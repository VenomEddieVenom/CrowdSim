#include "CrowdSystem.h"
#include <queue>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>

using namespace DirectX;

static constexpr int GS = CityLayout::GRID_SIZE;
static constexpr float CS = CityLayout::CELL_SIZE;
static constexpr float HCS = CS * 0.5f;
static constexpr float HW = CityLayout::CELL_SIZE * GS * 0.5f;

static const int DDX[] = {1, -1, 0, 0};
static const int DDZ[] = {0, 0, 1, -1};

// ============================================================
//  Initialize
// ============================================================
void CrowdSystem::Initialize()
{
    activeCount_ = 0;
    posX_.resize(MAX_PEDS, 0.f);
    posZ_.resize(MAX_PEDS, 0.f);
    speed_.resize(MAX_PEDS, 0.f);
    heading_.resize(MAX_PEDS, 0.f);
    state_.resize(MAX_PEDS, State::IDLE);
    dir_.resize(MAX_PEDS, 0);
    wpBuf_.resize(MAX_PEDS);
    wpCurr_.resize(MAX_PEDS, 0);
    cellHead_.assign(GS * GS, UINT32_MAX);
    cellNext_.resize(MAX_PEDS, UINT32_MAX);
    visIdx_.resize(MAX_VISIBLE);
    CreateInstPool();
}

// ============================================================
//  SpawnPed
// ============================================================
uint32_t CrowdSystem::SpawnPed(int homeGX, int homeGZ, int workGX, int workGZ,
                                const CityLayout& city)
{
    if (activeCount_ >= MAX_PEDS) return UINT32_MAX;

    auto path = FindPedPath(homeGX, homeGZ, workGX, workGZ, city);
    if (path.size() < 2) return UINT32_MAX;

    uint32_t i = activeCount_++;
    posX_[i] = path[0].x;
    posZ_[i] = path[0].y;

    // Random speed
    static std::mt19937 rng(42);
    std::uniform_real_distribution<float> spdDist(PED_SPEED_MIN, PED_SPEED_MAX);
    speed_[i] = spdDist(rng);

    // Direction from first waypoint
    float dx = path[1].x - path[0].x;
    float dz = path[1].y - path[0].y;
    heading_[i] = std::atan2(dx, dz);

    state_[i] = State::WALKING;
    dir_[i] = 0;
    wpBuf_[i] = std::move(path);
    wpCurr_[i] = 1;

    return i;
}

// ============================================================
//  FindPedPath — Dijkstra on road cells → sidewalk waypoints
// ============================================================
std::vector<XMFLOAT2> CrowdSystem::FindPedPath(
    int srcGX, int srcGZ, int dstGX, int dstGZ,
    const CityLayout& city)
{
    auto InBounds = [](int gx, int gz) { return gx >= 0 && gx < GS && gz >= 0 && gz < GS; };

    // Find road cells adjacent to src/dst
    int srcRX = -1, srcRZ = -1, dstRX = -1, dstRZ = -1;
    for (int d = 0; d < 4; d++) {
        int nx = srcGX + DDX[d], nz = srcGZ + DDZ[d];
        if (InBounds(nx, nz) && city.IsRoadLike(nx, nz)) { srcRX = nx; srcRZ = nz; break; }
    }
    for (int d = 0; d < 4; d++) {
        int nx = dstGX + DDX[d], nz = dstGZ + DDZ[d];
        if (InBounds(nx, nz) && city.IsRoadLike(nx, nz)) { dstRX = nx; dstRZ = nz; break; }
    }
    if (srcRX < 0 || dstRX < 0) return {};

    // Dijkstra on road-like cells
    int srcId = srcRZ * GS + srcRX;
    int dstId = dstRZ * GS + dstRX;

    std::vector<float> cost(GS * GS, 1e9f);
    std::vector<int>   from(GS * GS, -1);
    using PQ = std::priority_queue<std::pair<float,int>,
                                    std::vector<std::pair<float,int>>,
                                    std::greater<std::pair<float,int>>>;
    PQ pq;
    cost[srcId] = 0.f;
    from[srcId] = srcId;
    pq.push({0.f, srcId});
    bool found = (srcId == dstId);

    while (!pq.empty() && !found) {
        auto [cc, cur] = pq.top(); pq.pop();
        if (cc > cost[cur]) continue;
        int cx = cur % GS, cz = cur / GS;
        for (int d = 0; d < 4; d++) {
            int nx = cx + DDX[d], nz = cz + DDZ[d];
            if (!InBounds(nx, nz) || !city.IsRoadLike(nx, nz)) continue;
            int nid = nz * GS + nx;
            float nc = cc + 1.f;
            if (nc < cost[nid]) {
                cost[nid] = nc;
                from[nid] = cur;
                if (nid == dstId) { found = true; break; }
                pq.push({nc, nid});
            }
        }
    }
    if (!found) return {};

    // Trace back cell path
    std::vector<int> cellPath;
    for (int cur = dstId; cur != srcId; cur = from[cur])
        cellPath.push_back(cur);
    cellPath.push_back(srcId);
    std::reverse(cellPath.begin(), cellPath.end());

    if (cellPath.size() < 2) {
        auto cc = city.GridCellCenter(srcRX, srcRZ);
        return {{cc.x + SIDEWALK_MID, cc.y}};
    }

    // Convert cell path to sidewalk waypoints.
    // Peds walk ONLY on sidewalks and crosswalks — never on road surface.
    // Right-hand traffic convention: walk on right side of road.
    std::vector<XMFLOAT2> waypoints;

    auto sidewalkOff = [](int dx, int dz, float& ox, float& oz) {
        ox = oz = 0.f;
        if (dx == 0) ox = (dz < 0) ? SIDEWALK_MID : -SIDEWALK_MID;
        if (dz == 0) oz = (dx > 0) ? SIDEWALK_MID : -SIDEWALK_MID;
    };

    for (size_t i = 0; i < cellPath.size(); i++) {
        int cx = cellPath[i] % GS, cz = cellPath[i] / GS;
        auto cc = city.GridCellCenter(cx, cz);

        // Incoming direction
        int pddx = 0, pddz = 0;
        if (i > 0) {
            int px = cellPath[i-1] % GS, pz = cellPath[i-1] / GS;
            pddx = cx - px; pddz = cz - pz;
        } else if (cellPath.size() > 1) {
            int nx = cellPath[1] % GS, nz = cellPath[1] / GS;
            pddx = nx - cx; pddz = nz - cz;
        }

        // Outgoing direction
        int nddx = pddx, nddz = pddz;
        if (i + 1 < cellPath.size()) {
            int nx = cellPath[i+1] % GS, nz = cellPath[i+1] / GS;
            nddx = nx - cx; nddz = nz - cz;
        }

        // Detect turn
        bool isTurn = (i > 0 && i + 1 < cellPath.size() && (pddx != nddx || pddz != nddz));

        if (!isTurn) {
            // Straight: sidewalk waypoint
            float ox, oz;
            sidewalkOff(pddx, pddz, ox, oz);
            waypoints.push_back({cc.x + ox, cc.y + oz});
            continue;
        }

        // === TURN at intersection cell ===
        // Previous cell = approach crosswalk. Crossing happens there.
        int px = cellPath[i-1] % GS, pz = cellPath[i-1] / GS;
        auto prevC = city.GridCellCenter(px, pz);

        float pox, poz;  // incoming sidewalk offset
        sidewalkOff(pddx, pddz, pox, poz);
        float nox, noz;  // outgoing sidewalk offset
        sidewalkOff(nddx, nddz, nox, noz);

        // Inner edge of approach crosswalk cell (boundary with intersection)
        float edgeAlongX = (float)pddx * HCS;
        float edgeAlongZ = (float)pddz * HCS;

        int turn = pddx * nddz - pddz * nddx; // >0 right, <0 left

        if (turn > 0) {
            // RIGHT turn: stay on same sidewalk, round the corner
            // 1. Approach: inner edge of crosswalk cell, on current sidewalk
            waypoints.push_back({prevC.x + pox + edgeAlongX,
                                 prevC.y + poz + edgeAlongZ});
            // 2. Corner: intersection outer corner where two sidewalks meet
            float crnX = (pox != 0.f) ? pox : nox;
            float crnZ = (poz != 0.f) ? poz : noz;
            waypoints.push_back({cc.x + crnX, cc.y + crnZ});
        } else {
            // LEFT turn: cross at approach crosswalk, then walk around
            // 1. Approach: inner edge of crosswalk, current sidewalk
            waypoints.push_back({prevC.x + pox + edgeAlongX,
                                 prevC.y + poz + edgeAlongZ});
            // 2. Cross road at crosswalk inner edge (opposite sidewalk)
            waypoints.push_back({prevC.x - pox + edgeAlongX,
                                 prevC.y - poz + edgeAlongZ});
            // 3. Far corner of intersection
            float crnX = (pox != 0.f) ? -pox : nox;
            float crnZ = (poz != 0.f) ? -poz : noz;
            waypoints.push_back({cc.x + crnX, cc.y + crnZ});
        }
        // Exit waypoint on next cell is handled when that cell is processed
    }

    if (waypoints.size() > MAX_WP)
        waypoints.resize(MAX_WP);

    return waypoints;
}

// ============================================================
//  CanPedCross — query traffic light for crosswalk
// ============================================================
bool CrowdSystem::CanPedCross(int gx, int gz,
                               const CityLayout& city,
                               const TrafficLightSystem& lights)
{
    auto InBounds = [](int gx, int gz) { return gx >= 0 && gx < GS && gz >= 0 && gz < GS; };

    for (int d = 0; d < 4; d++) {
        int nx = gx + DDX[d], nz = gz + DDZ[d];
        if (!InBounds(nx, nz) || !lights.IsIntersection(nx, nz)) continue;
        int dx2 = gx - nx, dz2 = gz - nz;
        bool crossAxisNS = (dx2 == 0);
        auto axis = crossAxisNS ? TrafficLightSystem::Axis::NS
                                : TrafficLightSystem::Axis::EW;
        return lights.CanPedestrianCross(nx, nz, axis);
    }
    return true; // no intersection nearby → safe to cross
}

// ============================================================
//  RebuildSpatialHash
// ============================================================
void CrowdSystem::RebuildSpatialHash()
{
    std::fill(cellHead_.begin(), cellHead_.end(), UINT32_MAX);
    std::fill(cellNext_.begin(), cellNext_.begin() + activeCount_, UINT32_MAX);

    for (uint32_t i = 0; i < activeCount_; i++) {
        if (state_[i] == State::IDLE) continue;
        int gx = (int)std::floor((posX_[i] + HW) / CS);
        int gz = (int)std::floor((posZ_[i] + HW) / CS);
        if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) continue;
        int key = gz * GS + gx;
        cellNext_[i] = cellHead_[key];
        cellHead_[key] = i;
    }
}

// ============================================================
//  Update
// ============================================================
void CrowdSystem::Update(float dt, const CityLayout& city,
                          const TrafficLightSystem& lights,
                          const CarView* carView)
{
    RebuildSpatialHash();

    for (uint32_t i = 0; i < activeCount_; i++) {
        if (state_[i] == State::IDLE) continue;

        auto& wps = wpBuf_[i];
        uint16_t& wn = wpCurr_[i];
        uint16_t wc = (uint16_t)wps.size();

        // End of path → reverse
        if (wn >= wc) {
            std::reverse(wps.begin(), wps.end());
            wn = 1;
            dir_[i] ^= 1;
            continue;
        }

        XMFLOAT2 target = wps[wn];
        float dx = target.x - posX_[i];
        float dz = target.y - posZ_[i];
        float distToTarget = std::sqrt(dx * dx + dz * dz);

        // Crosswalk check
        int myGX, myGZ;
        bool onGrid = city.WorldToGrid(posX_[i], posZ_[i], myGX, myGZ);

        // Only stop at red if ped hasn't started crossing yet.
        // If already on a crosswalk or intersection, keep going to finish crossing.
        bool alreadyCrossing = false;
        if (onGrid) {
            auto ct = city.GetCellType(myGX, myGZ);
            alreadyCrossing = city.IsCrosswalk(myGX, myGZ) ||
                              (ct == CityLayout::CellType::ROAD && lights.IsIntersection(myGX, myGZ));
        }

        if (onGrid && !alreadyCrossing) {
            int tgtGX, tgtGZ;
            if (city.WorldToGrid(target.x, target.y, tgtGX, tgtGZ)) {
                if (city.IsCrosswalk(tgtGX, tgtGZ) && !CanPedCross(tgtGX, tgtGZ, city, lights)) {
                    state_[i] = State::WAITING_CROSS;
                    continue;
                }
            }
        }

        state_[i] = State::WALKING;
        float spd = speed_[i];

        // Ped yields to approaching cars (skip if on sidewalk)
        if (carView && carView->count > 0 && onGrid && !city.IsOnSidewalk(posX_[i], posZ_[i]) && distToTarget > 0.01f) {
            constexpr int GS = CityLayout::GRID_SIZE;
            for (int dg = -1; dg <= 1 && spd > 0.0f; dg++)
            for (int dh = -1; dh <= 1 && spd > 0.0f; dh++) {
                int cgx = myGX + dh, cgz = myGZ + dg;
                if (cgx < 0 || cgx >= GS || cgz < 0 || cgz >= GS) continue;
                int ck = cgz * GS + cgx;
                for (uint32_t c = carView->cellHead[ck]; c != UINT32_MAX; c = carView->cellNext[c]) {
                    if (carView->speed[c] < 0.5f) continue;
                    float toX = posX_[i] - carView->posX[c];
                    float toZ = posZ_[i] - carView->posZ[c];
                    float d2 = toX * toX + toZ * toZ;
                    if (d2 > 64.0f || d2 < 0.01f) continue; // within 8m
                    float cfx = std::sin(carView->heading[c]);
                    float cfz = std::cos(carView->heading[c]);
                    float ahead = toX * cfx + toZ * cfz;
                    float lat = std::abs(toX * cfz - toZ * cfx);
                    if (ahead > 0.0f && ahead < 8.0f && lat < 2.5f) { spd = 0.0f; break; }
                }
            }
        }

        if (distToTarget < 0.3f) {
            ++wn;
            continue;
        }

        float ndx = dx / distToTarget, ndz = dz / distToTarget;
        float step = std::min(spd * dt, distToTarget);
        posX_[i] += ndx * step;
        posZ_[i] += ndz * step;
        heading_[i] = std::atan2(ndx, ndz);

        // Snap to sidewalk layer if strayed off
        if (!city.IsOnSidewalk(posX_[i], posZ_[i]))
            city.SnapToSidewalk(posX_[i], posZ_[i]);
    }
}

// ============================================================
//  GetView — return lightweight snapshot for CarSystem
// ============================================================
PedestrianView CrowdSystem::GetView() const
{
    PedestrianView v;
    v.posX     = posX_.data();
    v.posZ     = posZ_.data();
    v.state    = reinterpret_cast<const uint8_t*>(state_.data());
    v.count    = activeCount_;
    v.cellHead = cellHead_.data();
    v.cellNext = cellNext_.data();
    return v;
}

uint32_t CrowdSystem::GetWalkingCount() const
{
    uint32_t c = 0;
    for (uint32_t i = 0; i < activeCount_; i++)
        if (state_[i] == State::WALKING) c++;
    return c;
}

uint32_t CrowdSystem::GetWaitingCount() const
{
    uint32_t c = 0;
    for (uint32_t i = 0; i < activeCount_; i++)
        if (state_[i] == State::WAITING_CROSS) c++;
    return c;
}

// ============================================================
//  CreateInstPool — instanced cubes for pedestrians
// ============================================================
void CrowdSystem::CreateInstPool()
{
    auto& scene = wi::scene::GetScene();

    meshEnt_ = scene.Entity_CreateCube("ped_mesh");
    scene.materials.GetComponent(meshEnt_)->SetBaseColor(
        XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f));
    auto* mt = scene.transforms.GetComponent(meshEnt_);
    mt->Scale(XMFLOAT3(0.f, 0.f, 0.f));
    mt->UpdateTransform();

    instPool_.resize(MAX_VISIBLE);
    for (uint32_t j = 0; j < MAX_VISIBLE; ++j) {
        wi::ecs::Entity e = wi::ecs::CreateEntity();
        scene.layers.Create(e);
        auto& tr = scene.transforms.Create(e);
        tr.Translate(XMFLOAT3(0.f, -1000.f, 0.f));
        tr.UpdateTransform();
        auto& obj = scene.objects.Create(e);
        obj.meshID = meshEnt_;
        obj.color  = XMFLOAT4(0.2f, 0.3f, 0.7f, 1.f); // blue-ish default
        instPool_[j] = e;
    }
}

// ============================================================
//  Render — place visible ped instances
// ============================================================
void CrowdSystem::Render(const XMFLOAT3& cameraPos)
{
    auto& scene = wi::scene::GetScene();

    const float camX = cameraPos.x, camZ = cameraPos.z;
    const float farSq = 400.f * 400.f;   // ped draw distance
    const float midSq = 200.f * 200.f;

    static constexpr float PED_HW = 0.15f;  // half-width  (30 cm shoulder width)
    static constexpr float PED_HH = 0.45f;  // half-height (90 cm → 1.8m tall)
    static constexpr float PED_HD = 0.10f;  // half-depth  (20 cm front-to-back)

    uint32_t count = 0;
    for (uint32_t i = 0; i < activeCount_ && count < MAX_VISIBLE; ++i) {
        if (state_[i] == State::IDLE) continue;
        float dx = posX_[i] - camX, dz = posZ_[i] - camZ;
        float d2 = dx * dx + dz * dz;
        if (d2 >= farSq) continue;
        if (d2 >= midSq && (i & 1u) != 0) continue;
        visIdx_[count++] = i;
    }
    visCount_ = count;

    // Random but deterministic colors per ped
    static const XMFLOAT4 pedColors[] = {
        {0.20f, 0.30f, 0.70f, 1.f}, // blue
        {0.70f, 0.20f, 0.20f, 1.f}, // red
        {0.20f, 0.60f, 0.20f, 1.f}, // green
        {0.65f, 0.55f, 0.10f, 1.f}, // yellow
        {0.50f, 0.25f, 0.60f, 1.f}, // purple
        {0.70f, 0.40f, 0.15f, 1.f}, // orange
        {0.30f, 0.55f, 0.55f, 1.f}, // teal
        {0.55f, 0.35f, 0.25f, 1.f}, // brown
    };
    static constexpr int NUM_COLORS = sizeof(pedColors) / sizeof(pedColors[0]);

    for (uint32_t s = 0; s < count; ++s) {
        const uint32_t i = visIdx_[s];
        auto* tr = scene.transforms.GetComponent(instPool_[s]);
        if (!tr) continue;

        float pedY = PED_HH + 0.14f; // sidewalk height
        tr->ClearTransform();
        tr->Scale(XMFLOAT3(PED_HW * 2.f, PED_HH * 2.f, PED_HD * 2.f));
        tr->RotateRollPitchYaw(XMFLOAT3(0.f, heading_[i], 0.f));
        tr->Translate(XMFLOAT3(posX_[i], pedY, posZ_[i]));
        tr->SetDirty();

        auto* obj = scene.objects.GetComponent(instPool_[s]);
        if (obj) obj->color = pedColors[i % NUM_COLORS];
    }

    // Hide unused slots
    for (uint32_t j = count; j < MAX_VISIBLE; ++j) {
        auto* tr = scene.transforms.GetComponent(instPool_[j]);
        if (tr) { tr->ClearTransform(); tr->Translate(XMFLOAT3(0.f, -1000.f, 0.f)); tr->SetDirty(); }
    }
}