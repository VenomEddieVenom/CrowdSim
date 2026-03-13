#pragma once
#include "WickedEngine.h"
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <fstream>

// ============================================================
//  CityLayout  –  user-editable grid city
//
//  40×40 cells, 20 m each (world: ±400 m).
//  Cell types: EMPTY, ROAD, HOUSE, WORKPLACE.
//  FindPath() does BFS on ROAD cells and returns world-space
//  waypoints for CrowdSystem::SpawnAgents().
// ============================================================
class CityLayout
{
public:
    static constexpr int   GRID_SIZE  = 40;
    static constexpr float CELL_SIZE  = 20.0f;
    static constexpr float HALF_WORLD = GRID_SIZE * CELL_SIZE * 0.5f;   // 400 m
    static constexpr float SIDEWALK_W = 2.0f;
    static constexpr float GRID_CELL  = 2.0f;  // legacy API compat

    // Sidewalk walkability layer: 10×10 sub-cells per grid cell (2 m resolution)
    static constexpr int   SW_SUB      = 10;
    static constexpr float SW_SUB_SIZE = CELL_SIZE / SW_SUB;          // 2 m
    static constexpr int   SW_DIM      = GRID_SIZE * SW_SUB;          // 400
    bool sidewalkLayer[SW_DIM * SW_DIM] = {};

    enum class CellType : uint8_t { EMPTY = 0, ROAD, HOUSE, WORKPLACE, CROSSWALK, PARKING };

    // Flat arrays indexed [gz * GRID_SIZE + gx]
    CellType                     cellType   [GRID_SIZE * GRID_SIZE] = {};
    std::vector<wi::ecs::Entity> cellEntities[GRID_SIZE * GRID_SIZE];
    // Per-road-cell sidewalk entity cache [idx][side]  side: 0=E,1=W,2=S,3=N
    // Sidewalks are never removed during neighbor rebuilds — only moved underground.
    wi::ecs::Entity roadSidewalks[GRID_SIZE * GRID_SIZE][4] = {};
    // Per-road-cell corner fill entities [idx][corner]  corner: 0=NE,1=NW,2=SE,3=SW
    // These fill the gap where two perpendicular connected sides meet.
    wi::ecs::Entity roadCorners[GRID_SIZE * GRID_SIZE][4] = {};

    // Per-cell lane count: 2, 4, or 6 total lanes.  Default = 4.
    int roadLanes_[GRID_SIZE * GRID_SIZE] = {};

    int  GetRoadLanes(int gx, int gz) const {
        if (gx < 0 || gx >= GRID_SIZE || gz < 0 || gz >= GRID_SIZE) return 4;
        int v = roadLanes_[gz * GRID_SIZE + gx];
        return (v == 2 || v == 6) ? v : 4;
    }
    void SetRoadLanes(int gx, int gz, int lanes) {
        if (gx >= 0 && gx < GRID_SIZE && gz >= 0 && gz < GRID_SIZE)
            roadLanes_[gz * GRID_SIZE + gx] = lanes;
    }
    void CycleRoadLanes(int gx, int gz) {
        int cur = GetRoadLanes(gx, gz);
        int next = (cur == 2) ? 4 : (cur == 4) ? 6 : 2;
        SetRoadLanes(gx, gz, next);
    }

    // Legacy occupancy (kept for API compat)
    int gridW = 0, gridH = 0;
    std::vector<uint8_t> occupancy;

    bool IsBlocked(float wx, float wz) const
    {
        int gx, gz;
        if (!WorldToGrid(wx, wz, gx, gz)) return false;
        auto t = GetCellType(gx, gz);
        return t == CellType::HOUSE || t == CellType::WORKPLACE;
    }

    bool IsCrosswalk(int gx, int gz) const
    {
        if (!InBounds(gx, gz)) return false;
        return cellType[gz * GRID_SIZE + gx] == CellType::CROSSWALK;
    }

    bool IsRoadLike(int gx, int gz) const
    {
        if (!InBounds(gx, gz)) return false;
        auto t = cellType[gz * GRID_SIZE + gx];
        return t == CellType::ROAD || t == CellType::CROSSWALK;
    }

    // ----------------------------------------------------------
    //  Init – ground plane only; user builds the rest
    // ----------------------------------------------------------
    void Initialize()
    {
        gridW = gridH = static_cast<int>(HALF_WORLD * 2.0f / GRID_CELL) + 1;
        occupancy.assign(static_cast<size_t>(gridW) * gridH, 0);

        auto& scene = wi::scene::GetScene();
        auto  ground = scene.Entity_CreateCube("ground");
        auto* tr  = scene.transforms.GetComponent(ground);
        tr->Scale(XMFLOAT3(HALF_WORLD, 0.05f, HALF_WORLD));
        tr->Translate(XMFLOAT3(0.0f, -0.05f, 0.0f));
        tr->UpdateTransform();
        scene.materials.GetComponent(ground)->SetBaseColor(
            XMFLOAT4(0.18f, 0.38f, 0.12f, 1.0f)); // green grass (desaturated)

        // Load dom.wiscene ONCE into a prefab scene, then merge its meshes/materials
        // into the main scene. All houses share the same mesh entity IDs → GPU instancing.
        wi::scene::LoadModel(domPrefab_, "models/dom.wiscene");
        if (domPrefab_.objects.GetCount() > 0) {
            domPrefabLoaded_ = true;
            // Compute world matrices for all prefab entities
            domPrefab_.Update(0.0f);
            // Collect (meshID, world-space local matrix) for every sub-object in the prefab
            std::vector<wi::ecs::Entity> protoEnts;
            for (size_t pi = 0; pi < domPrefab_.objects.GetCount(); ++pi) {
                wi::ecs::Entity ent = domPrefab_.objects.GetEntity(pi);
                auto& objComp = domPrefab_.objects[pi];
                DomSubObject sub;
                sub.meshID = objComp.meshID;
                sub.color  = objComp.color;
                auto* tr = domPrefab_.transforms.GetComponent(ent);
                if (tr) sub.localMatrix = tr->world;
                else XMStoreFloat4x4(&sub.localMatrix, XMMatrixIdentity());
                domSubObjects_.push_back(sub);
                protoEnts.push_back(ent);
            }
            // Also collect hierarchy-only (non-mesh, non-material) transforms for removal
            for (size_t pi = 0; pi < domPrefab_.transforms.GetCount(); ++pi) {
                wi::ecs::Entity ent = domPrefab_.transforms.GetEntity(pi);
                if (!domPrefab_.meshes.GetComponent(ent) &&
                    !domPrefab_.materials.GetComponent(ent) &&
                    !domPrefab_.objects.GetComponent(ent))
                    protoEnts.push_back(ent);
            }
            // Merge meshes + materials (and everything else) into main scene
            scene.Merge(domPrefab_);
            // Remove only the ObjectComponent from prototype placement entities so they
            // don't render at the origin. MeshComponent must stay – it is the shared mesh
            // that all house instances reference via meshID. Entity_Remove would also kill
            // the MeshComponent when mesh and object live on the same entity.
            for (auto e : protoEnts)
                scene.objects.Remove(e);
            wi::backlog::post("[CityLayout] dom.wiscene merged – GPU instancing enabled",
                              wi::backlog::LogLevel::Default);
        } else {
            wi::backlog::post("[CityLayout] dom.wiscene not found, falling back to procedural houses",
                              wi::backlog::LogLevel::Warning);
        }
    }

    // ----------------------------------------------------------
    //  Coordinate utilities
    // ----------------------------------------------------------
    bool WorldToGrid(float wx, float wz, int& gx, int& gz) const
    {
        gx = static_cast<int>(std::floor((wx + HALF_WORLD) / CELL_SIZE));
        gz = static_cast<int>(std::floor((wz + HALF_WORLD) / CELL_SIZE));
        return InBounds(gx, gz);
    }

    XMFLOAT2 GridCellCenter(int gx, int gz) const
    {
        return { -HALF_WORLD + (gx + 0.5f) * CELL_SIZE,
                 -HALF_WORLD + (gz + 0.5f) * CELL_SIZE };
    }

    CellType GetCellType(int gx, int gz) const
    {
        if (!InBounds(gx, gz)) return CellType::EMPTY;
        return cellType[gz * GRID_SIZE + gx];
    }

    // ----------------------------------------------------------
    //  Sidewalk walkability layer
    // ----------------------------------------------------------
    void BuildSidewalkLayer(const std::function<bool(int,int)>& isIntersection)
    {
        std::memset(sidewalkLayer, 0, sizeof(sidewalkLayer));
        for (int gz = 0; gz < GRID_SIZE; gz++)
        for (int gx = 0; gx < GRID_SIZE; gx++) {
            CellType ct = GetCellType(gx, gz);
            bool hasN = IsRoadLike(gx, gz-1), hasS = IsRoadLike(gx, gz+1);
            bool hasE = IsRoadLike(gx+1, gz), hasW = IsRoadLike(gx-1, gz);
            int bx = gx * SW_SUB, bz = gz * SW_SUB;

            if (ct == CellType::CROSSWALK) {
                // Find which neighbour is the intersection → determines road direction
                bool intN = gz > 0 && isIntersection(gx, gz-1);
                bool intS = gz < GRID_SIZE-1 && isIntersection(gx, gz+1);
                bool intE = gx < GRID_SIZE-1 && isIntersection(gx+1, gz);
                bool intW = gx > 0 && isIntersection(gx-1, gz);
                bool nsRoad = intN || intS;   // road runs N↔S
                bool ewRoad = intE || intW;   // road runs E↔W
                for (int sz = 0; sz < SW_SUB; sz++)
                for (int sx = 0; sx < SW_SUB; sx++) {
                    bool mark = false;
                    if (nsRoad) {
                        // Sidewalk strips only on edges facing non-road neighbours
                        if (!hasW && sx == 0) mark = true;
                        if (!hasE && sx == SW_SUB-1) mark = true;
                        // Crossing band: 2 sub-cells at intersection-facing edge (1 more in intersection cell)
                        if (intN && sz <= 1) mark = true;
                        if (intS && sz >= SW_SUB-2) mark = true;
                    }
                    if (ewRoad) {
                        if (!hasN && sz == 0) mark = true;
                        if (!hasS && sz == SW_SUB-1) mark = true;
                        // Crossing band: 2 sub-cells at intersection-facing edge
                        if (intE && sx >= SW_SUB-2) mark = true;
                        if (intW && sx <= 1) mark = true;
                    }
                    if (mark) sidewalkLayer[(bz+sz)*SW_DIM + bx+sx] = true;
                }
            }
            else if (ct == CellType::ROAD && isIntersection(gx, gz)) {
                // Only mark edges facing non-road neighbours
                for (int sz = 0; sz < SW_SUB; sz++)
                for (int sx = 0; sx < SW_SUB; sx++) {
                    bool mark = false;
                    if (!hasW && sx == 0) mark = true;
                    if (!hasE && sx == SW_SUB-1) mark = true;
                    if (!hasN && sz == 0) mark = true;
                    if (!hasS && sz == SW_SUB-1) mark = true;
                    if (mark) sidewalkLayer[(bz+sz)*SW_DIM + bx+sx] = true;
                }
                // Mark visible corners (where two connected sides meet)
                auto setBit = [&](int sx, int sz){ sidewalkLayer[(bz+sz)*SW_DIM+bx+sx] = true; };
                if (hasE && hasN) setBit(SW_SUB-1, 0);
                if (hasW && hasN) setBit(0, 0);
                if (hasE && hasS) setBit(SW_SUB-1, SW_SUB-1);
                if (hasW && hasS) setBit(0, SW_SUB-1);
                // Crossing band: 1 sub-cell at edge facing a crosswalk (extends the band from the crosswalk cell)
                auto isCW = [&](int cx, int cz){ return InBounds(cx,cz) && cellType[cz*GRID_SIZE+cx]==CellType::CROSSWALK; };
                if (isCW(gx, gz-1)) for (int sx=0; sx<SW_SUB; sx++) sidewalkLayer[bz*SW_DIM+bx+sx] = true;
                if (isCW(gx, gz+1)) for (int sx=0; sx<SW_SUB; sx++) sidewalkLayer[(bz+SW_SUB-1)*SW_DIM+bx+sx] = true;
                if (isCW(gx+1, gz)) for (int sz=0; sz<SW_SUB; sz++) sidewalkLayer[(bz+sz)*SW_DIM+bx+SW_SUB-1] = true;
                if (isCW(gx-1, gz)) for (int sz=0; sz<SW_SUB; sz++) sidewalkLayer[(bz+sz)*SW_DIM+bx] = true;
            }
            else if (ct == CellType::ROAD) {
                // Only mark edges facing non-road neighbours (1 sub-cell wide)
                for (int sz = 0; sz < SW_SUB; sz++)
                for (int sx = 0; sx < SW_SUB; sx++) {
                    if (!hasW && sx == 0) sidewalkLayer[(bz+sz)*SW_DIM + bx+sx] = true;
                    if (!hasE && sx == SW_SUB-1) sidewalkLayer[(bz+sz)*SW_DIM + bx+sx] = true;
                    if (!hasN && sz == 0) sidewalkLayer[(bz+sz)*SW_DIM + bx+sx] = true;
                    if (!hasS && sz == SW_SUB-1) sidewalkLayer[(bz+sz)*SW_DIM + bx+sx] = true;
                }
                // Mark visible corners
                auto setBit2 = [&](int sx, int sz){ sidewalkLayer[(bz+sz)*SW_DIM+bx+sx] = true; };
                if (hasE && hasN) setBit2(SW_SUB-1, 0);
                if (hasW && hasN) setBit2(0, 0);
                if (hasE && hasS) setBit2(SW_SUB-1, SW_SUB-1);
                if (hasW && hasS) setBit2(0, SW_SUB-1);
            }
            // Buildings don't mark sidewalk — road cells own their edges
        }
    }

    bool IsOnSidewalk(float wx, float wz) const
    {
        int sx = (int)std::floor((wx + HALF_WORLD) / SW_SUB_SIZE);
        int sz = (int)std::floor((wz + HALF_WORLD) / SW_SUB_SIZE);
        if (sx < 0 || sx >= SW_DIM || sz < 0 || sz >= SW_DIM) return false;
        return sidewalkLayer[sz * SW_DIM + sx];
    }

    void SnapToSidewalk(float& wx, float& wz) const
    {
        int sx = (int)std::floor((wx + HALF_WORLD) / SW_SUB_SIZE);
        int sz = (int)std::floor((wz + HALF_WORLD) / SW_SUB_SIZE);
        sx = std::clamp(sx, 0, SW_DIM-1);
        sz = std::clamp(sz, 0, SW_DIM-1);
        if (sidewalkLayer[sz * SW_DIM + sx]) return;
        for (int r = 1; r <= 10; r++) {
            float bestD2 = 1e9f; int bestSX = sx, bestSZ = sz;
            for (int dz = -r; dz <= r; dz++)
            for (int dx = -r; dx <= r; dx++) {
                if (std::abs(dx) != r && std::abs(dz) != r) continue;
                int nx = sx + dx, nz = sz + dz;
                if (nx < 0 || nx >= SW_DIM || nz < 0 || nz >= SW_DIM) continue;
                if (!sidewalkLayer[nz * SW_DIM + nx]) continue;
                float cx = -HALF_WORLD + (nx + 0.5f) * SW_SUB_SIZE;
                float cz = -HALF_WORLD + (nz + 0.5f) * SW_SUB_SIZE;
                float dd = (cx-wx)*(cx-wx) + (cz-wz)*(cz-wz);
                if (dd < bestD2) { bestD2 = dd; bestSX = nx; bestSZ = nz; }
            }
            if (bestD2 < 1e8f) {
                wx = -HALF_WORLD + (bestSX + 0.5f) * SW_SUB_SIZE;
                wz = -HALF_WORLD + (bestSZ + 0.5f) * SW_SUB_SIZE;
                return;
            }
        }
    }

    // ----------------------------------------------------------
    //  Placement – returns true when something changed
    // ----------------------------------------------------------
    bool PlaceCell(int gx, int gz, CellType type)
    {
        if (!InBounds(gx, gz)) return false;
        int idx = gz * GRID_SIZE + gx;
        if (cellType[idx] == type) return false;
        ClearCell(gx, gz);
        cellType[idx] = type;
        if (type == CellType::ROAD || type == CellType::CROSSWALK)
            if (roadLanes_[idx] == 0) roadLanes_[idx] = 4;  // default 4-lane
        auto c  = GridCellCenter(gx, gz);
        auto& s = wi::scene::GetScene();
        switch (type)
        {
        case CellType::ROAD:      BuildRoad(idx, gx, gz, c, s);      break;
        case CellType::HOUSE:     BuildHouse(idx, gx, gz, c, s);     break;
        case CellType::WORKPLACE: BuildWorkplace(idx, gx, gz, c, s); break;
        case CellType::CROSSWALK: BuildCrosswalk(idx, gx, gz, c, s); break;
        case CellType::PARKING:   BuildParking(idx, gx, gz, c, s);   break;
        default: break;
        }
        // Rebuild neighboring roads so they update their sidewalk open-sides
        const int ddx[4] = { 1,-1, 0, 0 };
        const int ddz[4] = { 0, 0, 1,-1 };
        for (int d = 0; d < 4; ++d)
            RebuildRoadCell(gx + ddx[d], gz + ddz[d]);
        return true;
    }

    void ClearCell(int gx, int gz)
    {
        if (!InBounds(gx, gz)) return;
        int idx = gz * GRID_SIZE + gx;
        auto& s = wi::scene::GetScene();
        for (auto e : cellEntities[idx]) s.Entity_Remove(e);
        cellEntities[idx].clear();
        cellType[idx] = CellType::EMPTY;
        // Reset the sidewalk entity cache so BuildRoad creates fresh ones if reused.
        for (int d = 0; d < 4; ++d) roadSidewalks[idx][d] = wi::ecs::INVALID_ENTITY;
        for (int d = 0; d < 4; ++d) roadCorners[idx][d] = wi::ecs::INVALID_ENTITY;
    }

    // ----------------------------------------------------------
    //  Save / Load  –  binary file: 4-byte magic + cell type grid
    // ----------------------------------------------------------
    bool SaveToFile(const char* path) const
    {
        std::ofstream f(path, std::ios::binary);
        if (!f) return false;
        uint32_t magic = 0x43495459; // "CITY"
        uint32_t ver   = 2;
        uint32_t gs    = GRID_SIZE;
        f.write(reinterpret_cast<const char*>(&magic), 4);
        f.write(reinterpret_cast<const char*>(&ver), 4);
        f.write(reinterpret_cast<const char*>(&gs), 4);
        f.write(reinterpret_cast<const char*>(cellType), GRID_SIZE * GRID_SIZE);
        f.write(reinterpret_cast<const char*>(housePopulation_), GRID_SIZE * GRID_SIZE * sizeof(int));
        f.write(reinterpret_cast<const char*>(roadLanes_), GRID_SIZE * GRID_SIZE * sizeof(int));
        return f.good();
    }

    // Returns a temporary grid of cell types; caller must rebuild the scene.
    // Does NOT modify this object — caller uses PlaceCell to rebuild.
    struct SaveData {
        CellType cells[GRID_SIZE * GRID_SIZE] = {};
        int      housePop[GRID_SIZE * GRID_SIZE] = {};
        int      roadLanes[GRID_SIZE * GRID_SIZE] = {};
        bool     valid = false;
    };

    static SaveData LoadFromFile(const char* path)
    {
        SaveData sd;
        std::ifstream f(path, std::ios::binary);
        if (!f) return sd;
        uint32_t magic = 0, ver = 0, gs = 0;
        f.read(reinterpret_cast<char*>(&magic), 4);
        f.read(reinterpret_cast<char*>(&ver), 4);
        f.read(reinterpret_cast<char*>(&gs), 4);
        if (magic != 0x43495459 || (ver != 1 && ver != 2)) return sd;
        if (gs != (uint32_t)GRID_SIZE) return sd;
        f.read(reinterpret_cast<char*>(sd.cells),    gs * gs);
        f.read(reinterpret_cast<char*>(sd.housePop), gs * gs * sizeof(int));
        if (ver >= 2)
            f.read(reinterpret_cast<char*>(sd.roadLanes), gs * gs * sizeof(int));
        if (!f.good()) return sd;
        sd.valid = true;
        return sd;
    }

    // Traffic density per cell — updated by CarSystem externally
    mutable int trafficCount_[GRID_SIZE * GRID_SIZE] = {};

    void SetTrafficCount(int gx, int gz, int count) const
    {
        if (InBounds(gx, gz)) trafficCount_[gz * GRID_SIZE + gx] = count;
    }
    int GetTrafficCount(int gx, int gz) const
    {
        if (!InBounds(gx, gz)) return 0;
        return trafficCount_[gz * GRID_SIZE + gx];
    }

    // ----------------------------------------------------------
    //  Pathfinding – Dijkstra on ROAD cells with traffic cost
    //  Returns world-space (x,z) waypoints:
    //    src building center → road path → dst building center
    // ----------------------------------------------------------
    std::vector<XMFLOAT2> FindPath(int srcGX, int srcGZ,
                                    int dstGX, int dstGZ) const
    {
        const int ddx[] = { 1, -1,  0,  0 };
        const int ddz[] = { 0,  0,  1, -1 };

        // Find exit road cells adjacent to each building
        int srcRX = -1, srcRZ = -1, dstRX = -1, dstRZ = -1;
        for (int d = 0; d < 4; ++d)
        {
            int nx = srcGX + ddx[d], nz = srcGZ + ddz[d];
            if (InBounds(nx, nz)) {
                auto ct = cellType[nz * GRID_SIZE + nx];
                if (ct == CellType::ROAD || ct == CellType::CROSSWALK)
                    { srcRX = nx; srcRZ = nz; break; }
            }
        }
        for (int d = 0; d < 4; ++d)
        {
            int nx = dstGX + ddx[d], nz = dstGZ + ddz[d];
            if (InBounds(nx, nz)) {
                auto ct = cellType[nz * GRID_SIZE + nx];
                if (ct == CellType::ROAD || ct == CellType::CROSSWALK)
                    { dstRX = nx; dstRZ = nz; break; }
            }
        }
        if (srcRX < 0 || dstRX < 0) return {};

        int srcId = srcRZ * GRID_SIZE + srcRX;
        int dstId = dstRZ * GRID_SIZE + dstRX;

        // Dijkstra with traffic-weighted cost
        constexpr float INF_COST = 1e9f;
        std::vector<float> cost(GRID_SIZE * GRID_SIZE, INF_COST);
        std::vector<int> from(GRID_SIZE * GRID_SIZE, -1);
        // min-heap: (cost, cellId)
        using PQ = std::priority_queue<std::pair<float,int>,
                   std::vector<std::pair<float,int>>,
                   std::greater<std::pair<float,int>>>;
        PQ pq;
        cost[srcId] = 0.0f;
        from[srcId] = srcId;
        pq.push({0.0f, srcId});
        bool found = (srcId == dstId);
        while (!pq.empty() && !found)
        {
            auto [curCost, cur] = pq.top(); pq.pop();
            if (curCost > cost[cur]) continue;
            int cx = cur % GRID_SIZE, cz = cur / GRID_SIZE;
            for (int d = 0; d < 4; ++d)
            {
                int nx = cx + ddx[d], nz = cz + ddz[d];
                if (!InBounds(nx, nz)) continue;
                int nid = nz * GRID_SIZE + nx;
                if (cellType[nid] != CellType::ROAD && cellType[nid] != CellType::CROSSWALK) continue;
                // Base cost 1.0 + 0.5 per car on the cell (traffic penalty)
                float edgeCost = 1.0f + 0.5f * (float)trafficCount_[nid];
                float newCost = curCost + edgeCost;
                if (newCost < cost[nid])
                {
                    cost[nid] = newCost;
                    from[nid] = cur;
                    if (nid == dstId) { found = true; break; }
                    pq.push({newCost, nid});
                }
            }
        }
        if (!found) return {};

        // Trace road path back from dst to src
        std::vector<XMFLOAT2> road;
        for (int cur = dstId; cur != srcId; cur = from[cur])
            road.push_back(GridCellCenter(cur % GRID_SIZE, cur / GRID_SIZE));
        road.push_back(GridCellCenter(srcRX, srcRZ));
        std::reverse(road.begin(), road.end());

        // Path is road cells only — agents walk on sidewalks, not inside buildings
        return road;
    }

    // ----------------------------------------------------------
    //  FindPathRoad – Dijkstra between two ROAD cells directly
    //  Used when re-routing (source/dest are already road cells).
    // ----------------------------------------------------------
    std::vector<XMFLOAT2> FindPathRoad(int srcRX, int srcRZ,
                                        int dstRX, int dstRZ) const
    {
        if (!InBounds(srcRX, srcRZ) || !InBounds(dstRX, dstRZ)) return {};
        auto srcCT = cellType[srcRZ * GRID_SIZE + srcRX];
        auto dstCT = cellType[dstRZ * GRID_SIZE + dstRX];
        if (srcCT != CellType::ROAD && srcCT != CellType::CROSSWALK) return {};
        if (dstCT != CellType::ROAD && dstCT != CellType::CROSSWALK) return {};

        int srcId = srcRZ * GRID_SIZE + srcRX;
        int dstId = dstRZ * GRID_SIZE + dstRX;

        const int ddx[] = { 1, -1,  0,  0 };
        const int ddz[] = { 0,  0,  1, -1 };

        constexpr float INF_COST = 1e9f;
        std::vector<float> cost(GRID_SIZE * GRID_SIZE, INF_COST);
        std::vector<int> from(GRID_SIZE * GRID_SIZE, -1);
        using PQ = std::priority_queue<std::pair<float,int>,
                   std::vector<std::pair<float,int>>,
                   std::greater<std::pair<float,int>>>;
        PQ pq;
        cost[srcId] = 0.0f;
        from[srcId] = srcId;
        pq.push({0.0f, srcId});
        bool found = (srcId == dstId);
        while (!pq.empty() && !found)
        {
            auto [curCost, cur] = pq.top(); pq.pop();
            if (curCost > cost[cur]) continue;
            int cx = cur % GRID_SIZE, cz = cur / GRID_SIZE;
            for (int d = 0; d < 4; ++d)
            {
                int nx = cx + ddx[d], nz = cz + ddz[d];
                if (!InBounds(nx, nz)) continue;
                int nid = nz * GRID_SIZE + nx;
                if (cellType[nid] != CellType::ROAD && cellType[nid] != CellType::CROSSWALK) continue;
                float edgeCost = 1.0f + 0.5f * (float)trafficCount_[nid];
                float newCost = curCost + edgeCost;
                if (newCost < cost[nid])
                {
                    cost[nid] = newCost;
                    from[nid] = cur;
                    if (nid == dstId) { found = true; break; }
                    pq.push({newCost, nid});
                }
            }
        }
        if (!found) return {};

        std::vector<XMFLOAT2> road;
        for (int cur = dstId; cur != srcId; cur = from[cur])
            road.push_back(GridCellCenter(cur % GRID_SIZE, cur / GRID_SIZE));
        road.push_back(GridCellCenter(srcRX, srcRZ));
        std::reverse(road.begin(), road.end());
        return road;
    }

private:
    bool InBounds(int gx, int gz) const
    {
        return gx >= 0 && gx < GRID_SIZE && gz >= 0 && gz < GRID_SIZE;
    }

    // Rebuild an existing road cell's connectivity display — NO entity removal.
    // Just moves each of the 4 pre-allocated sidewalk entities above or below ground.
    void RebuildRoadCell(int gx, int gz)
    {
        if (!InBounds(gx, gz)) return;
        int idx = gz * GRID_SIZE + gx;
        if (cellType[idx] != CellType::ROAD && cellType[idx] != CellType::CROSSWALK) return;

        const float h   = CELL_SIZE * 0.5f;
        const float swh = SIDEWALK_W * 0.5f;
        const XMFLOAT2 c = GridCellCenter(gx, gz);

        auto isRoadLike = [&](int cx, int cz) {
            if (!InBounds(cx, cz)) return false;
            auto t = cellType[cz * GRID_SIZE + cx];
            return t == CellType::ROAD || t == CellType::CROSSWALK;
        };

        const bool connected[4] = {
            isRoadLike(gx+1, gz),  // E
            isRoadLike(gx-1, gz),  // W
            isRoadLike(gx, gz+1),  // S
            isRoadLike(gx, gz-1),  // N
        };
        const float soff = 0.0f;  // no shift (unused, kept for reference)
        const float ox[4] = {  h - swh, -(h-swh), 0.f,      0.f      };
        const float oz[4] = {  0.f,      0.f,      h - swh, -(h-swh) };
        const float hx[4] = {  swh,      swh,      h,        h        };
        const float hz[4] = {  h,        h,        swh,      swh      };

        auto& s = wi::scene::GetScene();
        for (int d = 0; d < 4; ++d)
        {
            wi::ecs::Entity se = roadSidewalks[idx][d];
            if (se == wi::ecs::INVALID_ENTITY) continue;
            auto* st = s.transforms.GetComponent(se);
            if (!st) continue;
            // Show (0.14 above ground) or hide (100 below ground) without any entity creation/deletion.
            const float visY = connected[d] ? -100.0f : 0.14f;
            st->ClearTransform();
            st->Scale(XMFLOAT3(hx[d], 0.14f, hz[d]));
            st->Translate(XMFLOAT3(c.x + ox[d], visY, c.y + oz[d]));
            st->UpdateTransform();
        }

        // Update corner fill visibility.
        // Corners: 0=NE(E+N), 1=NW(W+N), 2=SE(E+S), 3=SW(W+S)
        // A corner is visible when BOTH adjacent sides are connected to road.
        const bool cornerVis[4] = {
            connected[0] && connected[3],  // NE: E connected AND N connected
            connected[1] && connected[3],  // NW: W connected AND N connected
            connected[0] && connected[2],  // SE: E connected AND S connected
            connected[1] && connected[2],  // SW: W connected AND S connected
        };
        const float cox[4] = {  h - swh, -(h-swh),  h - swh, -(h-swh) };
        const float coz[4] = { -(h-swh), -(h-swh),  h - swh,  h - swh };
        for (int d = 0; d < 4; ++d)
        {
            wi::ecs::Entity ce = roadCorners[idx][d];
            if (ce == wi::ecs::INVALID_ENTITY) continue;
            auto* ct = s.transforms.GetComponent(ce);
            if (!ct) continue;
            const float visY = cornerVis[d] ? 0.14f : -100.0f;
            ct->ClearTransform();
            ct->Scale(XMFLOAT3(swh, 0.14f, swh));
            ct->Translate(XMFLOAT3(c.x + cox[d], visY, c.y + coz[d]));
            ct->UpdateTransform();
        }
    }

    void BuildRoad(int idx, int gx, int gz, XMFLOAT2 c, wi::scene::Scene& s)
    {
        const float h   = CELL_SIZE * 0.5f;
        const float swh = SIDEWALK_W * 0.5f;

        // Asphalt base (full cell)
        {
            auto e  = s.Entity_CreateCube("road");
            auto* tr = s.transforms.GetComponent(e);
            if (tr) {
                tr->Scale(XMFLOAT3(h, 0.10f, h));
                tr->Translate(XMFLOAT3(c.x, 0.10f, c.y));
                tr->UpdateTransform();
            }
            auto* mat = s.materials.GetComponent(e);
            if (mat) mat->SetBaseColor(XMFLOAT4(0.10f, 0.10f, 0.12f, 1.0f));
            cellEntities[idx].push_back(e);
        }

        auto isRoadLike = [&](int cx, int cz) {
            if (!InBounds(cx, cz)) return false;
            auto t = cellType[cz * GRID_SIZE + cx];
            return t == CellType::ROAD || t == CellType::CROSSWALK;
        };

        // 4 permanent sidewalk entities — one per side (E,W,S,N).
        // Connected sides start underground; others visible at 0.14 m.
        const bool connected[4] = {
            isRoadLike(gx+1, gz),  // E
            isRoadLike(gx-1, gz),  // W
            isRoadLike(gx, gz+1),  // S
            isRoadLike(gx, gz-1),  // N
        };
        const float ox[4] = {  h - swh, -(h-swh), 0.f,      0.f      };
        const float oz[4] = {  0.f,      0.f,      h - swh, -(h-swh) };
        const float hx[4] = {  swh,      swh,      h,        h        };
        const float hz[4] = {  h,        h,        swh,      swh      };

        for (int d = 0; d < 4; ++d)
        {
            auto se = s.Entity_CreateCube("sidewalk");
            auto* st = s.transforms.GetComponent(se);
            if (st) {
                const float visY = connected[d] ? -100.0f : 0.14f;
                st->Scale(XMFLOAT3(hx[d], 0.14f, hz[d]));
                st->Translate(XMFLOAT3(c.x + ox[d], visY, c.y + oz[d]));
                st->UpdateTransform();
            }
            auto* mat = s.materials.GetComponent(se);
            if (mat) mat->SetBaseColor(XMFLOAT4(0.74f, 0.72f, 0.65f, 1.0f));
            cellEntities[idx].push_back(se);
            roadSidewalks[idx][d] = se;
        }

        // Corner fill pieces: small square at each corner where two perpendicular
        // connected sides meet. This fills the gap pedestrians need to walk around.
        // Corners: 0=NE(E+N), 1=NW(W+N), 2=SE(E+S), 3=SW(W+S)
        {
            const bool cornerVis[4] = {
                connected[0] && connected[3],
                connected[1] && connected[3],
                connected[0] && connected[2],
                connected[1] && connected[2],
            };
            const float cox[4] = {  h - swh, -(h-swh),  h - swh, -(h-swh) };
            const float coz[4] = { -(h-swh), -(h-swh),  h - swh,  h - swh };
            for (int d = 0; d < 4; ++d)
            {
                auto ce = s.Entity_CreateCube("sidewalk_corner");
                auto* ct = s.transforms.GetComponent(ce);
                if (ct) {
                    const float visY = cornerVis[d] ? 0.14f : -100.0f;
                    ct->Scale(XMFLOAT3(swh, 0.14f, swh));
                    ct->Translate(XMFLOAT3(c.x + cox[d], visY, c.y + coz[d]));
                    ct->UpdateTransform();
                }
                auto* mat = s.materials.GetComponent(ce);
                if (mat) mat->SetBaseColor(XMFLOAT4(0.74f, 0.72f, 0.65f, 1.0f));
                cellEntities[idx].push_back(ce);
                roadCorners[idx][d] = ce;
            }
        }
    }

    void BuildCrosswalk(int idx, int gx, int gz, XMFLOAT2 c, wi::scene::Scene& s)
    {
        const float h   = CELL_SIZE * 0.5f;
        const float swh = SIDEWALK_W * 0.5f;

        // Detect road direction from neighbors
        auto isRoadLike2 = [&](int cx, int cz) {
            if (!InBounds(cx, cz)) return false;
            auto t = cellType[cz * GRID_SIZE + cx];
            return t == CellType::ROAD || t == CellType::CROSSWALK;
        };

        bool hasE = isRoadLike2(gx+1, gz);
        bool hasW = isRoadLike2(gx-1, gz);
        bool hasN = isRoadLike2(gx, gz-1);
        bool hasS = isRoadLike2(gx, gz+1);

        bool roadGoesEW = (hasE || hasW) && !(hasN || hasS);

        // Asphalt base (full cell, same as road)
        {
            auto e  = s.Entity_CreateCube("crosswalk");
            auto* tr = s.transforms.GetComponent(e);
            if (tr) {
                tr->Scale(XMFLOAT3(h, 0.10f, h));
                tr->Translate(XMFLOAT3(c.x, 0.10f, c.y));
                tr->UpdateTransform();
            }
            auto* mat = s.materials.GetComponent(e);
            if (mat) mat->SetBaseColor(XMFLOAT4(0.10f, 0.10f, 0.12f, 1.0f));
            cellEntities[idx].push_back(e);
        }

        // Zebra stripes — shifted to inner edge (touching intersection)
        {
            const float bandHalf = 3.0f;    // 6 m = 3 sub-tiles
            const int NUM_STRIPES = 6;
            const float stripeGap = (bandHalf * 2.0f) / (float)(NUM_STRIPES);
            const float stripeThick = stripeGap * 0.6f;

            // Detect which neighbor is the intersection to shift stripes there
            auto countRN = [&](int cx, int cz) -> int {
                int rn = 0;
                if (isRoadLike2(cx+1,cz)) rn++;
                if (isRoadLike2(cx-1,cz)) rn++;
                if (isRoadLike2(cx,cz+1)) rn++;
                if (isRoadLike2(cx,cz-1)) rn++;
                return rn;
            };
            float interOff = 0.0f;
            constexpr float CW_SHIFT = 2.0f;  // 1 sub-tile toward intersection
            if (roadGoesEW) {
                if (hasE && countRN(gx+1, gz) >= 3) interOff =  (h - bandHalf + CW_SHIFT);
                else if (hasW && countRN(gx-1, gz) >= 3) interOff = -(h - bandHalf + CW_SHIFT);
            } else {
                if (hasS && countRN(gx, gz+1) >= 3) interOff =  (h - bandHalf + CW_SHIFT);
                else if (hasN && countRN(gx, gz-1) >= 3) interOff = -(h - bandHalf + CW_SHIFT);
            }

            for (int stripe = 0; stripe < NUM_STRIPES; ++stripe)
            {
                auto e = s.Entity_CreateCube("zebra_stripe");
                auto* tr = s.transforms.GetComponent(e);
                if (tr) {
                    float along = -bandHalf + stripeGap * ((float)stripe + 0.5f);
                    if (roadGoesEW)
                    {
                        tr->Scale(XMFLOAT3(stripeThick * 0.5f, 0.02f, (h - SIDEWALK_W) * 0.95f));
                        tr->Translate(XMFLOAT3(c.x + along + interOff, 0.21f, c.y));
                    }
                    else
                    {
                        tr->Scale(XMFLOAT3((h - SIDEWALK_W) * 0.95f, 0.02f, stripeThick * 0.5f));
                        tr->Translate(XMFLOAT3(c.x, 0.21f, c.y + along + interOff));
                    }
                    tr->UpdateTransform();
                }
                auto* mat = s.materials.GetComponent(e);
                if (mat) mat->SetBaseColor(XMFLOAT4(0.95f, 0.95f, 0.90f, 1.0f));
                cellEntities[idx].push_back(e);
            }
        }

        // Sidewalks
        {
            const bool connected[4] = { hasE, hasW, hasS, hasN };
            const float ox[4] = {  h - swh, -(h-swh), 0.f,      0.f      };
            const float oz[4] = {  0.f,      0.f,      h - swh, -(h-swh) };
            const float hxArr[4] = {  swh,      swh,      h,        h        };
            const float hzArr[4] = {  h,        h,        swh,      swh      };

            for (int d = 0; d < 4; ++d)
            {
                auto se = s.Entity_CreateCube("sidewalk");
                auto* st = s.transforms.GetComponent(se);
                if (st) {
                    const float visY = connected[d] ? -100.0f : 0.14f;
                    st->Scale(XMFLOAT3(hxArr[d], 0.14f, hzArr[d]));
                    st->Translate(XMFLOAT3(c.x + ox[d], visY, c.y + oz[d]));
                    st->UpdateTransform();
                }
                auto* mat = s.materials.GetComponent(se);
                if (mat) mat->SetBaseColor(XMFLOAT4(0.74f, 0.72f, 0.65f, 1.0f));
                cellEntities[idx].push_back(se);
                roadSidewalks[idx][d] = se;
            }

            // Corner fill pieces for crosswalk cells
            const bool cornerVis[4] = {
                connected[0] && connected[3],  // NE
                connected[1] && connected[3],  // NW
                connected[0] && connected[2],  // SE
                connected[1] && connected[2],  // SW
            };
            const float cox[4] = {  h - swh, -(h-swh),  h - swh, -(h-swh) };
            const float coz[4] = { -(h-swh), -(h-swh),  h - swh,  h - swh };
            for (int d = 0; d < 4; ++d)
            {
                auto ce = s.Entity_CreateCube("sidewalk_corner");
                auto* ct = s.transforms.GetComponent(ce);
                if (ct) {
                    const float visY = cornerVis[d] ? 0.14f : -100.0f;
                    ct->Scale(XMFLOAT3(swh, 0.14f, swh));
                    ct->Translate(XMFLOAT3(c.x + cox[d], visY, c.y + coz[d]));
                    ct->UpdateTransform();
                }
                auto* mat = s.materials.GetComponent(ce);
                if (mat) mat->SetBaseColor(XMFLOAT4(0.74f, 0.72f, 0.65f, 1.0f));
                cellEntities[idx].push_back(ce);
                roadCorners[idx][d] = ce;
            }
        }
    }

    // Per-house population tracking for reproduction + height growth
public:
    int housePopulation_[GRID_SIZE * GRID_SIZE] = {};
    float houseBaseHeight_[GRID_SIZE * GRID_SIZE] = {};

    int  GetHousePop(int gx, int gz) const { return InBounds(gx,gz) ? housePopulation_[gz*GRID_SIZE+gx] : 0; }
    void AddHousePop(int gx, int gz, int n) { if (InBounds(gx,gz)) housePopulation_[gz*GRID_SIZE+gx] += n; }
    void SetHousePop(int gx, int gz, int n) { if (InBounds(gx,gz)) housePopulation_[gz*GRID_SIZE+gx] = n; }

    // Workplace worker tracking
    int workplaceWorkers_[GRID_SIZE * GRID_SIZE] = {};
    int workplaceCapacity_[GRID_SIZE * GRID_SIZE] = {};

    int  GetWorkCap(int gx, int gz) const { return InBounds(gx,gz) ? workplaceCapacity_[gz*GRID_SIZE+gx] : 0; }
    int  GetWorkCount(int gx, int gz) const { return InBounds(gx,gz) ? workplaceWorkers_[gz*GRID_SIZE+gx] : 0; }
    void AddWorkers(int gx, int gz, int n) { if (InBounds(gx,gz)) workplaceWorkers_[gz*GRID_SIZE+gx] += n; }
    bool WorkplaceHasRoom(int gx, int gz) const {
        if (!InBounds(gx,gz)) return false;
        int idx = gz*GRID_SIZE+gx;
        return workplaceWorkers_[idx] < workplaceCapacity_[idx];
    }

    // Parking building tracking
    static constexpr int PARKING_FLOORS = 4;
    static constexpr int PARKING_SPOTS_PER_FLOOR = 6;
    static constexpr int PARKING_TOTAL_SPOTS = PARKING_FLOORS * PARKING_SPOTS_PER_FLOOR;
    // Floor height in world units
    static constexpr float PARKING_FLOOR_H = 3.8f;  // per storey
    int parkingCapacity_[GRID_SIZE * GRID_SIZE] = {};
    int parkingOccupancy_[GRID_SIZE * GRID_SIZE] = {};
    // Per-floor occupancy count
    int parkingFloorOcc_[GRID_SIZE * GRID_SIZE][PARKING_FLOORS] = {};

    int  GetParkCap(int gx, int gz) const { return InBounds(gx,gz) ? parkingCapacity_[gz*GRID_SIZE+gx] : 0; }
    int  GetParkOcc(int gx, int gz) const { return InBounds(gx,gz) ? parkingOccupancy_[gz*GRID_SIZE+gx] : 0; }
    void AddParkOcc(int gx, int gz, int n) { if (InBounds(gx,gz)) parkingOccupancy_[gz*GRID_SIZE+gx] += n; }
    bool ParkingHasRoom(int gx, int gz) const {
        if (!InBounds(gx,gz)) return false;
        int idx = gz*GRID_SIZE+gx;
        return cellType[idx] == CellType::PARKING && parkingOccupancy_[idx] < parkingCapacity_[idx];
    }
    // Returns world-space Y of lowest floor that has a free spot; -1 if none
    float ParkingFreeFloorY(int gx, int gz) const {
        if (!InBounds(gx,gz)) return -1.f;
        int idx = gz*GRID_SIZE+gx;
        for (int f = 0; f < PARKING_FLOORS; ++f)
            if (parkingFloorOcc_[idx][f] < PARKING_SPOTS_PER_FLOOR)
                return PARKING_FLOOR_H * (float)f + PARKING_FLOOR_H * 0.5f;
        return -1.f;
    }
    // Find adjacent road cell for a parking building (for car routing)
    bool ParkingAdjacentRoad(int gx, int gz, int& rx, int& rz) const {
        const int ddx[] = { 1,-1, 0, 0 };
        const int ddz[] = { 0, 0, 1,-1 };
        for (int d = 0; d < 4; ++d) {
            int nx = gx + ddx[d], nz = gz + ddz[d];
            if (InBounds(nx, nz)) {
                auto ct = cellType[nz * GRID_SIZE + nx];
                if (ct == CellType::ROAD || ct == CellType::CROSSWALK) {
                    rx = nx; rz = nz; return true;
                }
            }
        }
        return false;
    }

    // Update house visual height based on population
    void UpdateHouseHeight(int gx, int gz)
    {
        if (!InBounds(gx,gz)) return;
        int idx = gz * GRID_SIZE + gx;
        if (cellType[idx] != CellType::HOUSE) return;
        // wiscene houses have a single root entity – visual scaling not applicable
        if (domPrefabLoaded_ || cellEntities[idx].size() < 2) return;

        auto& s = wi::scene::GetScene();
        float baseBH = houseBaseHeight_[idx];
        int pop = housePopulation_[idx];
        // Height grows: base + extra floor every 5 people
        int extraFloors = std::max(0, (pop - 5) / 5);
        float bh = baseBH + (float)extraFloors * 3.0f;
        float bw = CELL_SIZE * 0.38f;
        XMFLOAT2 c = GridCellCenter(gx, gz);

        // Update main body (entity 0)
        auto* tr = s.transforms.GetComponent(cellEntities[idx][0]);
        if (tr) {
            tr->ClearTransform();
            tr->Scale(XMFLOAT3(bw, bh, bw));
            tr->Translate(XMFLOAT3(c.x, bh, c.y));
            tr->UpdateTransform();
        }
        // Update roof (entity 1)
        auto* rt = s.transforms.GetComponent(cellEntities[idx][1]);
        if (rt) {
            rt->ClearTransform();
            rt->Scale(XMFLOAT3(bw * 1.06f, bh * 0.10f, bw * 1.06f));
            rt->Translate(XMFLOAT3(c.x, bh * 2.0f + bh * 0.10f, c.y));
            rt->UpdateTransform();
        }
    }

    // dom.wiscene shared instancing data
    wi::scene::Scene domPrefab_;       // emptied after Initialize() merges into main scene
    bool domPrefabLoaded_ = false;
    struct DomSubObject {
        wi::ecs::Entity meshID = wi::ecs::INVALID_ENTITY;
        XMFLOAT4X4 localMatrix;
        XMFLOAT4   color = {1.f, 1.f, 1.f, 1.f};
    };
    std::vector<DomSubObject> domSubObjects_;  // one entry per mesh object in dom.wiscene

private:
    void BuildHouse(int idx, int gx, int gz, XMFLOAT2 c, wi::scene::Scene& s)
    {
        float bh = CELL_SIZE * 0.30f + static_cast<float>((gx * 17 + gz * 31) % 5) * 1.2f;
        houseBaseHeight_[idx] = bh;
        housePopulation_[idx] = 0;

        if (domPrefabLoaded_)
        {
            // Create one entity per sub-object, all sharing the same mesh entity → GPU instanced
            float rot = static_cast<float>((gx * 7 + gz * 13) % 4) * XM_PIDIV2;
            XMMATRIX houseMat = XMMatrixRotationY(rot) * XMMatrixTranslation(c.x, 0.0f, c.y);
            for (auto& sub : domSubObjects_) {
                wi::ecs::Entity e = wi::ecs::CreateEntity();
                s.layers.Create(e);
                auto& tr = s.transforms.Create(e);
                XMMATRIX world = XMLoadFloat4x4(&sub.localMatrix) * houseMat;
                tr.MatrixTransform(world);
                tr.UpdateTransform();
                auto& obj = s.objects.Create(e);
                obj.meshID = sub.meshID;  // shared mesh → instanced by renderer
                obj.color  = sub.color;
                cellEntities[idx].push_back(e);
            }
        }
        else
        {
            // Fallback: procedural cube house
            float bw = CELL_SIZE * 0.38f;

            auto e = s.Entity_CreateCube("house");
            auto* tr = s.transforms.GetComponent(e);
            if (tr) {
                tr->Scale(XMFLOAT3(bw, bh, bw));
                tr->Translate(XMFLOAT3(c.x, bh, c.y));
                tr->UpdateTransform();
            }
            float hi = static_cast<float>((gx * 13 + gz * 7) % 8) / 7.0f;
            auto* mat = s.materials.GetComponent(e);
            if (mat) mat->SetBaseColor(
                XMFLOAT4(0.82f + hi * 0.10f, 0.35f + hi * 0.10f, 0.16f, 1.0f));
            cellEntities[idx].push_back(e);

            auto r = s.Entity_CreateCube("house_roof");
            auto* rt = s.transforms.GetComponent(r);
            if (rt) {
                rt->Scale(XMFLOAT3(bw * 1.06f, bh * 0.10f, bw * 1.06f));
                rt->Translate(XMFLOAT3(c.x, bh * 2.0f + bh * 0.10f, c.y));
                rt->UpdateTransform();
            }
            auto* rmat = s.materials.GetComponent(r);
            if (rmat) rmat->SetBaseColor(XMFLOAT4(0.30f, 0.14f, 0.09f, 1.0f));
            cellEntities[idx].push_back(r);
        }
    }

    void BuildWorkplace(int idx, int gx, int gz, XMFLOAT2 c, wi::scene::Scene& s)
    {
        float bw = CELL_SIZE * 0.40f;
        float bh = CELL_SIZE * 0.55f + static_cast<float>((gx * 11 + gz * 23) % 8) * 2.0f;
        // Capacity scales with building height: 10-25 workers
        workplaceCapacity_[idx] = 10 + (int)(bh / 3.0f);
        workplaceWorkers_[idx] = 0;

        // Tower body (cool blue-gray)
        auto e = s.Entity_CreateCube("workplace");
        auto* tr = s.transforms.GetComponent(e);
        if (tr) {
            tr->Scale(XMFLOAT3(bw, bh, bw));
            tr->Translate(XMFLOAT3(c.x, bh, c.y));
            tr->UpdateTransform();
        }
        float ti = static_cast<float>((gx * 7 + gz * 13) % 6) / 5.0f;
        auto* mat = s.materials.GetComponent(e);
        if (mat) mat->SetBaseColor(
            XMFLOAT4(0.22f + ti * 0.08f, 0.38f + ti * 0.10f, 0.72f + ti * 0.12f, 1.0f));
        cellEntities[idx].push_back(e);

        // Glass crown
        auto g = s.Entity_CreateCube("workplace_crown");
        auto* gt = s.transforms.GetComponent(g);
        if (gt) {
            gt->Scale(XMFLOAT3(bw * 1.03f, bh * 0.09f, bw * 1.03f));
            gt->Translate(XMFLOAT3(c.x, bh * 2.0f + bh * 0.09f, c.y));
            gt->UpdateTransform();
        }
        auto* gmat = s.materials.GetComponent(g);
        if (gmat) gmat->SetBaseColor(XMFLOAT4(0.62f, 0.82f, 0.97f, 1.0f));
        cellEntities[idx].push_back(g);
    }

    void BuildParking(int idx, int gx, int gz, XMFLOAT2 c, wi::scene::Scene& s)
    {
        // Simplified multi-story parking garage:
        //   - PARKING_FLOORS storeys, each with a floor slab
        //   - Straight ramp between floors along one side
        //   - Entrance face oriented toward adjacent road
        //   - Open sides for visual clarity, 4 corner columns
        parkingCapacity_[idx] = PARKING_TOTAL_SPOTS;
        parkingOccupancy_[idx] = 0;
        for (int f = 0; f < PARKING_FLOORS; ++f)
            parkingFloorOcc_[idx][f] = 0;

        const float bw = CELL_SIZE * 0.44f;        // half-width of structure
        const float fh = PARKING_FLOOR_H;           // floor-to-floor height
        const float slabThk = 0.20f;                // floor slab half-thickness
        const float colThk  = 0.30f;                // column half-size
        const float totalH  = fh * PARKING_FLOORS;

        // ---- Floor slabs + 4 corner columns per floor ----
        for (int f = 0; f < PARKING_FLOORS; ++f)
        {
            float slabY = fh * (float)f;

            // Floor slab
            {
                auto e = s.Entity_CreateCube("park_slab");
                auto* tr = s.transforms.GetComponent(e);
                tr->Scale(XMFLOAT3(bw, slabThk, bw));
                tr->Translate(XMFLOAT3(c.x, slabY + slabThk, c.y));
                tr->UpdateTransform();
                s.materials.GetComponent(e)->SetBaseColor(XMFLOAT4(0.56f, 0.55f, 0.52f, 1.0f));
                cellEntities[idx].push_back(e);
            }

            // 4 corner columns
            const float cx4[4] = { bw - colThk, -(bw - colThk),  bw - colThk, -(bw - colThk) };
            const float cz4[4] = { bw - colThk,   bw - colThk, -(bw - colThk), -(bw - colThk) };
            for (int col = 0; col < 4; ++col)
            {
                auto e = s.Entity_CreateCube("park_col");
                auto* tr = s.transforms.GetComponent(e);
                float colH = (fh - slabThk * 2.f) * 0.5f;
                tr->Scale(XMFLOAT3(colThk, colH, colThk));
                tr->Translate(XMFLOAT3(c.x + cx4[col], slabY + slabThk * 2.f + colH, c.y + cz4[col]));
                tr->UpdateTransform();
                s.materials.GetComponent(e)->SetBaseColor(XMFLOAT4(0.52f, 0.51f, 0.48f, 1.0f));
                cellEntities[idx].push_back(e);
            }
        }

        // ---- Roof slab ----
        {
            auto e = s.Entity_CreateCube("park_roof");
            auto* tr = s.transforms.GetComponent(e);
            tr->Scale(XMFLOAT3(bw, slabThk, bw));
            tr->Translate(XMFLOAT3(c.x, totalH + slabThk, c.y));
            tr->UpdateTransform();
            s.materials.GetComponent(e)->SetBaseColor(XMFLOAT4(0.38f, 0.36f, 0.33f, 1.0f));
            cellEntities[idx].push_back(e);
        }

        // ---- Straight ramp on +X side between each pair of floors ----
        for (int f = 0; f < PARKING_FLOORS - 1; ++f)
        {
            float botY = fh * (float)f + slabThk * 2.f;
            float topY = fh * (float)(f + 1);
            float midY = (botY + topY) * 0.5f;
            float rampLen = bw * 0.8f;   // half-length of the ramp slab
            float rampW   = bw * 0.20f;  // half-width
            float pitch = std::atan2(topY - botY, rampLen * 2.f);

            auto e = s.Entity_CreateCube("park_ramp");
            auto* tr = s.transforms.GetComponent(e);
            tr->Scale(XMFLOAT3(rampW, slabThk, rampLen));
            tr->RotateRollPitchYaw(XMFLOAT3(pitch, 0.f, 0.f));
            tr->Translate(XMFLOAT3(c.x + bw - rampW - colThk, midY, c.y));
            tr->UpdateTransform();
            s.materials.GetComponent(e)->SetBaseColor(XMFLOAT4(0.50f, 0.49f, 0.46f, 1.0f));
            cellEntities[idx].push_back(e);
        }

        // ---- Blue "P" sign on front face ----
        {
            auto p = s.Entity_CreateCube("parking_sign");
            auto* pt = s.transforms.GetComponent(p);
            pt->Scale(XMFLOAT3(bw * 0.25f, fh * 0.35f, 0.14f));
            pt->Translate(XMFLOAT3(c.x, fh * 0.5f + fh * 0.35f, c.y - bw - 0.15f));
            pt->UpdateTransform();
            auto* pm = s.materials.GetComponent(p);
            pm->SetBaseColor(XMFLOAT4(0.10f, 0.28f, 0.82f, 1.0f));
            pm->SetEmissiveColor(XMFLOAT4(0.10f, 0.28f, 0.82f, 3.0f));
            cellEntities[idx].push_back(p);
        }
    }
};