#include "CrowdSystem.h"
#include "CityLayout.h"

// ============================================================
//  Initialize
// ============================================================
void CrowdSystem::Initialize()
{
    posX.resize(MAX_AGENTS, 0.0f);
    posZ.resize(MAX_AGENTS, 0.0f);
    speed.resize(MAX_AGENTS, 3.0f);
    agentName.resize(MAX_AGENTS);
    agentAge.resize(MAX_AGENTS, 0);
    agentDir.resize(MAX_AGENTS, 0);

    waypointBuf.resize(static_cast<size_t>(MAX_AGENTS) * MAX_WAYPOINTS);
    waypointCount_.resize(MAX_AGENTS, 0);
    waypointCurr_.resize(MAX_AGENTS, 0);

    agentMoney.resize(MAX_AGENTS, 0.0f);
    agentWage.resize(MAX_AGENTS,  0.0f);
    agentSidewalkOff_.resize(MAX_AGENTS, 0.0f);
    lastFrameTax_ = 0.0f;

    visIndices.resize(MAX_VISIBLE);
    activeCount = 0;

    CreateInstancePool();

    wi::backlog::post("[CrowdSystem] Ready – 0 agents, place houses near roads to spawn people",
                      wi::backlog::LogLevel::Default);
}

// ============================================================
//  SpawnAgents
// ============================================================
void CrowdSystem::SpawnAgents(const std::vector<XMFLOAT2>& path,
                               int homeGX, int homeGZ, int workGX, int workGZ,
                               uint32_t count)
{
    static const char* FIRST[] = {
        "Alice","Bob","Carol","Dave","Eve","Frank","Grace","Hank",
        "Iris","Jack","Kate","Leo","Mia","Noah","Olivia","Pete"
    };
    static const char* LAST[] = {
        "Smith","Jones","Brown","Davis","Wilson","Moore","Taylor","Anderson",
        "Thomas","Jackson","White","Harris","Martin","Lee","Perez","Walker"
    };
    if (path.size() < 2) return;
    const uint8_t wc = static_cast<uint8_t>(
        std::min(path.size(), static_cast<size_t>(MAX_WAYPOINTS)));

    for (uint32_t n = 0; n < count; ++n)
    {
        if (activeCount >= MAX_AGENTS) break;
        const uint32_t i    = activeCount++;
        const uint32_t seed = i * 2654435761u ^ n * 1013904223u;

        // Assign sidewalk side: ±9 m from road centre, small scatter within 2 m sidewalk
        float sw = ((seed & 1u) ? 1.f : -1.f) * 9.0f;
        sw += (static_cast<float>((seed >> 24) & 0x7) / 7.f - 0.5f) * 1.0f;
        agentSidewalkOff_[i] = sw;

        // Initial position on sidewalk (perpendicular to first road segment)
        float sdx = (wc >= 2) ? path[1].x - path[0].x : 1.f;
        float sdz = (wc >= 2) ? path[1].y - path[0].y : 0.f;
        float slen = std::sqrt(sdx * sdx + sdz * sdz);
        float rx = 0.f, rz = 0.f;
        if (slen > 0.1f) { rx = sdz / slen; rz = -sdx / slen; }

        // Scatter start around path[0]
        float ox = (static_cast<float>((seed      ) & 0xFF) / 255.0f - 0.5f) * 1.0f;
        float oz = (static_cast<float>((seed >>  8) & 0xFF) / 255.0f - 0.5f) * 1.0f;
        posX[i] = path[0].x + ox + rx * sw;
        posZ[i] = path[0].y + oz + rz * sw;   // XMFLOAT2.y stores the Z coord

        speed[i] = 2.5f + static_cast<float>((seed >> 16) & 0x3F) / 63.0f * 2.0f; // 2.5–4.5 m/s

        // Name and age
        uint32_t ni = (seed ^ (seed >> 7))  & 0xF;
        uint32_t li = ((seed * 1103515245u) >> 17) & 0xF;
        agentName[i]  = std::string(FIRST[ni]) + " " + LAST[li];
        agentAge[i]   = 18 + static_cast<uint8_t>((seed >> 20) % 50);
        agentDir[i]   = 0;
        agentMoney[i] = 0.0f;
        // Wage: $0.40 – $1.80 per game-second (scales with simSpeed via dt)
        agentWage[i]  = 0.40f + static_cast<float>((seed >> 12) & 0x3F) / 63.0f * 1.40f;

        XMFLOAT2* wp = &waypointBuf[static_cast<size_t>(i) * MAX_WAYPOINTS];
        for (uint8_t w = 0; w < wc; ++w) wp[w] = path[w];
        waypointCount_[i] = wc;
        waypointCurr_[i]  = 1; // first target = index 1
    }
}

// ============================================================
//  Target accessors
// ============================================================
float CrowdSystem::GetTargetX(uint32_t i) const
{
    uint8_t cur = waypointCurr_[i];
    if (cur >= waypointCount_[i]) cur = 0;
    return waypointBuf[static_cast<size_t>(i) * MAX_WAYPOINTS + cur].x;
}

float CrowdSystem::GetTargetZ(uint32_t i) const
{
    uint8_t cur = waypointCurr_[i];
    if (cur >= waypointCount_[i]) cur = 0;
    return waypointBuf[static_cast<size_t>(i) * MAX_WAYPOINTS + cur].y;
}

// ============================================================
//  Update — parallel waypoint walking
//  REWRITE: preventive sidewalk corridor constraint.
//  Pedestrians compute their desired position, then PROJECT it
//  onto the nearest valid sidewalk corridor. They never enter
//  the road surface.
//
//  Sidewalk corridors: for each road cell, the sidewalk is a
//  2m wide band at [8.0, 10.0] m from the cell centre,
//  perpendicular to the road axis.
//
//  At intersections (3+ road neighbours): pedestrians are only
//  allowed on the outer perimeter ring (within 2m of the cell
//  edge), simulating the crosswalk around the intersection.
// ============================================================
void CrowdSystem::Update(float dt, const XMFLOAT3& /*cameraPos*/, const CityLayout& city)
{
    wi::jobsystem::Wait(jobCtx);
    wi::jobsystem::Dispatch(jobCtx, activeCount, JOB_GROUP_SIZE,
        [this, dt, &city](wi::jobsystem::JobArgs args)
        {
            const uint32_t i  = args.jobIndex;
            const uint8_t  wc = waypointCount_[i];
            if (wc < 2) return;

            uint8_t& wCurr = waypointCurr_[i];

            if (wCurr == 0) wCurr = 1;
            if (wCurr >= wc)
            {
                XMFLOAT2* wp = &waypointBuf[static_cast<size_t>(i) * MAX_WAYPOINTS];
                for (uint8_t a = 0, b = wc - 1; a < b; ++a, --b)
                    std::swap(wp[a], wp[b]);
                wCurr = 1;
                agentDir[i] ^= 1;
                agentSidewalkOff_[i] = -agentSidewalkOff_[i];
            }

            const XMFLOAT2& tgt  = waypointBuf[static_cast<size_t>(i) * MAX_WAYPOINTS + wCurr];
            const XMFLOAT2& prev = waypointBuf[static_cast<size_t>(i) * MAX_WAYPOINTS + (wCurr - 1)];

            // Compute sidewalk-offset target
            float sdx = tgt.x - prev.x, sdz = tgt.y - prev.y;
            float slen = std::sqrt(sdx * sdx + sdz * sdz);
            float effX = tgt.x, effZ = tgt.y;
            float sw  = agentSidewalkOff_[i];

            if (slen > 0.1f)
            {
                float rx = sdz / slen, rz = -sdx / slen;

                float nrx = rx, nrz = rz;
                bool hasTurn = false;
                if (wCurr + 1 < wc)
                {
                    const XMFLOAT2& next2 = waypointBuf[
                        static_cast<size_t>(i) * MAX_WAYPOINTS + wCurr + 1];
                    float ndx = next2.x - tgt.x, ndz = next2.y - tgt.y;
                    float nlen = std::sqrt(ndx * ndx + ndz * ndz);
                    if (nlen > 0.1f)
                    {
                        ndx /= nlen; ndz /= nlen;
                        float hcos = (sdx / slen) * ndx + (sdz / slen) * ndz;
                        if (hcos < 0.7f)
                        {
                            hasTurn = true;
                            nrx = ndz; nrz = -ndx;
                        }
                    }
                }

                if (hasTurn)
                {
                    float sx = rx + nrx, sz = rz + nrz;
                    float sn = std::sqrt(sx*sx + sz*sz);
                    if (sn > 0.01f) { sx /= sn; sz /= sn; }
                    // At corners, push the target further out so pedestrians
                    // walk around the corner block on the sidewalk surface.
                    effX = tgt.x + sx * sw;
                    effZ = tgt.y + sz * sw;
                }
                else
                {
                    effX = tgt.x + rx * sw;
                    effZ = tgt.y + rz * sw;
                }
            }

            float dxT = effX - posX[i];
            float dzT = effZ - posZ[i];
            float d2 = dxT * dxT + dzT * dzT;

            if (d2 < 1.0f)
            {
                ++wCurr;
            }
            else
            {
                float inv = speed[i] * dt / std::sqrt(d2);
                float newX = posX[i] + dxT * inv;
                float newZ = posZ[i] + dzT * inv;

                // ============================================
                // PREVENTIVE sidewalk corridor constraint.
                // After computing the proposed position, clamp
                // it to the nearest valid sidewalk surface.
                // ============================================
                constexpr float SW_INNER = 8.0f;  // sidewalk starts here (road edge)
                constexpr float SW_OUTER = 10.0f;  // sidewalk ends here (cell edge)

                auto clampToSidewalk = [&](float& bx, float& bz) {
                    int cgx = 0, cgz = 0;
                    if (!city.WorldToGrid(bx, bz, cgx, cgz)) return;
                    auto ctype = city.GetCellType(cgx, cgz);
                    if (ctype != CityLayout::CellType::ROAD &&
                        ctype != CityLayout::CellType::CROSSWALK)
                        return; // not on a road cell — OK

                    XMFLOAT2 cc = city.GridCellCenter(cgx, cgz);
                    float relX = bx - cc.x;
                    float relZ = bz - cc.y;

                    // Count road neighbours
                    bool hasE = city.IsRoadLike(cgx+1, cgz);
                    bool hasW = city.IsRoadLike(cgx-1, cgz);
                    bool hasN = city.IsRoadLike(cgx, cgz-1);
                    bool hasS = city.IsRoadLike(cgx, cgz+1);
                    int rn = (hasE?1:0) + (hasW?1:0) + (hasN?1:0) + (hasS?1:0);

                    float halfC = CityLayout::CELL_SIZE * 0.5f;

                    if (rn >= 3)
                    {
                        // INTERSECTION: only allow on outer perimeter ring.
                        // The ring is [halfC - 2, halfC] from centre on each axis.
                        // Check if we're deep inside — if so, push to nearest perimeter.
                        float absX = std::abs(relX);
                        float absZ = std::abs(relZ);
                        float edgeDistX = halfC - absX; // distance from X edge
                        float edgeDistZ = halfC - absZ; // distance from Z edge

                        // Allowed if at least one axis is within 2m of the edge
                        bool onPerimeter = (edgeDistX <= 2.0f || edgeDistZ <= 2.0f);
                        if (!onPerimeter)
                        {
                            // Push to nearest edge
                            if (edgeDistX < edgeDistZ)
                                bx = cc.x + ((relX >= 0) ? (halfC - 1.0f) : -(halfC - 1.0f));
                            else
                                bz = cc.y + ((relZ >= 0) ? (halfC - 1.0f) : -(halfC - 1.0f));
                        }
                        // Clamp to stay within cell boundaries
                        bx = std::clamp(bx, cc.x - halfC, cc.x + halfC);
                        bz = std::clamp(bz, cc.y - halfC, cc.y + halfC);
                        return;
                    }

                    // Normal road: determine axis (EW or NS)
                    bool goesEW = (hasE || hasW) && !(hasN && hasS);
                    bool goesNS = (hasN || hasS) && !(hasE && hasW);

                    if (goesEW)
                    {
                        // Road runs E-W: Z is across the road
                        float absZ2 = std::abs(relZ);
                        if (absZ2 < SW_INNER)
                        {
                            // Inside road surface — snap to nearest sidewalk inner edge
                            bz = cc.y + ((relZ >= 0) ? SW_INNER : -SW_INNER);
                        }
                        else if (absZ2 > SW_OUTER)
                        {
                            // Past sidewalk outer edge — clamp back
                            bz = cc.y + ((relZ >= 0) ? SW_OUTER : -SW_OUTER);
                        }
                        // else: already on sidewalk, OK
                    }
                    else if (goesNS)
                    {
                        // Road runs N-S: X is across the road
                        float absX2 = std::abs(relX);
                        if (absX2 < SW_INNER)
                        {
                            bx = cc.x + ((relX >= 0) ? SW_INNER : -SW_INNER);
                        }
                        else if (absX2 > SW_OUTER)
                        {
                            bx = cc.x + ((relX >= 0) ? SW_OUTER : -SW_OUTER);
                        }
                    }
                    else
                    {
                        // Unknown axis: constrain both
                        float absX2 = std::abs(relX);
                        float absZ2 = std::abs(relZ);
                        if (absX2 < SW_INNER && absZ2 < SW_INNER)
                        {
                            // Deep inside — push out on whichever is closer
                            if (absX2 > absZ2)
                                bx = cc.x + ((relX >= 0) ? SW_INNER : -SW_INNER);
                            else
                                bz = cc.y + ((relZ >= 0) ? SW_INNER : -SW_INNER);
                        }
                    }
                };

                clampToSidewalk(newX, newZ);
                posX[i] = newX;
                posZ[i] = newZ;
            }
        }
    );
    wi::jobsystem::Wait(jobCtx);

    // Sequential wage + tax accumulation (after parallel walk is done)
    float tax = 0.0f;
    for (uint32_t i = 0; i < activeCount; ++i)
    {
        if (agentDir[i] == 1)   // returning from work — just put in a shift
        {
            float earned  = agentWage[i] * dt;
            agentMoney[i] += earned;
            tax           += earned * taxRate_;
        }
    }
    lastFrameTax_ = tax;
}

// ============================================================
//  CreateInstancePool — shared mesh + reusable objects
// ============================================================
void CrowdSystem::CreateInstancePool()
{
    auto& scene = wi::scene::GetScene();

    meshEntity = scene.Entity_CreateCube("agent_mesh");
    scene.materials.GetComponent(meshEntity)->SetBaseColor(
        XMFLOAT4(0.15f, 0.78f, 0.28f, 1.0f)); // bright green agents
    auto* meshTr = scene.transforms.GetComponent(meshEntity);
    meshTr->Scale(XMFLOAT3(0.0f, 0.0f, 0.0f));
    meshTr->UpdateTransform();

    instancePool.resize(MAX_VISIBLE);
    for (uint32_t i = 0; i < MAX_VISIBLE; ++i)
    {
        wi::ecs::Entity e = wi::ecs::CreateEntity();
        scene.layers.Create(e);
        auto& tr = scene.transforms.Create(e);
        tr.Translate(XMFLOAT3(0.0f, -1000.0f, 0.0f));
        tr.UpdateTransform();
        auto& obj = scene.objects.Create(e);
        obj.meshID = meshEntity;
        instancePool[i] = e;
    }
}

// ============================================================
//  RenderSolidAgents — deterministic sequential LOD scan
// ============================================================
void CrowdSystem::RenderSolidAgents(const XMFLOAT3& cameraPos)
{
    auto& scene = wi::scene::GetScene();

    static constexpr float HW = 0.25f;
    static constexpr float HH = 0.90f;
    static constexpr float HD = 0.25f;

    const float nearSq = LOD_NEAR * LOD_NEAR;
    const float midSq  = LOD_MID  * LOD_MID;
    const float farSq  = LOD_FAR  * LOD_FAR;
    const float camX   = cameraPos.x, camZ = cameraPos.z;

    uint32_t count = 0;
    for (uint32_t i = 0; i < activeCount && count < MAX_VISIBLE; ++i)
    {
        const float dx = posX[i] - camX;
        const float dz = posZ[i] - camZ;
        const float d2 = dx * dx + dz * dz;
        if (d2 >= farSq) continue;
        if (d2 >= midSq && (i & 3u) != 0) continue;
        if (d2 >= nearSq && (i & 1u) != 0) continue;
        visIndices[count++] = i;
    }
    visCount_ = count;

    for (uint32_t s = 0; s < count; ++s)
    {
        const uint32_t i = visIndices[s];
        auto* tr = scene.transforms.GetComponent(instancePool[s]);
        if (tr)
        {
            tr->ClearTransform();
            tr->Scale(XMFLOAT3(HW * 2.0f, HH * 2.0f, HD * 2.0f));
            tr->Translate(XMFLOAT3(posX[i], HH, posZ[i]));
            tr->SetDirty();
        }
    }

    for (uint32_t j = count; j < MAX_VISIBLE; ++j)
    {
        auto* tr = scene.transforms.GetComponent(instancePool[j]);
        if (tr)
        {
            tr->ClearTransform();
            tr->Translate(XMFLOAT3(0.0f, -1000.0f, 0.0f));
            tr->SetDirty();
        }
    }

    renderedCount.store(count, std::memory_order_relaxed);
}
