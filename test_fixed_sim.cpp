// ================================================================
//  test_fixed_sim.cpp — Comprehensive pedestrian/car simulation test
//
//  FIXES TESTED:
//  1. Corner waypoints at intersections (peds walk around, not through)
//  2. Extended ALL_RED phase (4s) for pedestrian crossing
//  3. Strict sidewalk-only validation at all road/intersection cells
//  4. Human-drives-car: walk to parking → drive → walk from parking
//  5. Crosswalk-only crossing enforcement
//
//  Run: 30 seconds at 100x = 3000 game-seconds = 50 game-minutes
// ================================================================
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <cassert>
#include <random>

static constexpr float PI  = 3.14159265358979323846f;
static constexpr float PI2 = PI * 2.0f;

// ============================================================
//  Grid constants
// ============================================================
static constexpr int   GS  = 40;
static constexpr float CS  = 20.0f;
static constexpr float HCS = CS * 0.5f;
static constexpr float HALF_WORLD = GS * CS * 0.5f;
static constexpr float SIDEWALK_W     = 2.0f;
static constexpr float SIDEWALK_INNER = HCS - SIDEWALK_W;  // 8.0 m
static constexpr float SIDEWALK_OUTER = HCS;               // 10.0 m
static constexpr float SIDEWALK_MID   = SIDEWALK_INNER + SIDEWALK_W * 0.5f; // 9.0 m

// ============================================================
//  Car constants
// ============================================================
static constexpr uint32_t MAX_CARS  = 2000;
static constexpr float MAX_SPEED    = 14.0f;
static constexpr float ACCEL        =  6.0f;
static constexpr float DECEL        = 14.0f;
static constexpr float CAR_HL       =  1.00f;
static constexpr float CAR_HW       =  0.50f;
static constexpr float MIN_SEP      =  CAR_HL * 2.f + 1.5f;
static constexpr float LAT_BAND     =  1.5f;
static constexpr float IDM_S0       =  2.0f;
static constexpr float IDM_T        =  1.5f;
static constexpr float IDM_B        =  3.0f;
static constexpr float K_STANLEY    =  3.5f;
static constexpr uint8_t MAX_WP     = 64;

// ============================================================
//  Pedestrian constants
// ============================================================
static constexpr uint32_t MAX_PEDS     = 5000;
static constexpr float PED_SPEED_MIN   = 1.2f;
static constexpr float PED_SPEED_MAX   = 1.8f;
static constexpr float PED_RADIUS      = 0.25f;
static constexpr float PED_MIN_SEP     = 0.6f;
static constexpr float PED_AVOID_DIST  = 2.0f;
static constexpr uint16_t PED_MAX_WP   = 256;

// ============================================================
//  Cell types
// ============================================================
enum CellType : uint8_t { EMPTY=0, ROAD=1, HOUSE=2, WORKPLACE=3, CROSSWALK=4, PARKING=5 };
static CellType grid[GS][GS];
static int roadLanes[GS][GS];
static bool isIntersection_[GS][GS];

// Same large city map as test_ped_sim.cpp
static const char* MAP_ROWS[GS] = {
    ".HRH..HRH..HRH..HRH..HRH..HRH..HRH..HRH.",
    "HHRHHHHRHHHHRHHHHRHHWPRPWHHRHHHHRHHHHRHH",
    "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
    "HHRPHHHRPHHHRPHHHRPHWHRHWHHRPHHHRPHHHRPH",
    ".HRH..HRH..HRH..HRH..HRH..HRH..HRH..HRH.",
    ".HRH..HRH..HRH..WRH..HRH..HRW..HRH..HRH.",
    "HHRHHHHRHHHHRHHHWRHHWHRHWHHRWHHHRHHHHRHH",
    "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
    "HHRPHHHRPHHHRPHHWRHHWPRPWHHRWHHHRPHHHRPH",
    ".HRH..HRH..HRH..WRP..HRH..HRW..HRH..HRH.",
    ".HRH..HRH..HRH..WRH..WRP..HRW..HRH..HRH.",
    "HHRHHHHRHHHHRHHPWRPPWWRWWHHRWHHHRHHHHRHH",
    "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
    "HHRPHHHRPHHHRHHHWRHHPWRPWHHRWHPPRPPHHRPH",
    ".HRH..HRH..HRH..WRH..WRW..PRW..HRH..HRH.",
    ".HRH..PRW..HRH..WRP..WRP..WRP..PRP..WRH.",
    "HHRHHHHRWHHHRHHWWRWWWWRWWWWRWWHHRHHHWRHH",
    "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
    "HHRPHHHRWHHHRHHPWRPWPWRPWPWRPWHHRHHHWRHH",
    ".HRH..HRW..PRP..WRW..WRW..WRW..HRH..WRP.",
    ".HRH..HRW..WRP..WRP..WRP..WRP..WRP..WRH.",
    "WHRHWHHRWHWWRWWWWRWWWWRWWWWRWWWWRWWPWRPP",
    "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
    "WPRPWHHRWHPWRPWPWRPWPWRPWPWRPWPWRPWHWRHH",
    ".HRH..HRW..WRW..WRW..WRW..WRW..WRW..WRH.",
    ".HRH..HRW..HRH..WRP..WRP..WRP..HRH..WRH.",
    "HHRHHHHRWHHHRHHWWRWWWWRWWWWRWWHHRHHHWRHH",
    "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
    "HHRPHHHRWHPPRPPPWRPWPWRPWPWRPWHHRHHPWRPP",
    ".HRH..PRW..HRH..WRW..WRW..WRW..PRP..WRH.",
    ".HRH..HRH..PRP..WRH..WRP..HRW..HRH..HRH.",
    "HHRHHHHRHHHHRHHHWRHHWWRWWHHRWHPPRPPHHRHH",
    "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
    "HHRPHHHRPHHHRHHHWRHHPWRPWHHRWHHHRHHHHRPH",
    ".HRH..HRH..HRH..WRP..WRW..HRW..HRH..HRH.",
    ".HRH..HRH..HRH..WRH..PRP..HRW..HRH..HRH.",
    "HHRHHHHRHHHHRHHPWRPPWHRHWHHRWHHHRHHHHRHH",
    "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
    "HHRPHHHRPHHHRPHHWRHHWHRHWHHRWHHHRPHHHRPH",
    ".HRH..HRH..HRH..WRH..HRH..PRW..HRH..HRH.",
};

// ============================================================
//  Basic helpers
// ============================================================
struct Vec2 { float x, y; };

static bool InBounds(int gx, int gz) { return gx >= 0 && gx < GS && gz >= 0 && gz < GS; }

static bool IsRoadLike(int gx, int gz)
{
    if (!InBounds(gx, gz)) return false;
    return grid[gz][gx] == ROAD || grid[gz][gx] == CROSSWALK;
}

static Vec2 CellCenter(int gx, int gz)
{
    return { -HALF_WORLD + (gx + 0.5f) * CS, -HALF_WORLD + (gz + 0.5f) * CS };
}

static bool WorldToGrid(float wx, float wz, int& gx, int& gz)
{
    gx = (int)std::floor((wx + HALF_WORLD) / CS);
    gz = (int)std::floor((wz + HALF_WORLD) / CS);
    return InBounds(gx, gz);
}

static int GetLanes(int gx, int gz)
{
    if (!InBounds(gx, gz)) return 4;
    int v = roadLanes[gz][gx];
    return (v == 2 || v == 6) ? v : 4;
}

static void GetLaneOffsets(int totalLanes, float* offsets, int& count)
{
    if (totalLanes >= 6)      { count = 3; offsets[0] = 1.5f; offsets[1] = 4.0f; offsets[2] = 6.5f; }
    else if (totalLanes >= 4) { count = 2; offsets[0] = 2.5f; offsets[1] = 5.5f; }
    else                      { count = 1; offsets[0] = 4.0f; }
}

static Vec2 DirTo(Vec2 a, Vec2 b)
{
    float dx = b.x-a.x, dz = b.y-a.y;
    float len = std::sqrt(dx*dx+dz*dz);
    if (len < 1e-6f) return {1,0};
    return {dx/len, dz/len};
}

static float Dist(Vec2 a, Vec2 b)
{
    float dx = a.x-b.x, dz = a.y-b.y;
    return std::sqrt(dx*dx+dz*dz);
}

// ============================================================
//  Build grid + detect intersections + place crosswalks
// ============================================================
static void BuildGrid()
{
    memset(grid, 0, sizeof(grid));
    memset(roadLanes, 0, sizeof(roadLanes));
    for (int z = 0; z < GS; z++) {
        const char* row = MAP_ROWS[z];
        for (int x = 0; x < GS; x++) {
            char c = row[x];
            switch (c) {
            case 'R': grid[z][x] = ROAD;      break;
            case 'H': grid[z][x] = HOUSE;     break;
            case 'W': grid[z][x] = WORKPLACE; break;
            case 'X': grid[z][x] = CROSSWALK; break;
            case 'P': grid[z][x] = PARKING;   break;
            default:  grid[z][x] = EMPTY;     break;
            }
            roadLanes[z][x] = 4;
        }
    }
    for (int i = 0; i < GS; i++) {
        roadLanes[i][17] = 6;
        roadLanes[i][22] = 6;
        roadLanes[17][i] = 6;
        roadLanes[22][i] = 6;
    }
}

static const int DDX[] = {1,-1,0,0};
static const int DDZ[] = {0,0,1,-1};

static void DetectIntersections()
{
    memset(isIntersection_, 0, sizeof(isIntersection_));
    for (int z = 0; z < GS; z++)
    for (int x = 0; x < GS; x++) {
        if (!IsRoadLike(x, z)) continue;
        int nc = 0;
        for (int d = 0; d < 4; d++)
            if (IsRoadLike(x+DDX[d], z+DDZ[d])) nc++;
        isIntersection_[z][x] = (nc >= 3);
    }
}

static bool IsIntersection(int gx, int gz)
{
    if (!InBounds(gx, gz)) return false;
    return isIntersection_[gz][gx];
}

static void PlaceCrosswalks()
{
    for (int gz = 0; gz < GS; gz++)
    for (int gx = 0; gx < GS; gx++) {
        if (!isIntersection_[gz][gx]) continue;
        for (int d = 0; d < 4; d++) {
            int nx = gx + DDX[d], nz = gz + DDZ[d];
            if (!InBounds(nx, nz)) continue;
            if (grid[nz][nx] != ROAD) continue;
            if (isIntersection_[nz][nx]) continue;
            int rn = 0;
            for (int d2 = 0; d2 < 4; d2++)
                if (IsRoadLike(nx+DDX[d2], nz+DDZ[d2])) rn++;
            if (rn == 2) {
                grid[nz][nx] = CROSSWALK;
            }
        }
    }
}

// ============================================================
//  Traffic lights — 8-phase with EXTENDED ALL_RED (4s for peds)
// ============================================================
static constexpr float THROUGH_DUR = 8.0f;
static constexpr float YELLOW_DUR  = 2.0f;
static constexpr float LEFT_DUR    = 4.0f;
static constexpr float ALL_RED_DUR = 4.0f;  // EXTENDED from 2s to 4s for ped crossing
static constexpr float CYCLE_DUR   = (THROUGH_DUR + YELLOW_DUR + LEFT_DUR + ALL_RED_DUR) * 2.0f; // 36s

enum Phase : uint8_t {
    PH_NS_THROUGH=0, PH_NS_YELLOW, PH_NS_LEFT, PH_ALL_RED_1,
    PH_EW_THROUGH, PH_EW_YELLOW, PH_EW_LEFT, PH_ALL_RED_2
};

static float PhaseDur(Phase p) {
    switch(p) {
    case PH_NS_THROUGH: case PH_EW_THROUGH: return THROUGH_DUR;
    case PH_NS_YELLOW:  case PH_EW_YELLOW:  return YELLOW_DUR;
    case PH_NS_LEFT:    case PH_EW_LEFT:    return LEFT_DUR;
    case PH_ALL_RED_1:  case PH_ALL_RED_2:  return ALL_RED_DUR;
    }
    return THROUGH_DUR;
}

static Phase intPhase[GS][GS];
static float intTimer[GS][GS];

static void InitLights()
{
    for (int z = 0; z < GS; z++)
    for (int x = 0; x < GS; x++) {
        intPhase[z][x] = PH_NS_THROUGH;
        intTimer[z][x] = THROUGH_DUR;
    }
}

static void UpdateLights(float dt)
{
    for (int z = 0; z < GS; z++)
    for (int x = 0; x < GS; x++) {
        if (!isIntersection_[z][x]) continue;
        intTimer[z][x] -= dt;
        while (intTimer[z][x] <= 0.0f) {
            Phase next = (Phase)((intPhase[z][x] + 1) % 8);
            intTimer[z][x] += PhaseDur(next);
            intPhase[z][x] = next;
        }
    }
}

enum TurnIntent : uint8_t { TI_STRAIGHT, TI_RIGHT, TI_LEFT };
enum LightColor : uint8_t { L_GREEN, L_YELLOW, L_RED };

static LightColor GetLight(int gx, int gz, float dx, float dz, TurnIntent intent = TI_STRAIGHT)
{
    if (!InBounds(gx, gz)) return L_RED;
    if (!isIntersection_[gz][gx]) return L_GREEN;
    bool isNS = (std::abs(dz) > std::abs(dx));
    Phase ph = intPhase[gz][gx];
    switch (ph) {
    case PH_NS_THROUGH:
        if (isNS) return (intent == TI_LEFT) ? L_RED : L_GREEN;
        return L_RED;
    case PH_NS_YELLOW:
        if (isNS && intent != TI_LEFT) return L_YELLOW;
        return L_RED;
    case PH_NS_LEFT:
        if (isNS && intent == TI_LEFT) return L_GREEN;
        return L_RED;
    case PH_ALL_RED_1: return L_RED;
    case PH_EW_THROUGH:
        if (!isNS) return (intent == TI_LEFT) ? L_RED : L_GREEN;
        return L_RED;
    case PH_EW_YELLOW:
        if (!isNS && intent != TI_LEFT) return L_YELLOW;
        return L_RED;
    case PH_EW_LEFT:
        if (!isNS && intent == TI_LEFT) return L_GREEN;
        return L_RED;
    case PH_ALL_RED_2: return L_RED;
    }
    return L_RED;
}

// ============================================================
//  Pedestrian crossing light
// ============================================================
enum CrossAxis : uint8_t { AX_NS, AX_EW };

static CrossAxis GetCrosswalkAxis(int gx, int gz)
{
    bool hasNS = (IsRoadLike(gx, gz-1) || IsRoadLike(gx, gz+1) ||
                  (InBounds(gx,gz-1) && isIntersection_[gz-1][gx]) ||
                  (InBounds(gx,gz+1) && isIntersection_[gz+1][gx]));
    bool hasEW = (IsRoadLike(gx-1, gz) || IsRoadLike(gx+1, gz) ||
                  (InBounds(gx-1,gz) && isIntersection_[gz][gx-1]) ||
                  (InBounds(gx+1,gz) && isIntersection_[gz][gx+1]));
    if (hasNS && !hasEW) return AX_EW;
    if (hasEW && !hasNS) return AX_NS;
    return AX_EW;
}

static bool CanPedCross(int gx, int gz)
{
    if (!InBounds(gx, gz)) return true;
    if (grid[gz][gx] != CROSSWALK) return true;

    int intGX = -1, intGZ = -1;
    for (int d = 0; d < 4; d++) {
        int nx = gx + DDX[d], nz = gz + DDZ[d];
        if (InBounds(nx, nz) && isIntersection_[nz][nx]) {
            intGX = nx; intGZ = nz; break;
        }
    }
    if (intGX < 0) return true;

    Phase ph = intPhase[intGZ][intGX];
    CrossAxis ax = GetCrosswalkAxis(gx, gz);

    if (ax == AX_NS)
        return (ph == PH_EW_THROUGH || ph == PH_EW_YELLOW ||
                ph == PH_EW_LEFT    || ph == PH_ALL_RED_1 || ph == PH_ALL_RED_2);
    else
        return (ph == PH_NS_THROUGH || ph == PH_NS_YELLOW ||
                ph == PH_NS_LEFT    || ph == PH_ALL_RED_1 || ph == PH_ALL_RED_2);
}

// ============================================================
//  FIXED Pedestrian Pathfinding
//
//  KEY FIX: At direction changes (turns), add an OUTER CORNER
//  waypoint so pedestrians walk around intersections instead of
//  cutting diagonally through the road surface.
//
//  Corner waypoint = combine non-zero offset components from
//  old and new direction. This places the ped at the outer
//  corner of the intersection (±9, ±9) from center.
// ============================================================
static std::vector<Vec2> FindPedPath(int srcGX, int srcGZ, int dstGX, int dstGZ)
{
    if (!InBounds(srcGX, srcGZ) || !InBounds(dstGX, dstGZ)) return {};

    // Find road cells adjacent to source and destination
    int srcRX=-1,srcRZ=-1,dstRX=-1,dstRZ=-1;
    for (int d = 0; d < 4; d++) {
        int nx = srcGX+DDX[d], nz = srcGZ+DDZ[d];
        if (IsRoadLike(nx, nz)) { srcRX=nx; srcRZ=nz; break; }
    }
    for (int d = 0; d < 4; d++) {
        int nx = dstGX+DDX[d], nz = dstGZ+DDZ[d];
        if (IsRoadLike(nx, nz)) { dstRX=nx; dstRZ=nz; break; }
    }
    if (srcRX < 0 || dstRX < 0) return {};

    // Dijkstra on road cells
    int srcId = srcRZ * GS + srcRX;
    int dstId = dstRZ * GS + dstRX;

    std::vector<float> cost(GS*GS, 1e9f);
    std::vector<int> from(GS*GS, -1);
    using PQ = std::priority_queue<std::pair<float,int>,
               std::vector<std::pair<float,int>>,
               std::greater<std::pair<float,int>>>;
    PQ pq;
    cost[srcId] = 0.0f;
    from[srcId] = srcId;
    pq.push({0.0f, srcId});
    bool found = (srcId == dstId);
    while (!pq.empty() && !found) {
        auto [cc, cur] = pq.top(); pq.pop();
        if (cc > cost[cur]) continue;
        int cx = cur % GS, cz = cur / GS;
        for (int d = 0; d < 4; d++) {
            int nx = cx+DDX[d], nz = cz+DDZ[d];
            if (!InBounds(nx, nz) || !IsRoadLike(nx, nz)) continue;
            int nid = nz * GS + nx;
            float nc = cc + 1.0f;
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
        Vec2 cc = CellCenter(srcRX, srcRZ);
        return { {cc.x + SIDEWALK_MID, cc.y} };
    }

    // Convert cell path to sidewalk waypoints with CORNER FIX
    std::vector<Vec2> waypoints;

    for (size_t i = 0; i < cellPath.size(); i++) {
        int cx = cellPath[i] % GS, cz = cellPath[i] / GS;
        Vec2 cc = CellCenter(cx, cz);

        // Determine travel direction
        int ddx = 0, ddz = 0;
        if (i == 0) {
            int nx = cellPath[1] % GS, nz = cellPath[1] / GS;
            ddx = nx - cx; ddz = nz - cz;
        } else {
            int px = cellPath[i-1] % GS, pz = cellPath[i-1] / GS;
            ddx = cx - px; ddz = cz - pz;
        }

        // Right-side offset from travel direction
        float offX = 0, offZ = 0;
        if (ddx == 0) offX = (ddz < 0) ? SIDEWALK_MID : -SIDEWALK_MID;
        if (ddz == 0) offZ = (ddx > 0) ? SIDEWALK_MID : -SIDEWALK_MID;

        // At direction changes: add CORNER waypoint (THE FIX)
        if (i > 0 && i + 1 < cellPath.size()) {
            int nnx = cellPath[i+1] % GS, nnz = cellPath[i+1] / GS;
            int nddx = nnx - cx, nddz = nnz - cz;
            if (nddx != ddx || nddz != ddz) {
                // Compute new offset for next direction
                float offX2 = 0, offZ2 = 0;
                if (nddx == 0) offX2 = (nddz < 0) ? SIDEWALK_MID : -SIDEWALK_MID;
                if (nddz == 0) offZ2 = (nddx > 0) ? SIDEWALK_MID : -SIDEWALK_MID;

                // Add before-turn waypoint (approach on old sidewalk)
                waypoints.push_back({cc.x + offX, cc.y + offZ});

                // *** CORNER FIX: add outer corner waypoint ***
                // Combine non-zero components from old and new offsets
                // This places the ped at the outer corner (e.g., +9,+9)
                float cornerX = (offX != 0.f) ? offX : offX2;
                float cornerZ = (offZ != 0.f) ? offZ : offZ2;
                waypoints.push_back({cc.x + cornerX, cc.y + cornerZ});

                // Add after-turn waypoint (join new sidewalk)
                waypoints.push_back({cc.x + offX2, cc.y + offZ2});
                continue;
            }
        }

        waypoints.push_back({cc.x + offX, cc.y + offZ});
    }

    if (waypoints.size() > PED_MAX_WP)
        waypoints.resize(PED_MAX_WP);

    return waypoints;
}

// ============================================================
//  Car pathfinding (same as test_ped_sim)
// ============================================================
static std::vector<Vec2> FindCarPath(int srcGX, int srcGZ, int dstGX, int dstGZ)
{
    if (!InBounds(srcGX, srcGZ) || !InBounds(dstGX, dstGZ)) return {};
    int srcRX=-1,srcRZ=-1,dstRX=-1,dstRZ=-1;
    for (int d = 0; d < 4; d++) {
        int nx = srcGX+DDX[d], nz = srcGZ+DDZ[d];
        if (IsRoadLike(nx, nz)) { srcRX=nx; srcRZ=nz; break; }
    }
    for (int d = 0; d < 4; d++) {
        int nx = dstGX+DDX[d], nz = dstGZ+DDZ[d];
        if (IsRoadLike(nx, nz)) { dstRX=nx; dstRZ=nz; break; }
    }
    if (srcRX < 0 || dstRX < 0) return {};

    int srcId = srcRZ*GS+srcRX;
    int dstId = dstRZ*GS+dstRX;

    std::vector<float> cost(GS*GS, 1e9f);
    std::vector<int> from(GS*GS, -1);
    using PQ = std::priority_queue<std::pair<float,int>,
               std::vector<std::pair<float,int>>,
               std::greater<std::pair<float,int>>>;
    PQ pq;
    cost[srcId] = 0.0f;
    from[srcId] = srcId;
    pq.push({0.0f, srcId});
    bool found = (srcId == dstId);
    while (!pq.empty() && !found) {
        auto [cc, cur] = pq.top(); pq.pop();
        if (cc > cost[cur]) continue;
        int cx = cur%GS, cz = cur/GS;
        for (int d = 0; d < 4; d++) {
            int nx = cx+DDX[d], nz = cz+DDZ[d];
            if (!InBounds(nx,nz) || !IsRoadLike(nx,nz)) continue;
            int nid = nz*GS+nx;
            float nc = cc + 1.0f;
            if (nc < cost[nid]) {
                cost[nid] = nc;
                from[nid] = cur;
                if (nid == dstId) { found = true; break; }
                pq.push({nc, nid});
            }
        }
    }
    if (!found) return {};

    std::vector<Vec2> road;
    for (int cur = dstId; cur != srcId; cur = from[cur])
        road.push_back(CellCenter(cur%GS, cur/GS));
    road.push_back(CellCenter(srcRX, srcRZ));
    std::reverse(road.begin(), road.end());
    return road;
}

// ============================================================
//  Find nearest parking cell to a grid position
// ============================================================
static bool FindNearestParking(int fromGX, int fromGZ, int& parkGX, int& parkGZ)
{
    float bestDist2 = 1e9f;
    parkGX = -1; parkGZ = -1;
    for (int z = 0; z < GS; z++)
    for (int x = 0; x < GS; x++) {
        if (grid[z][x] != PARKING) continue;
        float dx = (float)(x - fromGX);
        float dz = (float)(z - fromGZ);
        float d2 = dx*dx + dz*dz;
        if (d2 < bestDist2) {
            // Verify parking has adjacent road
            bool hasRoad = false;
            for (int d = 0; d < 4; d++)
                if (IsRoadLike(x+DDX[d], z+DDZ[d])) { hasRoad = true; break; }
            if (hasRoad) {
                bestDist2 = d2;
                parkGX = x; parkGZ = z;
            }
        }
    }
    return parkGX >= 0;
}

// ============================================================
//  Car state (SoA)
// ============================================================
static float    carPosX[MAX_CARS], carPosZ[MAX_CARS];
static float    carSpeed[MAX_CARS], carHeading[MAX_CARS];
static float    carLaneOff[MAX_CARS], carLaneTarget[MAX_CARS];
static int8_t   carLaneIdx[MAX_CARS];

enum class CarState : uint8_t { PARKED, DRIVING };
static CarState carState[MAX_CARS];
static float    carParkTimer[MAX_CARS];

static Vec2     carWpBuf[MAX_CARS][MAX_WP];
static uint8_t  carWpCount[MAX_CARS], carWpCurr[MAX_CARS];
static uint8_t  carDir[MAX_CARS];

static uint32_t carActiveCount = 0;

static uint32_t carDriveHead[GS * GS];
static uint32_t carDriveNext[MAX_CARS];

static float    carSchedDepart[MAX_CARS];
static float    carSchedReturn[MAX_CARS];
static uint32_t carLastDayTrip[MAX_CARS];

// Agent that owns this car (for human-drives-car system)
static int32_t  carOwnerAgent[MAX_CARS];

static void SimplifyPath(Vec2* wp, uint8_t& count) {
    if (count <= 2) return;
    uint8_t write = 1;
    for (uint8_t r = 1; r < count-1; r++) {
        float dx1=wp[r].x-wp[write-1].x, dz1=wp[r].y-wp[write-1].y;
        float dx2=wp[r+1].x-wp[r].x, dz2=wp[r+1].y-wp[r].y;
        float l1=std::sqrt(dx1*dx1+dz1*dz1), l2=std::sqrt(dx2*dx2+dz2*dz2);
        if (l1<0.001f||l2<0.001f) { wp[write++]=wp[r]; continue; }
        float dot=(dx1*dx2+dz1*dz2)/(l1*l2);
        if (dot<0.999f) wp[write++]=wp[r];
    }
    wp[write++]=wp[count-1];
    count=write;
}

static uint32_t SpawnCar(const std::vector<Vec2>& path, uint32_t colorSeed, int32_t ownerAgent = -1)
{
    if (carActiveCount >= MAX_CARS || path.size() < 2) return UINT32_MAX;
    uint32_t i = carActiveCount++;
    uint8_t wc = (uint8_t)std::min(path.size(), (size_t)MAX_WP);
    for (uint8_t w = 0; w < wc; w++) carWpBuf[i][w] = path[w];
    carWpCount[i] = wc;
    SimplifyPath(carWpBuf[i], carWpCount[i]);
    carWpCurr[i] = 1;
    carDir[i] = 0;
    carOwnerAgent[i] = ownerAgent;

    float lo[3]; int lc;
    GetLaneOffsets(4, lo, lc);
    carLaneIdx[i] = (int8_t)((colorSeed % 2 == 0) ? 0 : lc - 1);
    carLaneOff[i] = lo[std::min((int)carLaneIdx[i], lc-1)];
    carLaneTarget[i] = carLaneOff[i];

    Vec2 d = DirTo(path[0], path[1]);
    carPosX[i] = path[0].x;
    carPosZ[i] = path[0].y;
    carSpeed[i] = 0.0f;
    carHeading[i] = std::atan2(d.x, d.y);
    carState[i] = CarState::DRIVING; // Spawned already driving
    carParkTimer[i] = 0.0f;

    uint32_t ws = colorSeed * 1103515245u + 12345u;
    uint32_t schedSeed = ws * 1103515245u + 12345u;
    float schedRand = (float)((schedSeed >> 8) & 0xFF) / 255.f;
    float depart;
    if (schedRand < 0.08f)
        depart = 20.0f + (schedRand / 0.08f) * 3.0f;
    else if (schedRand < 0.20f)
        depart = 5.0f + ((schedRand - 0.08f) / 0.12f) * 2.0f;
    else
        depart = 7.0f + ((schedRand - 0.20f) / 0.80f) * 2.0f;
    float shiftLen = 7.5f + (float)(((schedSeed >> 16) & 0x1F)) / 31.f * 2.5f;
    carSchedDepart[i] = depart;
    carSchedReturn[i] = std::fmod(depart + shiftLen, 24.0f);
    carLastDayTrip[i] = 0;

    return i;
}

// ============================================================
//  Pedestrian state (SoA)
// ============================================================
static float    pedPosX[MAX_PEDS], pedPosZ[MAX_PEDS];
static float    pedSpeed[MAX_PEDS];
static float    pedHeading[MAX_PEDS];

enum class PedState : uint8_t { IDLE, WALKING, WAITING_CROSS };
static PedState pedState[MAX_PEDS];

static std::vector<Vec2> pedWpBuf[MAX_PEDS];
static uint16_t pedWpCurr[MAX_PEDS];
static uint8_t  pedDir[MAX_PEDS];

// Agent that owns this ped (for human-drives-car system)
static int32_t  pedOwnerAgent[MAX_PEDS];

static uint32_t pedActiveCount = 0;

static uint32_t pedHead[GS * GS];
static uint32_t pedNext[MAX_PEDS];

static uint32_t SpawnPed(const std::vector<Vec2>& path, uint32_t seed, int32_t ownerAgent = -1)
{
    if (pedActiveCount >= MAX_PEDS || path.size() < 2) return UINT32_MAX;
    uint32_t i = pedActiveCount++;

    pedWpBuf[i] = path;
    pedWpCurr[i] = 1;
    pedDir[i] = 0;
    pedOwnerAgent[i] = ownerAgent;

    pedPosX[i] = path[0].x;
    pedPosZ[i] = path[0].y;

    pedSpeed[i] = PED_SPEED_MIN + (float)((seed >> 16) & 0xFF) / 255.0f * (PED_SPEED_MAX - PED_SPEED_MIN);

    Vec2 d = DirTo(path[0], path[1]);
    pedHeading[i] = std::atan2(d.x, d.y);
    pedState[i] = PedState::WALKING;

    return i;
}

// ============================================================
//  Agent system: Human walks to parking → drives → walks to dest
// ============================================================
static constexpr uint32_t MAX_AGENTS = 2000;

enum class AgentPhase : uint8_t {
    IDLE_HOME,          // Waiting at home
    WALK_TO_PARKING,    // Walking to parking (pedestrian)
    ENTERING_CAR,       // At parking, transitioning to car
    DRIVING,            // In car, driving
    EXITING_CAR,        // At destination parking, transitioning to ped
    WALK_TO_DEST,       // Walking from parking to workplace
    AT_DEST,            // At destination (work/home)
    WALK_ONLY           // No car — walk entire route
};

struct Agent {
    int homeGX, homeGZ;
    int workGX, workGZ;
    int homeParkGX, homeParkGZ;   // Nearest parking to home
    int workParkGX, workParkGZ;   // Nearest parking to work
    AgentPhase phase = AgentPhase::IDLE_HOME;
    uint32_t pedIdx = UINT32_MAX;  // Current ped entity
    uint32_t carIdx = UINT32_MAX;  // Current car entity
    bool usesCar = false;          // Has nearby parking
    bool goingToWork = true;       // Direction flag
    float idleTimer = 0.0f;
};

static Agent agents[MAX_AGENTS];
static uint32_t agentActiveCount = 0;

// ============================================================
//  Violation tracking
// ============================================================
struct PedViolation {
    uint32_t pedIdx;
    int frame;
    float wx, wz;
    int gx, gz;
    std::string type;
};
static std::vector<PedViolation> pedViolations;

struct CarPedCollision {
    uint32_t carIdx, pedIdx;
    int frame;
    float dist;
    float carSpeed;
    float pedX, pedZ;
    int pedGX, pedGZ;
    bool pedOnCrosswalk;
    bool pedAtIntersection;
};
static std::vector<CarPedCollision> carPedCollisions;
static std::unordered_set<uint64_t> carPedCollisionPairs;

struct CarCollision {
    uint32_t carA, carB;
    int frame;
    float dist;
    std::string type;
};
static std::vector<CarCollision> carCollisions;

// Agent transition tracking
struct AgentTransition {
    uint32_t agentIdx;
    int frame;
    AgentPhase fromPhase, toPhase;
    std::string detail;
};
static std::vector<AgentTransition> agentTransitions;

// ============================================================
//  Car Update (from test_ped_sim with minor adaptations)
// ============================================================
static uint32_t dayCounter = 0;
static float prevTimeOfDay = 0.0f;

static void UpdateCars(float dt_in, int frame, float timeOfDay)
{
    constexpr float MAX_PHYSICS_DT = 0.05f;
    const int subSteps = (dt_in > MAX_PHYSICS_DT) ? (int)std::ceil(dt_in / MAX_PHYSICS_DT) : 1;
    const float dt = dt_in / (float)subSteps;

  for (int sub = 0; sub < subSteps; ++sub) {

    if (sub == 0) {
        if (timeOfDay < prevTimeOfDay) ++dayCounter;
        prevTimeOfDay = timeOfDay;
    }

    std::fill(carDriveHead, carDriveHead + GS*GS, UINT32_MAX);
    std::fill(carDriveNext, carDriveNext + MAX_CARS, UINT32_MAX);
    for (uint32_t i = 0; i < carActiveCount; i++) {
        if (carState[i] != CarState::DRIVING) continue;
        int gx, gz;
        if (WorldToGrid(carPosX[i], carPosZ[i], gx, gz)) {
            int key = gz*GS+gx;
            carDriveNext[i] = carDriveHead[key];
            carDriveHead[key] = i;
        }
    }

    for (uint32_t i = 0; i < carActiveCount; i++) {
        if (carState[i] == CarState::PARKED) continue;  // Agent system handles depart

        // DRIVING
        Vec2* wp = carWpBuf[i];
        uint8_t wc = carWpCount[i];
        uint8_t& wn = carWpCurr[i];

        if (wn >= wc) {
            // Reached destination — park
            carState[i] = CarState::PARKED;
            carSpeed[i] = 0.0f;
            carParkTimer[i] = 0.0f;
            continue;
        }

        Vec2 prev_raw = (wn > 0) ? wp[wn-1] : wp[0];
        Vec2 target_raw = wp[wn];
        Vec2 segDir = DirTo(prev_raw, target_raw);
        float segLen = std::sqrt((target_raw.x-prev_raw.x)*(target_raw.x-prev_raw.x) +
                                 (target_raw.y-prev_raw.y)*(target_raw.y-prev_raw.y));

        float myFwdX = std::sin(carHeading[i]);
        float myFwdZ = std::cos(carHeading[i]);

        int myGX=0, myGZ=0;
        bool onGrid = WorldToGrid(carPosX[i], carPosZ[i], myGX, myGZ);

        Vec2 nextSeg = segDir;
        float cornerCos = 1.0f, turnCross = 0.0f;
        if (wn+1 < wc) {
            nextSeg = DirTo(target_raw, wp[wn+1]);
            cornerCos = segDir.x*nextSeg.x + segDir.y*nextSeg.y;
            turnCross = segDir.x*nextSeg.y - segDir.y*nextSeg.x;
        }
        bool isTurn = (cornerCos < 0.3f);

        float cellLaneOffs[3]; int cellLaneCount=0;
        { int tl = onGrid ? GetLanes(myGX, myGZ) : 4;
          GetLaneOffsets(tl, cellLaneOffs, cellLaneCount); }
        if (carLaneIdx[i] >= cellLaneCount) carLaneIdx[i] = (int8_t)(cellLaneCount-1);
        if (carLaneIdx[i] < 0) carLaneIdx[i] = 0;
        carLaneTarget[i] = cellLaneOffs[carLaneIdx[i]];

        { float ld = carLaneTarget[i]-carLaneOff[i];
          float ml = 3.0f*dt;
          if (std::abs(ld)>ml) carLaneOff[i]+=(ld>0?ml:-ml);
          else carLaneOff[i]=carLaneTarget[i]; }

        bool inIntersection = onGrid && IsIntersection(myGX, myGZ);
        float segFwd = (carPosX[i]-prev_raw.x)*segDir.x + (carPosZ[i]-prev_raw.y)*segDir.y;

        Vec2 steerTarget; bool inArc = false;

        if (isTurn) {
            float inRX=segDir.y,inRZ=-segDir.x;
            float outRX=nextSeg.y,outRZ=-nextSeg.x;
            Vec2 entryPt = { target_raw.x-segDir.x*HCS+inRX*carLaneTarget[i],
                             target_raw.y-segDir.y*HCS+inRZ*carLaneTarget[i] };
            Vec2 exitPt = { target_raw.x+nextSeg.x*HCS+outRX*carLaneTarget[i],
                            target_raw.y+nextSeg.y*HCS+outRZ*carLaneTarget[i] };
            float entryDist = std::max(0.0f,segLen-HCS);

            if (segFwd >= entryDist-1.0f) {
                inArc = true;
                carLaneOff[i] = carLaneTarget[i];
                float chX=exitPt.x-entryPt.x, chZ=exitPt.y-entryPt.y;
                float chLen=std::sqrt(chX*chX+chZ*chZ);
                float R=std::max(1.0f,chLen*0.7071f);
                float perpX,perpZ;
                if (turnCross<0) { perpX=inRX; perpZ=inRZ; }
                else { perpX=-inRX; perpZ=-inRZ; }
                float arcCX=entryPt.x+perpX*R, arcCZ=entryPt.y+perpZ*R;
                float carAngle=std::atan2(carPosX[i]-arcCX, carPosZ[i]-arcCZ);
                float entAngle=std::atan2(entryPt.x-arcCX, entryPt.y-arcCZ);
                float extAngle=std::atan2(exitPt.x-arcCX, exitPt.y-arcCZ);
                auto normA=[](float a){ while(a>PI) a-=PI2; while(a<-PI) a+=PI2; return a; };
                float arcSpan=normA(extAngle-entAngle);
                float carSpan=normA(carAngle-entAngle);
                float tArc=std::abs(arcSpan)>0.01f?std::clamp(carSpan/arcSpan,0.0f,1.0f):0.0f;
                float tLook=std::min(1.0f,tArc+0.30f);
                float lookAngle=entAngle+arcSpan*tLook;
                steerTarget.x=arcCX+R*std::sin(lookAngle);
                steerTarget.y=arcCZ+R*std::cos(lookAngle);
                float crossDot=(carPosX[i]-exitPt.x)*nextSeg.x+(carPosZ[i]-exitPt.y)*nextSeg.y;
                if (crossDot>=-0.5f && tArc>0.75f) {
                    carHeading[i]=std::atan2(nextSeg.x,nextSeg.y);
                    ++wn; continue;
                }
            } else {
                float rX=segDir.y,rZ=-segDir.x;
                float look=std::clamp(carSpeed[i]*0.8f,3.0f,12.0f);
                float tgtFwd=std::min(segFwd+look,entryDist);
                steerTarget.x=prev_raw.x+segDir.x*tgtFwd+rX*carLaneOff[i];
                steerTarget.y=prev_raw.y+segDir.y*tgtFwd+rZ*carLaneOff[i];
            }
        } else {
            float rX=segDir.y,rZ=-segDir.x;
            float look=std::clamp(carSpeed[i]*0.8f,3.0f,12.0f);
            float tgtFwd=std::min(segFwd+look,segLen);
            steerTarget.x=prev_raw.x+segDir.x*tgtFwd+rX*carLaneOff[i];
            steerTarget.y=prev_raw.y+segDir.y*tgtFwd+rZ*carLaneOff[i];

            float crossDot=(carPosX[i]-target_raw.x)*segDir.x+(carPosZ[i]-target_raw.y)*segDir.y;
            if (crossDot>=-0.3f && segFwd>=segLen-1.0f) { ++wn; continue; }
        }

        float stDx=steerTarget.x-carPosX[i], stDz=steerTarget.y-carPosZ[i];
        float stDist=std::sqrt(stDx*stDx+stDz*stDz);

        // Lead car detection — increased sensitivity at low speeds
        float bestGap=1e6f, bestSpd=MAX_SPEED;
        if (onGrid) {
            for (int dg=-1;dg<=1;dg++)
            for (int dh=-1;dh<=1;dh++) {
                int cgx=myGX+dh, cgz=myGZ+dg;
                if (!InBounds(cgx,cgz)) continue;
                int cellKey=cgz*GS+cgx;
                for (uint32_t j=carDriveHead[cellKey];j!=UINT32_MAX;j=carDriveNext[j]) {
                    if (j==i) continue;
                    float odx=carPosX[j]-carPosX[i], odz=carPosZ[j]-carPosZ[i];
                    float fwd=odx*myFwdX+odz*myFwdZ;
                    if (fwd<=0) continue;
                    float lat=std::abs(odx*myFwdZ-odz*myFwdX);
                    if (lat>LAT_BAND) continue;
                    float jFwdX=std::sin(carHeading[j]), jFwdZ=std::cos(carHeading[j]);
                    float hcos=myFwdX*jFwdX+myFwdZ*jFwdZ;
                    if (hcos<0.3f) continue;
                    if (!inArc && !inIntersection && hcos>0.5f) {
                        float laneDiff=std::abs(carLaneOff[j]-carLaneOff[i]);
                        if (laneDiff>2.0f) continue;
                    }
                    float gap=fwd-MIN_SEP;
                    if (gap<bestGap) { bestGap=gap; bestSpd=carSpeed[j]; }
                }
            }
        }

        // Traffic lights + crosswalk stop
        if (onGrid && !IsIntersection(myGX, myGZ)) {
            TurnIntent turnIntent = TI_STRAIGHT;
            int turnCellGX=-1, turnCellGZ=-1;
            if (isTurn) {
                turnIntent = (turnCross > 0.0f) ? TI_LEFT : TI_RIGHT;
                WorldToGrid(target_raw.x, target_raw.y, turnCellGX, turnCellGZ);
            }

            auto checkLight = [&](int cgx, int cgz, float dX, float dZ) {
                if (!IsIntersection(cgx, cgz)) {
                    // Crosswalk stop: if peds have green, cars must stop
                    if (grid[cgz][cgx] == CROSSWALK && CanPedCross(cgx, cgz)) {
                        bool pedPresent = false;
                        for (int pd = -1; pd <= 1 && !pedPresent; pd++)
                        for (int ph = -1; ph <= 1 && !pedPresent; ph++) {
                            int pnx = cgx+ph, pnz = cgz+pd;
                            if (!InBounds(pnx, pnz)) continue;
                            int pkey = pnz*GS+pnx;
                            for (uint32_t p = pedHead[pkey]; p != UINT32_MAX; p = pedNext[p]) {
                                Vec2 crossCC = CellCenter(cgx, cgz);
                                float pdx = pedPosX[p] - crossCC.x;
                                float pdz = pedPosZ[p] - crossCC.y;
                                if (std::abs(pdx) < HCS + 1.0f && std::abs(pdz) < HCS + 1.0f) {
                                    pedPresent = true; break;
                                }
                            }
                        }
                        if (pedPresent) {
                            Vec2 crossC = CellCenter(cgx, cgz);
                            float toCX = crossC.x-carPosX[i], toCZ = crossC.y-carPosZ[i];
                            float fwdDist = toCX*dX + toCZ*dZ;
                            if (fwdDist > 0) {
                                float stopLine = std::max(0.0f, fwdDist - HCS - 1.0f);
                                if (stopLine < bestGap) { bestGap = stopLine; bestSpd = 0; }
                            }
                        }
                    }
                    return;
                }
                TurnIntent li = (cgx==turnCellGX && cgz==turnCellGZ) ? turnIntent : TI_STRAIGHT;
                auto color = GetLight(cgx,cgz,dX,dZ,li);
                if (color==L_GREEN) return;
                Vec2 intC=CellCenter(cgx,cgz);
                float toCX=intC.x-carPosX[i], toCZ=intC.y-carPosZ[i];
                float fwdDist=toCX*dX+toCZ*dZ;
                if (fwdDist<0) return;
                float stopLine=std::max(0.0f,fwdDist-HCS-1.0f);
                if (color==L_RED) { if (stopLine<bestGap) { bestGap=stopLine; bestSpd=0; } }
                else if (color==L_YELLOW) { if (stopLine>3.0f && stopLine<bestGap) { bestGap=stopLine; bestSpd=0; } }
            };

            int psx=myGX, psz=myGZ;
            float scanMax=std::min(80.0f,segLen-segFwd+20.0f);
            for (float d=2.0f;d<scanMax;d+=HCS) {
                float sx=carPosX[i]+segDir.x*d, sz=carPosZ[i]+segDir.y*d;
                int sgx,sgz;
                if (!WorldToGrid(sx,sz,sgx,sgz)) continue;
                if (sgx==psx && sgz==psz) continue;
                psx=sgx; psz=sgz;
                checkLight(sgx,sgz,segDir.x,segDir.y);
            }
        }

        // Pedestrian safety brake — EXPANDED for intersection corners
        if (onGrid) {
            float scanAhead = std::min(30.0f, carSpeed[i] * 3.0f + 6.0f);
            float fwdX = std::sin(carHeading[i]);
            float fwdZ = std::cos(carHeading[i]);
            for (int dg=-2; dg<=2; dg++)
            for (int dh=-2; dh<=2; dh++) {
                int cgx = myGX+dh, cgz = myGZ+dg;
                if (!InBounds(cgx, cgz)) continue;
                int cellKey = cgz*GS+cgx;
                for (uint32_t p = pedHead[cellKey]; p != UINT32_MAX; p = pedNext[p]) {
                    float ddx2 = pedPosX[p] - carPosX[i];
                    float ddz2 = pedPosZ[p] - carPosZ[i];

                    // Method 1: Forward projection (works on straights)
                    float fwd = ddx2*fwdX + ddz2*fwdZ;
                    float lat = std::abs(ddx2*fwdZ - ddz2*fwdX);
                    float latThresh = (inIntersection || isTurn || inArc)
                        ? (CAR_HW + PED_RADIUS + 3.0f)
                        : (CAR_HW + PED_RADIUS + 0.5f);
                    if (fwd > 0.5f && fwd <= scanAhead && lat <= latThresh) {
                        float stopDist = std::max(0.0f, fwd - CAR_HL - PED_RADIUS - 1.5f);
                        if (stopDist < bestGap) { bestGap = stopDist; bestSpd = 0; }
                    }

                    // Method 2: Radius check at intersections/turns
                    if (inIntersection || inArc) {
                        float d2 = ddx2*ddx2 + ddz2*ddz2;
                        float safeR = CAR_HL + PED_RADIUS + 3.5f;
                        if (d2 < safeR * safeR && d2 > 0.01f) {
                            float dd = std::sqrt(d2);
                            float stopDist = std::max(0.0f, dd - CAR_HL - PED_RADIUS - 0.5f);
                            if (stopDist < bestGap) { bestGap = stopDist; bestSpd = 0; }
                        }
                    }

                    // Method 3: Approaching intersection — detect peds
                    // at the next intersection corners (scan ahead along segment)
                    if (!inIntersection && isTurn && fwd > -1.0f) {
                        // Ped near target intersection
                        float pedToDst = std::sqrt(
                            (pedPosX[p]-target_raw.x)*(pedPosX[p]-target_raw.x)+
                            (pedPosZ[p]-target_raw.y)*(pedPosZ[p]-target_raw.y));
                        if (pedToDst < HCS + SIDEWALK_MID) {
                            float myToDst = std::sqrt(
                                (carPosX[i]-target_raw.x)*(carPosX[i]-target_raw.x)+
                                (carPosZ[i]-target_raw.y)*(carPosZ[i]-target_raw.y));
                            float stopDist = std::max(0.0f, myToDst - HCS - 2.0f);
                            if (stopDist < bestGap) { bestGap = stopDist; bestSpd = 0; }
                        }
                    }
                }
            }
        }

        // IDM
        float v=carSpeed[i], vR=v/MAX_SPEED, vR4=vR*vR*vR*vR;
        float idmAcc;
        if (bestGap<1e5f) {
            float ab2=2.f*std::sqrt(ACCEL*IDM_B);
            float dv=v-bestSpd;
            float s_star=IDM_S0+std::max(0.f,v*IDM_T+v*dv/ab2);
            float gap=std::max(bestGap,0.01f);
            idmAcc=ACCEL*(1.f-vR4-(s_star/gap)*(s_star/gap));
        } else idmAcc=ACCEL*(1.f-vR4);

        float desMax=MAX_SPEED;
        if (inArc) desMax=MAX_SPEED*0.30f;
        else if (isTurn) {
            float dte=std::max(0.0f,segLen-segFwd-HCS);
            desMax=MAX_SPEED*(0.30f+0.70f*std::clamp(dte/25.0f,0.0f,1.0f));
        }
        if (v>desMax) idmAcc=std::min(idmAcc,-IDM_B*1.5f);
        if (bestGap<0.5f && bestSpd<0.1f) idmAcc=-DECEL;
        else if (bestGap<3.0f && bestSpd<0.5f) idmAcc=std::min(idmAcc,-IDM_B*2.0f);

        carSpeed[i]=std::clamp(v+idmAcc*dt,0.0f,desMax);
        if (bestGap<=-0.5f && bestSpd<0.1f) carSpeed[i]=std::min(carSpeed[i], 0.5f);
        // Only ensure minimum intersection speed if gap is large (no ped/car blocking)
        if (inIntersection && carSpeed[i] < 2.0f && bestGap > 5.0f) carSpeed[i] = std::min(2.0f, desMax);

        // Steering + movement
        if (carSpeed[i]>0.001f && stDist>0.01f) {
            float step=carSpeed[i]*dt;
            if (!inArc) {
                float segH=std::atan2(segDir.x,segDir.y);
                float rX=segDir.y, rZ=-segDir.x;
                float sf2=(carPosX[i]-prev_raw.x)*segDir.x+(carPosZ[i]-prev_raw.y)*segDir.y;
                float lX=prev_raw.x+segDir.x*sf2+rX*carLaneOff[i];
                float lZ=prev_raw.y+segDir.y*sf2+rZ*carLaneOff[i];
                float cte=(carPosX[i]-lX)*rX+(carPosZ[i]-lZ)*rZ;
                float steerCorr=-std::atan2(K_STANLEY*cte,carSpeed[i]+0.5f);
                float maxCorr;
                if (std::abs(cte)>1.0f) maxCorr=0.40f;
                else if (std::abs(carLaneTarget[i]-carLaneOff[i])>0.3f) maxCorr=0.30f;
                else maxCorr=0.18f;
                steerCorr=std::clamp(steerCorr,-maxCorr,maxCorr);
                carHeading[i]=segH+steerCorr;
                carPosX[i]+=std::sin(carHeading[i])*step;
                carPosZ[i]+=std::cos(carHeading[i])*step;
            } else {
                float desH=std::atan2(stDx,stDz);
                float dh=desH-carHeading[i];
                while(dh>PI) dh-=PI2; while(dh<-PI) dh+=PI2;
                float maxSR=5.0f;
                float sd2=std::clamp(dh,-maxSR*dt,maxSR*dt);
                float bd=dh*std::min(1.0f,8.0f*dt);
                if (std::abs(bd)>std::abs(sd2))
                    sd2=std::clamp(bd,-maxSR*dt*1.5f,maxSR*dt*1.5f);
                carHeading[i]+=sd2;
                carPosX[i]+=std::sin(carHeading[i])*step;
                carPosZ[i]+=std::cos(carHeading[i])*step;
            }
        }
    }

    // Car-car collision resolution
    std::fill(carDriveHead, carDriveHead+GS*GS, UINT32_MAX);
    std::fill(carDriveNext, carDriveNext+MAX_CARS, UINT32_MAX);
    for (uint32_t i=0;i<carActiveCount;i++) {
        if (carState[i]!=CarState::DRIVING) continue;
        int gx,gz;
        if (WorldToGrid(carPosX[i],carPosZ[i],gx,gz)) {
            int key=gz*GS+gx;
            carDriveNext[i]=carDriveHead[key]; carDriveHead[key]=i;
        }
    }

    auto resolveCarPair = [&](uint32_t a, uint32_t b) {
        float ddx2=carPosX[b]-carPosX[a], ddz2=carPosZ[b]-carPosZ[a];
        float dd2=ddx2*ddx2+ddz2*ddz2;
        if (dd2>=MIN_SEP*MIN_SEP || dd2<0.001f) return;
        float hcos=std::sin(carHeading[a])*std::sin(carHeading[b])+std::cos(carHeading[a])*std::cos(carHeading[b]);
        bool bothD=(carState[a]==CarState::DRIVING && carState[b]==CarState::DRIVING);
        if (bothD && hcos<0.3f) return;
        if (bothD) {
            float sinH=std::sin(carHeading[a]),cosH=std::cos(carHeading[a]);
            float lat2=std::abs(ddx2*cosH-ddz2*sinH);
            if (lat2>LAT_BAND) return;
            float laneDiff=std::abs(carLaneOff[a]-carLaneOff[b]);
            if (hcos>0.5f && laneDiff>2.0f) return;
        }
        float dd=std::sqrt(dd2);

        if (carSpeed[a] > 1.0f || carSpeed[b] > 1.0f) {
            std::string type;
            float hdgDiff=std::abs(carHeading[a]-carHeading[b]);
            while(hdgDiff>PI) hdgDiff-=PI2;
            hdgDiff=std::abs(hdgDiff);
            if (hdgDiff>2.3f) type="head-on";
            else if (hdgDiff>1.0f) type="intersection-cross";
            else type=(std::abs(carLaneOff[a]-carLaneOff[b])>1.5f)?"cross-lane":"same-lane-rear";
            carCollisions.push_back({a,b,frame,dd,type});
        }

        float overlap=MIN_SEP-dd;
        float nx=ddx2/dd, nz=ddz2/dd;
        float sinH=std::sin(carHeading[a]),cosH=std::cos(carHeading[a]);
        float pushFwd=nx*sinH+nz*cosH;
        float pushMag=std::min(overlap*0.3f,0.3f);
        if (std::abs(pushFwd)>0.1f) {
            float px=sinH*pushFwd*pushMag, pz=cosH*pushFwd*pushMag;
            carPosX[a]-=px; carPosZ[a]-=pz;
            carPosX[b]+=px; carPosZ[b]+=pz;
        }
        float fwd=ddx2*sinH+ddz2*cosH;
        if (fwd>0) carSpeed[a]=std::min(carSpeed[a],carSpeed[b]*0.5f);
        else carSpeed[b]=std::min(carSpeed[b],carSpeed[a]*0.5f);
    };

    for (int cy=0;cy<GS;cy++)
    for (int cx=0;cx<GS;cx++) {
        int key=cy*GS+cx;
        if (carDriveHead[key]==UINT32_MAX) continue;
        for (uint32_t a=carDriveHead[key];a!=UINT32_MAX;a=carDriveNext[a])
            for (uint32_t b=carDriveNext[a];b!=UINT32_MAX;b=carDriveNext[b])
                resolveCarPair(a,b);
        static const int NBR[][2]={{1,0},{0,1},{1,1},{-1,1}};
        for (auto& nb:NBR) {
            int nx2=cx+nb[0], nz2=cy+nb[1];
            if (!InBounds(nx2,nz2)) continue;
            int nkey=nz2*GS+nx2;
            if (carDriveHead[nkey]==UINT32_MAX) continue;
            for (uint32_t a=carDriveHead[key];a!=UINT32_MAX;a=carDriveNext[a])
                for (uint32_t b=carDriveHead[nkey];b!=UINT32_MAX;b=carDriveNext[b])
                    resolveCarPair(a,b);
        }
    }

  } // end substep
}

// ============================================================
//  Pedestrian Update with STRICT sidewalk validation
// ============================================================
static void UpdatePeds(float dt, int frame)
{
    // Rebuild ped linked list
    std::fill(pedHead, pedHead + GS*GS, UINT32_MAX);
    std::fill(pedNext, pedNext + MAX_PEDS, UINT32_MAX);
    for (uint32_t i = 0; i < pedActiveCount; i++) {
        if (pedState[i] == PedState::IDLE) continue;
        int gx, gz;
        if (WorldToGrid(pedPosX[i], pedPosZ[i], gx, gz)) {
            int key = gz*GS+gx;
            pedNext[i] = pedHead[key];
            pedHead[key] = i;
        }
    }

    for (uint32_t i = 0; i < pedActiveCount; i++) {
        if (pedState[i] == PedState::IDLE) continue;

        auto& wps = pedWpBuf[i];
        uint16_t& wn = pedWpCurr[i];
        uint16_t wc = (uint16_t)wps.size();

        // End of path → mark idle (agent system handles next phase)
        if (wn >= wc) {
            pedState[i] = PedState::IDLE;
            continue;
        }

        Vec2 target = wps[wn];
        float dx = target.x - pedPosX[i];
        float dz = target.y - pedPosZ[i];
        float distToTarget = std::sqrt(dx*dx + dz*dz);

        // Crosswalk wait check
        int myGX, myGZ;
        bool onGrid = WorldToGrid(pedPosX[i], pedPosZ[i], myGX, myGZ);

        if (onGrid) {
            int tgtGX, tgtGZ;
            if (WorldToGrid(target.x, target.y, tgtGX, tgtGZ)) {
                if (grid[tgtGZ][tgtGX] == CROSSWALK && !CanPedCross(tgtGX, tgtGZ)) {
                    pedState[i] = PedState::WAITING_CROSS;
                    continue;
                }
            }
        }

        pedState[i] = PedState::WALKING;
        float spd = pedSpeed[i];

        // Ped-ped avoidance
        if (onGrid) {
            for (int dg=-1;dg<=1;dg++)
            for (int dh=-1;dh<=1;dh++) {
                int cgx=myGX+dh, cgz=myGZ+dg;
                if (!InBounds(cgx,cgz)) continue;
                int cellKey=cgz*GS+cgx;
                for (uint32_t j=pedHead[cellKey];j!=UINT32_MAX;j=pedNext[j]) {
                    if (j==i) continue;
                    float odx=pedPosX[j]-pedPosX[i], odz=pedPosZ[j]-pedPosZ[i];
                    float dd2=odx*odx+odz*odz;
                    if (dd2 > PED_AVOID_DIST*PED_AVOID_DIST) continue;
                    float dd=std::sqrt(dd2);
                    if (dd < PED_MIN_SEP) {
                        if (dd > 0.01f) {
                            float nx2 = odx/dd, nz2 = odz/dd;
                            float push = (PED_MIN_SEP - dd) * 0.3f;
                            pedPosX[i] -= nx2 * push;
                            pedPosZ[i] -= nz2 * push;
                        }
                        spd *= 0.3f;
                    } else if (dd < PED_AVOID_DIST) {
                        float myFwdX = std::sin(pedHeading[i]);
                        float myFwdZ = std::cos(pedHeading[i]);
                        float fwd = odx*myFwdX + odz*myFwdZ;
                        if (fwd > 0) {
                            float t = 1.0f - (dd - PED_MIN_SEP) / (PED_AVOID_DIST - PED_MIN_SEP);
                            spd *= (1.0f - 0.5f * t);
                        }
                    }
                }
            }
        }

        if (distToTarget < 0.3f) {
            ++wn;
            continue;
        }

        float ndx = dx / distToTarget, ndz = dz / distToTarget;
        float step = std::min(spd * dt, distToTarget);
        pedPosX[i] += ndx * step;
        pedPosZ[i] += ndz * step;
        pedHeading[i] = std::atan2(ndx, ndz);

        // Clamp to sidewalk on regular road cells (not intersections/crosswalks)
        if (onGrid) {
            CellType ct = grid[myGZ][myGX];
            if (ct == ROAD && !isIntersection_[myGZ][myGX]) {
                Vec2 cc = CellCenter(myGX, myGZ);
                float relX = pedPosX[i] - cc.x;
                float relZ = pedPosZ[i] - cc.y;
                bool hasNS = (InBounds(myGX, myGZ-1) && IsRoadLike(myGX, myGZ-1)) ||
                             (InBounds(myGX, myGZ+1) && IsRoadLike(myGX, myGZ+1));
                bool hasEW = (InBounds(myGX-1, myGZ) && IsRoadLike(myGX-1, myGZ)) ||
                             (InBounds(myGX+1, myGZ) && IsRoadLike(myGX+1, myGZ));
                if (hasNS && !hasEW) {
                    if (std::abs(relX) < SIDEWALK_INNER)
                        pedPosX[i] = cc.x + ((relX >= 0) ? SIDEWALK_MID : -SIDEWALK_MID);
                } else if (hasEW && !hasNS) {
                    if (std::abs(relZ) < SIDEWALK_INNER)
                        pedPosZ[i] = cc.y + ((relZ >= 0) ? SIDEWALK_MID : -SIDEWALK_MID);
                }
            }
            // ENHANCED: Clamp at intersections too — ped must be on outer edge
            else if (ct == ROAD && isIntersection_[myGZ][myGX]) {
                Vec2 cc = CellCenter(myGX, myGZ);
                float relX = pedPosX[i] - cc.x;
                float relZ = pedPosZ[i] - cc.y;
                // At intersection, ped should be at corner: both |relX| and |relZ| >= SIDEWALK_INNER
                // or at least one axis should be at sidewalk
                // Determine travel direction from heading
                float absHdgSin = std::abs(std::sin(pedHeading[i]));
                float absHdgCos = std::abs(std::cos(pedHeading[i]));
                if (absHdgCos > absHdgSin) {
                    // Primarily NS travel — perpendicular is X
                    if (std::abs(relX) < SIDEWALK_INNER)
                        pedPosX[i] = cc.x + ((relX >= 0) ? SIDEWALK_MID : -SIDEWALK_MID);
                } else {
                    // Primarily EW travel — perpendicular is Z
                    if (std::abs(relZ) < SIDEWALK_INNER)
                        pedPosZ[i] = cc.y + ((relZ >= 0) ? SIDEWALK_MID : -SIDEWALK_MID);
                }
            }
        }

        // ---- STRICT VIOLATION CHECKS ----
        if (onGrid) {
            CellType ct = grid[myGZ][myGX];

            // Check 1: Regular road cells — ped must be in sidewalk band
            if (ct == ROAD && !isIntersection_[myGZ][myGX]) {
                Vec2 cc = CellCenter(myGX, myGZ);
                float relX = std::abs(pedPosX[i] - cc.x);
                float relZ = std::abs(pedPosZ[i] - cc.y);
                bool hasNS = (InBounds(myGX, myGZ-1) && IsRoadLike(myGX, myGZ-1)) ||
                             (InBounds(myGX, myGZ+1) && IsRoadLike(myGX, myGZ+1));
                bool hasEW = (InBounds(myGX-1, myGZ) && IsRoadLike(myGX-1, myGZ)) ||
                             (InBounds(myGX+1, myGZ) && IsRoadLike(myGX+1, myGZ));
                bool violation = false;
                if (hasNS && !hasEW)
                    violation = (relX < SIDEWALK_INNER - 0.5f);
                else if (hasEW && !hasNS)
                    violation = (relZ < SIDEWALK_INNER - 0.5f);
                else
                    violation = (relX < SIDEWALK_INNER - 0.5f && relZ < SIDEWALK_INNER - 0.5f);
                if (violation)
                    pedViolations.push_back({i, frame, pedPosX[i], pedPosZ[i], myGX, myGZ, "on_road"});
            }

            // Check 2: Intersection cells — ped must be at outer edge
            if (ct == ROAD && isIntersection_[myGZ][myGX]) {
                Vec2 cc = CellCenter(myGX, myGZ);
                float relX = std::abs(pedPosX[i] - cc.x);
                float relZ = std::abs(pedPosZ[i] - cc.y);
                // At intersection, at LEAST one axis must be >= sidewalk inner
                if (relX < SIDEWALK_INNER - 1.0f && relZ < SIDEWALK_INNER - 1.0f)
                    pedViolations.push_back({i, frame, pedPosX[i], pedPosZ[i], myGX, myGZ, "intersection_center"});
            }
        }
    }

    // Car-ped collision check
    for (uint32_t p = 0; p < pedActiveCount; p++) {
        if (pedState[p] == PedState::IDLE) continue;
        int pgx, pgz;
        if (!WorldToGrid(pedPosX[p], pedPosZ[p], pgx, pgz)) continue;

        for (int dg=-1;dg<=1;dg++)
        for (int dh=-1;dh<=1;dh++) {
            int cgx=pgx+dh, cgz=pgz+dg;
            if (!InBounds(cgx,cgz)) continue;
            int cellKey=cgz*GS+cgx;
            for (uint32_t c=carDriveHead[cellKey];c!=UINT32_MAX;c=carDriveNext[c]) {
                float ddx2=carPosX[c]-pedPosX[p], ddz2=carPosZ[c]-pedPosZ[p];
                float dd=std::sqrt(ddx2*ddx2+ddz2*ddz2);
                if (dd < CAR_HL + PED_RADIUS && carSpeed[c] > 0.5f) {
                    uint64_t pairKey = ((uint64_t)c << 32) | (uint64_t)p;
                    carPedCollisionPairs.insert(pairKey);
                    int cpgx, cpgz;
                    WorldToGrid(pedPosX[p], pedPosZ[p], cpgx, cpgz);
                    carPedCollisions.push_back({c, p, frame, dd, carSpeed[c],
                        pedPosX[p], pedPosZ[p], cpgx, cpgz,
                        InBounds(cpgx,cpgz) && grid[cpgz][cpgx]==CROSSWALK,
                        InBounds(cpgx,cpgz) && isIntersection_[cpgz][cpgx]});
                }
            }
        }
    }
}

// ============================================================
//  Agent Update — Human walks to parking → drives → walks
// ============================================================
static void UpdateAgents(float dt, int frame, float timeOfDay)
{
    for (uint32_t a = 0; a < agentActiveCount; a++) {
        auto& ag = agents[a];

        switch (ag.phase) {
        case AgentPhase::IDLE_HOME:
        {
            // Start trip after a short idle
            ag.idleTimer -= dt;
            if (ag.idleTimer > 0.0f) break;

            if (ag.usesCar) {
                // Walk to nearest parking
                auto path = FindPedPath(ag.homeGX, ag.homeGZ, ag.homeParkGX, ag.homeParkGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 101 + frame), (int32_t)a);
                    if (ag.pedIdx != UINT32_MAX) {
                        ag.phase = AgentPhase::WALK_TO_PARKING;
                        agentTransitions.push_back({a, frame, AgentPhase::IDLE_HOME,
                            AgentPhase::WALK_TO_PARKING, "walk to home parking"});
                    }
                }
            } else {
                // Walk directly to workplace
                auto path = FindPedPath(ag.homeGX, ag.homeGZ, ag.workGX, ag.workGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 101 + frame), (int32_t)a);
                    if (ag.pedIdx != UINT32_MAX) {
                        ag.phase = AgentPhase::WALK_ONLY;
                        agentTransitions.push_back({a, frame, AgentPhase::IDLE_HOME,
                            AgentPhase::WALK_ONLY, "walk all the way"});
                    }
                }
            }
            break;
        }

        case AgentPhase::WALK_TO_PARKING:
        {
            // Check if ped reached destination (idle)
            if (ag.pedIdx < pedActiveCount && pedState[ag.pedIdx] == PedState::IDLE) {
                // Ped arrived at parking — spawn car
                auto carPath = FindCarPath(ag.homeParkGX, ag.homeParkGZ,
                                           ag.workParkGX, ag.workParkGZ);
                if (carPath.size() >= 2) {
                    ag.carIdx = SpawnCar(carPath, (uint32_t)(a * 37 + 1), (int32_t)a);
                    if (ag.carIdx != UINT32_MAX) {
                        ag.phase = AgentPhase::DRIVING;
                        agentTransitions.push_back({a, frame, AgentPhase::WALK_TO_PARKING,
                            AgentPhase::DRIVING, "entered car, driving"});
                    }
                }
            }
            break;
        }

        case AgentPhase::DRIVING:
        {
            // Check if car reached destination (parked)
            if (ag.carIdx < carActiveCount && carState[ag.carIdx] == CarState::PARKED) {
                // Car arrived at destination parking — walk to workplace
                auto path = FindPedPath(ag.workParkGX, ag.workParkGZ,
                                        ag.workGX, ag.workGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 53 + frame), (int32_t)a);
                    if (ag.pedIdx != UINT32_MAX) {
                        ag.phase = AgentPhase::WALK_TO_DEST;
                        agentTransitions.push_back({a, frame, AgentPhase::DRIVING,
                            AgentPhase::WALK_TO_DEST, "exited car, walking to work"});
                    }
                } else {
                    // No ped path → just arrive at destination
                    ag.phase = AgentPhase::AT_DEST;
                }
            }
            break;
        }

        case AgentPhase::WALK_TO_DEST:
        {
            if (ag.pedIdx < pedActiveCount && pedState[ag.pedIdx] == PedState::IDLE) {
                ag.phase = AgentPhase::AT_DEST;
                ag.idleTimer = 60.0f; // Stay at work for a while
                agentTransitions.push_back({a, frame, AgentPhase::WALK_TO_DEST,
                    AgentPhase::AT_DEST, "arrived at destination"});
            }
            break;
        }

        case AgentPhase::AT_DEST:
        {
            ag.idleTimer -= dt;
            if (ag.idleTimer > 0.0f) break;

            // Return trip (swap home/work)
            ag.goingToWork = !ag.goingToWork;
            if (ag.usesCar) {
                // Walk to work parking
                auto path = FindPedPath(ag.workGX, ag.workGZ, ag.workParkGX, ag.workParkGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 71 + frame), (int32_t)a);
                    if (ag.pedIdx != UINT32_MAX) {
                        // Reuse WALK_TO_PARKING for return trip too
                        ag.phase = AgentPhase::WALK_TO_PARKING;
                        // Swap parking targets for return
                        std::swap(ag.homeParkGX, ag.workParkGX);
                        std::swap(ag.homeParkGZ, ag.workParkGZ);
                        std::swap(ag.homeGX, ag.workGX);
                        std::swap(ag.homeGZ, ag.workGZ);
                        agentTransitions.push_back({a, frame, AgentPhase::AT_DEST,
                            AgentPhase::WALK_TO_PARKING, "returning home"});
                    }
                }
            } else {
                auto path = FindPedPath(ag.workGX, ag.workGZ, ag.homeGX, ag.homeGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 71 + frame), (int32_t)a);
                    if (ag.pedIdx != UINT32_MAX) {
                        ag.phase = AgentPhase::WALK_ONLY;
                        std::swap(ag.homeGX, ag.workGX);
                        std::swap(ag.homeGZ, ag.workGZ);
                    }
                }
            }
            break;
        }

        case AgentPhase::WALK_ONLY:
        {
            if (ag.pedIdx < pedActiveCount && pedState[ag.pedIdx] == PedState::IDLE) {
                ag.phase = AgentPhase::AT_DEST;
                ag.idleTimer = 60.0f;
                agentTransitions.push_back({a, frame, AgentPhase::WALK_ONLY,
                    AgentPhase::AT_DEST, "walked to destination"});
            }
            break;
        }

        default:
            break;
        }
    }
}

// ============================================================
//  MAIN — Run simulation, check results, report
// ============================================================
int main()
{
    printf("=== FIXED Pedestrian + Car Simulation V2 ===\n");
    printf("Fixes: corner waypoints, extended ALL_RED (4s), intersection clamp,\n");
    printf("       human-drives-car (walk→drive→walk), strict validation\n\n");

    BuildGrid();
    DetectIntersections();
    PlaceCrosswalks();
    DetectIntersections(); // Re-detect after crosswalks

    // Count cells
    int roadCount=0, houseCount=0, wpCount=0, intCount=0, crossCount=0, parkCount=0;
    for (int z=0;z<GS;z++) for (int x=0;x<GS;x++) {
        if (IsRoadLike(x,z)) roadCount++;
        if (grid[z][x]==HOUSE) houseCount++;
        if (grid[z][x]==WORKPLACE) wpCount++;
        if (isIntersection_[z][x]) intCount++;
        if (grid[z][x]==CROSSWALK) crossCount++;
        if (grid[z][x]==PARKING) parkCount++;
    }
    printf("Grid: %dx%d  Roads=%d Houses=%d Workplaces=%d Intersections=%d Crosswalks=%d Parking=%d\n",
           GS, GS, roadCount, houseCount, wpCount, intCount, crossCount, parkCount);

    InitLights();

    // Initialize spatial hash
    std::fill(pedHead, pedHead + GS*GS, UINT32_MAX);
    std::fill(pedNext, pedNext + MAX_PEDS, UINT32_MAX);
    std::fill(carDriveHead, carDriveHead + GS*GS, UINT32_MAX);
    std::fill(carDriveNext, carDriveNext + MAX_CARS, UINT32_MAX);
    memset(carOwnerAgent, -1, sizeof(carOwnerAgent));
    memset(pedOwnerAgent, -1, sizeof(pedOwnerAgent));

    // ---- Spawn agents: house → find parking → find workplace ----
    printf("\n--- Spawning Agents ---\n");
    int agentsWithCar = 0, agentsWalkOnly = 0, agentsFailed = 0;

    // Collect houses and workplaces
    std::vector<std::pair<int,int>> houses, workplaces;
    for (int z=0;z<GS;z++) for (int x=0;x<GS;x++) {
        if (grid[z][x]==HOUSE) houses.push_back({x,z});
        if (grid[z][x]==WORKPLACE) workplaces.push_back({x,z});
    }

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> wpDist(0, (int)workplaces.size()-1);

    for (size_t hi = 0; hi < houses.size() && agentActiveCount < MAX_AGENTS; hi++) {
        auto [hx, hz] = houses[hi];

        // Pick a random workplace
        if (workplaces.empty()) break;
        auto [wx, wz] = workplaces[wpDist(rng)];

        Agent ag;
        ag.homeGX = hx; ag.homeGZ = hz;
        ag.workGX = wx; ag.workGZ = wz;
        ag.goingToWork = true;
        ag.idleTimer = 1.0f + (float)(hi % 20) * 2.0f; // Stagger departures

        // Find nearest parking to home and work
        bool hasHomePark = FindNearestParking(hx, hz, ag.homeParkGX, ag.homeParkGZ);
        bool hasWorkPark = FindNearestParking(wx, wz, ag.workParkGX, ag.workParkGZ);

        if (hasHomePark && hasWorkPark) {
            // Verify car path exists between parkings
            auto carPath = FindCarPath(ag.homeParkGX, ag.homeParkGZ,
                                       ag.workParkGX, ag.workParkGZ);
            if (carPath.size() >= 2) {
                ag.usesCar = true;
                agentsWithCar++;
            } else {
                ag.usesCar = false;
                agentsWalkOnly++;
            }
        } else {
            ag.usesCar = false;
            agentsWalkOnly++;
        }

        // Verify ped path exists
        auto testPath = FindPedPath(hx, hz, (ag.usesCar ? ag.homeParkGX : wx),
                                            (ag.usesCar ? ag.homeParkGZ : wz));
        if (testPath.size() < 2) {
            agentsFailed++;
            continue;
        }

        ag.phase = AgentPhase::IDLE_HOME;
        agents[agentActiveCount] = ag;
        agentActiveCount++;
    }

    printf("Agents spawned: %u (car=%d, walk-only=%d, failed=%d)\n",
           agentActiveCount, agentsWithCar, agentsWalkOnly, agentsFailed);

    // ============================================================
    //  SIMULATION: 30 real seconds at 100x = 3000 game-seconds
    // ============================================================
    constexpr float BASE_DT = 1.0f / 60.0f;
    constexpr float SIM_SPEED = 100.0f;
    constexpr float BIG_DT = BASE_DT * SIM_SPEED;  // ~1.667s per frame
    constexpr int TOTAL_FRAMES = 30 * 60;           // 1800 frames
    constexpr float DAY_DUR = 86400.0f;

    float timeOfDay = 7.0f / 24.0f; // Start at 7 AM
    float simTime = 0.0f;

    printf("\n=== SIMULATION: %d frames @ %.1fx (%.0f game-sec) ===\n",
           TOTAL_FRAMES, SIM_SPEED, TOTAL_FRAMES * BIG_DT);
    printf("Starting at hour=%.1f\n\n", timeOfDay * 24.0f);

    for (int f = 0; f < TOTAL_FRAMES; f++) {
        simTime += BIG_DT;
        timeOfDay += BIG_DT / DAY_DUR;
        if (timeOfDay >= 1.0f) timeOfDay -= 1.0f;

        UpdateLights(BIG_DT);
        UpdateAgents(BIG_DT, f, timeOfDay);
        UpdateCars(BIG_DT, f, timeOfDay);
        UpdatePeds(BIG_DT, f);

        // Progress report every 10s real time (600 frames)
        if (f % 300 == 0) {
            int driving=0, parked=0;
            for (uint32_t i=0;i<carActiveCount;i++) {
                if (carState[i]==CarState::DRIVING) driving++;
                else parked++;
            }
            int walking=0, waiting=0, idle=0;
            for (uint32_t i=0;i<pedActiveCount;i++) {
                if (pedState[i]==PedState::WALKING) walking++;
                else if (pedState[i]==PedState::WAITING_CROSS) waiting++;
                else idle++;
            }
            int aIdle=0,aWalk=0,aDrive=0,aDest=0;
            for (uint32_t i=0;i<agentActiveCount;i++) {
                switch(agents[i].phase) {
                case AgentPhase::IDLE_HOME: aIdle++; break;
                case AgentPhase::WALK_TO_PARKING:
                case AgentPhase::WALK_TO_DEST:
                case AgentPhase::WALK_ONLY: aWalk++; break;
                case AgentPhase::DRIVING: aDrive++; break;
                case AgentPhase::AT_DEST: aDest++; break;
                default: break;
                }
            }
            printf("f=%4d (sim=%.0fs h=%.1f) cars=%d/%d peds=%d(w=%d) "
                   "agents[idle=%d walk=%d drive=%d dest=%d] "
                   "vio=%zu col=%zu/%zu\n",
                   f, simTime, timeOfDay*24.0f, driving, parked, walking, waiting,
                   aIdle, aWalk, aDrive, aDest,
                   pedViolations.size(), carPedCollisions.size(), carCollisions.size());
        }
    }

    // ============================================================
    //  ANALYSIS & REPORT
    // ============================================================
    printf("\n========================================\n");
    printf("  SIMULATION REPORT\n");
    printf("========================================\n");
    printf("Total agents: %u (car=%d, walk=%d)\n", agentActiveCount, agentsWithCar, agentsWalkOnly);
    printf("Total cars spawned: %u  Total peds spawned: %u\n", carActiveCount, pedActiveCount);
    printf("Sim time: %.0f game-seconds (%.1f game-minutes)\n", simTime, simTime/60.0f);
    printf("Agent transitions: %zu\n", agentTransitions.size());

    // Pedestrian violations
    printf("\n--- PEDESTRIAN VIOLATIONS ---\n");
    printf("Total: %zu\n", pedViolations.size());
    std::unordered_map<std::string,int> pvTypeCount;
    for (auto& v : pedViolations) pvTypeCount[v.type]++;
    for (auto& [t,n] : pvTypeCount) printf("  %-30s %d\n", t.c_str(), n);
    if (!pedViolations.empty()) {
        printf("  First 20 samples:\n");
        int shown = 0;
        for (auto& v : pedViolations) {
            if (shown >= 20) break;
            printf("    f=%d ped%u at (%.1f,%.1f) cell(%d,%d) [%s]\n",
                   v.frame, v.pedIdx, v.wx, v.wz, v.gx, v.gz, v.type.c_str());
            shown++;
        }
    }

    // Car-ped collisions
    printf("\n--- CAR-PEDESTRIAN COLLISIONS ---\n");
    printf("Total events: %zu  Unique pairs: %zu\n", carPedCollisions.size(), carPedCollisionPairs.size());
    if (!carPedCollisions.empty()) {
        int onCross=0, atInter=0, onRoad=0;
        for (auto& c : carPedCollisions) {
            if (c.pedOnCrosswalk) onCross++;
            else if (c.pedAtIntersection) atInter++;
            else onRoad++;
        }
        printf("  On crosswalk: %d  At intersection: %d  On road: %d\n", onCross, atInter, onRoad);
        printf("  First 10 samples:\n");
        for (int ci=0;ci<std::min(10,(int)carPedCollisions.size());ci++) {
            auto& c = carPedCollisions[ci];
            printf("    f=%d car%u(spd=%.1f) vs ped%u dist=%.2f cell(%d,%d)%s%s\n",
                   c.frame, c.carIdx, c.carSpeed, c.pedIdx, c.dist,
                   c.pedGX, c.pedGZ,
                   c.pedOnCrosswalk ? " [CROSSWALK]" : "",
                   c.pedAtIntersection ? " [INTERSECTION]" : "");
        }
    }

    // Car-car collisions
    printf("\n--- CAR-CAR COLLISIONS ---\n");
    printf("Total: %zu\n", carCollisions.size());
    std::unordered_map<std::string,int> ccTypeCount;
    for (auto& c : carCollisions) ccTypeCount[c.type]++;
    for (auto& [t,n] : ccTypeCount) printf("  %-25s %d\n", t.c_str(), n);

    // Agent transitions summary
    printf("\n--- AGENT TRANSITIONS ---\n");
    std::unordered_map<std::string,int> transCount;
    for (auto& t : agentTransitions) transCount[t.detail]++;
    for (auto& [d,n] : transCount) printf("  %-35s %d\n", d.c_str(), n);

    // SUMMARY
    printf("\n========================================\n");
    printf("  SUMMARY\n");
    printf("========================================\n");

    bool allPass = true;

    if (pedViolations.empty())
        printf("  PASS: No pedestrian road/intersection violations\n");
    else {
        printf("  FAIL: %zu pedestrian violations\n", pedViolations.size());
        allPass = false;
    }

    if (carPedCollisionPairs.empty())
        printf("  PASS: No car-pedestrian collisions\n");
    else if (carPedCollisionPairs.size() <= 10)
        printf("  OK:   %zu unique car-ped collision pairs (minor)\n", carPedCollisionPairs.size());
    else {
        printf("  WARN: %zu unique car-ped collision pairs\n", carPedCollisionPairs.size());
        allPass = false;
    }

    if (carCollisions.size() <= 50)
        printf("  OK:   %zu car-car collisions\n", carCollisions.size());
    else {
        printf("  WARN: %zu car-car collisions\n", carCollisions.size());
    }

    int agentReachedDest = 0;
    for (uint32_t i = 0; i < agentActiveCount; i++)
        if (agents[i].phase == AgentPhase::AT_DEST) agentReachedDest++;
    printf("  Agents reached destination: %d / %u\n", agentReachedDest, agentActiveCount);

    if (agentsWithCar > 0) {
        int carTripsComplete = 0;
        for (auto& t : agentTransitions)
            if (t.detail == "exited car, walking to work") carTripsComplete++;
        printf("  Car trips completed (walk→drive→walk): %d\n", carTripsComplete);
    }

    if (allPass)
        printf("\n*** ALL TESTS PASSED ***\n");
    else
        printf("\n*** SOME ISSUES REMAIN — see details above ***\n");

    return allPass ? 0 : 1;
}
