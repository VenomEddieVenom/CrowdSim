// ================================================================
//  Standalone pedestrian + car simulation
//  Tests: sidewalk-only pathing, crosswalk crossing, ped/car lights,
//         cars stopping for pedestrians, collision detection.
//  Modeled after test_sim.cpp — no WickedEngine dependency.
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
static constexpr float SIDEWALK_INNER = HCS - SIDEWALK_W;  // 8.0 m from center
static constexpr float SIDEWALK_OUTER = HCS;               // 10.0 m from center

// ============================================================
//  Car constants (exact copies from CarSystem / test_sim)
// ============================================================
static constexpr uint32_t MAX_CARS  = 2000;
static constexpr float MAX_SPEED    = 14.0f;
static constexpr float ACCEL        =  6.0f;
static constexpr float DECEL        = 14.0f;
static constexpr float CAR_HL       =  1.00f;
static constexpr float CAR_HW       =  0.50f;
static constexpr float MIN_SEP      =  CAR_HL * 2.f + 0.5f;
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
static constexpr float PED_SPEED_MIN   = 1.2f;   // m/s
static constexpr float PED_SPEED_MAX   = 1.8f;   // m/s
static constexpr float PED_RADIUS      = 0.25f;   // collision radius
static constexpr float PED_MIN_SEP     = 0.6f;    // min separation
static constexpr float PED_AVOID_DIST  = 2.0f;    // avoidance lookahead
static constexpr uint16_t PED_MAX_WP   = 256;     // max waypoints per ped

// ============================================================
//  Cell types + grid
// ============================================================
enum CellType : uint8_t { EMPTY=0, ROAD=1, HOUSE=2, WORKPLACE=3, CROSSWALK=4, PARKING=5 };
static CellType grid[GS][GS];
static int roadLanes[GS][GS];
static bool isIntersection_[GS][GS];
static bool isCrosswalk_[GS][GS];

// The same big_city map from test_sim.cpp
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
    // 6-lane boulevards
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

// Place crosswalks on road cells adjacent to intersections (same logic as main.cpp)
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
    // Mark crosswalk lookup
    memset(isCrosswalk_, 0, sizeof(isCrosswalk_));
    for (int z = 0; z < GS; z++)
    for (int x = 0; x < GS; x++)
        isCrosswalk_[z][x] = (grid[z][x] == CROSSWALK);
}

// ============================================================
//  8-phase traffic lights (same as test_sim / TrafficLightSystem)
// ============================================================
static constexpr float THROUGH_DUR = 8.0f;
static constexpr float YELLOW_DUR  = 2.0f;
static constexpr float LEFT_DUR    = 4.0f;
static constexpr float ALL_RED_DUR = 2.0f;
static constexpr float CYCLE_DUR   = (THROUGH_DUR + YELLOW_DUR + LEFT_DUR + ALL_RED_DUR) * 2.0f;

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
//  Pedestrian crossing lights
//  Crosswalks inherit phase from adjacent intersection.
//  A crosswalk cell sits between intersection and road.
//  crossAxis: 0=NS (crossing NS, i.e. ped walks EW), 1=EW (crossing EW, i.e. ped walks NS)
//
//  Pedestrians can cross when the perpendicular traffic direction has its phases.
//  For a crosswalk on the N/S arm of an intersection (crossAxis=NS):
//    ped walks EW across the road → safe when NS cars have green (EW cars stopped)
//    Wait, that's wrong. Let me think...
//
//  crossAxis = axis of the road the crosswalk is ON.
//  If crosswalk is on a NS road segment (road goes N-S), the crossing direction is EW.
//  Pedestrians cross EW. They need EW traffic to be stopped → NS phases active.
//  Actually: CanPedestrianCross from TrafficLightSystem.h says:
//    crossAxis=NS → allowed when EW phases active (EW_THROUGH, EW_YELLOW, EW_LEFT, ALL_RED)
//    crossAxis=EW → allowed when NS phases active (NS_THROUGH, NS_YELLOW, NS_LEFT, ALL_RED)
//  Wait, that doesn't make sense to me. Let me re-read:
//    "crossAxis == NS" means the CROSS axis is NS, so ped is crossing in NS direction.
//    When ped crosses NS, they cross perpendicular to EW traffic.
//    They're safe when EW traffic is stopped → during NS phases!
//    But the code says allowed when phase = EW_THROUGH etc. Let me just copy the logic.
// ============================================================

enum CrossAxis : uint8_t { AX_NS, AX_EW };

// Determines the "cross axis" for a crosswalk cell:
// which direction pedestrians walk across it.
// If the road is NS (has road neighbors to N and/or S), crosswalk is on NS road,
// pedestrians cross EW → crossAxis = EW.
static CrossAxis GetCrosswalkAxis(int gx, int gz)
{
    bool hasNS = (IsRoadLike(gx, gz-1) || IsRoadLike(gx, gz+1) ||
                  (InBounds(gx,gz-1) && isIntersection_[gz-1][gx]) ||
                  (InBounds(gx,gz+1) && isIntersection_[gz+1][gx]));
    bool hasEW = (IsRoadLike(gx-1, gz) || IsRoadLike(gx+1, gz) ||
                  (InBounds(gx-1,gz) && isIntersection_[gz][gx-1]) ||
                  (InBounds(gx+1,gz) && isIntersection_[gz][gx+1]));
    // If road runs NS, ped crosses EW
    if (hasNS && !hasEW) return AX_EW;
    if (hasEW && !hasNS) return AX_NS;
    return AX_EW; // default
}

// Can a pedestrian cross this crosswalk right now?
// Uses matching logic from TrafficLightSystem::CanPedestrianCross
static bool CanPedCross(int gx, int gz)
{
    if (!InBounds(gx, gz)) return true;
    if (!isCrosswalk_[gz][gx]) return true;

    // Find the adjacent intersection to get its phase
    int intGX = -1, intGZ = -1;
    for (int d = 0; d < 4; d++) {
        int nx = gx + DDX[d], nz = gz + DDZ[d];
        if (InBounds(nx, nz) && isIntersection_[nz][nx]) {
            intGX = nx; intGZ = nz; break;
        }
    }
    if (intGX < 0) return true; // no adjacent intersection

    Phase ph = intPhase[intGZ][intGX];
    CrossAxis ax = GetCrosswalkAxis(gx, gz);

    if (ax == AX_NS)
        return (ph == PH_EW_THROUGH || ph == PH_EW_YELLOW ||
                ph == PH_EW_LEFT    || ph == PH_ALL_RED_1 || ph == PH_ALL_RED_2);
    else
        return (ph == PH_NS_THROUGH || ph == PH_NS_YELLOW ||
                ph == PH_NS_LEFT    || ph == PH_ALL_RED_1 || ph == PH_ALL_RED_2);
}

// Do cars need to stop at this crosswalk? (inverse of CanPedCross for the car's direction)
static bool CarMustStopAtCrosswalk(int gx, int gz, float carDX, float carDZ)
{
    if (!InBounds(gx, gz)) return false;
    if (!isCrosswalk_[gz][gx]) return false;
    // If pedestrians CAN cross, cars must stop
    return CanPedCross(gx, gz);
}

// ============================================================
//  Pedestrian sidewalk graph + pathfinding
//
//  Sidewalk nodes: for each road/crosswalk cell, there are 4 sidewalk
//  "edges" (N,S,E,W side). We define sidewalk nodes as the midpoints
//  of sidewalk segments along each side of each road cell.
//
//  Simplified approach: we treat sidewalk-walkable positions as:
//  - Along the edge of each road cell at ±SIDEWALK_MID from center
//  - Pedestrians can walk along a road's sidewalk (same road, same side)
//  - At intersections, pedestrians can cross on the crosswalk if ped light is green
//
//  For pathfinding, we use a simpler grid-based approach:
//  Each road/crosswalk cell has 4 sidewalk zones (N/S/E/W side).
//  Adjacency: same-cell opposite sides connected, neighboring cells same-side connected.
// ============================================================

// Sidewalk node ID: 4 nodes per road/crosswalk cell
// nodeID = cellIndex * 4 + side (0=N, 1=S, 2=E, 3=W)
static constexpr int SIDES = 4;
static constexpr int SIDE_N = 0, SIDE_S = 1, SIDE_E = 2, SIDE_W = 3;
static constexpr float SIDEWALK_MID = SIDEWALK_INNER + SIDEWALK_W * 0.5f; // 9.0 m from center

// World position of a sidewalk node
static Vec2 SidewalkNodePos(int gx, int gz, int side)
{
    Vec2 cc = CellCenter(gx, gz);
    switch (side) {
    case SIDE_N: return { cc.x, cc.y - SIDEWALK_MID }; // N side (negative Z)
    case SIDE_S: return { cc.x, cc.y + SIDEWALK_MID }; // S side (positive Z)
    case SIDE_E: return { cc.x + SIDEWALK_MID, cc.y }; // E side
    case SIDE_W: return { cc.x - SIDEWALK_MID, cc.y }; // W side
    }
    return cc;
}

// Build a pedestrian path from a house cell to a target cell.
// Path uses sidewalk positions (not cell centers).
// Returns a series of world-space waypoints along sidewalks.
static std::vector<Vec2> FindPedPath(int srcGX, int srcGZ, int dstGX, int dstGZ)
{
    if (!InBounds(srcGX, srcGZ) || !InBounds(dstGX, dstGZ)) return {};

    // First find road cells adjacent to src and dst
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

    // Dijkstra on road cells (same as car pathfinding, but we'll convert to sidewalk positions)
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
        // Same cell: just return the sidewalk position
        Vec2 cc = CellCenter(srcRX, srcRZ);
        return { {cc.x + SIDEWALK_MID, cc.y} };
    }

    // Convert cell path to sidewalk waypoints.
    // For each cell, determine which side the sidewalk is on based on direction of travel.
    // Pedestrians walk on the RIGHT side of the road (like cars drive on the right).
    std::vector<Vec2> waypoints;

    for (size_t i = 0; i < cellPath.size(); i++) {
        int cx = cellPath[i] % GS, cz = cellPath[i] / GS;
        Vec2 cc = CellCenter(cx, cz);

        if (i == 0) {
            // First cell: determine walking direction
            int nx = cellPath[1] % GS, nz = cellPath[1] / GS;
            int ddx = nx - cx, ddz = nz - cz;
            // Walk right side: perpendicular right of travel direction
            // If going N (ddz=-1): right side = E (+x)
            // If going S (ddz=+1): right side = W (-x)
            // If going E (ddx=+1): right side = S (+z)
            // If going W (ddx=-1): right side = N (-z)
            float offX = 0, offZ = 0;
            if (ddx == 0) offX = (ddz < 0) ? SIDEWALK_MID : -SIDEWALK_MID;
            if (ddz == 0) offZ = (ddx > 0) ? SIDEWALK_MID : -SIDEWALK_MID;
            waypoints.push_back({cc.x + offX, cc.y + offZ});
        }
        else {
            int px = cellPath[i-1] % GS, pz = cellPath[i-1] / GS;
            int ddx = cx - px, ddz = cz - pz;

            // Determine right-side offset
            float offX = 0, offZ = 0;
            if (ddx == 0) offX = (ddz < 0) ? SIDEWALK_MID : -SIDEWALK_MID;
            if (ddz == 0) offZ = (ddx > 0) ? SIDEWALK_MID : -SIDEWALK_MID;

            // At direction changes, add a corner waypoint
            if (i + 1 < cellPath.size()) {
                int nnx = cellPath[i+1] % GS, nnz = cellPath[i+1] / GS;
                int nddx = nnx - cx, nddz = nnz - cz;
                if (nddx != ddx || nddz != ddz) {
                    // Direction change: add current position first
                    waypoints.push_back({cc.x + offX, cc.y + offZ});
                    // Then compute new offset for next direction
                    float offX2 = 0, offZ2 = 0;
                    if (nddx == 0) offX2 = (nddz < 0) ? SIDEWALK_MID : -SIDEWALK_MID;
                    if (nddz == 0) offZ2 = (nddx > 0) ? SIDEWALK_MID : -SIDEWALK_MID;
                    // Corner point: average of the two offsets (diagonal cut)
                    waypoints.push_back({cc.x + offX2, cc.y + offZ2});
                    continue;
                }
            }

            waypoints.push_back({cc.x + offX, cc.y + offZ});
        }
    }

    // Limit waypoints
    if (waypoints.size() > PED_MAX_WP)
        waypoints.resize(PED_MAX_WP);

    return waypoints;
}

// ============================================================
//  Dijkstra for car path (exact copy from test_sim)
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
//  Car state (SoA) — simplified from test_sim
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

// Per-cell linked lists for cars
static uint32_t carDriveHead[GS * GS];
static uint32_t carDriveNext[MAX_CARS];

// Schedule
static float    carSchedDepart[MAX_CARS];
static float    carSchedReturn[MAX_CARS];
static uint32_t carLastDayTrip[MAX_CARS];

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

static uint32_t SpawnCar(const std::vector<Vec2>& path, uint32_t colorSeed)
{
    if (carActiveCount >= MAX_CARS || path.size() < 2) return UINT32_MAX;
    uint32_t i = carActiveCount++;
    uint8_t wc = (uint8_t)std::min(path.size(), (size_t)MAX_WP);
    for (uint8_t w = 0; w < wc; w++) carWpBuf[i][w] = path[w];
    carWpCount[i] = wc;
    SimplifyPath(carWpBuf[i], carWpCount[i]);
    carWpCurr[i] = 1;
    carDir[i] = 0;

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
    carState[i] = CarState::PARKED;
    carParkTimer[i] = 0.3f + (float)(colorSeed % 8) * 0.6f;

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
static uint8_t  pedDir[MAX_PEDS]; // 0 = going to work, 1 = going home

static uint32_t pedActiveCount = 0;

// Per-cell linked list for pedestrians
static uint32_t pedHead[GS * GS];
static uint32_t pedNext[MAX_PEDS];

static uint32_t SpawnPed(const std::vector<Vec2>& path, uint32_t seed)
{
    if (pedActiveCount >= MAX_PEDS || path.size() < 2) return UINT32_MAX;
    uint32_t i = pedActiveCount++;

    pedWpBuf[i] = path;
    pedWpCurr[i] = 1;
    pedDir[i] = 0;

    // Scatter start position slightly
    float ox = (float)((seed      ) & 0xFF) / 255.0f * 0.5f - 0.25f;
    float oz = (float)((seed >> 8 ) & 0xFF) / 255.0f * 0.5f - 0.25f;
    pedPosX[i] = path[0].x + ox;
    pedPosZ[i] = path[0].y + oz;

    // Speed: 1.2 - 1.8 m/s
    pedSpeed[i] = PED_SPEED_MIN + (float)((seed >> 16) & 0xFF) / 255.0f * (PED_SPEED_MAX - PED_SPEED_MIN);

    Vec2 d = DirTo(path[0], path[1]);
    pedHeading[i] = std::atan2(d.x, d.y);
    pedState[i] = PedState::WALKING;

    return i;
}

// ============================================================
//  Collision / violation tracking
// ============================================================
struct PedViolation {
    uint32_t pedIdx;
    int frame;
    float wx, wz;
    int gx, gz;
    std::string type; // "on_road", "ran_red_light"
};
static std::vector<PedViolation> pedViolations;

struct CarPedCollision {
    uint32_t carIdx, pedIdx;
    int frame;
    float dist;
    float carSpeed;
    float pedX, pedZ; // ped position for debugging
    int pedGX, pedGZ; // ped grid cell
    bool pedOnCrosswalk;
    bool pedAtIntersection;
};
static std::vector<CarPedCollision> carPedCollisions;
static std::unordered_set<uint64_t> carPedCollisionPairs; // unique car-ped pairs

struct CarCollision {
    uint32_t carA, carB;
    int frame;
    float dist;
    std::string type;
};
static std::vector<CarCollision> carCollisions;

// ============================================================
//  Car Update (simplified from test_sim — key parts retained)
// ============================================================
static uint32_t dayCounter = 0;
static float prevTimeOfDay = 0.0f;

static void UpdateCars(float dt_in, int frame, float timeOfDay)
{
    constexpr float MAX_PHYSICS_DT = 0.05f;
    const int subSteps = (dt_in > MAX_PHYSICS_DT) ? (int)std::ceil(dt_in / MAX_PHYSICS_DT) : 1;
    const float dt = dt_in / (float)subSteps;

  for (int sub = 0; sub < subSteps; ++sub) {
    float currentHour = timeOfDay * 24.0f;

    if (sub == 0) {
        if (timeOfDay < prevTimeOfDay) ++dayCounter;
        prevTimeOfDay = timeOfDay;
    }

    // Rebuild linked list
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
        if (carState[i] == CarState::PARKED) {
            if (carParkTimer[i] > 0.0f) { carParkTimer[i] -= dt; continue; }

            bool shouldDepart = false;
            if (carDir[i] == 0) {
                float dep = carSchedDepart[i];
                float diff = currentHour - dep;
                if (diff < 0.0f) diff += 24.0f;
                if (diff >= 0.0f && diff < 0.5f && carLastDayTrip[i] != dayCounter * 2 + 1) {
                    shouldDepart = true;
                    carLastDayTrip[i] = dayCounter * 2 + 1;
                }
            } else {
                float ret = carSchedReturn[i];
                float diff = currentHour - ret;
                if (diff < 0.0f) diff += 24.0f;
                if (diff >= 0.0f && diff < 0.5f && carLastDayTrip[i] != dayCounter * 2 + 2) {
                    shouldDepart = true;
                    carLastDayTrip[i] = dayCounter * 2 + 2;
                }
            }
            if (!shouldDepart) continue;

            // Proximity check
            {
                bool blocked = false;
                int cgx, cgz;
                if (WorldToGrid(carPosX[i], carPosZ[i], cgx, cgz)) {
                    for (int dg = -1; dg <= 1 && !blocked; dg++)
                    for (int dh = -1; dh <= 1 && !blocked; dh++) {
                        int nx2 = cgx+dh, nz2 = cgz+dg;
                        if (!InBounds(nx2, nz2)) continue;
                        int nkey = nz2*GS+nx2;
                        for (uint32_t j = carDriveHead[nkey]; j != UINT32_MAX; j = carDriveNext[j]) {
                            float ddx = carPosX[j]-carPosX[i], ddz = carPosZ[j]-carPosZ[i];
                            if (ddx*ddx+ddz*ddz < MIN_SEP*MIN_SEP)
                            { blocked = true; break; }
                        }
                    }
                }
                if (blocked) { carParkTimer[i] = 0.5f; continue; }
            }

            Vec2* wp = carWpBuf[i];
            uint8_t wc = carWpCount[i];
            for (uint8_t a = 0, b = wc-1; a < b; a++, b--)
                std::swap(wp[a], wp[b]);
            carWpCurr[i] = 1;
            carDir[i] ^= 1;

            if (wc >= 2) {
                Vec2 d = DirTo(wp[0], wp[1]);
                carHeading[i] = std::atan2(d.x, d.y);
            }
            carSpeed[i] = 0.0f;
            carState[i] = CarState::DRIVING;
            {
                int dgx, dgz;
                if (WorldToGrid(carPosX[i], carPosZ[i], dgx, dgz)) {
                    int dkey = dgz*GS+dgx;
                    carDriveNext[i] = carDriveHead[dkey];
                    carDriveHead[dkey] = i;
                }
            }
            continue;
        }

        // ---- DRIVING ----
        Vec2* wp = carWpBuf[i];
        uint8_t wc = carWpCount[i];
        uint8_t& wn = carWpCurr[i];

        if (wn >= wc) {
            for (uint8_t a = 0, b = wc-1; a < b; a++, b--)
                std::swap(wp[a], wp[b]);
            wn = 1;
            carDir[i] ^= 1;
            if (wc >= 2) {
                Vec2 d = DirTo(wp[0], wp[1]);
                carHeading[i] = std::atan2(d.x, d.y);
            }
            carSpeed[i] = std::min(carSpeed[i], 3.0f);
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

        // Lead car detection
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
                    // Check crosswalk stop
                    if (isCrosswalk_[cgz][cgx] && CarMustStopAtCrosswalk(cgx, cgz, dX, dZ)) {
                        // Check if any pedestrians are actually on/near the crosswalk
                        bool pedPresent = false;
                        for (int pd = -1; pd <= 1 && !pedPresent; pd++)
                        for (int ph = -1; ph <= 1 && !pedPresent; ph++) {
                            int pnx = cgx+ph, pnz = cgz+pd;
                            if (!InBounds(pnx, pnz)) continue;
                            int pkey = pnz*GS+pnx;
                            for (uint32_t p = pedHead[pkey]; p != UINT32_MAX; p = pedNext[p]) {
                                // Check if ped is within crosswalk area
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

            // Don't block the box
            if (!inArc && !inIntersection) {
                int px2=myGX, pz2=myGZ;
                for (float d=HCS;d<60.0f;d+=HCS) {
                    float sx=carPosX[i]+segDir.x*d, sz=carPosZ[i]+segDir.y*d;
                    int sgx,sgz;
                    if (!WorldToGrid(sx,sz,sgx,sgz)) break;
                    if (sgx==px2&&sgz==pz2) continue;
                    px2=sgx; pz2=sgz;
                    if (!IsIntersection(sgx,sgz)) continue;
                    int ecx=sgx+(int)std::round(segDir.x);
                    int ecz=sgz+(int)std::round(segDir.y);
                    if (isTurn) { ecx=sgx+(int)std::round(nextSeg.x); ecz=sgz+(int)std::round(nextSeg.y); }
                    if (!InBounds(ecx,ecz)) break;
                    int ekey=ecz*GS+ecx;
                    int jam=0;
                    for (uint32_t j2=carDriveHead[ekey];j2!=UINT32_MAX;j2=carDriveNext[j2]) {
                        if (carSpeed[j2]>=1.0f) continue;
                        float j2Fx=std::sin(carHeading[j2]), j2Fz=std::cos(carHeading[j2]);
                        float dirDot=segDir.x*j2Fx+segDir.y*j2Fz;
                        if (dirDot>0.3f) jam++;
                    }
                    int exitLanes = GetLanes(ecx, ecz);
                    int jamThreshold = exitLanes * 2;
                    if (jam>=jamThreshold) {
                        Vec2 ic=CellCenter(sgx,sgz);
                        float fd=(ic.x-carPosX[i])*segDir.x+(ic.y-carPosZ[i])*segDir.y;
                        if (fd>0) { float sd=std::max(0.0f,fd-HCS-1.0f); if (sd<bestGap) { bestGap=sd; bestSpd=0; } }
                    }
                    break;
                }
            }
        }

        // Pedestrian safety brake: scan ahead for pedestrians in our path
        if (onGrid) {
            float scanAhead = std::min(20.0f, carSpeed[i] * 2.0f + 3.0f);
            float fwdX = std::sin(carHeading[i]);
            float fwdZ = std::cos(carHeading[i]);
            // Check nearby cells for peds
            for (int dg=-1; dg<=1; dg++)
            for (int dh=-1; dh<=1; dh++) {
                int cgx = myGX+dh, cgz = myGZ+dg;
                if (!InBounds(cgx, cgz)) continue;
                int cellKey = cgz*GS+cgx;
                for (uint32_t p = pedHead[cellKey]; p != UINT32_MAX; p = pedNext[p]) {
                    float dx = pedPosX[p] - carPosX[i];
                    float dz = pedPosZ[p] - carPosZ[i];
                    float fwd = dx*fwdX + dz*fwdZ;
                    if (fwd < 1.0f || fwd > scanAhead) continue;
                    float lat = std::abs(dx*fwdZ - dz*fwdX);
                    if (lat > CAR_HW + PED_RADIUS + 0.5f) continue;
                    float stopDist = std::max(0.0f, fwd - CAR_HL - PED_RADIUS - 0.5f);
                    if (stopDist < bestGap) { bestGap = stopDist; bestSpd = 0; }
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
        if (inIntersection && carSpeed[i] < 2.0f && bestGap > 0.0f) carSpeed[i] = std::min(2.0f, desMax);

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

  } // end substep loop
}

// ============================================================
//  Pedestrian Update
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

        // End of path → reverse
        if (wn >= wc) {
            std::reverse(wps.begin(), wps.end());
            wn = 1;
            pedDir[i] ^= 1;
            continue;
        }

        Vec2 target = wps[wn];
        float dx = target.x - pedPosX[i];
        float dz = target.y - pedPosZ[i];
        float distToTarget = std::sqrt(dx*dx + dz*dz);

        // Check if approaching a crosswalk and need to wait
        int myGX, myGZ;
        bool onGrid = WorldToGrid(pedPosX[i], pedPosZ[i], myGX, myGZ);

        if (onGrid) {
            // Check if we're at or approaching a crosswalk
            // Scan the next waypoint's grid cell
            int tgtGX, tgtGZ;
            if (WorldToGrid(target.x, target.y, tgtGX, tgtGZ)) {
                if (isCrosswalk_[tgtGZ][tgtGX] && !CanPedCross(tgtGX, tgtGZ)) {
                    // Wait at crosswalk edge
                    pedState[i] = PedState::WAITING_CROSS;
                    continue;
                }
                // Also check if we're ON a crosswalk
                if (isCrosswalk_[myGZ][myGX] && !CanPedCross(myGX, myGZ)) {
                    // Already on crosswalk but light changed — keep going, don't stop mid-crossing
                }
            }
        }

        pedState[i] = PedState::WALKING;

        // Move toward target
        float spd = pedSpeed[i];

        // Ped-ped avoidance: slow down if pedestrian ahead
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
                        // Push apart
                        if (dd > 0.01f) {
                            float nx = odx/dd, nz = odz/dd;
                            float push = (PED_MIN_SEP - dd) * 0.3f;
                            pedPosX[i] -= nx * push;
                            pedPosZ[i] -= nz * push;
                        }
                        spd *= 0.3f;
                    } else if (dd < PED_AVOID_DIST) {
                        // Check if ahead in direction of travel
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

        // Clamp ped to sidewalk on regular road cells (not intersections/crosswalks)
        // This prevents ped-ped avoidance from pushing peds onto the road
        if (onGrid) {
            CellType ct = grid[myGZ][myGX];
            if (ct == ROAD && !isIntersection_[myGZ][myGX]) {
                Vec2 cc = CellCenter(myGX, myGZ);
                float relX = pedPosX[i] - cc.x;
                float relZ = pedPosZ[i] - cc.y;
                // Determine road direction
                bool hasNS = (InBounds(myGX, myGZ-1) && IsRoadLike(myGX, myGZ-1)) ||
                             (InBounds(myGX, myGZ+1) && IsRoadLike(myGX, myGZ+1));
                bool hasEW = (InBounds(myGX-1, myGZ) && IsRoadLike(myGX-1, myGZ)) ||
                             (InBounds(myGX+1, myGZ) && IsRoadLike(myGX+1, myGZ));
                if (hasNS && !hasEW) {
                    // NS road: clamp X to sidewalk band
                    if (std::abs(relX) < SIDEWALK_INNER) {
                        pedPosX[i] = cc.x + ((relX >= 0) ? SIDEWALK_MID : -SIDEWALK_MID);
                    }
                } else if (hasEW && !hasNS) {
                    // EW road: clamp Z to sidewalk band
                    if (std::abs(relZ) < SIDEWALK_INNER) {
                        pedPosZ[i] = cc.y + ((relZ >= 0) ? SIDEWALK_MID : -SIDEWALK_MID);
                    }
                }
            }
        }

        // ---- Violation check: is pedestrian on the road surface (not sidewalk)? ----
        // Intersections: peds are allowed to walk through (that's where crosswalks are)
        // Crosswalks: ct == CROSSWALK, not ROAD, so already excluded
        // Regular road cells: ped must be on sidewalk (8-10m from center on perpendicular axis)
        if (onGrid) {
            CellType ct = grid[myGZ][myGX];
            if (ct == ROAD && !isIntersection_[myGZ][myGX]) {
                Vec2 cc = CellCenter(myGX, myGZ);
                float relX = std::abs(pedPosX[i] - cc.x);
                float relZ = std::abs(pedPosZ[i] - cc.y);
                // Determine road direction: check neighbors to see if road runs NS or EW
                bool hasNS = (InBounds(myGX, myGZ-1) && IsRoadLike(myGX, myGZ-1)) ||
                             (InBounds(myGX, myGZ+1) && IsRoadLike(myGX, myGZ+1));
                bool hasEW = (InBounds(myGX-1, myGZ) && IsRoadLike(myGX-1, myGZ)) ||
                             (InBounds(myGX+1, myGZ) && IsRoadLike(myGX+1, myGZ));
                bool violation = false;
                if (hasNS && !hasEW) {
                    // NS road: check X axis (perpendicular to road direction)
                    violation = (relX < SIDEWALK_INNER - 0.5f);
                } else if (hasEW && !hasNS) {
                    // EW road: check Z axis (perpendicular to road direction)
                    violation = (relZ < SIDEWALK_INNER - 0.5f);
                } else {
                    // Both or neither: check both axes (conservative)
                    violation = (relX < SIDEWALK_INNER - 0.5f && relZ < SIDEWALK_INNER - 0.5f);
                }
                if (violation) {
                    pedViolations.push_back({i, frame, pedPosX[i], pedPosZ[i], myGX, myGZ, "on_road"});
                }
            }
        }
    }

    // ---- Car-ped collision check ----
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
                float ddx=carPosX[c]-pedPosX[p], ddz=carPosZ[c]-pedPosZ[p];
                float dd=std::sqrt(ddx*ddx+ddz*ddz);
                // Only count as collision if car is actually moving
                if (dd < CAR_HL + PED_RADIUS && carSpeed[c] > 0.5f) {
                    uint64_t pairKey = ((uint64_t)c << 32) | (uint64_t)p;
                    carPedCollisionPairs.insert(pairKey);
                    int cpgx, cpgz;
                    WorldToGrid(pedPosX[p], pedPosZ[p], cpgx, cpgz);
                    carPedCollisions.push_back({c, p, frame, dd, carSpeed[c],
                        pedPosX[p], pedPosZ[p], cpgx, cpgz,
                        InBounds(cpgx,cpgz) && isCrosswalk_[cpgz][cpgx],
                        InBounds(cpgx,cpgz) && isIntersection_[cpgz][cpgx]});
                }
            }
        }
    }
}

// ============================================================
//  MAIN
// ============================================================
int main()
{
    printf("=== Pedestrian + Car Simulation — BIG CITY Map ===\n");

    BuildGrid();
    DetectIntersections();
    PlaceCrosswalks();
    // Re-detect intersections after crosswalk placement
    DetectIntersections();
    InitLights();

    // Count cell types
    int roadCount=0, houseCount=0, wpCount2=0, intCount=0, crossCount=0;
    for (int z=0;z<GS;z++) for (int x=0;x<GS;x++) {
        if (IsRoadLike(x,z)) roadCount++;
        if (grid[z][x]==HOUSE) houseCount++;
        if (grid[z][x]==WORKPLACE) wpCount2++;
        if (isIntersection_[z][x]) intCount++;
        if (isCrosswalk_[z][x]) crossCount++;
    }
    printf("Grid: %dx%d  Roads=%d Houses=%d Workplaces=%d Intersections=%d Crosswalks=%d\n",
           GS, GS, roadCount, houseCount, wpCount2, intCount, crossCount);

    // Spawn cars (same as test_sim)
    int housesSpawnedCar = 0;
    for (int hz=0;hz<GS;hz++)
    for (int hx=0;hx<GS;hx++) {
        if (grid[hz][hx] != HOUSE) continue;
        bool found = false;
        for (int wz=0;wz<GS && !found;wz++)
        for (int wx=0;wx<GS && !found;wx++) {
            if (grid[wz][wx] != WORKPLACE) continue;
            auto path = FindCarPath(hx,hz,wx,wz);
            if (path.size() < 2) continue;
            int toSpawn = ((int)path.size() > 3) ? 3 : 5;
            uint32_t seed = (uint32_t)(hx*37 + hz*19 + wx*7 + wz*3);
            int ok = 0;
            for (int c = 0; c < toSpawn && carActiveCount < MAX_CARS; c++)
                if (SpawnCar(path, seed + (uint32_t)c) != UINT32_MAX) ok++;
            if (ok == 0) continue;
            found = true;
            housesSpawnedCar++;
        }
    }
    printf("Car houses spawned: %d  Total cars: %u\n", housesSpawnedCar, carActiveCount);

    // Spawn pedestrians (house → workplace, walking on sidewalks)
    int housesSpawnedPed = 0;
    for (int hz=0;hz<GS;hz++)
    for (int hx=0;hx<GS;hx++) {
        if (grid[hz][hx] != HOUSE) continue;
        bool found = false;
        for (int wz=0;wz<GS && !found;wz++)
        for (int wx=0;wx<GS && !found;wx++) {
            if (grid[wz][wx] != WORKPLACE) continue;
            auto path = FindPedPath(hx,hz,wx,wz);
            if (path.size() < 2) continue;
            uint32_t seed = (uint32_t)(hx*53 + hz*31 + wx*11 + wz*7);
            // Spawn 2 pedestrians per house-workplace pair
            int ok = 0;
            for (int p = 0; p < 2 && pedActiveCount < MAX_PEDS; p++)
                if (SpawnPed(path, seed + (uint32_t)p) != UINT32_MAX) ok++;
            if (ok == 0) continue;
            found = true;
            housesSpawnedPed++;
        }
    }
    printf("Ped houses spawned: %d  Total pedestrians: %u\n", housesSpawnedPed, pedActiveCount);

    // Set all cars to depart at 7:00 AM
    for (uint32_t i = 0; i < carActiveCount; i++) {
        carState[i] = CarState::PARKED;
        carParkTimer[i] = 0.0f;
        carSchedDepart[i] = 7.0f;
    }

    constexpr float DT = 1.0f / 60.0f;

    // Initialize pedHead spatial hash so cars can safely scan for peds on first frame
    std::fill(pedHead, pedHead + GS*GS, UINT32_MAX);
    std::fill(pedNext, pedNext + MAX_PEDS, UINT32_MAX);
    constexpr float DAY_DUR = 86400.0f;

    // ---- WARM-UP: 30s at 1x ----
    constexpr int WARMUP_FRAMES = 30 * 60;
    float timeOfDay = 7.0f / 24.0f;

    printf("\n=== WARM-UP: %d frames (%.0f seconds at 1x) from hour %.1f ===\n",
           WARMUP_FRAMES, WARMUP_FRAMES * DT, timeOfDay * 24.0f);

    for (int f = 0; f < WARMUP_FRAMES; f++) {
        timeOfDay += DT / DAY_DUR;
        if (timeOfDay >= 1.0f) timeOfDay -= 1.0f;
        UpdateLights(DT);
        UpdateCars(DT, f, timeOfDay);
        UpdatePeds(DT, f);
        if (f % 600 == 0) {
            int dr=0; for (uint32_t i=0;i<carActiveCount;i++) if (carState[i]==CarState::DRIVING) dr++;
            int pw=0,wt=0; for (uint32_t i=0;i<pedActiveCount;i++) {
                if (pedState[i]==PedState::WALKING) pw++;
                if (pedState[i]==PedState::WAITING_CROSS) wt++;
            }
            printf("  Warmup f=%d h=%.2f cars_driving=%d peds_walking=%d peds_waiting=%d\n",
                   f, timeOfDay*24.0f, dr, pw, wt);
        }
    }
    int warmupCarCols = (int)carCollisions.size();
    int warmupPedVio = (int)pedViolations.size();
    int warmupCarPedCols = (int)carPedCollisions.size();
    printf("After warm-up: carCols=%d pedViolations=%d carPedCols=%d\n",
           warmupCarCols, warmupPedVio, warmupCarPedCols);

    carCollisions.clear();
    pedViolations.clear();
    carPedCollisions.clear();

    // ---- MAIN TEST: 10x for 60 real seconds ----
    constexpr float BIG_DT = DT * 10.0f;
    constexpr int TEST_FRAMES = 60 * 60;
    float simTime = 0.0f;

    printf("\n=== TEST: 10x for 60 real seconds (%d frames, dt=%.3fs) ===\n",
           TEST_FRAMES, BIG_DT);
    printf("Starting at hour=%.2f with %u cars, %u pedestrians\n",
           timeOfDay*24.0f, carActiveCount, pedActiveCount);

    for (int f = 0; f < TEST_FRAMES; f++) {
        simTime += BIG_DT;
        timeOfDay += BIG_DT / DAY_DUR;
        if (timeOfDay >= 1.0f) timeOfDay -= 1.0f;
        UpdateLights(BIG_DT);
        UpdateCars(BIG_DT, f, timeOfDay);
        UpdatePeds(BIG_DT, f);

        if (f % 600 == 0) {
            int dr=0,cstp=0;
            for (uint32_t i=0;i<carActiveCount;i++) {
                if (carState[i]==CarState::DRIVING) { dr++; if (carSpeed[i]<0.5f) cstp++; }
            }
            int pw=0,wt=0;
            for (uint32_t i=0;i<pedActiveCount;i++) {
                if (pedState[i]==PedState::WALKING) pw++;
                if (pedState[i]==PedState::WAITING_CROSS) wt++;
            }
            printf("  f=%d (sim %.0fs h=%.2f) cars=%d(stop=%d) peds=%d(wait=%d) "
                   "carCols=%zu pedVio=%zu carPed=%zu\n",
                   f, simTime, timeOfDay*24.0f, dr, cstp, pw, wt,
                   carCollisions.size(), pedViolations.size(), carPedCollisions.size());
        }
    }

    // ============================================================
    //  ANALYSIS
    // ============================================================
    printf("\n========================================\n");
    printf("  SIMULATION REPORT\n");
    printf("========================================\n");
    printf("Total cars: %u  Total pedestrians: %u\n", carActiveCount, pedActiveCount);
    printf("Sim time: %.0f seconds at 10x\n", simTime);

    // Car-car collisions
    printf("\n--- CAR-CAR COLLISIONS ---\n");
    printf("Total: %zu\n", carCollisions.size());
    std::unordered_map<std::string,int> ccTypeCount;
    for (auto& c : carCollisions) ccTypeCount[c.type]++;
    for (auto& [t,n] : ccTypeCount) printf("  %-25s %d\n", t.c_str(), n);

    // Car-ped collisions
    printf("\n--- CAR-PEDESTRIAN COLLISIONS ---\n");
    printf("Total events: %zu  Unique car-ped pairs: %zu\n", carPedCollisions.size(), carPedCollisionPairs.size());
    if (!carPedCollisions.empty()) {
        int highSpeed = 0, onCrosswalk = 0, atIntersection = 0, onRoad = 0;
        for (auto& c : carPedCollisions) {
            if (c.carSpeed > 5.0f) highSpeed++;
            if (c.pedOnCrosswalk) onCrosswalk++;
            else if (c.pedAtIntersection) atIntersection++;
            else onRoad++;
        }
        printf("  High-speed (>5 m/s): %d\n", highSpeed);
        printf("  Ped on crosswalk: %d  Ped at intersection: %d  Ped on road: %d\n", onCrosswalk, atIntersection, onRoad);
        printf("  First 20 samples:\n");
        for (int ci=0;ci<std::min(20,(int)carPedCollisions.size());ci++) {
            auto& c = carPedCollisions[ci];
            printf("    f=%d car%u(spd=%.1f) vs ped%u dist=%.2f cell(%d,%d)%s%s\n",
                   c.frame, c.carIdx, c.carSpeed, c.pedIdx, c.dist,
                   c.pedGX, c.pedGZ,
                   c.pedOnCrosswalk ? " [CROSSWALK]" : "",
                   c.pedAtIntersection ? " [INTERSECTION]" : "");
        }
    }

    // Pedestrian violations
    printf("\n--- PEDESTRIAN VIOLATIONS ---\n");
    printf("Total: %zu\n", pedViolations.size());
    std::unordered_map<std::string,int> pvTypeCount;
    for (auto& v : pedViolations) pvTypeCount[v.type]++;
    for (auto& [t,n] : pvTypeCount) printf("  %-25s %d\n", t.c_str(), n);
    if (!pedViolations.empty()) {
        printf("  First 20 samples:\n");
        for (int vi=0;vi<std::min(20,(int)pedViolations.size());vi++) {
            auto& v = pedViolations[vi];
            printf("    f=%d ped%u at (%.1f,%.1f) cell(%d,%d) [%s]\n",
                   v.frame, v.pedIdx, v.wx, v.wz, v.gx, v.gz, v.type.c_str());
        }
    }

    // Crosswalk usage stats
    printf("\n--- CROSSWALK STATS ---\n");
    int pedsWaiting = 0, pedsWalking = 0;
    for (uint32_t i = 0; i < pedActiveCount; i++) {
        if (pedState[i] == PedState::WAITING_CROSS) pedsWaiting++;
        if (pedState[i] == PedState::WALKING) pedsWalking++;
    }
    printf("Final: %d walking, %d waiting at crosswalks\n", pedsWalking, pedsWaiting);

    // Summary
    printf("\n========================================\n");
    printf("  SUMMARY\n");
    printf("========================================\n");
    bool pass = true;
    if (pedViolations.size() > 0) {
        printf("  FAIL: %zu pedestrian road violations\n", pedViolations.size());
        pass = false;
    } else {
        printf("  PASS: No pedestrian road violations\n");
    }
    if (carPedCollisionPairs.size() > 50) {
        printf("  WARN: %zu unique car-pedestrian collision pairs (%zu events)\n", carPedCollisionPairs.size(), carPedCollisions.size());
    } else if (carPedCollisionPairs.empty()) {
        printf("  PASS: No car-pedestrian collisions\n");
    } else {
        printf("  OK:   %zu unique car-pedestrian collision pairs\n", carPedCollisionPairs.size());
    }
    if (carCollisions.size() > 1000) {
        printf("  WARN: %zu car-car collisions\n", carCollisions.size());
    } else {
        printf("  OK:   %zu car-car collisions\n", carCollisions.size());
    }

    if (pass)
        printf("\n*** ALL PEDESTRIAN TESTS PASSED ***\n");
    else
        printf("\n*** SOME TESTS FAILED — see details above ***\n");

    return 0;
}
