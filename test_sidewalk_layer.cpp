// ================================================================
//  test_sidewalk_layer.cpp — Sidewalk-layer pedestrian simulation
//
//  CHANGES FROM PREVIOUS TEST:
//  1. Sidewalk layer: bool array marking every valid ped position
//  2. No ped-ped collisions
//  3. Peds on crosswalk/road keep walking even on red
//  4. Shorter ped crossing phase (ALL_RED=3s, ped gets 14s per half)
//  5. Crosswalks shifted 1 SIDEWALK_W closer to intersection
//  6. Car stop line at traffic light (behind crosswalk)
//  7. Cars don't double-stop (skip light check on crosswalk cells)
//
//  Run: 30 seconds at 100x = 3000 game-seconds
// ================================================================
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <string>
#include <queue>
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
static constexpr uint16_t PED_MAX_WP   = 256;

// ============================================================
//  Cell types
// ============================================================
enum CellType : uint8_t { EMPTY=0, ROAD=1, HOUSE=2, WORKPLACE=3, CROSSWALK=4, PARKING=5 };
static CellType grid[GS][GS];
static int roadLanes[GS][GS];
static bool isIntersection_[GS][GS];

// ============================================================
//  SIDEWALK LAYER — 4x4 sub-grid per cell (5m resolution)
//  Marks every position where a pedestrian is allowed to be.
//  Sub-cell (sx,sz) in [0..3]: world pos = cellCenter + (sx-1.5)*5, (sz-1.5)*5
//  For CELL_SIZE=20m, sub-cells at offsets: -7.5, -2.5, +2.5, +7.5
//  Sidewalk sub-cells: 0 and 3 (at ±7.5m, within 8-10m sidewalk band)
// ============================================================
static constexpr int SUB = 4;        // sub-cells per cell per axis
static constexpr float SUB_SIZE = CS / SUB;  // 5m
static bool sidewalkLayer[GS * SUB][GS * SUB];  // true = ped allowed

static void BuildSidewalkLayer()
{
    memset(sidewalkLayer, 0, sizeof(sidewalkLayer));

    auto isRoad = [](int gx, int gz) -> bool {
        if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) return false;
        return grid[gz][gx] == ROAD || grid[gz][gx] == CROSSWALK;
    };

    for (int gz = 0; gz < GS; gz++)
    for (int gx = 0; gx < GS; gx++) {
        CellType ct = grid[gz][gx];

        if (ct == ROAD || ct == CROSSWALK) {
            bool hasN = isRoad(gx, gz-1);
            bool hasS = isRoad(gx, gz+1);
            bool hasE = isRoad(gx+1, gz);
            bool hasW = isRoad(gx-1, gz);
            bool isInt = isIntersection_[gz][gx];

            // Base sub-cell coords
            int bx = gx * SUB;
            int bz = gz * SUB;

            if (ct == CROSSWALK) {
                // Entire crosswalk is walkable
                for (int sz = 0; sz < SUB; sz++)
                for (int sx = 0; sx < SUB; sx++)
                    sidewalkLayer[bz + sz][bx + sx] = true;
            }
            else if (isInt) {
                // Intersection: only outer ring is walkable (corners)
                for (int sz = 0; sz < SUB; sz++)
                for (int sx = 0; sx < SUB; sx++) {
                    bool edgeX = (sx == 0 || sx == SUB-1);
                    bool edgeZ = (sz == 0 || sz == SUB-1);
                    if (edgeX || edgeZ)
                        sidewalkLayer[bz + sz][bx + sx] = true;
                }
            }
            else {
                // Regular road: sidewalks on non-connected sides
                // NS road (hasN or hasS, no E/W connection): sidewalks at E(sx=3) and W(sx=0)
                // EW road: sidewalks at N(sz=0) and S(sz=3)
                bool isNS = (hasN || hasS) && !(hasE && hasW);
                bool isEW = (hasE || hasW) && !(hasN && hasS);

                for (int sz = 0; sz < SUB; sz++)
                for (int sx = 0; sx < SUB; sx++) {
                    if (isNS) {
                        // Sidewalks on E(3) and W(0) edges, full length
                        if (sx == 0 || sx == SUB-1)
                            sidewalkLayer[bz + sz][bx + sx] = true;
                    }
                    else if (isEW) {
                        // Sidewalks on N(0) and S(3) edges, full length
                        if (sz == 0 || sz == SUB-1)
                            sidewalkLayer[bz + sz][bx + sx] = true;
                    }
                    else {
                        // Isolated or complex: sidewalks on all non-connected edges
                        if (!hasE && sx == SUB-1) sidewalkLayer[bz + sz][bx + sx] = true;
                        if (!hasW && sx == 0)     sidewalkLayer[bz + sz][bx + sx] = true;
                        if (!hasS && sz == SUB-1) sidewalkLayer[bz + sz][bx + sx] = true;
                        if (!hasN && sz == 0)     sidewalkLayer[bz + sz][bx + sx] = true;
                    }
                }
            }
        }
        else if (ct == HOUSE || ct == WORKPLACE || ct == PARKING) {
            // Buildings: edge sub-cells adjacent to road are walkable
            int bx = gx * SUB;
            int bz = gz * SUB;
            if (isRoad(gx+1, gz)) for (int sz=0;sz<SUB;sz++) sidewalkLayer[bz+sz][bx+SUB-1] = true;
            if (isRoad(gx-1, gz)) for (int sz=0;sz<SUB;sz++) sidewalkLayer[bz+sz][bx+0] = true;
            if (isRoad(gx, gz+1)) for (int sx=0;sx<SUB;sx++) sidewalkLayer[bz+SUB-1][bx+sx] = true;
            if (isRoad(gx, gz-1)) for (int sx=0;sx<SUB;sx++) sidewalkLayer[bz+0][bx+sx] = true;
        }
    }
}

// Check if a world position is on sidewalk layer
static bool IsOnSidewalk(float wx, float wz)
{
    int sx = (int)std::floor((wx + HALF_WORLD) / SUB_SIZE);
    int sz = (int)std::floor((wz + HALF_WORLD) / SUB_SIZE);
    if (sx < 0 || sx >= GS*SUB || sz < 0 || sz >= GS*SUB) return false;
    return sidewalkLayer[sz][sx];
}

// Find nearest sidewalk sub-cell to a world position
static void SnapToSidewalk(float& wx, float& wz)
{
    int sx = (int)std::floor((wx + HALF_WORLD) / SUB_SIZE);
    int sz = (int)std::floor((wz + HALF_WORLD) / SUB_SIZE);
    sx = std::clamp(sx, 0, GS*SUB-1);
    sz = std::clamp(sz, 0, GS*SUB-1);
    if (sidewalkLayer[sz][sx]) return; // already on sidewalk

    // Search nearby sub-cells
    for (int r = 1; r <= 4; r++) {
        float bestD2 = 1e9f;
        int bestSX = sx, bestSZ = sz;
        for (int dz = -r; dz <= r; dz++)
        for (int dx = -r; dx <= r; dx++) {
            if (std::abs(dx) != r && std::abs(dz) != r) continue; // only ring
            int nx = sx + dx, nz = sz + dz;
            if (nx < 0 || nx >= GS*SUB || nz < 0 || nz >= GS*SUB) continue;
            if (!sidewalkLayer[nz][nx]) continue;
            float cx = -HALF_WORLD + (nx + 0.5f) * SUB_SIZE;
            float cz2 = -HALF_WORLD + (nz + 0.5f) * SUB_SIZE;
            float dd = (cx-wx)*(cx-wx)+(cz2-wz)*(cz2-wz);
            if (dd < bestD2) { bestD2 = dd; bestSX = nx; bestSZ = nz; }
        }
        if (bestD2 < 1e8f) {
            wx = -HALF_WORLD + (bestSX + 0.5f) * SUB_SIZE;
            wz = -HALF_WORLD + (bestSZ + 0.5f) * SUB_SIZE;
            return;
        }
    }
}

// Same large city map
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
    // Place crosswalks on the cell adjacent to each intersection arm
    // (same as before — crosswalk is the first straight road cell next to intersection)
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
            if (rn == 2)
                grid[nz][nx] = CROSSWALK;
        }
    }
}

// ============================================================
//  Traffic lights — shorter ped phase
// ============================================================
static constexpr float THROUGH_DUR = 8.0f;
static constexpr float YELLOW_DUR  = 2.0f;
static constexpr float LEFT_DUR    = 4.0f;
static constexpr float ALL_RED_DUR = 3.0f;  // Shorter for peds
static constexpr float CYCLE_DUR   = (THROUGH_DUR + YELLOW_DUR + LEFT_DUR + ALL_RED_DUR) * 2.0f; // 34s

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
//  Ped crossing light
// ============================================================
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

    // Determine crosswalk axis: which way does the road through crosswalk go?
    bool hasNS = IsRoadLike(gx, gz-1) || IsRoadLike(gx, gz+1);
    bool hasEW = IsRoadLike(gx-1, gz) || IsRoadLike(gx+1, gz);
    // Crosswalk on NS road → peds cross EW → need NS traffic stopped
    bool crossAxisNS = !(hasNS && !hasEW); // true if peds cross in NS direction

    if (crossAxisNS)
        return (ph == PH_EW_THROUGH || ph == PH_EW_YELLOW ||
                ph == PH_EW_LEFT    || ph == PH_ALL_RED_1 || ph == PH_ALL_RED_2);
    else
        return (ph == PH_NS_THROUGH || ph == PH_NS_YELLOW ||
                ph == PH_NS_LEFT    || ph == PH_ALL_RED_1 || ph == PH_ALL_RED_2);
}

// ============================================================
//  Pedestrian Pathfinding — uses sidewalk layer for waypoints
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

    // Convert cell path to sidewalk waypoints
    std::vector<Vec2> waypoints;

    auto sidewalkOff = [](int dx, int dz, float& ox, float& oz) {
        ox = oz = 0.f;
        if (dx == 0) ox = (dz < 0) ? SIDEWALK_MID : -SIDEWALK_MID;
        if (dz == 0) oz = (dx > 0) ? SIDEWALK_MID : -SIDEWALK_MID;
    };

    for (size_t i = 0; i < cellPath.size(); i++) {
        int cx = cellPath[i] % GS, cz = cellPath[i] / GS;
        Vec2 cc = CellCenter(cx, cz);

        // Incoming direction
        int pddx = 0, pddz = 0;
        if (i > 0) {
            int px = cellPath[i-1] % GS, pz = cellPath[i-1] / GS;
            pddx = cx - px; pddz = cz - pz;
        } else if (cellPath.size() > 1) {
            int nx2 = cellPath[1] % GS, nz2 = cellPath[1] / GS;
            pddx = nx2 - cx; pddz = nz2 - cz;
        }

        // Outgoing direction
        int nddx = pddx, nddz = pddz;
        if (i + 1 < cellPath.size()) {
            int nx2 = cellPath[i+1] % GS, nz2 = cellPath[i+1] / GS;
            nddx = nx2 - cx; nddz = nz2 - cz;
        }

        bool isTurn = (i > 0 && i + 1 < cellPath.size() && (pddx != nddx || pddz != nddz));

        if (!isTurn) {
            float ox, oz;
            sidewalkOff(pddx, pddz, ox, oz);
            waypoints.push_back({cc.x + ox, cc.y + oz});
            continue;
        }

        // === TURN at intersection cell ===
        int px = cellPath[i-1] % GS, pz = cellPath[i-1] / GS;
        Vec2 prevC = CellCenter(px, pz);

        float pox, poz;
        sidewalkOff(pddx, pddz, pox, poz);
        float nox, noz;
        sidewalkOff(nddx, nddz, nox, noz);

        // Inner edge of approach crosswalk cell (boundary with intersection)
        // Shifted 1 SIDEWALK_W closer: use HCS - SIDEWALK_W instead of HCS
        float edgeDist = HCS - SIDEWALK_W;
        float edgeAlongX = (float)pddx * edgeDist;
        float edgeAlongZ = (float)pddz * edgeDist;

        int turn = pddx * nddz - pddz * nddx;

        if (turn > 0) {
            // RIGHT turn
            waypoints.push_back({prevC.x + pox + edgeAlongX,
                                 prevC.y + poz + edgeAlongZ});
            float crnX = (pox != 0.f) ? pox : nox;
            float crnZ = (poz != 0.f) ? poz : noz;
            waypoints.push_back({cc.x + crnX, cc.y + crnZ});
        } else {
            // LEFT turn: cross at approach crosswalk, walk around
            waypoints.push_back({prevC.x + pox + edgeAlongX,
                                 prevC.y + poz + edgeAlongZ});
            waypoints.push_back({prevC.x - pox + edgeAlongX,
                                 prevC.y - poz + edgeAlongZ});
            float crnX = (pox != 0.f) ? -pox : nox;
            float crnZ = (poz != 0.f) ? -poz : noz;
            waypoints.push_back({cc.x + crnX, cc.y + crnZ});
        }
    }

    // Snap all waypoints to sidewalk layer
    for (auto& wp : waypoints)
        SnapToSidewalk(wp.x, wp.y);

    if (waypoints.size() > PED_MAX_WP)
        waypoints.resize(PED_MAX_WP);

    return waypoints;
}

// ============================================================
//  Car pathfinding
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
    std::vector<float> costV(GS*GS, 1e9f);
    std::vector<int> fromV(GS*GS, -1);
    using PQ = std::priority_queue<std::pair<float,int>,
               std::vector<std::pair<float,int>>,
               std::greater<std::pair<float,int>>>;
    PQ pq;
    costV[srcId] = 0.0f;
    fromV[srcId] = srcId;
    pq.push({0.0f, srcId});
    bool found = (srcId == dstId);
    while (!pq.empty() && !found) {
        auto [cc, cur] = pq.top(); pq.pop();
        if (cc > costV[cur]) continue;
        int cx = cur%GS, cz = cur/GS;
        for (int d = 0; d < 4; d++) {
            int nx = cx+DDX[d], nz = cz+DDZ[d];
            if (!InBounds(nx,nz) || !IsRoadLike(nx,nz)) continue;
            int nid = nz*GS+nx;
            float nc = cc + 1.0f;
            if (nc < costV[nid]) {
                costV[nid] = nc;
                fromV[nid] = cur;
                if (nid == dstId) { found = true; break; }
                pq.push({nc, nid});
            }
        }
    }
    if (!found) return {};

    std::vector<Vec2> road;
    for (int cur = dstId; cur != srcId; cur = fromV[cur])
        road.push_back(CellCenter(cur%GS, cur/GS));
    road.push_back(CellCenter(srcRX, srcRZ));
    std::reverse(road.begin(), road.end());
    return road;
}

// ============================================================
//  Find nearest parking
// ============================================================
static bool FindNearestParking(int fromGX, int fromGZ, int& parkGX, int& parkGZ)
{
    float bestDist2 = 1e9f;
    parkGX = -1; parkGZ = -1;
    for (int z = 0; z < GS; z++)
    for (int x = 0; x < GS; x++) {
        if (grid[z][x] != PARKING) continue;
        float dx = (float)(x - fromGX), dz = (float)(z - fromGZ);
        float d2 = dx*dx + dz*dz;
        if (d2 < bestDist2) {
            bool hasRoad = false;
            for (int d = 0; d < 4; d++)
                if (IsRoadLike(x+DDX[d], z+DDZ[d])) { hasRoad = true; break; }
            if (hasRoad) { bestDist2 = d2; parkGX = x; parkGZ = z; }
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
    carState[i] = CarState::DRIVING;
    carParkTimer[i] = 0.0f;
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

static uint32_t pedActiveCount = 0;
static uint32_t pedHead[GS * GS];
static uint32_t pedNext[MAX_PEDS];

static uint32_t SpawnPed(const std::vector<Vec2>& path, uint32_t seed)
{
    if (pedActiveCount >= MAX_PEDS || path.size() < 2) return UINT32_MAX;
    uint32_t i = pedActiveCount++;
    pedWpBuf[i] = path;
    pedWpCurr[i] = 1;
    pedDir[i] = 0;
    pedPosX[i] = path[0].x;
    pedPosZ[i] = path[0].y;
    pedSpeed[i] = PED_SPEED_MIN + (float)((seed >> 16) & 0xFF) / 255.0f * (PED_SPEED_MAX - PED_SPEED_MIN);
    Vec2 d = DirTo(path[0], path[1]);
    pedHeading[i] = std::atan2(d.x, d.y);
    pedState[i] = PedState::WALKING;
    return i;
}

// ============================================================
//  Agent system
// ============================================================
static constexpr uint32_t MAX_AGENTS = 2000;

enum class AgentPhase : uint8_t {
    IDLE_HOME, WALK_TO_PARKING, DRIVING, WALK_TO_DEST, AT_DEST, WALK_ONLY
};

struct Agent {
    int homeGX, homeGZ, workGX, workGZ;
    int homeParkGX, homeParkGZ, workParkGX, workParkGZ;
    AgentPhase phase = AgentPhase::IDLE_HOME;
    uint32_t pedIdx = UINT32_MAX;
    uint32_t carIdx = UINT32_MAX;
    bool usesCar = false;
    bool goingToWork = true;
    float idleTimer = 0.0f;
};

static Agent agents[MAX_AGENTS];
static uint32_t agentActiveCount = 0;

// ============================================================
//  Violation tracking
// ============================================================
static uint32_t pedOnRoadViolations = 0;
static uint32_t pedIntersectionViolations = 0;
static uint32_t carPedCollisionCount = 0;
static uint32_t carCarCollisionCount = 0;
static uint32_t tripsCompleted = 0;
static uint32_t pedsStuckOnStreet = 0;

// ============================================================
//  Car Update (simplified from full test — focus on ped interaction)
// ============================================================
static void UpdateCars(float dt_in, int frame)
{
    constexpr float MAX_PHYSICS_DT = 0.05f;
    const int subSteps = (dt_in > MAX_PHYSICS_DT) ? (int)std::ceil(dt_in / MAX_PHYSICS_DT) : 1;
    const float dt = dt_in / (float)subSteps;

    for (int sub = 0; sub < subSteps; ++sub) {
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
            if (carState[i] == CarState::PARKED) continue;

            Vec2* wp = carWpBuf[i];
            uint8_t wc = carWpCount[i];
            uint8_t& wn = carWpCurr[i];

            if (wn >= wc) {
                carState[i] = CarState::PARKED;
                carSpeed[i] = 0.0f;
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
            bool onCrosswalk = onGrid && grid[myGZ][myGX] == CROSSWALK;
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
                    float extAngle=std::atan2(exitPt.x-arcCX, exitPt.y-arcCZ);
                    float entAngle=std::atan2(entryPt.x-arcCX, entryPt.y-arcCZ);
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

            // Traffic lights — only check if NOT already on crosswalk or intersection
            if (onGrid && !inIntersection && !onCrosswalk) {
                TurnIntent turnIntent = TI_STRAIGHT;
                int turnCellGX=-1, turnCellGZ=-1;
                if (isTurn) {
                    turnIntent = (turnCross > 0.0f) ? TI_LEFT : TI_RIGHT;
                    WorldToGrid(target_raw.x, target_raw.y, turnCellGX, turnCellGZ);
                }

                auto checkLight = [&](int cgx, int cgz, float dX, float dZ) {
                    if (!IsIntersection(cgx, cgz)) return;
                    TurnIntent li = (cgx==turnCellGX && cgz==turnCellGZ) ? turnIntent : TI_STRAIGHT;
                    auto color = GetLight(cgx,cgz,dX,dZ,li);
                    if (color==L_GREEN) return;
                    Vec2 intC=CellCenter(cgx,cgz);
                    float toCX=intC.x-carPosX[i], toCZ=intC.y-carPosZ[i];
                    float fwdDist=toCX*dX+toCZ*dZ;
                    if (fwdDist<0) return;
                    // Stop at traffic light: HALF_CS + SIDEWALK_W behind intersection edge
                    float stopLine=std::max(0.0f, fwdDist - HCS - SIDEWALK_W);
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

            // Pedestrian safety brake
            if (onGrid) {
                float scanAhead = std::min(40.0f, carSpeed[i] * 3.5f + 8.0f);
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
                        float fwd = ddx2*fwdX + ddz2*fwdZ;
                        float lat = std::abs(ddx2*fwdZ - ddz2*fwdX);
                        float latThresh = (inIntersection || isTurn || inArc)
                            ? (CAR_HW + PED_RADIUS + 4.0f) : (CAR_HW + PED_RADIUS + 1.0f);
                        if (fwd > 0.3f && fwd <= scanAhead && lat <= latThresh) {
                            float stopDist = std::max(0.0f, fwd - CAR_HL - PED_RADIUS - 2.0f);
                            if (stopDist < bestGap) { bestGap = stopDist; bestSpd = 0; }
                        }
                        if (inIntersection || inArc) {
                            float d2 = ddx2*ddx2 + ddz2*ddz2;
                            float safeR = CAR_HL + PED_RADIUS + 4.5f;
                            if (d2 < safeR * safeR && d2 > 0.01f) {
                                float dd = std::sqrt(d2);
                                float stopDist = std::max(0.0f, dd - CAR_HL - PED_RADIUS - 1.0f);
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
            if (inIntersection && carSpeed[i] < 2.0f && bestGap > 5.0f) carSpeed[i] = std::min(2.0f, desMax);

            // Steering
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
                    float maxCorr = std::abs(cte)>1.0f ? 0.40f : 0.18f;
                    steerCorr=std::clamp(steerCorr,-maxCorr,maxCorr);
                    carHeading[i]=segH+steerCorr;
                    carPosX[i]+=std::sin(carHeading[i])*step;
                    carPosZ[i]+=std::cos(carHeading[i])*step;
                } else {
                    float desH=std::atan2(stDx,stDz);
                    float dh=desH-carHeading[i];
                    while(dh>PI) dh-=PI2; while(dh<-PI) dh+=PI2;
                    float sd2=std::clamp(dh,-5.0f*dt,5.0f*dt);
                    carHeading[i]+=sd2;
                    carPosX[i]+=std::sin(carHeading[i])*step;
                    carPosZ[i]+=std::cos(carHeading[i])*step;
                }
            }
        }
    }
}

// ============================================================
//  Pedestrian Update — NO ped-ped collisions, sidewalk-layer clamping
// ============================================================
static void UpdatePeds(float dt, int frame)
{
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

        if (wn >= wc) {
            pedState[i] = PedState::IDLE;
            continue;
        }

        Vec2 target = wps[wn];
        float dx = target.x - pedPosX[i];
        float dz = target.y - pedPosZ[i];
        float distToTarget = std::sqrt(dx*dx + dz*dz);

        int myGX, myGZ;
        bool onGrid = WorldToGrid(pedPosX[i], pedPosZ[i], myGX, myGZ);

        // Red light check: ONLY if not already on a crosswalk or road surface
        // (if already crossing, keep going!)
        bool alreadyCrossing = false;
        if (onGrid) {
            CellType ct = grid[myGZ][myGX];
            alreadyCrossing = (ct == CROSSWALK) ||
                              (ct == ROAD && isIntersection_[myGZ][myGX]);
        }

        if (onGrid && !alreadyCrossing) {
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

        // NO ped-ped collisions — removed entirely

        // Ped yields to nearby moving cars (even while crossing)
        if (onGrid) {
            float ndx2 = (distToTarget > 0.01f) ? dx/distToTarget : 0.f;
            float ndz2 = (distToTarget > 0.01f) ? dz/distToTarget : 0.f;
            for (int dg=-1;dg<=1;dg++)
            for (int dh=-1;dh<=1;dh++) {
                int cgx=myGX+dh, cgz=myGZ+dg;
                if (!InBounds(cgx,cgz)) continue;
                int cellKey=cgz*GS+cgx;
                for (uint32_t c=carDriveHead[cellKey];c!=UINT32_MAX;c=carDriveNext[c]) {
                    if (carSpeed[c] < 0.5f) continue;
                    float cdx=carPosX[c]-pedPosX[i], cdz=carPosZ[c]-pedPosZ[i];
                    float d2=cdx*cdx+cdz*cdz;
                    if (d2 < 25.0f) { // within 5m
                        float fwd=cdx*ndx2+cdz*ndz2;
                        if (fwd > 0.0f && fwd < 5.0f) { // car ahead in walk direction
                            spd = 0.0f;
                            break;
                        }
                    }
                }
                if (spd <= 0.0f) break;
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

        // Snap to sidewalk layer if off it
        if (!IsOnSidewalk(pedPosX[i], pedPosZ[i])) {
            SnapToSidewalk(pedPosX[i], pedPosZ[i]);
        }

        // ---- VIOLATION CHECKS ----
        if (onGrid) {
            CellType ct = grid[myGZ][myGX];
            if (ct == ROAD && !isIntersection_[myGZ][myGX]) {
                Vec2 cc = CellCenter(myGX, myGZ);
                float relX = std::abs(pedPosX[i] - cc.x);
                float relZ = std::abs(pedPosZ[i] - cc.y);
                bool hasNS = IsRoadLike(myGX, myGZ-1) || IsRoadLike(myGX, myGZ+1);
                bool hasEW = IsRoadLike(myGX-1, myGZ) || IsRoadLike(myGX+1, myGZ);
                bool violation = false;
                if (hasNS && !hasEW) violation = (relX < SIDEWALK_INNER - 0.5f);
                else if (hasEW && !hasNS) violation = (relZ < SIDEWALK_INNER - 0.5f);
                else violation = (relX < SIDEWALK_INNER - 0.5f && relZ < SIDEWALK_INNER - 0.5f);
                if (violation) pedOnRoadViolations++;
            }
            if (ct == ROAD && isIntersection_[myGZ][myGX]) {
                Vec2 cc = CellCenter(myGX, myGZ);
                float relX = std::abs(pedPosX[i] - cc.x);
                float relZ = std::abs(pedPosZ[i] - cc.y);
                if (relX < SIDEWALK_INNER - 1.0f && relZ < SIDEWALK_INNER - 1.0f)
                    pedIntersectionViolations++;
            }
        }

        // Car-ped collision check (inline)
        if (onGrid) {
            for (int dg=-1;dg<=1;dg++)
            for (int dh=-1;dh<=1;dh++) {
                int cgx=myGX+dh, cgz=myGZ+dg;
                if (!InBounds(cgx,cgz)) continue;
                int cellKey=cgz*GS+cgx;
                for (uint32_t c=carDriveHead[cellKey];c!=UINT32_MAX;c=carDriveNext[c]) {
                    float ddx2=carPosX[c]-pedPosX[i], ddz2=carPosZ[c]-pedPosZ[i];
                    float dd=std::sqrt(ddx2*ddx2+ddz2*ddz2);
                    if (dd < CAR_HL + PED_RADIUS && carSpeed[c] > 0.5f)
                        carPedCollisionCount++;
                }
            }
        }
    }
}

// ============================================================
//  Agent Update
// ============================================================
static void UpdateAgents(float dt, int frame)
{
    for (uint32_t a = 0; a < agentActiveCount; a++) {
        auto& ag = agents[a];

        switch (ag.phase) {
        case AgentPhase::IDLE_HOME: {
            ag.idleTimer -= dt;
            if (ag.idleTimer > 0.0f) break;
            if (ag.usesCar) {
                auto path = FindPedPath(ag.homeGX, ag.homeGZ, ag.homeParkGX, ag.homeParkGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 101 + frame));
                    if (ag.pedIdx != UINT32_MAX) ag.phase = AgentPhase::WALK_TO_PARKING;
                }
            } else {
                auto path = FindPedPath(ag.homeGX, ag.homeGZ, ag.workGX, ag.workGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 101 + frame));
                    if (ag.pedIdx != UINT32_MAX) ag.phase = AgentPhase::WALK_ONLY;
                }
            }
            break;
        }
        case AgentPhase::WALK_TO_PARKING: {
            if (ag.pedIdx < pedActiveCount && pedState[ag.pedIdx] == PedState::IDLE) {
                auto carPath = FindCarPath(ag.homeParkGX, ag.homeParkGZ, ag.workParkGX, ag.workParkGZ);
                if (carPath.size() >= 2) {
                    ag.carIdx = SpawnCar(carPath, (uint32_t)(a * 37 + 1));
                    if (ag.carIdx != UINT32_MAX) ag.phase = AgentPhase::DRIVING;
                }
            }
            break;
        }
        case AgentPhase::DRIVING: {
            if (ag.carIdx < carActiveCount && carState[ag.carIdx] == CarState::PARKED) {
                auto path = FindPedPath(ag.workParkGX, ag.workParkGZ, ag.workGX, ag.workGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 53 + frame));
                    if (ag.pedIdx != UINT32_MAX) ag.phase = AgentPhase::WALK_TO_DEST;
                } else ag.phase = AgentPhase::AT_DEST;
            }
            break;
        }
        case AgentPhase::WALK_TO_DEST: {
            if (ag.pedIdx < pedActiveCount && pedState[ag.pedIdx] == PedState::IDLE) {
                ag.phase = AgentPhase::AT_DEST;
                ag.idleTimer = 60.0f;
                tripsCompleted++;
            }
            break;
        }
        case AgentPhase::AT_DEST: {
            ag.idleTimer -= dt;
            if (ag.idleTimer > 0.0f) break;
            ag.goingToWork = !ag.goingToWork;
            if (ag.usesCar) {
                auto path = FindPedPath(ag.workGX, ag.workGZ, ag.workParkGX, ag.workParkGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 71 + frame));
                    if (ag.pedIdx != UINT32_MAX) {
                        ag.phase = AgentPhase::WALK_TO_PARKING;
                        std::swap(ag.homeParkGX, ag.workParkGX);
                        std::swap(ag.homeParkGZ, ag.workParkGZ);
                        std::swap(ag.homeGX, ag.workGX);
                        std::swap(ag.homeGZ, ag.workGZ);
                    }
                }
            } else {
                auto path = FindPedPath(ag.workGX, ag.workGZ, ag.homeGX, ag.homeGZ);
                if (path.size() >= 2) {
                    ag.pedIdx = SpawnPed(path, (uint32_t)(a * 71 + frame));
                    if (ag.pedIdx != UINT32_MAX) {
                        ag.phase = AgentPhase::WALK_ONLY;
                        std::swap(ag.homeGX, ag.workGX);
                        std::swap(ag.homeGZ, ag.workGZ);
                    }
                }
            }
            break;
        }
        case AgentPhase::WALK_ONLY: {
            if (ag.pedIdx < pedActiveCount && pedState[ag.pedIdx] == PedState::IDLE) {
                ag.phase = AgentPhase::AT_DEST;
                ag.idleTimer = 60.0f;
                tripsCompleted++;
            }
            break;
        }
        }
    }
}

// ============================================================
//  MAIN
// ============================================================
int main()
{
    printf("=== SIDEWALK LAYER Simulation Test ===\n");
    printf("Changes: sidewalk layer, no ped-ped collision, keep-crossing,\n");
    printf("         shorter ped phase (ALL_RED=%.0fs), crosswalk 1 SW closer\n\n", ALL_RED_DUR);

    BuildGrid();
    DetectIntersections();
    PlaceCrosswalks();
    DetectIntersections();
    BuildSidewalkLayer();

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

    // Count sidewalk sub-cells
    int swCount = 0;
    for (int z = 0; z < GS*SUB; z++)
    for (int x = 0; x < GS*SUB; x++)
        if (sidewalkLayer[z][x]) swCount++;

    printf("Grid: %dx%d  Roads=%d Houses=%d Workplaces=%d Intersections=%d Crosswalks=%d Parking=%d\n",
           GS, GS, roadCount, houseCount, wpCount, intCount, crossCount, parkCount);
    printf("Sidewalk layer: %d sub-cells marked (%.1f%% of %dx%d)\n",
           swCount, 100.0f * swCount / (GS*SUB*GS*SUB), GS*SUB, GS*SUB);

    InitLights();

    std::fill(pedHead, pedHead + GS*GS, UINT32_MAX);
    std::fill(pedNext, pedNext + MAX_PEDS, UINT32_MAX);
    std::fill(carDriveHead, carDriveHead + GS*GS, UINT32_MAX);
    std::fill(carDriveNext, carDriveNext + MAX_CARS, UINT32_MAX);

    // Spawn agents
    std::vector<std::pair<int,int>> houses, workplaces;
    for (int z=0;z<GS;z++) for (int x=0;x<GS;x++) {
        if (grid[z][x]==HOUSE) houses.push_back({x,z});
        if (grid[z][x]==WORKPLACE) workplaces.push_back({x,z});
    }

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> wpDist(0, (int)workplaces.size()-1);

    int agentsWithCar = 0, agentsWalkOnly = 0, agentsFailed = 0;

    for (size_t hi = 0; hi < houses.size() && agentActiveCount < MAX_AGENTS; hi++) {
        auto [hx, hz] = houses[hi];
        if (workplaces.empty()) break;
        auto [wx, wz] = workplaces[wpDist(rng)];

        Agent ag;
        ag.homeGX = hx; ag.homeGZ = hz;
        ag.workGX = wx; ag.workGZ = wz;
        ag.goingToWork = true;
        ag.idleTimer = 1.0f + (float)(hi % 20) * 2.0f;

        bool hasHomePark = FindNearestParking(hx, hz, ag.homeParkGX, ag.homeParkGZ);
        bool hasWorkPark = FindNearestParking(wx, wz, ag.workParkGX, ag.workParkGZ);

        if (hasHomePark && hasWorkPark) {
            auto carPath = FindCarPath(ag.homeParkGX, ag.homeParkGZ, ag.workParkGX, ag.workParkGZ);
            if (carPath.size() >= 2) { ag.usesCar = true; agentsWithCar++; }
            else { ag.usesCar = false; agentsWalkOnly++; }
        } else { ag.usesCar = false; agentsWalkOnly++; }

        auto testPath = FindPedPath(hx, hz, (ag.usesCar ? ag.homeParkGX : wx),
                                            (ag.usesCar ? ag.homeParkGZ : wz));
        if (testPath.size() < 2) { agentsFailed++; continue; }

        ag.phase = AgentPhase::IDLE_HOME;
        agents[agentActiveCount] = ag;
        agentActiveCount++;
    }

    printf("Agents: %u (car=%d, walk=%d, failed=%d)\n\n",
           agentActiveCount, agentsWithCar, agentsWalkOnly, agentsFailed);

    // Run simulation
    constexpr float BIG_DT = (1.0f / 60.0f) * 100.0f;
    constexpr int TOTAL_FRAMES = 30 * 60;

    printf("=== SIMULATION: %d frames @ 100x (%.0f game-sec) ===\n\n", TOTAL_FRAMES, TOTAL_FRAMES * BIG_DT);

    for (int f = 0; f < TOTAL_FRAMES; f++) {
        UpdateLights(BIG_DT);
        UpdateAgents(BIG_DT, f);
        UpdateCars(BIG_DT, f);
        UpdatePeds(BIG_DT, f);

        if (f % 600 == 0) {
            int driving=0;
            for (uint32_t i=0;i<carActiveCount;i++)
                if (carState[i]==CarState::DRIVING) driving++;
            int walking=0, waiting=0;
            for (uint32_t i=0;i<pedActiveCount;i++) {
                if (pedState[i]==PedState::WALKING) walking++;
                if (pedState[i]==PedState::WAITING_CROSS) waiting++;
            }
            printf("  f=%5d  cars=%u(drv=%d) peds=%u(walk=%d,wait=%d) trips=%u violations=%u/%u carped=%u\n",
                   f, carActiveCount, driving, pedActiveCount, walking, waiting,
                   tripsCompleted, pedOnRoadViolations, pedIntersectionViolations, carPedCollisionCount);
        }
    }

    // Final report
    printf("\n========================================\n");
    printf("  RESULTS\n");
    printf("========================================\n");
    printf("  Agents:            %u\n", agentActiveCount);
    printf("  Trips completed:   %u\n", tripsCompleted);
    printf("  Cars spawned:      %u\n", carActiveCount);
    printf("  Peds spawned:      %u\n", pedActiveCount);
    printf("  Ped on-road:       %u\n", pedOnRoadViolations);
    printf("  Ped intersection:  %u\n", pedIntersectionViolations);
    printf("  Car-ped collisions:%u\n", carPedCollisionCount);
    printf("========================================\n");

    bool pass = (pedOnRoadViolations == 0 && carPedCollisionCount == 0);
    printf("\n%s\n", pass ? "*** PASS ***" : "*** FAIL ***");

    return pass ? 0 : 1;
}
