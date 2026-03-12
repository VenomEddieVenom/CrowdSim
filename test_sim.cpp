// ================================================================
//  Standalone terminal traffic simulation — BIG CITY map replica
//  Mirrors CarSystem + TrafficLightSystem + CityLayout exactly.
//  100x speed, 30 seconds real = 3000 seconds sim time.
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
#include <functional>

static constexpr float PI  = 3.14159265358979323846f;
static constexpr float PI2 = PI * 2.0f;

// ============================================================
//  Constants (exact copies from CarSystem)
// ============================================================
static constexpr uint32_t MAX_CARS  = 2000;
static constexpr float MAX_SPEED    = 14.0f;
static constexpr float ACCEL        =  6.0f;
static constexpr float DECEL        = 14.0f;
static constexpr float CAR_HL       =  1.00f;
static constexpr float CAR_HW       =  0.50f;
static constexpr float MIN_SEP      =  CAR_HL * 2.f + 0.5f;   // 2.5 m
static constexpr float LAT_BAND     =  1.5f;
static constexpr float IDM_S0       =  2.0f;
static constexpr float IDM_T        =  1.5f;
static constexpr float IDM_B        =  3.0f;
static constexpr float K_STANLEY    =  3.5f;
static constexpr uint8_t MAX_WP     = 64;
static constexpr float PARK_PAUSE   =  2.0f;

// ============================================================
//  Grid — exact big_city map (40x40)
// ============================================================
static constexpr int GS  = 40;
static constexpr float CS = 20.0f;
static constexpr float HCS = CS * 0.5f;
static constexpr float HALF_WORLD = GS * CS * 0.5f; // 400 m

enum CellType : uint8_t { EMPTY=0, ROAD=1, HOUSE=2, WORKPLACE=3, CROSSWALK=4, PARKING=5 };

// Hardcoded big_city map — rows 0..39
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

static CellType grid[GS][GS];
static int roadLanes[GS][GS];

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
            roadLanes[z][x] = 4; // default
        }
    }
    // 6-lane boulevards: columns 17 and 22 (all rows), rows 17 and 22 (all cols)
    for (int i = 0; i < GS; i++) {
        roadLanes[i][17] = 6;
        roadLanes[i][22] = 6;
        roadLanes[17][i] = 6;
        roadLanes[22][i] = 6;
    }
}

static bool InBounds(int gx, int gz) { return gx >= 0 && gx < GS && gz >= 0 && gz < GS; }

static bool IsRoadLike(int gx, int gz)
{
    if (!InBounds(gx, gz)) return false;
    return grid[gz][gx] == ROAD || grid[gz][gx] == CROSSWALK;
}

struct Vec2 { float x, y; };

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

// ============================================================
//  Lane offsets (exact copy)
// ============================================================
static void GetLaneOffsets(int totalLanes, float* offsets, int& count)
{
    if (totalLanes >= 6) { count = 3; offsets[0] = 1.5f; offsets[1] = 4.0f; offsets[2] = 6.5f; }
    else if (totalLanes >= 4) { count = 2; offsets[0] = 2.5f; offsets[1] = 5.5f; }
    else { count = 1; offsets[0] = 4.0f; }
}

// ============================================================
//  Intersection detection (3+ road neighbors)
// ============================================================
static bool isIntersection[GS][GS];

static void DetectIntersections()
{
    memset(isIntersection, 0, sizeof(isIntersection));
    const int dx[] = {1,-1,0,0};
    const int dz[] = {0,0,1,-1};
    for (int z = 0; z < GS; z++)
    for (int x = 0; x < GS; x++) {
        if (!IsRoadLike(x, z)) continue;
        int ncount = 0;
        for (int d = 0; d < 4; d++)
            if (IsRoadLike(x + dx[d], z + dz[d])) ncount++;
        isIntersection[z][x] = (ncount >= 3);
    }
}

static bool IsIntersection(int gx, int gz)
{
    if (!InBounds(gx, gz)) return false;
    return isIntersection[gz][gx];
}

// ============================================================
//  8-phase traffic lights (exact copy of TrafficLightSystem)
// ============================================================
static float g_lightTime = 0.0f;
static constexpr float THROUGH_DUR = 8.0f;
static constexpr float YELLOW_DUR  = 2.0f;
static constexpr float LEFT_DUR    = 4.0f;
static constexpr float ALL_RED_DUR = 2.0f;
static constexpr float CYCLE_DUR   = (THROUGH_DUR + YELLOW_DUR + LEFT_DUR + ALL_RED_DUR) * 2.0f; // 32s

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
        if (!isIntersection[z][x]) continue;
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
    if (!isIntersection[gz][gx]) return L_GREEN;

    // NS = abs(dz) > abs(dx)
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
//  Dijkstra pathfinding (exact copy)
// ============================================================
static std::vector<Vec2> FindPath(int srcGX, int srcGZ, int dstGX, int dstGZ)
{
    if (!InBounds(srcGX, srcGZ) || !InBounds(dstGX, dstGZ)) return {};
    const int ddx[] = {1,-1,0,0};
    const int ddz[] = {0,0,1,-1};

    // Find road adjacent to src/dst
    int srcRX=-1,srcRZ=-1,dstRX=-1,dstRZ=-1;
    for (int d = 0; d < 4; d++) {
        int nx = srcGX+ddx[d], nz = srcGZ+ddz[d];
        if (IsRoadLike(nx, nz)) { srcRX=nx; srcRZ=nz; break; }
    }
    for (int d = 0; d < 4; d++) {
        int nx = dstGX+ddx[d], nz = dstGZ+ddz[d];
        if (IsRoadLike(nx, nz)) { dstRX=nx; dstRZ=nz; break; }
    }
    if (srcRX < 0 || dstRX < 0) return {};

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
            int nx = cx+ddx[d], nz = cz+ddz[d];
            if (!InBounds(nx, nz)) continue;
            int nid = nz * GS + nx;
            if (!IsRoadLike(nx, nz)) continue;
            float ec = 1.0f;
            float nc = cc + ec;
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
        road.push_back(CellCenter(cur % GS, cur / GS));
    road.push_back(CellCenter(srcRX, srcRZ));
    std::reverse(road.begin(), road.end());
    return road;
}

// ============================================================
//  Car state (SoA)
// ============================================================
static float    posX[MAX_CARS], posZ[MAX_CARS];
static float    speed_[MAX_CARS], heading_[MAX_CARS];
static float    laneOff[MAX_CARS], laneTarget[MAX_CARS];
static int8_t   laneIdx_[MAX_CARS];

enum class State : uint8_t { PARKED, DRIVING };
static State    state_[MAX_CARS];
static float    parkTimer[MAX_CARS];

static Vec2     wpBuf[MAX_CARS][MAX_WP];
static uint8_t  wpCount[MAX_CARS], wpCurr[MAX_CARS];
static uint8_t  carDir_[MAX_CARS];

static uint32_t activeCount = 0;

// Per-cell linked list
static uint32_t driveHead[GS * GS];
static uint32_t driveNext[MAX_CARS];

// Schedule
static float    schedDepart[MAX_CARS];
static float    schedReturn[MAX_CARS];
static uint32_t lastDayTrip[MAX_CARS];

// ============================================================
//  Helpers
// ============================================================
static Vec2 DirTo(Vec2 a, Vec2 b) {
    float dx = b.x-a.x, dz = b.y-a.y;
    float len = std::sqrt(dx*dx+dz*dz);
    if (len < 1e-6f) return {1,0};
    return {dx/len, dz/len};
}

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

// ============================================================
//  SpawnCar (mirrors real SpawnCar exactly)
// ============================================================
static uint32_t SpawnCar(const std::vector<Vec2>& path, uint32_t colorSeed)
{
    if (activeCount >= MAX_CARS || path.size() < 2) return UINT32_MAX;
    uint32_t i = activeCount++;
    uint8_t wc = (uint8_t)std::min(path.size(), (size_t)MAX_WP);
    for (uint8_t w = 0; w < wc; w++) wpBuf[i][w] = path[w];
    wpCount[i] = wc;
    SimplifyPath(wpBuf[i], wpCount[i]);
    wpCurr[i] = 1;
    carDir_[i] = 0;

    // Lane assignment (50/50 inner/outer)
    float lo[3]; int lc;
    GetLaneOffsets(4, lo, lc);
    laneIdx_[i] = (int8_t)((colorSeed % 2 == 0) ? 0 : lc - 1);
    laneOff[i] = lo[std::min((int)laneIdx_[i], lc-1)];
    laneTarget[i] = laneOff[i];

    Vec2 d = DirTo(path[0], path[1]);
    posX[i] = path[0].x;
    posZ[i] = path[0].y;
    speed_[i] = 0.0f;
    heading_[i] = std::atan2(d.x, d.y);
    state_[i] = State::PARKED;
    parkTimer[i] = 0.3f + (float)(colorSeed % 8) * 0.6f;

    // Schedule
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
    schedDepart[i] = depart;
    schedReturn[i] = std::fmod(depart + shiftLen, 24.0f);
    lastDayTrip[i] = 0;

    return i;
}

// ============================================================
//  Collision tracking
// ============================================================
struct CollisionRecord {
    uint32_t carA, carB;
    int frame;
    float dist, hcos;
    float hdgA, hdgB;
    float laneA, laneB;
    int cellGX, cellGZ;
    std::string type;
};
static std::vector<CollisionRecord> collisions;

// ============================================================
//  UPDATE (mirrors CarSystem::Update exactly)
// ============================================================
static uint32_t dayCounter = 0;
static float prevTimeOfDay = 0.0f;

static void Update(float dt_in, int frame, float timeOfDay)
{
    // ---- DT clamping with substeps ----
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
    std::fill(driveHead, driveHead + GS*GS, UINT32_MAX);
    std::fill(driveNext, driveNext + MAX_CARS, UINT32_MAX);
    for (uint32_t i = 0; i < activeCount; i++) {
        if (state_[i] != State::DRIVING) continue;
        int gx, gz;
        if (WorldToGrid(posX[i], posZ[i], gx, gz)) {
            int key = gz*GS+gx;
            driveNext[i] = driveHead[key];
            driveHead[key] = i;
        }
    }

    for (uint32_t i = 0; i < activeCount; i++) {
        // ---- PARKED ----
        if (state_[i] == State::PARKED) {
            if (parkTimer[i] > 0.0f) { parkTimer[i] -= dt; continue; }

            bool shouldDepart = false;
            if (carDir_[i] == 0) {
                float dep = schedDepart[i];
                float diff = currentHour - dep;
                if (diff < 0.0f) diff += 24.0f;
                if (diff >= 0.0f && diff < 0.5f && lastDayTrip[i] != dayCounter * 2 + 1) {
                    shouldDepart = true;
                    lastDayTrip[i] = dayCounter * 2 + 1;
                }
            } else {
                float ret = schedReturn[i];
                float diff = currentHour - ret;
                if (diff < 0.0f) diff += 24.0f;
                if (diff >= 0.0f && diff < 0.5f && lastDayTrip[i] != dayCounter * 2 + 2) {
                    shouldDepart = true;
                    lastDayTrip[i] = dayCounter * 2 + 2;
                }
            }
            if (!shouldDepart) continue;

            // Departure proximity check: delay if a driving car is too close
            {
                bool blocked = false;
                int cgx, cgz;
                if (WorldToGrid(posX[i], posZ[i], cgx, cgz)) {
                    for (int dg = -1; dg <= 1 && !blocked; dg++)
                    for (int dh = -1; dh <= 1 && !blocked; dh++) {
                        int nx2 = cgx+dh, nz2 = cgz+dg;
                        if (!InBounds(nx2, nz2)) continue;
                        int nkey = nz2*GS+nx2;
                        for (uint32_t j = driveHead[nkey]; j != UINT32_MAX; j = driveNext[j]) {
                            float ddx = posX[j]-posX[i], ddz = posZ[j]-posZ[i];
                            if (ddx*ddx+ddz*ddz < MIN_SEP*MIN_SEP)
                            { blocked = true; break; }
                        }
                    }
                }
                if (blocked) { parkTimer[i] = 0.5f; continue; }
            }

            // Reverse path
            Vec2* wp = wpBuf[i];
            uint8_t wc = wpCount[i];
            for (uint8_t a = 0, b = wc-1; a < b; a++, b--)
                std::swap(wp[a], wp[b]);
            wpCurr[i] = 1;
            carDir_[i] ^= 1;

            if (wc >= 2) {
                Vec2 d = DirTo(wp[0], wp[1]);
                heading_[i] = std::atan2(d.x, d.y);
            }
            speed_[i] = 0.0f;
            state_[i] = State::DRIVING;
            // Insert into drive linked list so subsequent parked cars see us
            {
                int dgx, dgz;
                if (WorldToGrid(posX[i], posZ[i], dgx, dgz)) {
                    int dkey = dgz*GS+dgx;
                    driveNext[i] = driveHead[dkey];
                    driveHead[dkey] = i;
                }
            }
            continue;
        }

        // ---- DRIVING ----
        Vec2* wp = wpBuf[i];
        uint8_t wc = wpCount[i];
        uint8_t& wn = wpCurr[i];

        // End of path → immediately reverse (continuous driving for density test)
        if (wn >= wc) {
            for (uint8_t a = 0, b = wc-1; a < b; a++, b--)
                std::swap(wp[a], wp[b]);
            wn = 1;
            carDir_[i] ^= 1;
            if (wc >= 2) {
                Vec2 d = DirTo(wp[0], wp[1]);
                heading_[i] = std::atan2(d.x, d.y);
            }
            speed_[i] = std::min(speed_[i], 3.0f);
            continue;
        }

        Vec2 prev_raw = (wn > 0) ? wp[wn-1] : wp[0];
        Vec2 target_raw = wp[wn];
        Vec2 segDir = DirTo(prev_raw, target_raw);
        float segLen = std::sqrt((target_raw.x-prev_raw.x)*(target_raw.x-prev_raw.x) +
                                 (target_raw.y-prev_raw.y)*(target_raw.y-prev_raw.y));

        float myFwdX = std::sin(heading_[i]);
        float myFwdZ = std::cos(heading_[i]);

        int myGX=0, myGZ=0;
        bool onGrid = WorldToGrid(posX[i], posZ[i], myGX, myGZ);

        // Turn detection
        Vec2 nextSeg = segDir;
        float cornerCos = 1.0f, turnCross = 0.0f;
        if (wn+1 < wc) {
            nextSeg = DirTo(target_raw, wp[wn+1]);
            cornerCos = segDir.x*nextSeg.x + segDir.y*nextSeg.y;
            turnCross = segDir.x*nextSeg.y - segDir.y*nextSeg.x;
        }
        bool isTurn = (cornerCos < 0.3f);

        // Dynamic lane offsets
        float cellLaneOffs[3]; int cellLaneCount=0;
        { int tl = onGrid ? GetLanes(myGX, myGZ) : 4;
          GetLaneOffsets(tl, cellLaneOffs, cellLaneCount); }
        if (laneIdx_[i] >= cellLaneCount) laneIdx_[i] = (int8_t)(cellLaneCount-1);
        if (laneIdx_[i] < 0) laneIdx_[i] = 0;
        laneTarget[i] = cellLaneOffs[laneIdx_[i]];

        { float ld = laneTarget[i]-laneOff[i];
          float ml = 3.0f*dt;
          if (std::abs(ld)>ml) laneOff[i]+=(ld>0?ml:-ml);
          else laneOff[i]=laneTarget[i]; }

        bool inIntersection = onGrid && IsIntersection(myGX, myGZ);

        float segFwd = (posX[i]-prev_raw.x)*segDir.x + (posZ[i]-prev_raw.y)*segDir.y;

        // Steering
        Vec2 steerTarget; bool inArc = false;

        if (isTurn) {
            float inRX=segDir.y,inRZ=-segDir.x;
            float outRX=nextSeg.y,outRZ=-nextSeg.x;
            Vec2 entryPt = { target_raw.x-segDir.x*HCS+inRX*laneTarget[i],
                             target_raw.y-segDir.y*HCS+inRZ*laneTarget[i] };
            Vec2 exitPt = { target_raw.x+nextSeg.x*HCS+outRX*laneTarget[i],
                            target_raw.y+nextSeg.y*HCS+outRZ*laneTarget[i] };
            float entryDist = std::max(0.0f, segLen-HCS);

            if (segFwd >= entryDist-1.0f) {
                inArc = true;
                laneOff[i] = laneTarget[i];
                float chX=exitPt.x-entryPt.x, chZ=exitPt.y-entryPt.y;
                float chLen=std::sqrt(chX*chX+chZ*chZ);
                float R=std::max(1.0f, chLen*0.7071f);
                float perpX,perpZ;
                if (turnCross<0) { perpX=inRX; perpZ=inRZ; }
                else { perpX=-inRX; perpZ=-inRZ; }
                float arcCX=entryPt.x+perpX*R, arcCZ=entryPt.y+perpZ*R;
                float carAngle=std::atan2(posX[i]-arcCX, posZ[i]-arcCZ);
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
                float crossDot=(posX[i]-exitPt.x)*nextSeg.x+(posZ[i]-exitPt.y)*nextSeg.y;
                if (crossDot>=-0.5f && tArc>0.75f) {
                    heading_[i]=std::atan2(nextSeg.x,nextSeg.y);
                    ++wn; continue;
                }
            } else {
                float rX=segDir.y,rZ=-segDir.x;
                float look=std::clamp(speed_[i]*0.8f,3.0f,12.0f);
                float tgtFwd=std::min(segFwd+look,entryDist);
                steerTarget.x=prev_raw.x+segDir.x*tgtFwd+rX*laneOff[i];
                steerTarget.y=prev_raw.y+segDir.y*tgtFwd+rZ*laneOff[i];
            }
        } else {
            float rX=segDir.y,rZ=-segDir.x;
            float look=std::clamp(speed_[i]*0.8f,3.0f,12.0f);
            float tgtFwd=std::min(segFwd+look,segLen);
            steerTarget.x=prev_raw.x+segDir.x*tgtFwd+rX*laneOff[i];
            steerTarget.y=prev_raw.y+segDir.y*tgtFwd+rZ*laneOff[i];

            float crossDot=(posX[i]-target_raw.x)*segDir.x+(posZ[i]-target_raw.y)*segDir.y;
            if (crossDot>=-0.3f && segFwd>=segLen-1.0f) { ++wn; continue; }
        }

        float stDx=steerTarget.x-posX[i], stDz=steerTarget.y-posZ[i];
        float stDist=std::sqrt(stDx*stDx+stDz*stDz);

        // ---- Lead car detection ----
        float bestGap=1e6f, bestSpd=MAX_SPEED;
        if (onGrid) {
            for (int dg=-1;dg<=1;dg++)
            for (int dh=-1;dh<=1;dh++) {
                int cgx=myGX+dh, cgz=myGZ+dg;
                if (!InBounds(cgx,cgz)) continue;
                int cellKey=cgz*GS+cgx;
                for (uint32_t j=driveHead[cellKey];j!=UINT32_MAX;j=driveNext[j]) {
                    if (j==i) continue;
                    float odx=posX[j]-posX[i], odz=posZ[j]-posZ[i];
                    float fwd=odx*myFwdX+odz*myFwdZ;
                    if (fwd<=0) continue;
                    float lat=std::abs(odx*myFwdZ-odz*myFwdX);
                    if (lat>LAT_BAND) continue;
                    float jFwdX=std::sin(heading_[j]), jFwdZ=std::cos(heading_[j]);
                    float hcos=myFwdX*jFwdX+myFwdZ*jFwdZ;
                    if (hcos<0.3f) continue;
                    if (!inArc && !inIntersection && hcos>0.5f) {
                        float laneDiff=std::abs(laneOff[j]-laneOff[i]);
                        if (laneDiff>2.0f) continue;
                    }
                    float gap=fwd-MIN_SEP;
                    if (gap<bestGap) { bestGap=gap; bestSpd=speed_[j]; }
                }
            }
        }

        // ---- Traffic lights ----
        if (onGrid && !IsIntersection(myGX, myGZ)) {
            // Determine turn intent
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
                float toCX=intC.x-posX[i], toCZ=intC.y-posZ[i];
                float fwdDist=toCX*dX+toCZ*dZ;
                if (fwdDist<0) return;
                float stopLine=std::max(0.0f,fwdDist-HCS-1.0f);
                if (color==L_RED) { if (stopLine<bestGap) { bestGap=stopLine; bestSpd=0; } }
                else if (color==L_YELLOW) { if (stopLine>3.0f && stopLine<bestGap) { bestGap=stopLine; bestSpd=0; } }
            };

            int psx=myGX, psz=myGZ;
            float scanMax=std::min(80.0f,segLen-segFwd+20.0f);
            for (float d=HCS;d<scanMax;d+=HCS) {
                float sx=posX[i]+segDir.x*d, sz=posZ[i]+segDir.y*d;
                int sgx,sgz;
                if (!WorldToGrid(sx,sz,sgx,sgz)) continue;
                if (sgx==psx && sgz==psz) continue;
                psx=sgx; psz=sgz;
                checkLight(sgx,sgz,segDir.x,segDir.y);
            }

            // Don't block the box — but only for cars NOT already in intersection
            // Only count same-direction stopped cars in exit cell
            if (!inArc && !inIntersection) {
                int px2=myGX, pz2=myGZ;
                for (float d=HCS;d<60.0f;d+=HCS) {
                    float sx=posX[i]+segDir.x*d, sz=posZ[i]+segDir.y*d;
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
                    for (uint32_t j2=driveHead[ekey];j2!=UINT32_MAX;j2=driveNext[j2]) {
                        if (speed_[j2]>=1.0f) continue;
                        // Only count cars heading roughly the same direction
                        float j2Fx=std::sin(heading_[j2]), j2Fz=std::cos(heading_[j2]);
                        float dirDot=segDir.x*j2Fx+segDir.y*j2Fz;
                        if (dirDot>0.3f) jam++;
                    }
                    int exitLanes = GetLanes(ecx, ecz);
                    int jamThreshold = exitLanes * 2;
                    if (jam>=jamThreshold) {
                        Vec2 ic=CellCenter(sgx,sgz);
                        float fd=(ic.x-posX[i])*segDir.x+(ic.y-posZ[i])*segDir.y;
                        if (fd>0) { float sd=std::max(0.0f,fd-HCS-1.0f); if (sd<bestGap) { bestGap=sd; bestSpd=0; } }
                    }
                    break;
                }
            }
        }

        // ---- IDM ----
        float v=speed_[i], vR=v/MAX_SPEED, vR4=vR*vR*vR*vR;
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

        speed_[i]=std::clamp(v+idmAcc*dt,0.0f,desMax);

        // Allow creep instead of permanent hard-stop (prevents permanent gridlock)
        if (bestGap<=-0.5f && bestSpd<0.1f) speed_[i]=std::min(speed_[i], 0.5f);

        // Cars inside intersection creep through to clear the box, but only if room ahead
        if (inIntersection && speed_[i] < 2.0f && bestGap > 0.0f) speed_[i] = std::min(2.0f, desMax);

        // ---- Steering + movement ----
        if (speed_[i]>0.001f && stDist>0.01f) {
            float step=speed_[i]*dt;
            if (!inArc) {
                float segH=std::atan2(segDir.x,segDir.y);
                float rX=segDir.y, rZ=-segDir.x;
                float sf2=(posX[i]-prev_raw.x)*segDir.x+(posZ[i]-prev_raw.y)*segDir.y;
                float lX=prev_raw.x+segDir.x*sf2+rX*laneOff[i];
                float lZ=prev_raw.y+segDir.y*sf2+rZ*laneOff[i];
                float cte=(posX[i]-lX)*rX+(posZ[i]-lZ)*rZ;
                float steerCorr=-std::atan2(K_STANLEY*cte,speed_[i]+0.5f);
                float maxCorr;
                if (std::abs(cte)>1.0f) maxCorr=0.40f;
                else if (std::abs(laneTarget[i]-laneOff[i])>0.3f) maxCorr=0.30f;
                else maxCorr=0.18f;
                steerCorr=std::clamp(steerCorr,-maxCorr,maxCorr);
                heading_[i]=segH+steerCorr;
                posX[i]+=std::sin(heading_[i])*step;
                posZ[i]+=std::cos(heading_[i])*step;
            } else {
                float desH=std::atan2(stDx,stDz);
                float dh=desH-heading_[i];
                while(dh>PI) dh-=PI2; while(dh<-PI) dh+=PI2;
                float maxSR=5.0f;
                float sd2=std::clamp(dh,-maxSR*dt,maxSR*dt);
                float bd=dh*std::min(1.0f,8.0f*dt);
                if (std::abs(bd)>std::abs(sd2))
                    sd2=std::clamp(bd,-maxSR*dt*1.5f,maxSR*dt*1.5f);
                heading_[i]+=sd2;
                posX[i]+=std::sin(heading_[i])*step;
                posZ[i]+=std::cos(heading_[i])*step;
            }
        }
    }

    // ---- Collision resolution ----
    std::fill(driveHead, driveHead+GS*GS, UINT32_MAX);
    std::fill(driveNext, driveNext+MAX_CARS, UINT32_MAX);
    for (uint32_t i=0;i<activeCount;i++) {
        if (state_[i]!=State::DRIVING) continue;
        int gx,gz;
        if (WorldToGrid(posX[i],posZ[i],gx,gz)) {
            int key=gz*GS+gx;
            driveNext[i]=driveHead[key]; driveHead[key]=i;
        }
    }

    auto resolvePair = [&](uint32_t a, uint32_t b) {
        float ddx2=posX[b]-posX[a], ddz2=posZ[b]-posZ[a];
        float dd2=ddx2*ddx2+ddz2*ddz2;
        if (dd2>=MIN_SEP*MIN_SEP || dd2<0.001f) return;
        float hcos=std::sin(heading_[a])*std::sin(heading_[b])+std::cos(heading_[a])*std::cos(heading_[b]);
        bool bothD=(state_[a]==State::DRIVING && state_[b]==State::DRIVING);
        if (bothD && hcos<0.3f) return;
        if (bothD) {
            float sinH=std::sin(heading_[a]),cosH=std::cos(heading_[a]);
            float lat2=std::abs(ddx2*cosH-ddz2*sinH);
            if (lat2>LAT_BAND) return;
            // Skip if clearly in different lanes
            float laneDiff=std::abs(laneOff[a]-laneOff[b]);
            if (hcos>0.5f && laneDiff>2.0f) return;
        }
        float dd=std::sqrt(dd2);

        // Only record as collision if at least one car is moving (>1 m/s)
        if (speed_[a] > 1.0f || speed_[b] > 1.0f) {
        // Record collision
        float hdgDiff=std::abs(heading_[a]-heading_[b]);
        while(hdgDiff>PI) hdgDiff-=PI2;
        hdgDiff=std::abs(hdgDiff);
        std::string type;
        if (hdgDiff>2.3f) type="head-on";
        else if (hdgDiff>1.0f) type="intersection-cross";
        else {
            float ldiff=std::abs(laneOff[a]-laneOff[b]);
            type=(ldiff>1.5f)?"cross-lane":"same-lane-rear";
        }
        int ga,ga2; WorldToGrid(posX[a],posZ[a],ga,ga2);
        collisions.push_back({a,b,frame,dd,hcos,heading_[a],heading_[b],laneOff[a],laneOff[b],ga,ga2,type});
        }

        // Push
        float overlap=MIN_SEP-dd;
        float nx=ddx2/dd, nz=ddz2/dd;
        float sinH=std::sin(heading_[a]),cosH=std::cos(heading_[a]);
        float pushFwd=nx*sinH+nz*cosH;
        float pushMag=std::min(overlap*0.3f,0.3f);
        if (std::abs(pushFwd)>0.1f) {
            float px=sinH*pushFwd*pushMag, pz=cosH*pushFwd*pushMag;
            posX[a]-=px; posZ[a]-=pz;
            posX[b]+=px; posZ[b]+=pz;
        }
        float fwd=ddx2*sinH+ddz2*cosH;
        if (fwd>0) speed_[a]=std::min(speed_[a],speed_[b]*0.5f);
        else speed_[b]=std::min(speed_[b],speed_[a]*0.5f);
    };

    for (int cy=0;cy<GS;cy++)
    for (int cx=0;cx<GS;cx++) {
        int key=cy*GS+cx;
        if (driveHead[key]==UINT32_MAX) continue;
        for (uint32_t a=driveHead[key];a!=UINT32_MAX;a=driveNext[a])
            for (uint32_t b=driveNext[a];b!=UINT32_MAX;b=driveNext[b])
                resolvePair(a,b);
        static const int NBR[][2]={{1,0},{0,1},{1,1},{-1,1}};
        for (auto& nb:NBR) {
            int nx2=cx+nb[0], nz2=cy+nb[1];
            if (!InBounds(nx2,nz2)) continue;
            int nkey=nz2*GS+nx2;
            if (driveHead[nkey]==UINT32_MAX) continue;
            for (uint32_t a=driveHead[key];a!=UINT32_MAX;a=driveNext[a])
                for (uint32_t b=driveHead[nkey];b!=UINT32_MAX;b=driveNext[b])
                    resolvePair(a,b);
        }
    }

  } // end substep loop
}

// ============================================================
//  MAIN
// ============================================================
int main()
{
    printf("=== Traffic Simulation — BIG CITY Map ===\n");
    printf("Constants: MIN_SEP=%.1f LAT_BAND=%.1f K_STANLEY=%.1f\n", MIN_SEP, LAT_BAND, K_STANLEY);

    BuildGrid();
    DetectIntersections();
    InitLights();

    // Count cell types and intersections
    int roadCount=0, houseCount=0, wpCount2=0, intCount=0;
    for (int z=0;z<GS;z++) for (int x=0;x<GS;x++) {
        if (IsRoadLike(x,z)) roadCount++;
        if (grid[z][x]==HOUSE) houseCount++;
        if (grid[z][x]==WORKPLACE) wpCount2++;
        if (isIntersection[z][x]) intCount++;
    }
    printf("Grid: %dx%d  Roads=%d Houses=%d Workplaces=%d Intersections=%d\n",
           GS, GS, roadCount, houseCount, wpCount2, intCount);

    // Spawn cars: house → workplace pairs (mirrors TrySpawnFromHouse exactly)
    // All cars start PARKED with staggered departure timers (like real game)
    int housesSpawned = 0;
    for (int hz=0;hz<GS;hz++)
    for (int hx=0;hx<GS;hx++) {
        if (grid[hz][hx] != HOUSE) continue;
        bool found = false;
        for (int wz=0;wz<GS && !found;wz++)
        for (int wx=0;wx<GS && !found;wx++) {
            if (grid[wz][wx] != WORKPLACE) continue;
            auto path = FindPath(hx,hz,wx,wz);
            if (path.size() < 2) continue;
            int toSpawn = ((int)path.size() > 3) ? 3 : 5;
            uint32_t seed = (uint32_t)(hx*37 + hz*19 + wx*7 + wz*3);
            int ok = 0;
            for (int c = 0; c < toSpawn && activeCount < MAX_CARS; c++)
                if (SpawnCar(path, seed + (uint32_t)c) != UINT32_MAX) ok++;
            if (ok == 0) continue;
            found = true;
            housesSpawned++;
        }
    }
    printf("Houses spawned: %d  Total cars: %u\n", housesSpawned, activeCount);

    // All cars parked — force them to try departing ASAP (worst-case burst test)
    // Set schedDepart to 7.0 so all try to leave at hour 7.0
    for (uint32_t i = 0; i < activeCount; i++) {
        state_[i] = State::PARKED;
        parkTimer[i] = 0.0f;
        schedDepart[i] = 7.0f; // all want to leave at 7:00 AM
    }

    { int dr=0; for (uint32_t i=0;i<activeCount;i++) if (state_[i]==State::DRIVING) dr++;
      printf("Cars driving at start: %d (all parked, waiting for schedule)\n", dr); }

    constexpr float DT = 1.0f / 60.0f;
    constexpr float DAY_DUR = 86400.0f;

    // ---- WARM-UP at 1x for 30s to let early-scheduled cars activate ----
    constexpr int WARMUP_FRAMES = 30 * 60;
    float warmupTime = 0.0f;
    float timeOfDay = 7.0f / 24.0f; // Start at 7:00 AM (peak departure hour)

    printf("\n=== WARM-UP: %d frames (%.0f seconds at 1x) from hour %.1f ===\n",
           WARMUP_FRAMES, WARMUP_FRAMES * DT, timeOfDay * 24.0f);
    
    for (int f = 0; f < WARMUP_FRAMES; f++) {
        warmupTime += DT;
        timeOfDay += DT / DAY_DUR;
        if (timeOfDay >= 1.0f) timeOfDay -= 1.0f;
        UpdateLights(DT);
        Update(DT, f, timeOfDay);
        if (f % 600 == 0) {
            int dr=0; for (uint32_t i=0;i<activeCount;i++) if (state_[i]==State::DRIVING) dr++;
            printf("  Warmup f=%d h=%.2f driving=%d cols=%zu\n",
                   f, timeOfDay*24.0f, dr, collisions.size());
        }
    }
    int warmupCollisions = (int)collisions.size();
    { int dr=0; for (uint32_t i=0;i<activeCount;i++) if (state_[i]==State::DRIVING) dr++;
      printf("After warm-up: %d driving, hour=%.2f, warmupCols=%d\n",
             dr, timeOfDay*24.0f, warmupCollisions); }

    collisions.clear();

    // ---- REAL TEST: 100x for 30 real seconds ----
    constexpr float BIG_DT = DT * 100.0f;
    constexpr int TEST_FRAMES = 30 * 60;
    float simTime = 0.0f;
    int colFrames = 0;

    printf("\n=== TEST: 100x for 30 real seconds (%d frames, dt=%.3fs) ===\n",
           TEST_FRAMES, BIG_DT);
    printf("Starting at hour=%.2f with %u total cars\n", timeOfDay*24.0f, activeCount);

    int TOTAL_FRAMES = TEST_FRAMES;
    // Gridlock tracking
    int totalStoppedSamples = 0, totalDrivingSamples = 0;
    float totalSpeedSum = 0.0f;
    int gridlockFrames = 0; // frames where >50% of driving cars are stopped

    for (int f = 0; f < TEST_FRAMES; f++) {
        simTime += BIG_DT;
        timeOfDay += BIG_DT / DAY_DUR;
        if (timeOfDay >= 1.0f) timeOfDay -= 1.0f;
        UpdateLights(BIG_DT);
        size_t colBefore = collisions.size();
        Update(BIG_DT, f, timeOfDay);
        if (collisions.size() > colBefore) colFrames++;

        // Sample every 60 frames (~1 real second)
        if (f % 60 == 0) {
            int driving=0, stopped=0, parked=0;
            float spdSum=0;
            for (uint32_t i=0;i<activeCount;i++) {
                if (state_[i]==State::PARKED) { parked++; continue; }
                driving++;
                spdSum += speed_[i];
                if (speed_[i] < 0.5f) stopped++;
            }
            totalDrivingSamples += driving;
            totalStoppedSamples += stopped;
            totalSpeedSum += spdSum;
            if (driving > 0 && stopped > driving/2) gridlockFrames++;
        }

        if (f % 300 == 0) {
            int driving=0, stopped=0, parked=0;
            float spdSum=0;
            for (uint32_t i=0;i<activeCount;i++) {
                if (state_[i]==State::PARKED) { parked++; continue; }
                driving++;
                spdSum += speed_[i];
                if (speed_[i] < 0.5f) stopped++;
            }
            float avgSpd = driving > 0 ? spdSum/driving : 0;
            printf("  Frame %d/%d (sim %.0fs h=%.2f) driving=%d stopped=%d parked=%d avgSpd=%.1f cols=%zu\n",
                   f, TEST_FRAMES, simTime, timeOfDay*24.0f, driving, stopped, parked, avgSpd, collisions.size());
        }
    }

    // ============================================================
    //  ANALYSIS
    // ============================================================
    printf("\n========================================\n");
    printf("  COLLISION REPORT — BIG CITY\n");
    printf("========================================\n");
    printf("Total frames: %d (%.1f sim-seconds at 100x, dt=%.3fs)\n", TOTAL_FRAMES, simTime, BIG_DT);
    printf("Active cars: %u\n", activeCount);
    printf("Total collisions: %zu\n", collisions.size());
    printf("Frames with collisions: %d / %d (%.1f%%)\n",
           colFrames, TOTAL_FRAMES, 100.0f*colFrames/TOTAL_FRAMES);

    // By type
    std::unordered_map<std::string,int> typeCount;
    for (auto& c : collisions) typeCount[c.type]++;
    printf("\nBy type:\n");
    for (auto& [t,n] : typeCount) printf("  %-25s %d\n", t.c_str(), n);

    // By location
    std::unordered_map<int,int> cellCols;
    for (auto& c : collisions) cellCols[c.cellGZ*GS+c.cellGX]++;
    std::vector<std::pair<int,int>> sorted(cellCols.begin(), cellCols.end());
    std::sort(sorted.begin(), sorted.end(), [](auto& a, auto& b){return a.second>b.second;});
    printf("\nTop 15 collision cells:\n");
    for (int ci=0;ci<std::min(15,(int)sorted.size());ci++) {
        int key=sorted[ci].first;
        int gx=key%GS, gz=key/GS;
        const char* ct = IsIntersection(gx,gz)?"INTERSECT":(IsRoadLike(gx,gz)?"ROAD":"OTHER");
        printf("  Cell (%d,%d) %-10s lanes=%d : %d collisions\n",
               gx, gz, ct, GetLanes(gx,gz), sorted[ci].second);
    }

    // Sample collisions at different times
    printf("\nSample collisions (first 20):\n");
    for (int ci=0;ci<std::min(20,(int)collisions.size());ci++) {
        auto& c = collisions[ci];
        printf("  f=%6d car%03u(h=%.2f ln=%.1f spd=%.1f) vs car%03u(h=%.2f ln=%.1f spd=%.1f)"
               " d=%.2f hcos=%.2f (%d,%d) [%s]\n",
               c.frame, c.carA, c.hdgA, c.laneA, speed_[c.carA],
               c.carB, c.hdgB, c.laneB, speed_[c.carB],
               c.dist, c.hcos, c.cellGX, c.cellGZ, c.type.c_str());
    }

    // Time distribution
    int early=0,mid=0,late=0;
    int earlyF = TOTAL_FRAMES/10, midF = TOTAL_FRAMES/2;
    for (auto& c : collisions) {
        if (c.frame<earlyF) early++;
        else if (c.frame<midF) mid++;
        else late++;
    }
    printf("\nTime distribution:\n");
    printf("  Early (0-10%%):  %d\n  Mid (10-50%%):   %d\n  Late (50-100%%): %d\n", early, mid, late);

    // Unique pairs
    std::unordered_map<uint64_t,int> pairCount;
    for (auto& c : collisions) {
        uint32_t lo=std::min(c.carA,c.carB), hi=std::max(c.carA,c.carB);
        pairCount[((uint64_t)lo<<32)|hi]++;
    }
    printf("\nUnique colliding pairs: %zu\n", pairCount.size());
    int persistent=0;
    for (auto& [p,n] : pairCount) if (n>10) persistent++;
    printf("Persistent (>10 frames): %d\n", persistent);

    // Final states
    int moving=0,stopped=0,parked=0;
    for (uint32_t i=0;i<activeCount;i++) {
        if (state_[i]==State::PARKED) parked++;
        else if (speed_[i]<0.1f) stopped++;
        else moving++;
    }
    printf("\nFinal: %d moving, %d stopped, %d parked\n", moving, stopped, parked);

    // ---- GRIDLOCK ANALYSIS ----
    printf("\n========================================\n");
    printf("  GRIDLOCK ANALYSIS\n");
    printf("========================================\n");
    float avgStopped = totalDrivingSamples > 0 ? 100.0f * totalStoppedSamples / totalDrivingSamples : 0;
    float avgSpeed = totalDrivingSamples > 0 ? totalSpeedSum / totalDrivingSamples : 0;
    printf("Average %% of driving cars stopped: %.1f%%\n", avgStopped);
    printf("Average speed of driving cars: %.2f m/s (max=%.1f)\n", avgSpeed, MAX_SPEED);
    printf("Gridlock frames (>50%% stopped): %d / %d (%.1f%%)\n",
           gridlockFrames, TOTAL_FRAMES/60, 100.0f*gridlockFrames/(TOTAL_FRAMES/60));

    // Per-intersection analysis: count cars stuck near each intersection
    printf("\nPer-intersection jam (cars stopped within 1 cell):\n");
    struct IntJam { int gx, gz; int stopped; int total; };
    std::vector<IntJam> intJams;
    for (int z=0;z<GS;z++)
    for (int x=0;x<GS;x++) {
        if (!isIntersection[z][x]) continue;
        Vec2 ic = CellCenter(x,z);
        int nearStopped=0, nearTotal=0;
        for (uint32_t i=0;i<activeCount;i++) {
            if (state_[i]==State::PARKED) continue;
            float dx=posX[i]-ic.x, dz=posZ[i]-ic.y;
            if (dx*dx+dz*dz < (CS*2)*(CS*2)) {
                nearTotal++;
                if (speed_[i] < 0.5f) nearStopped++;
            }
        }
        if (nearTotal > 0)
            intJams.push_back({x, z, nearStopped, nearTotal});
    }
    std::sort(intJams.begin(), intJams.end(), [](auto& a, auto& b){return a.stopped>b.stopped;});
    for (int j=0;j<std::min(20,(int)intJams.size());j++) {
        auto& ij = intJams[j];
        Phase ph = intPhase[ij.gz][ij.gx];
        const char* phName[] = {"NS_THR","NS_YEL","NS_LEFT","ALL_RED","EW_THR","EW_YEL","EW_LEFT","ALL_RED"};
        printf("  Int (%d,%d) lanes=%d phase=%s: %d stopped / %d near\n",
               ij.gx, ij.gz, GetLanes(ij.gx,ij.gz), phName[ph], ij.stopped, ij.total);
    }

    // Per-road-cell jam analysis
    printf("\nTop 20 jammed road cells:\n");
    struct CellJam { int gx, gz; int stopped; int total; };
    std::vector<CellJam> cellJams;
    for (int z=0;z<GS;z++)
    for (int x=0;x<GS;x++) {
        if (!IsRoadLike(x,z)) continue;
        int key=z*GS+x;
        int cs=0, ct=0;
        for (uint32_t j=driveHead[key];j!=UINT32_MAX;j=driveNext[j]) {
            ct++;
            if (speed_[j]<0.5f) cs++;
        }
        if (ct>0) cellJams.push_back({x,z,cs,ct});
    }
    std::sort(cellJams.begin(), cellJams.end(), [](auto& a, auto& b){return a.stopped>b.stopped;});
    for (int j=0;j<std::min(20,(int)cellJams.size());j++) {
        auto& cj = cellJams[j];
        const char* ct = isIntersection[cj.gz][cj.gx]?"INT":"ROAD";
        printf("  Cell (%d,%d) %-4s lanes=%d: %d stopped / %d total\n",
               cj.gx, cj.gz, ct, GetLanes(cj.gx,cj.gz), cj.stopped, cj.total);
    }

    // Check for deadlock pattern: 4 directions all blocked at same intersection
    printf("\nDeadlock check (all 4 approaches blocked):\n");
    int deadlockCount = 0;
    for (int z=0;z<GS;z++)
    for (int x=0;x<GS;x++) {
        if (!isIntersection[z][x]) continue;
        // Check N,S,E,W adjacent cells for stopped cars
        const int ddx[]={1,-1,0,0}, ddz[]={0,0,1,-1};
        const char* dirName[]={"E","W","S","N"};
        int blockedDirs=0;
        for (int d=0;d<4;d++) {
            int nx=x+ddx[d], nz=z+ddz[d];
            if (!InBounds(nx,nz)) continue;
            if (!IsRoadLike(nx,nz)) continue;
            int nkey=nz*GS+nx;
            bool hasBlocked=false;
            for (uint32_t j=driveHead[nkey];j!=UINT32_MAX;j=driveNext[j]) {
                if (speed_[j]<0.5f) { hasBlocked=true; break; }
            }
            if (hasBlocked) blockedDirs++;
        }
        if (blockedDirs >= 3) {
            Phase ph = intPhase[z][x];
            const char* phName[] = {"NS_THR","NS_YEL","NS_LEFT","ALL_RED","EW_THR","EW_YEL","EW_LEFT","ALL_RED"};
            printf("  DEADLOCK at (%d,%d) lanes=%d phase=%s blocked_dirs=%d\n",
                   x, z, GetLanes(x,z), phName[ph], blockedDirs);
            deadlockCount++;
        }
    }
    if (deadlockCount == 0) printf("  No deadlocks detected\n");

    if (collisions.empty())
        printf("\n*** ZERO COLLISIONS ***\n");
    else
        printf("\n*** %zu COLLISIONS — investigation needed ***\n", collisions.size());

    return 0;
}
