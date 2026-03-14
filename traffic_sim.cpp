// traffic_sim.cpp — Standalone traffic simulation matching CrowdSim exactly
// Reads save_10.city, runs at 100x for 2 game-minutes, outputs diagnostics
// Build: cl /O2 /EHsc /std:c++17 traffic_sim.cpp /Fe:traffic_sim.exe

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <queue>
#include <algorithm>
#include <fstream>
#include <array>
#include <numeric>

static constexpr float PI  = 3.14159265358979f;
static constexpr float PI2 = 6.28318530717959f;

// ============================================================
//  CityLayout (minimal replica)
// ============================================================
static constexpr int   GS       = 40;
static constexpr float CELL_SIZE = 20.0f;
static constexpr float HALF_WORLD = GS * CELL_SIZE * 0.5f; // 400
static constexpr float HALF_CS   = CELL_SIZE * 0.5f;
static constexpr float SIDEWALK_W = 2.0f;

enum class CellType : uint8_t {
    EMPTY=0, ROAD, HOUSE, WORKPLACE, CROSSWALK, PARKING, LAKE,
    POWER_PLANT, WATER_PUMP, POLICE, FIRE_STATION, HOSPITAL,
    BUS_DEPOT, BUS_STOP
};

static constexpr int PARKING_FLOORS = 4;
static constexpr int PARKING_SPOTS_PER_FLOOR = 6;

struct City {
    CellType cells[GS * GS];
    int      housePop[GS * GS];
    int      roadLanes[GS * GS];
    int      parkOcc[GS * GS][PARKING_FLOORS];
    int      totalParkOcc[GS * GS]; // total occupied across floors

    bool InBounds(int x, int z) const { return x>=0 && x<GS && z>=0 && z<GS; }

    CellType GetCellType(int x, int z) const {
        if (!InBounds(x,z)) return CellType::EMPTY;
        return cells[z*GS+x];
    }
    int GetRoadLanes(int x, int z) const {
        if (!InBounds(x,z)) return 2;
        return roadLanes[z*GS+x];
    }
    bool IsRoadLike(int x, int z) const {
        auto ct = GetCellType(x,z);
        return ct==CellType::ROAD || ct==CellType::CROSSWALK || ct==CellType::BUS_STOP;
    }

    struct Vec2 { float x, y; };

    Vec2 GridCellCenter(int gx, int gz) const {
        return { -HALF_WORLD + (gx + 0.5f) * CELL_SIZE,
                 -HALF_WORLD + (gz + 0.5f) * CELL_SIZE };
    }
    bool WorldToGrid(float wx, float wz, int& gx, int& gz) const {
        gx = (int)std::floor((wx + HALF_WORLD) / CELL_SIZE);
        gz = (int)std::floor((wz + HALF_WORLD) / CELL_SIZE);
        if (gx < 0) gx = 0; if (gx >= GS) gx = GS-1;
        if (gz < 0) gz = 0; if (gz >= GS) gz = GS-1;
        return true;
    }

    bool ParkingHasRoom(int gx, int gz) const {
        if (!InBounds(gx,gz)) return false;
        if (cells[gz*GS+gx] != CellType::PARKING) return false;
        return totalParkOcc[gz*GS+gx] < PARKING_FLOORS * PARKING_SPOTS_PER_FLOOR;
    }
    bool ParkingAdjacentRoad(int pgx, int pgz, int& rx, int& rz) const {
        static const int dx[]={1,-1,0,0}, dz[]={0,0,1,-1};
        for (int d = 0; d < 4; ++d) {
            int nx=pgx+dx[d], nz=pgz+dz[d];
            if (InBounds(nx,nz) && IsRoadLike(nx,nz)) { rx=nx; rz=nz; return true; }
        }
        return false;
    }

    // A* pathfinding between road cells
    std::vector<Vec2> FindPathRoad(int sx, int sz, int dx, int dz) const {
        if (!InBounds(sx,sz) || !InBounds(dx,dz)) return {};
        auto isRoad = [&](int x, int z) {
            auto ct = GetCellType(x,z);
            return ct==CellType::ROAD || ct==CellType::CROSSWALK || ct==CellType::BUS_STOP || ct==CellType::BUS_DEPOT;
        };
        if (!isRoad(sx,sz) || !isRoad(dx,dz)) return {};

        int srcId = sz*GS+sx, dstId = dz*GS+dx;
        std::vector<float> dist(GS*GS, 1e9f);
        std::vector<int>   prev(GS*GS, -1);
        dist[srcId] = 0;

        using PQ = std::pair<float,int>;
        std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;
        pq.push({0, srcId});

        static const int ddx[]={1,-1,0,0}, ddz[]={0,0,1,-1};
        while (!pq.empty()) {
            auto [d, id] = pq.top(); pq.pop();
            if (d > dist[id]) continue;
            if (id == dstId) break;
            int cx=id%GS, cz=id/GS;
            for (int nd=0; nd<4; ++nd) {
                int nx=cx+ddx[nd], nz=cz+ddz[nd];
                if (!InBounds(nx,nz)) continue;
                int nid=nz*GS+nx;
                if (!isRoad(nx,nz) && nid != dstId) continue;
                float cost = 1.0f;
                float nd2 = dist[id] + cost;
                if (nd2 < dist[nid]) {
                    dist[nid] = nd2;
                    prev[nid] = id;
                    pq.push({nd2, nid});
                }
            }
        }
        if (dist[dstId] > 1e8f) return {};

        std::vector<Vec2> path;
        for (int id=dstId; id>=0; id=prev[id])
            path.push_back(GridCellCenter(id%GS, id/GS));
        std::reverse(path.begin(), path.end());
        return path;
    }

    // FindPath: from building cell to building cell via adjacent roads
    std::vector<Vec2> FindPath(int sx, int sz, int dx, int dz) const {
        // Find adjacent road for source
        static const int ddx[]={1,-1,0,0}, ddz[]={0,0,1,-1};
        int srx=-1,srz=-1, drx=-1,drz=-1;
        for (int d=0;d<4;++d) {
            int nx=sx+ddx[d], nz=sz+ddz[d];
            if (InBounds(nx,nz) && IsRoadLike(nx,nz)) { srx=nx; srz=nz; break; }
        }
        for (int d=0;d<4;++d) {
            int nx=dx+ddx[d], nz=dz+ddz[d];
            if (InBounds(nx,nz) && IsRoadLike(nx,nz)) { drx=nx; drz=nz; break; }
        }
        if (srx<0 || drx<0) return {};
        return FindPathRoad(srx, srz, drx, drz);
    }

    bool Load(const char* path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return false;
        uint32_t magic=0, ver=0, gs=0;
        f.read((char*)&magic, 4);
        f.read((char*)&ver, 4);
        f.read((char*)&gs, 4);
        if (magic != 0x43495459 || gs != GS) return false;
        f.read((char*)cells, GS*GS);
        f.read((char*)housePop, GS*GS*sizeof(int));
        if (ver >= 2) f.read((char*)roadLanes, GS*GS*sizeof(int));
        else std::fill(roadLanes, roadLanes+GS*GS, 4);
        memset(parkOcc, 0, sizeof(parkOcc));
        memset(totalParkOcc, 0, sizeof(totalParkOcc));
        return f.good();
    }
};

// ============================================================
//  TrafficLightSystem (exact replica)
// ============================================================
// ============================================================
//  TrafficLights — ORIGINAL (4-phase: N, E, S, W each solo)
// ============================================================
struct TrafficLightsOriginal {
    static constexpr float GREEN_DUR  = 8.0f;
    static constexpr float YELLOW_DUR = 2.0f;
    static constexpr int   NUM_PHASES = 8;

    enum class Phase : uint8_t {
        N_GREEN=0, N_YELLOW=1, E_GREEN=2, E_YELLOW=3,
        S_GREEN=4, S_YELLOW=5, W_GREEN=6, W_YELLOW=7
    };
    enum class ApproachDir : uint8_t { N=0, E=1, S=2, W=3 };
    enum class TurnIntent : uint8_t { STRAIGHT, RIGHT, LEFT };
    enum class LightColor : uint8_t { GREEN, YELLOW, RED };

    bool  isIntersection[GS*GS] = {};
    Phase phase[GS*GS] = {};
    float timer[GS*GS] = {};

    void RebuildIntersections(const City& city) {
        static const int dx[]={1,-1,0,0}, dz[]={0,0,1,-1};
        for (int gz=0; gz<GS; ++gz)
        for (int gx=0; gx<GS; ++gx) {
            int key = gz*GS+gx;
            isIntersection[key] = false;
            auto ct = city.GetCellType(gx,gz);
            if (ct != CellType::ROAD && ct != CellType::CROSSWALK) continue;
            int rn = 0;
            for (int d=0;d<4;++d) {
                auto nct = city.GetCellType(gx+dx[d], gz+dz[d]);
                if (nct==CellType::ROAD || nct==CellType::CROSSWALK) ++rn;
            }
            if (rn >= 3) {
                isIntersection[key] = true;
                phase[key] = (Phase)((gx+gz) % NUM_PHASES);
                timer[key] = 0;
            }
        }
    }

    bool IsIntersection(int gx, int gz) const {
        if (gx<0||gx>=GS||gz<0||gz>=GS) return false;
        return isIntersection[gz*GS+gx];
    }

    void Update(float dt) {
        for (int key=0; key<GS*GS; ++key) {
            if (!isIntersection[key]) continue;
            timer[key] += dt;
            float dur = PhaseDuration(phase[key]);
            while (timer[key] >= dur) {
                timer[key] -= dur;
                phase[key] = (Phase)(((int)phase[key] + 1) % NUM_PHASES);
                dur = PhaseDuration(phase[key]);
            }
        }
    }

    static float PhaseDuration(Phase p) {
        return ((int)p & 1) ? YELLOW_DUR : GREEN_DUR;
    }

    static ApproachDir GetApproachDir(float dx, float dz) {
        if (std::abs(dz) >= std::abs(dx))
            return (dz > 0.f) ? ApproachDir::N : ApproachDir::S;
        else
            return (dx > 0.f) ? ApproachDir::W : ApproachDir::E;
    }

    LightColor GetLight(int gx, int gz, float dx, float dz, TurnIntent intent = TurnIntent::STRAIGHT) const {
        if (gx<0||gx>=GS||gz<0||gz>=GS) return LightColor::RED;
        int key = gz*GS+gx;
        if (!isIntersection[key]) return LightColor::GREEN;

        ApproachDir ad = GetApproachDir(dx, dz);
        Phase ph = phase[key];
        int phIdx = (int)ph;
        bool isYellow = (phIdx & 1) != 0;
        int greenIdx = isYellow ? (phIdx - 1) : phIdx;

        ApproachDir dominant  = (ApproachDir)(greenIdx / 2);
        ApproachDir companion = (ApproachDir)(((int)dominant + 3) % 4);

        if (ad == dominant)
            return isYellow ? LightColor::YELLOW : LightColor::GREEN;
        if (ad == companion && intent == TurnIntent::RIGHT)
            return isYellow ? LightColor::YELLOW : LightColor::GREEN;
        return LightColor::RED;
    }
};

// ============================================================
//  TrafficLights — FIXED (lead-lag protected-permissive left)
//  6 phases: NS_LEFT_LEAD, NS_LEFT_Y, NS_THRU+LEFT_PERM, NS_THRU_Y,
//            EW_LEFT_LEAD, EW_LEFT_Y, EW_THRU+LEFT_PERM, EW_THRU_Y
//  Left gets a 3s head start (protected), then through+left share green.
//  Cycle = (3+2+10+2)*2 = 34s
// ============================================================
struct TrafficLights {
    static constexpr float LEFT_LEAD_DUR = 3.0f;    // protected left arrow head start
    static constexpr float THRU_GREEN_DUR = 10.0f;  // through + permissive left
    static constexpr float YELLOW_DUR     = 2.0f;
    static constexpr int   NUM_PHASES     = 8;

    enum class Phase : uint8_t {
        NS_LEFT_LEAD=0, NS_LEFT_YELLOW=1,   // only left-turners from NS
        NS_THRU_GREEN=2, NS_THRU_YELLOW=3,  // through + right + permissive left from NS
        EW_LEFT_LEAD=4, EW_LEFT_YELLOW=5,
        EW_THRU_GREEN=6, EW_THRU_YELLOW=7
    };
    enum class ApproachDir : uint8_t { N=0, E=1, S=2, W=3 };
    enum class TurnIntent : uint8_t { STRAIGHT, RIGHT, LEFT };
    enum class LightColor : uint8_t { GREEN, YELLOW, RED };

    bool  isIntersection[GS*GS] = {};
    Phase phase[GS*GS] = {};
    float timer[GS*GS] = {};

    void RebuildIntersections(const City& city) {
        static const int dx[]={1,-1,0,0}, dz[]={0,0,1,-1};
        for (int gz=0; gz<GS; ++gz)
        for (int gx=0; gx<GS; ++gx) {
            int key = gz*GS+gx;
            isIntersection[key] = false;
            auto ct = city.GetCellType(gx,gz);
            if (ct != CellType::ROAD && ct != CellType::CROSSWALK) continue;
            int rn = 0;
            for (int d=0;d<4;++d) {
                auto nct = city.GetCellType(gx+dx[d], gz+dz[d]);
                if (nct==CellType::ROAD || nct==CellType::CROSSWALK) ++rn;
            }
            if (rn >= 3) {
                isIntersection[key] = true;
                phase[key] = (Phase)((gx+gz) % NUM_PHASES);
                timer[key] = 0;
            }
        }
    }

    bool IsIntersection(int gx, int gz) const {
        if (gx<0||gx>=GS||gz<0||gz>=GS) return false;
        return isIntersection[gz*GS+gx];
    }

    void Update(float dt) {
        for (int key=0; key<GS*GS; ++key) {
            if (!isIntersection[key]) continue;
            timer[key] += dt;
            float dur = PhaseDuration(phase[key]);
            while (timer[key] >= dur) {
                timer[key] -= dur;
                phase[key] = (Phase)(((int)phase[key] + 1) % NUM_PHASES);
                dur = PhaseDuration(phase[key]);
            }
        }
    }

    static float PhaseDuration(Phase p) {
        int idx = (int)p;
        if (idx & 1) return YELLOW_DUR;
        return (idx == 0 || idx == 4) ? LEFT_LEAD_DUR : THRU_GREEN_DUR;
    }

    static ApproachDir GetApproachDir(float dx, float dz) {
        if (std::abs(dz) >= std::abs(dx))
            return (dz > 0.f) ? ApproachDir::N : ApproachDir::S;
        else
            return (dx > 0.f) ? ApproachDir::W : ApproachDir::E;
    }

    // Lead-lag protected-permissive:
    //   LEFT_LEAD: only left-turners get green (protected, no oncoming)
    //   THRU_GREEN: through + right + left all get green (permissive left)
    //   Right-on-red: always allowed
    LightColor GetLight(int gx, int gz, float dx, float dz, TurnIntent intent = TurnIntent::STRAIGHT) const {
        if (gx<0||gx>=GS||gz<0||gz>=GS) return LightColor::RED;
        int key = gz*GS+gx;
        if (!isIntersection[key]) return LightColor::GREEN;

        ApproachDir ad = GetApproachDir(dx, dz);
        Phase ph = phase[key];

        bool isNS = (ad == ApproachDir::N || ad == ApproachDir::S);
        bool isEW = (ad == ApproachDir::E || ad == ApproachDir::W);

        // Right-on-red: always allow right turns
        if (intent == TurnIntent::RIGHT) return LightColor::GREEN;

        switch (ph) {
            // --- NS left-turn lead (protected: only left gets green) ---
            case Phase::NS_LEFT_LEAD:
                if (isNS && intent == TurnIntent::LEFT) return LightColor::GREEN;
                return LightColor::RED;
            case Phase::NS_LEFT_YELLOW:
                if (isNS && intent == TurnIntent::LEFT) return LightColor::YELLOW;
                return LightColor::RED;

            // --- NS through (all NS movements: through + left permissive) ---
            case Phase::NS_THRU_GREEN:
                if (isNS) return LightColor::GREEN;  // all NS directions go
                return LightColor::RED;
            case Phase::NS_THRU_YELLOW:
                if (isNS) return LightColor::YELLOW;
                return LightColor::RED;

            // --- EW left-turn lead ---
            case Phase::EW_LEFT_LEAD:
                if (isEW && intent == TurnIntent::LEFT) return LightColor::GREEN;
                return LightColor::RED;
            case Phase::EW_LEFT_YELLOW:
                if (isEW && intent == TurnIntent::LEFT) return LightColor::YELLOW;
                return LightColor::RED;

            // --- EW through (all EW movements) ---
            case Phase::EW_THRU_GREEN:
                if (isEW) return LightColor::GREEN;
                return LightColor::RED;
            case Phase::EW_THRU_YELLOW:
                if (isEW) return LightColor::YELLOW;
                return LightColor::RED;
        }
        return LightColor::RED;
    }
};

// ============================================================
//  CarSystem (exact replica of driving physics)
// ============================================================
static constexpr uint32_t MAX_CARS  = 10000;
static constexpr uint8_t  MAX_WP    = 64;
static constexpr float MAX_SPEED    = 14.0f;
static constexpr float ACCEL        =  6.0f;
static constexpr float DECEL        = 14.0f;
static constexpr float PARK_PAUSE   =  2.0f;
static constexpr float CAR_HL       =  1.0f;
static constexpr float CAR_HW       =  0.45f;
static constexpr float MIN_SEP      =  CAR_HL * 2.f + 2.5f;  // FIX: wider min separation (was +1.5)
static constexpr float LAT_BAND     =  1.5f;
static constexpr float IDM_S0       =  3.0f;   // FIX: bigger jam distance (was 2.0)
static constexpr float IDM_T        =  1.5f;
static constexpr float IDM_B        =  3.0f;
static constexpr float JOYRIDE_CHANCE = 0.15f;
static constexpr float WANDER_HOME_CHANCE = 0.30f;

using Vec2 = City::Vec2;

static Vec2 DirTo(Vec2 a, Vec2 b) {
    float dx = b.x - a.x, dz = b.y - a.y;
    float len = std::sqrt(dx*dx + dz*dz);
    if (len < 1e-6f) return {0, 1};
    return {dx/len, dz/len};
}

static void GetLaneOffsets(int totalLanes, float* offsets, int& count) {
    if (totalLanes >= 6) { count=3; offsets[0]=1.33f; offsets[1]=4.0f; offsets[2]=6.67f; }
    else if (totalLanes >= 4) { count=2; offsets[0]=2.0f; offsets[1]=6.0f; }
    else { count=1; offsets[0]=4.0f; }
}

enum class State : uint8_t { PARKED, DRIVING };

struct CarSys {
    uint32_t activeCount = 0;

    // Per-car arrays
    std::vector<float>    posX, posZ, speed, heading;
    std::vector<Vec2>     wpBuf;   // MAX_WP per car
    std::vector<uint8_t>  wpCount, wpCurr;
    std::vector<State>    state;
    std::vector<float>    parkTimer;
    std::vector<uint32_t> parkCell;
    std::vector<uint8_t>  myParkSlot;
    std::vector<uint8_t>  carDir;  // 0=to work, 1=to home
    std::vector<float>    halfLen;
    std::vector<float>    laneOff, laneTarget;
    std::vector<int8_t>   laneIdx;
    std::vector<float>    schedDepart, schedReturn;
    std::vector<uint32_t> lastDayTrip;
    std::vector<bool>     wandering;

    // Spatial hash for driving cars
    std::vector<uint32_t> driveHead, driveNext;

    // Counters
    uint32_t dayCounter = 0;
    float    prevTimeOfDay = 0;

    void Initialize() {
        posX.resize(MAX_CARS, 0); posZ.resize(MAX_CARS, 0);
        speed.resize(MAX_CARS, 0); heading.resize(MAX_CARS, 0);
        wpBuf.resize((size_t)MAX_CARS * MAX_WP);
        wpCount.resize(MAX_CARS, 0); wpCurr.resize(MAX_CARS, 0);
        state.resize(MAX_CARS, State::PARKED);
        parkTimer.resize(MAX_CARS, 0);
        parkCell.resize(MAX_CARS, UINT32_MAX);
        myParkSlot.resize(MAX_CARS, 0xFF);
        carDir.resize(MAX_CARS, 0);
        halfLen.resize(MAX_CARS, CAR_HL);
        laneOff.resize(MAX_CARS, 2.0f);
        laneTarget.resize(MAX_CARS, 2.0f);
        laneIdx.resize(MAX_CARS, 0);
        schedDepart.resize(MAX_CARS, 7.0f);
        schedReturn.resize(MAX_CARS, 17.0f);
        lastDayTrip.resize(MAX_CARS, 0);
        wandering.resize(MAX_CARS, false);
        driveHead.resize(GS*GS, UINT32_MAX);
        driveNext.resize(MAX_CARS, UINT32_MAX);
    }

    uint32_t SpawnCar(const std::vector<Vec2>& roadPath, uint32_t colorSeed, int spawnLanes = 6) {
        if (activeCount >= MAX_CARS || roadPath.size() < 2) return UINT32_MAX;
        float spawnX = roadPath[0].x, spawnZ = roadPath[0].y;
        for (uint32_t j = 0; j < activeCount; ++j) {
            if (state[j] != State::DRIVING) continue;
            float dx = posX[j]-spawnX, dz = posZ[j]-spawnZ;
            float sep = CAR_HL + halfLen[j] + 1.5f;
            if (dx*dx+dz*dz < sep*sep) return UINT32_MAX;
        }

        uint32_t i = activeCount++;
        uint8_t wc = (uint8_t)std::min(roadPath.size(), (size_t)MAX_WP);
        Vec2* wp = &wpBuf[(size_t)i * MAX_WP];
        for (uint8_t w=0;w<wc;++w) wp[w] = roadPath[w];
        wpCount[i] = wc;
        wpCurr[i] = 1;
        carDir[i] = 0;

        float lo[3]; int lc;
        GetLaneOffsets(spawnLanes, lo, lc);
        laneIdx[i] = (int8_t)(colorSeed % lc);
        laneOff[i] = lo[laneIdx[i]];
        laneTarget[i] = laneOff[i];

        Vec2 d0 = DirTo(wp[0], wp[1]);
        float spawnRX = d0.y, spawnRZ = -d0.x;
        posX[i] = wp[0].x + spawnRX * laneOff[i];
        posZ[i] = wp[0].y + spawnRZ * laneOff[i];
        speed[i] = 0;
        heading[i] = std::atan2(d0.x, d0.y);
        state[i] = State::PARKED;
        parkTimer[i] = 0.3f + (float)(colorSeed % 8) * 0.6f;
        parkCell[i] = UINT32_MAX;
        myParkSlot[i] = 0xFF;
        halfLen[i] = CAR_HL;
        wandering[i] = false;

        // Schedule
        uint32_t ws = colorSeed * 1103515245u + 12345u;
        float schedRand = (float)((ws >> 8) & 0xFF) / 255.f;
        float depart;
        if      (schedRand < 0.15f) depart = 5.0f + (schedRand/0.15f)*2.0f;
        else if (schedRand < 0.40f) depart = 7.0f + ((schedRand-0.15f)/0.25f)*2.0f;
        else if (schedRand < 0.55f) depart = 9.0f + ((schedRand-0.40f)/0.15f)*2.0f;
        else if (schedRand < 0.70f) depart = 11.0f + ((schedRand-0.55f)/0.15f)*2.0f;
        else if (schedRand < 0.85f) depart = 14.0f + ((schedRand-0.70f)/0.15f)*2.0f;
        else                        depart = 20.0f + ((schedRand-0.85f)/0.15f)*3.0f;
        uint32_t shiftSeed = (ws >> 16) & 0xFF;
        float shiftLen;
        if (shiftSeed < 64)       shiftLen = 4.0f + (shiftSeed/64.f)*2.0f;
        else if (shiftSeed < 128) shiftLen = 6.0f + ((shiftSeed-64)/64.f)*2.0f;
        else                      shiftLen = 8.0f + ((shiftSeed-128)/128.f)*2.0f;
        schedDepart[i] = depart;
        schedReturn[i] = std::fmod(depart + shiftLen, 24.0f);
        lastDayTrip[i] = 0;
        return i;
    }

    // The big update — mirrors CarSystem::Update exactly
    void Update(float dt, City& city, const TrafficLights& lights,
                float timeOfDay, float dayDuration)
    {
        constexpr float MAX_PHYSICS_DT = 0.05f;
        int subSteps = (dt > MAX_PHYSICS_DT) ? (int)std::ceil(dt / MAX_PHYSICS_DT) : 1;
        float subDt = dt / (float)subSteps;

        for (int sub = 0; sub < subSteps; ++sub) {
            if (sub == 0) {
                if (timeOfDay < prevTimeOfDay) ++dayCounter;
                prevTimeOfDay = timeOfDay;
            }
            float currentHour = timeOfDay * 24.0f;
            float ddt = subDt;

            // Rebuild spatial hash
            std::fill(driveHead.begin(), driveHead.end(), UINT32_MAX);
            std::fill(driveNext.begin(), driveNext.end(), UINT32_MAX);
            for (uint32_t i = 0; i < activeCount; ++i) {
                if (state[i] != State::DRIVING) continue;
                int gx, gz;
                if (city.WorldToGrid(posX[i], posZ[i], gx, gz)) {
                    int key = gz*GS+gx;
                    driveNext[i] = driveHead[key];
                    driveHead[key] = i;
                }
            }

            for (uint32_t i = 0; i < activeCount; ++i) {
                // ---- PARKED ----
                if (state[i] == State::PARKED) {
                    if (parkTimer[i] > 0) { parkTimer[i] -= ddt; continue; }

                    bool shouldDepart = wandering[i];
                    if (!shouldDepart && carDir[i] == 0) {
                        float dep = schedDepart[i];
                        float diff = currentHour - dep;
                        if (diff < 0) diff += 24.0f;
                        if (diff >= 0 && diff < 0.5f && lastDayTrip[i] != dayCounter*2+1) {
                            shouldDepart = true;
                            lastDayTrip[i] = dayCounter*2+1;
                        }
                        if (!shouldDepart && lastDayTrip[i] != dayCounter*2+1) {
                            uint32_t jHash = (uint32_t)(i * 2654435761u + dayCounter*31u + (uint32_t)(currentHour*100.f));
                            float jRand = (float)(jHash & 0xFFFF) / 65535.f;
                            float hourFrac = ddt / (dayDuration / 24.0f);
                            if (jRand < JOYRIDE_CHANCE * hourFrac) {
                                shouldDepart = true;
                                wandering[i] = true;
                            }
                        }
                    } else if (!shouldDepart && carDir[i] == 1) {
                        float ret = schedReturn[i];
                        float diff = currentHour - ret;
                        if (diff < 0) diff += 24.0f;
                        if (diff >= 0 && diff < 0.5f && lastDayTrip[i] != dayCounter*2+2) {
                            shouldDepart = true;
                            lastDayTrip[i] = dayCounter*2+2;
                        }
                    }
                    if (!shouldDepart) continue;

                    // Proximity check
                    {
                        bool blocked = false;
                        int cgx, cgz;
                        if (city.WorldToGrid(posX[i], posZ[i], cgx, cgz)) {
                            for (int dg=-1; dg<=1 && !blocked; ++dg)
                            for (int dh=-1; dh<=1 && !blocked; ++dh) {
                                int nx=cgx+dh, nz=cgz+dg;
                                if (nx<0||nx>=GS||nz<0||nz>=GS) continue;
                                int nkey=nz*GS+nx;
                                for (uint32_t j=driveHead[nkey]; j!=UINT32_MAX; j=driveNext[j]) {
                                    float ddx=posX[j]-posX[i], ddz=posZ[j]-posZ[i];
                                    float sep2=halfLen[i]+halfLen[j]+1.5f;
                                    if (ddx*ddx+ddz*ddz < sep2*sep2) { blocked=true; break; }
                                }
                            }
                        }
                        if (blocked) { parkTimer[i] = 0.5f; continue; }
                    }

                    // Release parking
                    if (parkCell[i] != UINT32_MAX && myParkSlot[i] != 0xFF) {
                        int key = (int)parkCell[i];
                        int pgx=key%GS, pgz=key/GS;
                        if (city.GetCellType(pgx,pgz)==CellType::PARKING) {
                            int sl=(int)myParkSlot[i];
                            int fl=sl/PARKING_SPOTS_PER_FLOOR;
                            if (fl>=0 && fl<PARKING_FLOORS)
                                city.parkOcc[key][fl] = std::max(0, city.parkOcc[key][fl]-1);
                            city.totalParkOcc[key] = std::max(0, city.totalParkOcc[key]-1);

                            int rx, rz;
                            if (city.ParkingAdjacentRoad(pgx,pgz,rx,rz)) {
                                Vec2 roadC = city.GridCellCenter(rx,rz);
                                Vec2 parkC = city.GridCellCenter(pgx,pgz);
                                float dirX=parkC.x-roadC.x, dirZ=parkC.y-roadC.y;
                                float len=std::sqrt(dirX*dirX+dirZ*dirZ);
                                if (len>0.1f) { dirX/=len; dirZ/=len; }
                                posX[i]=roadC.x+dirX*3.f;
                                posZ[i]=roadC.y+dirZ*3.f;
                            }
                        }
                    }
                    parkCell[i] = UINT32_MAX;
                    myParkSlot[i] = 0xFF;

                    // Re-route
                    Vec2* wp = &wpBuf[(size_t)i*MAX_WP];
                    uint8_t wc = wpCount[i];
                    int srcGX, srcGZ;
                    bool haveSrc = city.WorldToGrid(posX[i], posZ[i], srcGX, srcGZ);
                    bool rerouted = false;

                    // Wandering
                    if (wandering[i] && haveSrc) {
                        uint32_t rh = (uint32_t)(i*2654435761u + dayCounter*997u + (uint32_t)(currentHour*1000.f));
                        for (int attempt=0; attempt<20; ++attempt) {
                            rh = rh*1664525u+1013904223u;
                            int rx=(int)(rh%GS);
                            rh = rh*1664525u+1013904223u;
                            int rz=(int)(rh%GS);
                            auto ct = city.GetCellType(rx,rz);
                            if (ct!=CellType::ROAD && ct!=CellType::BUS_STOP) continue;
                            if (std::abs(rx-srcGX)+std::abs(rz-srcGZ)<5) continue;
                            auto newPath = city.FindPathRoad(srcGX,srcGZ,rx,rz);
                            if (newPath.size()>=2) {
                                uint8_t nwc=(uint8_t)std::min(newPath.size(),(size_t)MAX_WP);
                                for (uint8_t w=0;w<nwc;++w) wp[w]=newPath[w];
                                wpCount[i]=nwc;
                                rerouted=true; break;
                            }
                        }
                    }

                    if (!rerouted && haveSrc) {
                        float d0sq = (posX[i]-wp[0].x)*(posX[i]-wp[0].x)+(posZ[i]-wp[0].y)*(posZ[i]-wp[0].y);
                        float dNsq = (posX[i]-wp[wc-1].x)*(posX[i]-wp[wc-1].x)+(posZ[i]-wp[wc-1].y)*(posZ[i]-wp[wc-1].y);
                        bool nearStart = (d0sq <= dNsq);
                        Vec2 dstPt = nearStart ? wp[wc-1] : wp[0];
                        int dstGX, dstGZ;
                        bool haveDst = city.WorldToGrid(dstPt.x, dstPt.y, dstGX, dstGZ);
                        if (haveDst && !(srcGX==dstGX && srcGZ==dstGZ)) {
                            auto newPath = city.FindPathRoad(srcGX,srcGZ,dstGX,dstGZ);
                            if (newPath.size()>=2) {
                                uint8_t nwc=(uint8_t)std::min(newPath.size(),(size_t)MAX_WP);
                                for (uint8_t w=0;w<nwc;++w) wp[w]=newPath[w];
                                wpCount[i]=nwc;
                                rerouted=true;
                            }
                        }
                        if (!rerouted && !nearStart) {
                            for (uint8_t a=0, b=wc-1; a<b; ++a,--b) std::swap(wp[a],wp[b]);
                        }
                    }

                    wpCurr[i] = 1;
                    carDir[i] ^= 1;
                    wc = wpCount[i];
                    if (wc >= 2) {
                        Vec2 d0 = DirTo(wp[0], wp[1]);
                        heading[i] = std::atan2(d0.x, d0.y);
                    }
                    speed[i] = 0;
                    state[i] = State::DRIVING;
                    {
                        int dgx, dgz;
                        if (city.WorldToGrid(posX[i],posZ[i],dgx,dgz)) {
                            int dkey=dgz*GS+dgx;
                            driveNext[i]=driveHead[dkey];
                            driveHead[dkey]=i;
                        }
                    }
                    continue;
                }

                // ---- DRIVING ----
                Vec2* wp = &wpBuf[(size_t)i*MAX_WP];
                uint8_t wc = wpCount[i];
                uint8_t& wn = wpCurr[i];

                // Reached last waypoint → park
                if (wn >= wc) {
                    bool parked = false;
                    constexpr int PARK_BLDG_RADIUS = 12;
                    int destGX, destGZ;
                    if (city.WorldToGrid(wp[wc-1].x, wp[wc-1].y, destGX, destGZ)) {
                        float bestDist2 = 1e9f;
                        int bestPGX=-1, bestPGZ=-1;
                        for (int dz=-PARK_BLDG_RADIUS; dz<=PARK_BLDG_RADIUS; ++dz)
                        for (int dx=-PARK_BLDG_RADIUS; dx<=PARK_BLDG_RADIUS; ++dx) {
                            int pgx=destGX+dx, pgz=destGZ+dz;
                            if (!city.ParkingHasRoom(pgx,pgz)) continue;
                            float d2=(float)(dx*dx+dz*dz);
                            if (d2<bestDist2) { bestDist2=d2; bestPGX=pgx; bestPGZ=pgz; }
                        }
                        if (bestPGX>=0) {
                            Vec2 pkC = city.GridCellCenter(bestPGX,bestPGZ);
                            int pidx=bestPGZ*GS+bestPGX;
                            int freeFl=0;
                            for (int f=0;f<PARKING_FLOORS;++f) {
                                if (city.parkOcc[pidx][f]<PARKING_SPOTS_PER_FLOOR) { freeFl=f; break; }
                            }
                            int slotN=city.parkOcc[pidx][freeFl]%PARKING_SPOTS_PER_FLOOR;
                            int col=slotN%3, row=slotN/3;
                            float xOff=(col-1)*5.0f, zOff=(row==0)?-3.0f:3.0f;

                            posX[i]=pkC.x+xOff; posZ[i]=pkC.y+zOff;
                            speed[i]=0; heading[i]=PI*0.5f;
                            state[i]=State::PARKED; parkTimer[i]=PARK_PAUSE;
                            city.parkOcc[pidx][freeFl]++;
                            city.totalParkOcc[pidx]++;
                            parkCell[i]=(uint32_t)pidx;
                            myParkSlot[i]=(uint8_t)(freeFl*PARKING_SPOTS_PER_FLOOR+slotN);
                            parked=true;

                            if (wandering[i]) {
                                parkTimer[i]=3.0f+(float)(i%5)*2.0f;
                                uint32_t wh=(uint32_t)(i*2654435761u+dayCounter*53u);
                                float goHome=(float)(wh&0xFFFF)/65535.f;
                                if (goHome<WANDER_HOME_CHANCE) wandering[i]=false;
                            }
                        }
                    }
                    if (!parked) {
                        for (uint8_t a=0, b=wc-1; a<b; ++a,--b) std::swap(wp[a],wp[b]);
                        wpCurr[i]=1; carDir[i]^=1;
                    }
                    continue;
                }

                // ---- Segment geometry ----
                Vec2 prev_raw = (wn>0) ? wp[wn-1] : wp[0];
                Vec2 target_raw = wp[wn];
                Vec2 segDir = DirTo(prev_raw, target_raw);
                float segLen = std::sqrt((target_raw.x-prev_raw.x)*(target_raw.x-prev_raw.x) +
                                         (target_raw.y-prev_raw.y)*(target_raw.y-prev_raw.y));

                float myFwdX = std::sin(heading[i]);
                float myFwdZ = std::cos(heading[i]);

                int myGX=0, myGZ=0;
                bool onGrid = city.WorldToGrid(posX[i], posZ[i], myGX, myGZ);

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
                {
                    int totalLanes = 6;
                    if (onGrid) {
                        totalLanes = city.GetRoadLanes(myGX, myGZ);
                        if (lights.IsIntersection(myGX, myGZ)) {
                            int stepX=(int)std::round(segDir.x);
                            int stepZ=(int)std::round(segDir.y);
                            for (int s=1;s<=3;++s) {
                                int bGX=myGX-stepX*s, bGZ=myGZ-stepZ*s;
                                auto bct=city.GetCellType(bGX,bGZ);
                                if (bct==CellType::ROAD && !lights.IsIntersection(bGX,bGZ)) {
                                    totalLanes=city.GetRoadLanes(bGX,bGZ); break;
                                }
                            }
                        }
                    }
                    GetLaneOffsets(totalLanes, cellLaneOffs, cellLaneCount);
                }

                if (laneIdx[i]>=cellLaneCount) laneIdx[i]=(int8_t)(cellLaneCount-1);
                if (laneIdx[i]<0) laneIdx[i]=0;
                laneTarget[i] = cellLaneOffs[laneIdx[i]];

                // Smooth lane transition
                {
                    float laneDiff = laneTarget[i] - laneOff[i];
                    float maxLat = 5.0f * ddt;
                    if (std::abs(laneDiff) > maxLat)
                        laneOff[i] += (laneDiff > 0 ? maxLat : -maxLat);
                    else
                        laneOff[i] = laneTarget[i];
                }

                bool inIntersection = onGrid && lights.IsIntersection(myGX, myGZ);
                bool laneSettled = std::abs(laneOff[i] - laneTarget[i]) < 0.1f;
                bool canChangeLane = !isTurn && !inIntersection && onGrid && laneSettled;

                // Helper: check if a target lane is clear of nearby same-direction cars
                auto isLaneClear = [&](float tgtLane) -> bool {
                    for (int dg=-1;dg<=1;++dg)
                    for (int dh2=-1;dh2<=1;++dh2) {
                        int cgx=myGX+dh2, cgz=myGZ+dg;
                        if (cgx<0||cgx>=GS||cgz<0||cgz>=GS) continue;
                        int cellKey=cgz*GS+cgx;
                        for (uint32_t j=driveHead[cellKey]; j!=UINT32_MAX; j=driveNext[j]) {
                            if (j==i) continue;
                            float jFwdX=std::sin(heading[j]);
                            float jFwdZ=std::cos(heading[j]);
                            float hcos=myFwdX*jFwdX+myFwdZ*jFwdZ;
                            if (hcos<0.5f) continue;
                            if (std::abs(laneOff[j]-tgtLane)>1.0f) continue;
                            float odx=posX[j]-posX[i], odz=posZ[j]-posZ[i];
                            float fwd2=odx*myFwdX+odz*myFwdZ;
                            if (fwd2>-8.f && fwd2<15.f) return false;
                        }
                    }
                    return true;
                };

                // ---- Turn lane preparation ----
                // Left turn → leftmost lane (0), right turn → rightmost lane (N-1),
                // straight → any lane (pick least busy)
                int8_t requiredIdx = laneIdx[i];
                bool turnPrepNeeded = false;
                float distToTurnWp = 1e6f;

                if (isTurn) {
                    requiredIdx = (turnCross > 0.0f) ? 0 : (int8_t)(cellLaneCount - 1);
                    turnPrepNeeded = true;
                    float tdx=target_raw.x-posX[i], tdz=target_raw.y-posZ[i];
                    distToTurnWp = std::sqrt(tdx*tdx+tdz*tdz);
                }

                if (!turnPrepNeeded) {
                    float tdx0=target_raw.x-posX[i], tdz0=target_raw.y-posZ[i];
                    float accumDist=std::sqrt(tdx0*tdx0+tdz0*tdz0);
                    for (uint8_t look=1; look<=8 && (wn+look)<wc; ++look) {
                        uint8_t futIdx=wn+look;
                        if (futIdx+1>=wc) break;
                        Vec2 fSeg=DirTo(wp[futIdx-1], wp[futIdx]);
                        Vec2 fNext=DirTo(wp[futIdx], wp[futIdx+1]);
                        float fCos=fSeg.x*fNext.x+fSeg.y*fNext.y;
                        float fCross2=fSeg.x*fNext.y-fSeg.y*fNext.x;
                        if (fCos<0.3f) {
                            requiredIdx=(fCross2>0.0f)?0:(int8_t)(cellLaneCount-1);
                            turnPrepNeeded=true;
                            distToTurnWp=accumDist;
                            break;
                        }
                        float sdx=wp[futIdx].x-wp[futIdx-1].x;
                        float sdz=wp[futIdx].y-wp[futIdx-1].y;
                        accumDist+=std::sqrt(sdx*sdx+sdz*sdz);
                    }
                }

                if (turnPrepNeeded && laneIdx[i]!=requiredIdx
                    && canChangeLane && isLaneClear(cellLaneOffs[requiredIdx])) {
                    laneIdx[i]=requiredIdx;
                    laneTarget[i]=cellLaneOffs[laneIdx[i]];
                }

                bool wrongLaneForTurn = turnPrepNeeded && (laneIdx[i] != requiredIdx);

                // Balance lane distribution: no turn ahead → prefer less crowded lane
                if (!turnPrepNeeded && canChangeLane && cellLaneCount > 1) {
                    int laneCounts[3]={0,0,0};
                    for (int dg=-1;dg<=1;++dg)
                    for (int dh2=-1;dh2<=1;++dh2) {
                        int cgx=myGX+dh2, cgz=myGZ+dg;
                        if (cgx<0||cgx>=GS||cgz<0||cgz>=GS) continue;
                        int cellKey=cgz*GS+cgx;
                        for (uint32_t j=driveHead[cellKey]; j!=UINT32_MAX; j=driveNext[j]) {
                            if (j==i) continue;
                            float jFwdX=std::sin(heading[j]), jFwdZ=std::cos(heading[j]);
                            float hcos=myFwdX*jFwdX+myFwdZ*jFwdZ;
                            if (hcos<0.5f) continue;
                            float odx2=posX[j]-posX[i], odz2=posZ[j]-posZ[i];
                            float fwd3=odx2*myFwdX+odz2*myFwdZ;
                            if (fwd3<0.f||fwd3>40.f) continue;
                            for (int l=0;l<cellLaneCount;++l) {
                                if (std::abs(laneOff[j]-cellLaneOffs[l])<1.5f)
                                { laneCounts[l]++; break; }
                            }
                        }
                    }
                    int myCount=laneCounts[laneIdx[i]];
                    int bestIdx2=laneIdx[i], bestCount=myCount;
                    for (int l=0;l<cellLaneCount;++l) {
                        if (l!=laneIdx[i] && laneCounts[l]<bestCount-1)
                        { bestCount=laneCounts[l]; bestIdx2=l; }
                    }
                    if (bestIdx2!=laneIdx[i] && isLaneClear(cellLaneOffs[bestIdx2])) {
                        laneIdx[i]=(int8_t)bestIdx2;
                        laneTarget[i]=cellLaneOffs[laneIdx[i]];
                    }
                }

                // Steering target
                float segFwd = (posX[i]-prev_raw.x)*segDir.x + (posZ[i]-prev_raw.y)*segDir.y;
                Vec2 steerTarget;
                bool inArc = false;

                if (isTurn) {
                    float inRX=segDir.y, inRZ=-segDir.x;
                    float outRX=nextSeg.y, outRZ=-nextSeg.x;
                    Vec2 entryPt = {target_raw.x - segDir.x*HALF_CS + inRX*laneTarget[i],
                                    target_raw.y - segDir.y*HALF_CS + inRZ*laneTarget[i]};
                    Vec2 exitPt = {target_raw.x + nextSeg.x*HALF_CS + outRX*laneTarget[i],
                                   target_raw.y + nextSeg.y*HALF_CS + outRZ*laneTarget[i]};
                    float entryDist = std::max(0.0f, segLen - HALF_CS);

                    if (segFwd >= entryDist - 1.0f) {
                        inArc = true;
                        laneOff[i] = laneTarget[i];
                        float etx=exitPt.x-entryPt.x, etz=exitPt.y-entryPt.y;
                        float chordLen = std::sqrt(etx*etx+etz*etz);
                        float R = std::max(1.0f, chordLen*0.7071f);
                        float perpX, perpZ;
                        if (turnCross < 0) { perpX=inRX; perpZ=inRZ; }
                        else { perpX=-inRX; perpZ=-inRZ; }
                        float arcCX=entryPt.x+perpX*R, arcCZ=entryPt.y+perpZ*R;

                        float carAngle=std::atan2(posX[i]-arcCX, posZ[i]-arcCZ);
                        float entAngle=std::atan2(entryPt.x-arcCX, entryPt.y-arcCZ);
                        float extAngle=std::atan2(exitPt.x-arcCX, exitPt.y-arcCZ);

                        auto normAngle=[](float a){ while(a>PI)a-=PI2; while(a<-PI)a+=PI2; return a; };
                        float arcSpan = normAngle(extAngle-entAngle);
                        float carSpan = normAngle(carAngle-entAngle);
                        float tArc = (std::abs(arcSpan)>0.01f) ? std::clamp(carSpan/arcSpan, 0.0f, 1.0f) : 0.0f;
                        float tLook = std::min(1.0f, tArc+0.30f);
                        float lookAngle = entAngle + arcSpan*tLook;
                        steerTarget.x = arcCX + R*std::sin(lookAngle);
                        steerTarget.y = arcCZ + R*std::cos(lookAngle);

                        float crossDot = (posX[i]-exitPt.x)*nextSeg.x + (posZ[i]-exitPt.y)*nextSeg.y;
                        if (crossDot >= -0.5f && tArc > 0.75f) {
                            heading[i] = std::atan2(nextSeg.x, nextSeg.y);
                            ++wn; continue;
                        }
                    } else {
                        float rightX=segDir.y, rightZ=-segDir.x;
                        float lookAhead=std::clamp(speed[i]*0.8f, 3.0f, 12.0f);
                        float tgtFwd=std::min(segFwd+lookAhead, entryDist);
                        steerTarget.x = prev_raw.x+segDir.x*tgtFwd+rightX*laneOff[i];
                        steerTarget.y = prev_raw.y+segDir.y*tgtFwd+rightZ*laneOff[i];
                    }
                } else {
                    float rightX=segDir.y, rightZ=-segDir.x;
                    float lookAhead=std::clamp(speed[i]*0.8f, 3.0f, 12.0f);
                    float tgtFwd=std::min(segFwd+lookAhead, segLen);
                    steerTarget.x = prev_raw.x+segDir.x*tgtFwd+rightX*laneOff[i];
                    steerTarget.y = prev_raw.y+segDir.y*tgtFwd+rightZ*laneOff[i];

                    float crossDot = (posX[i]-target_raw.x)*segDir.x + (posZ[i]-target_raw.y)*segDir.y;
                    if (crossDot >= -0.3f && segFwd >= segLen-1.0f) { ++wn; continue; }
                }

                float stDx=steerTarget.x-posX[i], stDz=steerTarget.y-posZ[i];
                float stDist=std::sqrt(stDx*stDx+stDz*stDz);

                // ---- Car following ----
                float bestGap = 1e6f, bestSpd = MAX_SPEED;
                if (onGrid) {
                    for (int dg=-1;dg<=1;++dg)
                    for (int dh=-1;dh<=1;++dh) {
                        int cgx=myGX+dh, cgz=myGZ+dg;
                        if (cgx<0||cgx>=GS||cgz<0||cgz>=GS) continue;
                        int cellKey=cgz*GS+cgx;
                        for (uint32_t j=driveHead[cellKey]; j!=UINT32_MAX; j=driveNext[j]) {
                            if (j==i) continue;
                            float odx=posX[j]-posX[i], odz=posZ[j]-posZ[i];
                            float fwd=odx*myFwdX+odz*myFwdZ;
                            if (fwd<=0) continue;
                            float lat=std::abs(odx*myFwdZ-odz*myFwdX);
                            if (lat>LAT_BAND) continue;
                            float jFwdX=std::sin(heading[j]), jFwdZ=std::cos(heading[j]);
                            float hcos=myFwdX*jFwdX+myFwdZ*jFwdZ;
                            if (hcos<0.3f) continue;
                            if (!inArc && !inIntersection && hcos>0.5f) {
                                float laneDiff2=std::abs(laneOff[j]-laneOff[i]);
                                if (laneDiff2>2.0f) continue;
                            }
                            float gap=fwd-(halfLen[i]+halfLen[j]+1.5f);
                            if (gap<bestGap) { bestGap=gap; bestSpd=speed[j]; }
                        }
                    }
                }

                // ---- Traffic lights ----
                if (onGrid) {
                    using TI = TrafficLights::TurnIntent;
                    TI turnIntent = TI::STRAIGHT;
                    int turnCellGX=-1, turnCellGZ=-1;
                    if (isTurn) {
                        turnIntent = (turnCross > 0) ? TI::LEFT : TI::RIGHT;
                        city.WorldToGrid(target_raw.x, target_raw.y, turnCellGX, turnCellGZ);
                    }

                    auto checkLight = [&](int cgx2, int cgz2, float dirX, float dirZ) {
                        if (!lights.IsIntersection(cgx2, cgz2)) return;
                        TI localIntent = (cgx2==turnCellGX && cgz2==turnCellGZ) ? turnIntent : TI::STRAIGHT;
                        auto color = lights.GetLight(cgx2, cgz2, dirX, dirZ, localIntent);
                        if (color == TrafficLights::LightColor::GREEN) return;

                        Vec2 intC = city.GridCellCenter(cgx2, cgz2);
                        float toCX=intC.x-posX[i], toCZ=intC.y-posZ[i];
                        float fwdDist=toCX*dirX+toCZ*dirZ;
                        if (fwdDist < 0) return;
                        float stopLine = fwdDist - HALF_CS - 3.0f - halfLen[i];
                        if (stopLine < -1.0f) return;
                        float gap2 = std::max(0.0f, stopLine);

                        if (color == TrafficLights::LightColor::RED) {
                            if (gap2 < bestGap) { bestGap=gap2; bestSpd=0; }
                        } else if (color == TrafficLights::LightColor::YELLOW) {
                            if (gap2 > 3.0f && gap2 < bestGap) { bestGap=gap2; bestSpd=0; }
                        }
                    };

                    if (!lights.IsIntersection(myGX, myGZ)) {
                        int prevScanGX=myGX, prevScanGZ=myGZ;
                        float scanMax = std::min(80.0f, segLen-segFwd+20.0f);
                        for (float d=HALF_CS; d<scanMax; d+=HALF_CS) {
                            float scanX=posX[i]+segDir.x*d, scanZ=posZ[i]+segDir.y*d;
                            int sgx, sgz;
                            if (!city.WorldToGrid(scanX,scanZ,sgx,sgz)) continue;
                            if (sgx==prevScanGX && sgz==prevScanGZ) continue;
                            prevScanGX=sgx; prevScanGZ=sgz;
                            checkLight(sgx, sgz, segDir.x, segDir.y);
                        }
                    }

                    // Gridlock prevention
                    if (!inArc && !lights.IsIntersection(myGX, myGZ)) {
                        int prevScanGX2=myGX, prevScanGZ2=myGZ;
                        for (float d=HALF_CS; d<60.0f; d+=HALF_CS) {
                            float scanX=posX[i]+segDir.x*d, scanZ=posZ[i]+segDir.y*d;
                            int sgx, sgz;
                            if (!city.WorldToGrid(scanX,scanZ,sgx,sgz)) break;
                            if (sgx==prevScanGX2 && sgz==prevScanGZ2) continue;
                            prevScanGX2=sgx; prevScanGZ2=sgz;
                            if (!lights.IsIntersection(sgx,sgz)) continue;

                            int exitCX=sgx+(int)std::round(segDir.x);
                            int exitCZ=sgz+(int)std::round(segDir.y);
                            if (isTurn) {
                                exitCX=sgx+(int)std::round(nextSeg.x);
                                exitCZ=sgz+(int)std::round(nextSeg.y);
                            }
                            if (exitCX<0||exitCX>=GS||exitCZ<0||exitCZ>=GS) break;

                            int exitKey=exitCZ*GS+exitCX;
                            int jamCount=0;
                            for (uint32_t j2=driveHead[exitKey]; j2!=UINT32_MAX; j2=driveNext[j2]) {
                                if (speed[j2]>=1.0f) continue;
                                float j2Fx=std::sin(heading[j2]), j2Fz=std::cos(heading[j2]);
                                float dirDot=segDir.x*j2Fx+segDir.y*j2Fz;
                                if (dirDot>0.3f) jamCount++;
                            }
                            int exitLanes=city.GetRoadLanes(exitCX,exitCZ);
                            // FIX: Lower threshold — was exitLanes*2, now exitLanes
                            // Prevents cars from piling into already-jammed intersections
                            int jamThreshold=exitLanes;
                            if (jamCount>=jamThreshold) {
                                Vec2 intC2=city.GridCellCenter(sgx,sgz);
                                float toCX2=intC2.x-posX[i], toCZ2=intC2.y-posZ[i];
                                float fwdDist2=toCX2*segDir.x+toCZ2*segDir.y;
                                if (fwdDist2>0) {
                                    float stopDist=fwdDist2-HALF_CS-3.0f;
                                    if (stopDist>-1.0f && stopDist<bestGap)
                                    { bestGap=std::max(0.0f,stopDist); bestSpd=0; }
                                }
                            }
                            break;
                        }
                    }
                }

                // ---- IDM acceleration ----
                float v = speed[i];
                float vRatio = v / MAX_SPEED;
                float vR4 = vRatio*vRatio*vRatio*vRatio;
                float idmAccel;
                if (bestGap < 1e5f) {
                    float idm_2ab = 2.f*std::sqrt(ACCEL*IDM_B);
                    float dv = v - bestSpd;
                    float s_star = IDM_S0 + std::max(0.f, v*IDM_T + v*dv/idm_2ab);
                    float gap2 = std::max(bestGap, 0.01f);
                    idmAccel = ACCEL * (1.f - vR4 - (s_star/gap2)*(s_star/gap2));
                } else {
                    idmAccel = ACCEL * (1.f - vR4);
                }

                // Corner speed limit
                float desiredMax = MAX_SPEED;
                if (inArc) desiredMax = MAX_SPEED * 0.30f;
                else if (isTurn) {
                    float distToEntry = std::max(0.0f, segLen-segFwd-HALF_CS);
                    float approachFactor = std::clamp(distToEntry/25.0f, 0.0f, 1.0f);
                    desiredMax = MAX_SPEED * (0.30f + 0.70f*approachFactor);
                }

                // Wrong lane for turn: slow down to allow lane change before intersection
                if (wrongLaneForTurn && !inArc) {
                    float urgency = 1.0f - std::clamp(distToTurnWp / 60.0f, 0.0f, 1.0f);
                    float laneChangeMax = MAX_SPEED * (1.0f - urgency * 0.85f);
                    if (distToTurnWp < 25.0f)
                        laneChangeMax = std::min(laneChangeMax, 2.0f);
                    desiredMax = std::min(desiredMax, laneChangeMax);
                }
                if (v > desiredMax) idmAccel = std::min(idmAccel, -IDM_B*1.5f);

                // Hard brake
                if (bestGap < 0.5f && bestSpd < 0.1f) idmAccel = -DECEL;
                else if (bestGap < 3.0f && bestSpd < 0.5f) idmAccel = std::min(idmAccel, -IDM_B*2.0f);

                speed[i] = std::clamp(v + idmAccel*ddt, 0.0f, desiredMax);
                if (bestGap <= -0.5f && bestSpd < 0.1f) speed[i] = std::min(speed[i], 0.5f);
                if (inIntersection && speed[i] < 2.0f && bestGap > 5.0f)
                    speed[i] = std::min(2.0f, desiredMax);

                // ---- Steering + movement ----
                if (speed[i] > 0.001f && stDist > 0.01f) {
                    float step = speed[i] * ddt;
                    if (!inArc) {
                        // Stanley controller
                        float segH = std::atan2(segDir.x, segDir.y);
                        float rightX=segDir.y, rightZ=-segDir.x;
                        float segFwd2 = (posX[i]-prev_raw.x)*segDir.x+(posZ[i]-prev_raw.y)*segDir.y;
                        float laneX = prev_raw.x+segDir.x*segFwd2+rightX*laneOff[i];
                        float laneZ = prev_raw.y+segDir.y*segFwd2+rightZ*laneOff[i];
                        float cte = (posX[i]-laneX)*rightX + (posZ[i]-laneZ)*rightZ;
                        constexpr float K_STANLEY = 3.5f;
                        float steerCorr = -std::atan2(K_STANLEY*cte, speed[i]+0.5f);
                        float maxCorr = (std::abs(cte)>1.0f) ? 0.40f : 0.18f;
                        steerCorr = std::clamp(steerCorr, -maxCorr, maxCorr);
                        heading[i] = segH + steerCorr;
                        float fwdX=std::sin(heading[i]), fwdZ=std::cos(heading[i]);
                        posX[i] += fwdX*step;
                        posZ[i] += fwdZ*step;
                    } else {
                        // Turn: smooth pursuit
                        float desiredH = std::atan2(stDx, stDz);
                        float dh2 = desiredH - heading[i];
                        while (dh2 > PI) dh2 -= PI2;
                        while (dh2 < -PI) dh2 += PI2;
                        float maxSteerRate = 5.0f;
                        float steerDelta = std::clamp(dh2, -maxSteerRate*ddt, maxSteerRate*ddt);
                        float blendDelta = dh2 * std::min(1.0f, 8.0f*ddt);
                        if (std::abs(blendDelta)>std::abs(steerDelta))
                            steerDelta=std::clamp(blendDelta, -maxSteerRate*ddt*1.5f, maxSteerRate*ddt*1.5f);
                        heading[i] += steerDelta;
                        float fwdX=std::sin(heading[i]), fwdZ=std::cos(heading[i]);
                        posX[i] += fwdX*step;
                        posZ[i] += fwdZ*step;
                    }
                }
            } // end per-car

            // ---- Soft collision resolution ----
            std::fill(driveHead.begin(), driveHead.end(), UINT32_MAX);
            std::fill(driveNext.begin(), driveNext.end(), UINT32_MAX);
            for (uint32_t i=0; i<activeCount; ++i) {
                if (state[i]!=State::DRIVING) continue;
                int gx, gz;
                if (city.WorldToGrid(posX[i],posZ[i],gx,gz)) {
                    int key=gz*GS+gx;
                    driveNext[i]=driveHead[key];
                    driveHead[key]=i;
                }
            }

            auto resolvePair = [&](uint32_t a, uint32_t b) {
                float ddx=posX[b]-posX[a], ddz=posZ[b]-posZ[a];
                float dd2=ddx*ddx+ddz*ddz;
                float sep=halfLen[a]+halfLen[b]+1.5f;
                if (dd2>=sep*sep||dd2<0.001f) return;

                float hcos=std::sin(heading[a])*std::sin(heading[b])
                          +std::cos(heading[a])*std::cos(heading[b]);
                bool bothDriving = (state[a]==State::DRIVING && state[b]==State::DRIVING);
                if (bothDriving && hcos<0.3f) return;

                if (bothDriving) {
                    float sinH=std::sin(heading[a]), cosH=std::cos(heading[a]);
                    float lat=std::abs(ddx*cosH-ddz*sinH);
                    if (lat>LAT_BAND) return;
                    float laneDiff2=std::abs(laneOff[a]-laneOff[b]);
                    if (hcos>0.5f && laneDiff2>2.0f) return;
                }

                float dd=std::sqrt(dd2);
                float overlap=sep-dd;
                float nx=ddx/dd, nz=ddz/dd;

                // FIX: Stronger push — 0.5× overlap, max 0.8m (was 0.3×, max 0.3m)
                float pushMag=std::min(overlap*0.5f, 0.8f);
                posX[a]-=nx*pushMag*0.5f; posZ[a]-=nz*pushMag*0.5f;
                posX[b]+=nx*pushMag*0.5f; posZ[b]+=nz*pushMag*0.5f;

                // FIX: Hard speed reduction for overlapping cars
                float sinH=std::sin(heading[a]), cosH=std::cos(heading[a]);
                float fwd=ddx*sinH+ddz*cosH;
                if (fwd>0) {
                    speed[a]=std::min(speed[a], std::max(speed[b]*0.3f, 0.0f));
                } else {
                    speed[b]=std::min(speed[b], std::max(speed[a]*0.3f, 0.0f));
                }
            };

            for (int cy=0;cy<GS;++cy)
            for (int cx=0;cx<GS;++cx) {
                int key=cy*GS+cx;
                if (driveHead[key]==UINT32_MAX) continue;
                for (uint32_t a=driveHead[key]; a!=UINT32_MAX; a=driveNext[a])
                    for (uint32_t b=driveNext[a]; b!=UINT32_MAX; b=driveNext[b])
                        resolvePair(a, b);
                static const int NBR[][2]={{1,0},{0,1},{1,1},{-1,1}};
                for (auto& nb : NBR) {
                    int nx2=cx+nb[0], nz2=cy+nb[1];
                    if (nx2<0||nx2>=GS||nz2<0||nz2>=GS) continue;
                    int nkey=nz2*GS+nx2;
                    if (driveHead[nkey]==UINT32_MAX) continue;
                    for (uint32_t a=driveHead[key]; a!=UINT32_MAX; a=driveNext[a])
                        for (uint32_t b=driveHead[nkey]; b!=UINT32_MAX; b=driveNext[b])
                            resolvePair(a, b);
                }
            }
        } // end substep
    }
};

// ============================================================
//  Diagnostics
// ============================================================
struct SimStats {
    // Per-snapshot
    uint32_t totalDriving = 0;
    uint32_t totalParked  = 0;
    uint32_t stoppedAtLight = 0;  // driving but speed < 0.1 near red light
    uint32_t stoppedByCarAhead = 0; // driving, speed < 0.1, not at light
    uint32_t collisionPairs = 0;    // pairs within MIN_SEP
    uint32_t stuckCars = 0;          // driving, speed == 0 for > 5s
    float    avgSpeed = 0;

    // Cumulative
    uint64_t totalCollisionFrames = 0;
    uint64_t totalStoppedLightFrames = 0;
    uint64_t totalStoppedCarFrames = 0;
    uint64_t totalStuckFrames = 0;
    uint64_t frameCount = 0;
    double   totalSpeedSum = 0;
    uint64_t totalDrivingFrames = 0;

    // Per-car stuck tracking
    std::vector<float> stuckTime;

    void Init(uint32_t maxCars) { stuckTime.resize(maxCars, 0); }

    void Accumulate(const SimStats& snap) {
        totalCollisionFrames += snap.collisionPairs;
        totalStoppedLightFrames += snap.stoppedAtLight;
        totalStoppedCarFrames += snap.stoppedByCarAhead;
        totalStuckFrames += snap.stuckCars;
        totalSpeedSum += snap.avgSpeed * snap.totalDriving;
        totalDrivingFrames += snap.totalDriving;
        frameCount++;
    }
};

SimStats ComputeSnapshot(CarSys& cars, City& city, TrafficLights& lights, float dt) {
    SimStats s;
    s.Init(MAX_CARS);
    float speedSum = 0;

    for (uint32_t i = 0; i < cars.activeCount; ++i) {
        if (cars.state[i] == State::PARKED) { s.totalParked++; continue; }
        s.totalDriving++;
        speedSum += cars.speed[i];

        if (cars.speed[i] < 0.1f) {
            // Stuck tracking
            s.stuckTime.resize(MAX_CARS, 0);

            // Check if near red light
            int gx, gz;
            city.WorldToGrid(cars.posX[i], cars.posZ[i], gx, gz);
            float fwdX = std::sin(cars.heading[i]);
            float fwdZ = std::cos(cars.heading[i]);

            bool nearRedLight = false;
            for (int d = 0; d <= 3; ++d) {
                float scanX = cars.posX[i] + fwdX * (HALF_CS * d);
                float scanZ = cars.posZ[i] + fwdZ * (HALF_CS * d);
                int sgx, sgz;
                city.WorldToGrid(scanX, scanZ, sgx, sgz);
                if (lights.IsIntersection(sgx, sgz)) {
                    auto color = lights.GetLight(sgx, sgz, fwdX, fwdZ);
                    if (color == TrafficLights::LightColor::RED || color == TrafficLights::LightColor::YELLOW) {
                        nearRedLight = true;
                        break;
                    }
                }
            }
            if (nearRedLight) s.stoppedAtLight++;
            else s.stoppedByCarAhead++;
        }
    }

    if (s.totalDriving > 0) s.avgSpeed = speedSum / s.totalDriving;

    // Count collision pairs
    for (uint32_t i = 0; i < cars.activeCount; ++i) {
        if (cars.state[i] != State::DRIVING) continue;
        for (uint32_t j = i+1; j < cars.activeCount; ++j) {
            if (cars.state[j] != State::DRIVING) continue;
            float dx = cars.posX[j]-cars.posX[i], dz = cars.posZ[j]-cars.posZ[i];
            float d2 = dx*dx+dz*dz;
            float sep = cars.halfLen[i]+cars.halfLen[j]+1.5f;
            if (d2 < sep*sep) s.collisionPairs++;
        }
    }

    return s;
}

// ============================================================
//  Spawn helpers
// ============================================================
struct HouseInfo { int gx, gz, pop; };
struct WorkInfo  { int gx, gz; };

uint32_t SpawnOneCar(CarSys& cars, City& city,
                     const std::vector<HouseInfo>& houses,
                     const std::vector<WorkInfo>& workplaces,
                     uint32_t& rng, bool startDriving = false)
{
    rng = rng * 1664525u + 1013904223u;
    int hi = (int)(rng % houses.size());
    rng = rng * 1664525u + 1013904223u;
    int wi = (int)(rng % workplaces.size());
    auto path = city.FindPath(houses[hi].gx, houses[hi].gz,
                              workplaces[wi].gx, workplaces[wi].gz);
    if (path.size() < 2) return UINT32_MAX;
    int pathLanes = city.GetRoadLanes(
        (int)std::floor((path[0].x+HALF_WORLD)/CELL_SIZE),
        (int)std::floor((path[0].y+HALF_WORLD)/CELL_SIZE));
    uint32_t id = cars.SpawnCar(path, rng, pathLanes);
    if (id != UINT32_MAX && startDriving) {
        cars.state[id] = State::DRIVING;
        cars.parkTimer[id] = 0;
        // Start mid-path for variety
        uint8_t mid = std::min((uint8_t)(cars.wpCount[id] / 3), (uint8_t)(cars.wpCount[id]-1));
        if (mid >= 1) {
            Vec2* wp = &cars.wpBuf[(size_t)id * MAX_WP];
            Vec2 dir = DirTo(wp[mid-1], wp[mid]);
            float rx = dir.y, rz = -dir.x;
            cars.posX[id] = wp[mid-1].x + rx * cars.laneOff[id];
            cars.posZ[id] = wp[mid-1].y + rz * cars.laneOff[id];
            cars.heading[id] = std::atan2(dir.x, dir.y);
            cars.wpCurr[id] = mid;
        }
    }
    return id;
}

// ============================================================
//  Run a simulation phase and report
// ============================================================
void RunPhase(const char* label, CarSys& cars, City& city, TrafficLights& lights,
              float simSpeed, float totalRealTime, float startTimeOfDay, float dayDuration)
{
    float realDt   = 1.0f / 60.0f;
    float simDt    = realDt * simSpeed;
    float timeOfDay = startTimeOfDay;

    printf("\n========================================\n");
    printf("  PHASE: %s\n", label);
    printf("========================================\n");
    printf("  Cars: %d, SimSpeed: %.0fx, RealTime: %.0fs = %.0f game-min\n",
           cars.activeCount, simSpeed, totalRealTime, totalRealTime * simSpeed / 60.0f);
    printf("  Starting at game time: %02d:%02d\n\n",
           (int)(timeOfDay*24), (int)(std::fmod(timeOfDay*24*60, 60)));

    SimStats cumulative;
    cumulative.Init(MAX_CARS);
    std::vector<float> stuckTime(MAX_CARS, 0);

    // Per-intersection cumulative stops for hotspot detection
    std::vector<uint64_t> intStopsCum(GS*GS, 0);

    float elapsed = 0;
    int reportInterval = 0;
    int totalFrames = (int)(totalRealTime / realDt);

    printf("%-6s  %-6s  %-6s  %-8s  %-8s  %-8s  %-8s  %-8s  %-6s\n",
           "Time", "Drive", "Park", "AvgSpd", "AtLight", "ByCar", "Collisn", "Stuck", "GameT");
    printf("------  ------  ------  --------  --------  --------  --------  --------  ------\n");

    for (int frame = 0; frame < totalFrames; ++frame) {
        lights.Update(simDt);
        cars.Update(simDt, city, lights, timeOfDay, dayDuration);

        timeOfDay += simDt / dayDuration;
        if (timeOfDay >= 1.0f) timeOfDay -= 1.0f;
        elapsed += realDt;

        for (uint32_t i = 0; i < cars.activeCount; ++i) {
            if (cars.state[i] == State::DRIVING && cars.speed[i] < 0.1f)
                stuckTime[i] += simDt;
            else
                stuckTime[i] = 0;
        }

        // Accumulate intersection stops every frame
        for (uint32_t ci = 0; ci < cars.activeCount; ++ci) {
            if (cars.state[ci] != State::DRIVING || cars.speed[ci] >= 0.5f) continue;
            int gx, gz;
            city.WorldToGrid(cars.posX[ci], cars.posZ[ci], gx, gz);
            for (int dg=-2;dg<=2;++dg)
            for (int dh=-2;dh<=2;++dh) {
                int nx=gx+dh, nz=gz+dg;
                if (nx>=0&&nx<GS&&nz>=0&&nz<GS && lights.isIntersection[nz*GS+nx])
                    intStopsCum[nz*GS+nx]++;
            }
        }

        if ((int)(elapsed / 5.0f) > reportInterval) {
            reportInterval = (int)(elapsed / 5.0f);
            SimStats snap = ComputeSnapshot(cars, city, lights, simDt);
            snap.stuckCars = 0;
            for (uint32_t i = 0; i < cars.activeCount; ++i)
                if (stuckTime[i] > 10.0f) snap.stuckCars++;
            cumulative.Accumulate(snap);

            float gameHour = timeOfDay * 24.0f;
            printf("%5.0fs  %5d   %5d   %6.1fm/s  %6d    %6d    %6d    %6d    %02d:%02d\n",
                   elapsed, snap.totalDriving, snap.totalParked,
                   snap.avgSpeed, snap.stoppedAtLight, snap.stoppedByCarAhead,
                   snap.collisionPairs, snap.stuckCars,
                   (int)gameHour, (int)(std::fmod(gameHour*60, 60)));
        }
    }

    // ---- Summary ----
    printf("\n  --- Summary ---\n");
    float avgAvgSpeed = cumulative.totalDrivingFrames > 0
        ? (float)(cumulative.totalSpeedSum / cumulative.totalDrivingFrames) : 0;
    printf("  Avg speed:        %.2f m/s (%.1f km/h)\n", avgAvgSpeed, avgAvgSpeed * 3.6f);

    float avgDriving = cumulative.frameCount > 0
        ? (float)cumulative.totalDrivingFrames / cumulative.frameCount : 0;
    float avgAtLight = cumulative.frameCount > 0
        ? (float)cumulative.totalStoppedLightFrames / cumulative.frameCount : 0;
    float avgByCar = cumulative.frameCount > 0
        ? (float)cumulative.totalStoppedCarFrames / cumulative.frameCount : 0;
    float avgCollisions = cumulative.frameCount > 0
        ? (float)cumulative.totalCollisionFrames / cumulative.frameCount : 0;
    float avgStuck = cumulative.frameCount > 0
        ? (float)cumulative.totalStuckFrames / cumulative.frameCount : 0;

    printf("  Avg driving:         %.0f cars\n", avgDriving);
    printf("  Avg stopped at lights: %.1f cars (%.0f%% of driving)\n",
           avgAtLight, avgDriving > 0 ? avgAtLight/avgDriving*100 : 0);
    printf("  Avg stopped by car:    %.1f cars (%.0f%% of driving)\n",
           avgByCar, avgDriving > 0 ? avgByCar/avgDriving*100 : 0);
    printf("  Avg collision pairs:   %.1f\n", avgCollisions);
    printf("  Avg stuck (>10s):      %.1f cars\n", avgStuck);

    // ---- Worst intersections ----
    printf("\n  --- Worst Intersections (cumulative stop-frames) ---\n");
    std::vector<std::pair<uint64_t,int>> intRank;
    for (int i=0;i<GS*GS;i++) if (intStopsCum[i]>0) intRank.push_back({intStopsCum[i], i});
    std::sort(intRank.begin(), intRank.end(), std::greater<>());
    for (int j = 0; j < std::min(15, (int)intRank.size()); ++j) {
        int key = intRank[j].second;
        int gx = key%GS, gz = key/GS;
        printf("    [%2d,%2d] lanes=%d  stop-frames=%llu\n",
               gx, gz, city.GetRoadLanes(gx,gz), (unsigned long long)intRank[j].first);
    }

    // ---- Speed distribution final snapshot ----
    printf("\n  --- Speed Distribution (final snapshot) ---\n");
    int speedBins[15] = {};
    int drivingNow = 0;
    for (uint32_t ci=0; ci<cars.activeCount; ++ci) {
        if (cars.state[ci]!=State::DRIVING) continue;
        drivingNow++;
        int bin = std::min(14, (int)cars.speed[ci]);
        speedBins[bin]++;
    }
    printf("    Driving now: %d\n", drivingNow);
    for (int b=0;b<15;++b) {
        if (speedBins[b]>0)
            printf("    %2d-%2d m/s: %4d cars (%4.1f%%)\n",
                   b, b+1, speedBins[b], 100.f*speedBins[b]/std::max(1,drivingNow));
    }
}

// ============================================================
//  Main
// ============================================================
int main() {
    printf("=== CrowdSim Traffic Simulation ===\n");
    printf("Loading save_10.city...\n");

    City city;
    if (!city.Load("saves/save_10.city")) {
        if (!city.Load("build/Release/saves/save_10.city")) {
            printf("ERROR: Cannot load save_10.city\n");
            return 1;
        }
    }

    int roadCount=0, houseCount=0, workCount=0, parkCount=0, totalPop=0;
    for (int i=0; i<GS*GS; i++) {
        auto ct = city.cells[i];
        if (ct==CellType::ROAD||ct==CellType::CROSSWALK||ct==CellType::BUS_STOP) roadCount++;
        if (ct==CellType::HOUSE) { houseCount++; totalPop+=city.housePop[i]; }
        if (ct==CellType::WORKPLACE) workCount++;
        if (ct==CellType::PARKING) parkCount++;
    }
    printf("  Roads: %d, Houses: %d (pop %d), Workplaces: %d, Parking: %d\n",
           roadCount, houseCount, totalPop, workCount, parkCount);

    TrafficLights lights;
    lights.RebuildIntersections(city);
    int intersections = 0;
    for (int i=0;i<GS*GS;i++) if (lights.isIntersection[i]) intersections++;
    printf("  Intersections: %d\n", intersections);

    std::vector<HouseInfo> houses;
    std::vector<WorkInfo>  workplaces;
    for (int gz=0; gz<GS; ++gz)
    for (int gx=0; gx<GS; ++gx) {
        if (city.GetCellType(gx,gz)==CellType::HOUSE && city.housePop[gz*GS+gx]>0)
            houses.push_back({gx, gz, city.housePop[gz*GS+gx]});
        if (city.GetCellType(gx,gz)==CellType::WORKPLACE)
            workplaces.push_back({gx, gz});
    }

    // ================================================================
    //  PHASE 1: Normal schedule-based sim (100x, 2 real-min)
    // ================================================================
    {
        CarSys cars;
        cars.Initialize();
        uint32_t rng = 42;
        int spawned = 0;
        int targetCars = totalPop / 3;
        for (int c = 0; c < targetCars; ++c) {
            if (SpawnOneCar(cars, city, houses, workplaces, rng) != UINT32_MAX) spawned++;
        }
        printf("\n  Phase 1: Spawned %d/%d cars (schedule-based)\n", spawned, targetCars);

        // Reset city parking
        memset(city.parkOcc, 0, sizeof(city.parkOcc));
        memset(city.totalParkOcc, 0, sizeof(city.totalParkOcc));

        RunPhase("NORMAL SCHEDULE (100x, 2min)", cars, city, lights, 100.0f, 120.0f, 0.30f, 86400.0f);
    }

    // ================================================================
    //  PHASE 2: Stress test — all cars start driving simultaneously
    // ================================================================
    {
        // Reset city parking
        memset(city.parkOcc, 0, sizeof(city.parkOcc));
        memset(city.totalParkOcc, 0, sizeof(city.totalParkOcc));

        CarSys cars;
        cars.Initialize();
        uint32_t rng = 12345;
        int spawned = 0;

        // Spawn many cars directly as DRIVING
        int stressTarget = 800;  // 800 concurrent driving cars
        for (int c = 0; c < stressTarget * 5 && spawned < stressTarget; ++c) {
            if (SpawnOneCar(cars, city, houses, workplaces, rng, true) != UINT32_MAX) spawned++;
        }
        printf("\n  Phase 2: Spawned %d/%d cars (all driving, stress test)\n", spawned, stressTarget);

        // Reset lights
        lights.RebuildIntersections(city);

        // Run at 10x for 30 real-seconds = 5 game-minutes — enough to see congestion
        RunPhase("STRESS TEST (800 cars, 10x, 30s)", cars, city, lights, 10.0f, 30.0f, 0.30f, 86400.0f);
    }

    // ================================================================
    //  PHASE 3: Extreme stress — 2000 cars
    // ================================================================
    {
        memset(city.parkOcc, 0, sizeof(city.parkOcc));
        memset(city.totalParkOcc, 0, sizeof(city.totalParkOcc));

        CarSys cars;
        cars.Initialize();
        uint32_t rng = 99999;
        int spawned = 0;
        int stressTarget = 2000;
        for (int c = 0; c < stressTarget * 5 && spawned < stressTarget; ++c) {
            if (SpawnOneCar(cars, city, houses, workplaces, rng, true) != UINT32_MAX) spawned++;
        }
        printf("\n  Phase 3: Spawned %d/%d cars (all driving, extreme stress)\n", spawned, stressTarget);

        lights.RebuildIntersections(city);
        RunPhase("EXTREME STRESS (2000 cars, 10x, 30s)", cars, city, lights, 10.0f, 30.0f, 0.30f, 86400.0f);
    }

    printf("\n\n=== ALL PHASES COMPLETE ===\n");
    return 0;
}
