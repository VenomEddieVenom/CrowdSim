#include "CarSystem.h"
#include "CrowdSystem.h"
#include "TrafficLightSystem.h"
#include <algorithm>
#include <cmath>

// ----- car colour palette -----
static const XMFLOAT4 PALETTE[] =
{
    {0.88f, 0.14f, 0.14f, 1.f},  // red
    {0.15f, 0.44f, 0.90f, 1.f},  // blue
    {0.82f, 0.82f, 0.82f, 1.f},  // silver
    {0.12f, 0.12f, 0.12f, 1.f},  // black
    {0.86f, 0.68f, 0.10f, 1.f},  // gold
    {0.14f, 0.70f, 0.26f, 1.f},  // green
    {0.90f, 0.45f, 0.10f, 1.f},  // orange
    {0.55f, 0.14f, 0.85f, 1.f},  // purple
    {0.90f, 0.90f, 0.40f, 1.f},  // yellow
    {0.20f, 0.76f, 0.82f, 1.f},  // teal
};
static constexpr int PALETTE_N = (int)(sizeof(PALETTE)/sizeof(PALETTE[0]));

// ============================================================
//  Initialize
// ============================================================
void CarSystem::Initialize()
{
    posX_.resize(MAX_CARS, 0.f);
    posZ_.resize(MAX_CARS, 0.f);
    speed_.resize(MAX_CARS, 0.f);
    heading_.resize(MAX_CARS, 0.f);
    state_.resize(MAX_CARS, State::PARKED);
    parkTimer_.resize(MAX_CARS, 0.f);
    carColor_.resize(MAX_CARS, {0.82f, 0.82f, 0.82f, 1.f});
    halfLen_.resize(MAX_CARS, CAR_HL);
    isBus_.resize(MAX_CARS, false);
    busRouteIdx_.resize(MAX_CARS, -1);
    busDirection_.resize(MAX_CARS, 0);
    busNextStop_.resize(MAX_CARS, -1);
    busStopTimer_.resize(MAX_CARS, 0.f);
    busPassengers_.resize(MAX_CARS, 0);
    wage_.resize(MAX_CARS, 0.f);
    money_.resize(MAX_CARS, 0.f);

    wpBuf_.resize((size_t)MAX_CARS * MAX_WP);
    wpCount_.resize(MAX_CARS, 0);
    wpCurr_.resize(MAX_CARS, 0);
    carDir_.resize(MAX_CARS, 0);
    wandering_.resize(MAX_CARS, false);
    laneOff_.resize(MAX_CARS, 4.5f);
    laneTarget_.resize(MAX_CARS, 4.5f);
    laneIdx_.resize(MAX_CARS, 0);
    parkAnim_.resize(MAX_CARS);

    schedDepart_.resize(MAX_CARS, 8.0f);
    schedReturn_.resize(MAX_CARS, 17.0f);
    lastDayTrip_.resize(MAX_CARS, 0);
    dayCounter_ = 0;
    prevTimeOfDay_ = 0.0f;

    driveHead_.resize(CityLayout::GRID_SIZE * CityLayout::GRID_SIZE, UINT32_MAX);
    driveNext_.resize(MAX_CARS, UINT32_MAX);
    parkCell_.resize(MAX_CARS, UINT32_MAX);
    myParkSlot_.resize(MAX_CARS, 0xFF);

    visIdx_.resize(MAX_VISIBLE);
    activeCount = 0;

    CreateInstPool();

    wi::backlog::post("[CarSystem] Ready – traffic system initialised",
                      wi::backlog::LogLevel::Default);
}

// ============================================================
//  SpawnCar
// ============================================================
uint32_t CarSystem::SpawnCar(const std::vector<XMFLOAT2>& roadPath, uint32_t colorSeed, int spawnLanes)
{
    if (activeCount >= MAX_CARS)    return UINT32_MAX;
    if (roadPath.size() < 2)        return UINT32_MAX;

    // ---- Spawn proximity check: refuse if too close to a DRIVING car ----
    const float spawnX = roadPath[0].x;
    const float spawnZ = roadPath[0].y;
    for (uint32_t j = 0; j < activeCount; ++j) {
        if (state_[j] != State::DRIVING && state_[j] != State::ENTERING_PARKING) continue;
        float dx = posX_[j] - spawnX;
        float dz = posZ_[j] - spawnZ;
        float sep = CAR_HL + halfLen_[j] + 1.5f;
        if (dx*dx + dz*dz < sep * sep)
            return UINT32_MAX;
    }

    const uint32_t i = activeCount++;

    const uint8_t wc = (uint8_t)std::min(roadPath.size(), (size_t)MAX_WP);
    XMFLOAT2* wp = &wpBuf_[(size_t)i * MAX_WP];
    for (uint8_t w = 0; w < wc; ++w) wp[w] = roadPath[w];
    wpCount_[i] = wc;
    SimplifyPath(wp, wpCount_[i]);
    wpCurr_[i]  = 1;
    carDir_[i]  = 0;

    // Assign lane: distribute evenly across all lanes
    {
        float lo[3]; int lc;
        GetLaneOffsets(spawnLanes, lo, lc);
        laneIdx_[i] = (int8_t)(colorSeed % lc);
        laneOff_[i] = lo[laneIdx_[i]];
        laneTarget_[i] = laneOff_[i];
    }

    // Spawn at lane-offset position (not centreline) so cars start centered
    XMFLOAT2 d0 = DirTo(wp[0], wp[1]);
    float spawnRX = d0.y, spawnRZ = -d0.x; // right of travel
    posX_[i]    = wp[0].x + spawnRX * laneOff_[i];
    posZ_[i]    = wp[0].y + spawnRZ * laneOff_[i];
    speed_[i]   = 0.f;
    heading_[i] = std::atan2(d0.x, d0.y);

    parkCell_[i]  = UINT32_MAX;
    myParkSlot_[i]= 0xFF;

    // Start PARKED with staggered departure so they enter the road sequentially
    state_[i]     = State::PARKED;
    parkTimer_[i] = 0.3f + (float)(colorSeed % 8) * 0.6f;
    parkCell_[i]  = UINT32_MAX;
    myParkSlot_[i]= 0xFF;

    carColor_[i] = PALETTE[colorSeed % PALETTE_N];
    halfLen_[i] = CAR_HL;
    isBus_[i]   = false;

    // Wage $0.50 – $2.00 /game-sec
    uint32_t ws = colorSeed * 1103515245u + 12345u;
    wage_[i]  = 0.50f + (float)((ws >> 8) & 0x3F) / 63.f * 1.50f;
    money_[i] = 0.f;

    // Work schedule: spread across 6 shift windows for even traffic distribution
    //   15% early morning  (5:00 – 7:00)   – bakeries, hospitals, logistics
    //   25% morning rush   (7:00 – 9:00)   – classic office workers
    //   15% late morning   (9:00 – 11:00)  – flexi workers, retail open
    //   15% midday         (11:00 – 13:00) – split-shift, hospitality
    //   15% afternoon      (14:00 – 16:00) – schools, part-time retail
    //   15% night shift    (20:00 – 23:00) – bars, factories, security
    uint32_t schedSeed = ws * 1103515245u + 12345u;
    float schedRand = (float)((schedSeed >> 8) & 0xFF) / 255.f;
    float depart;
    if      (schedRand < 0.15f) depart = 5.0f  + (schedRand / 0.15f) * 2.0f;
    else if (schedRand < 0.40f) depart = 7.0f  + ((schedRand - 0.15f) / 0.25f) * 2.0f;
    else if (schedRand < 0.55f) depart = 9.0f  + ((schedRand - 0.40f) / 0.15f) * 2.0f;
    else if (schedRand < 0.70f) depart = 11.0f + ((schedRand - 0.55f) / 0.15f) * 2.0f;
    else if (schedRand < 0.85f) depart = 14.0f + ((schedRand - 0.70f) / 0.15f) * 2.0f;
    else                        depart = 20.0f + ((schedRand - 0.85f) / 0.15f) * 3.0f;

    // Shift length 4–10 h (includes part-time workers at 4–6 h)
    uint32_t shiftSeed = (schedSeed >> 16) & 0xFF;
    float shiftLen;
    if (shiftSeed < 64)        shiftLen = 4.0f + (shiftSeed / 64.f) * 2.0f;   // 25% part-time  4-6 h
    else if (shiftSeed < 128)  shiftLen = 6.0f + ((shiftSeed - 64) / 64.f) * 2.0f; // 25% mid  6-8 h
    else                       shiftLen = 8.0f + ((shiftSeed - 128) / 128.f) * 2.0f; // 50% full 8-10 h
    schedDepart_[i] = depart;
    schedReturn_[i] = std::fmod(depart + shiftLen, 24.0f);
    lastDayTrip_[i] = 0;

    return i;
}

// ============================================================
//  SimplifyPath – merge consecutive collinear waypoints
//  Keeps only the first, last, and any point where direction changes.
// ============================================================
void CarSystem::SimplifyPath(XMFLOAT2* wp, uint8_t& count)
{
    if (count <= 2) return;
    uint8_t write = 1;
    for (uint8_t read = 1; read < count - 1; ++read)
    {
        float dx1 = wp[read].x - wp[write - 1].x;
        float dz1 = wp[read].y - wp[write - 1].y;
        float dx2 = wp[read + 1].x - wp[read].x;
        float dz2 = wp[read + 1].y - wp[read].y;
        float len1 = std::sqrt(dx1 * dx1 + dz1 * dz1);
        float len2 = std::sqrt(dx2 * dx2 + dz2 * dz2);
        if (len1 < 0.001f || len2 < 0.001f) { wp[write++] = wp[read]; continue; }
        float dot = (dx1 * dx2 + dz1 * dz2) / (len1 * len2);
        if (dot < 0.999f)
            wp[write++] = wp[read];
    }
    wp[write++] = wp[count - 1];
    count = write;
}

// ============================================================
//  SpawnBus – spawn a bus that drives a loop through the city
// ============================================================
uint32_t CarSystem::SpawnBus(const std::vector<XMFLOAT2>& roadPath, uint32_t seed)
{
    if (activeCount >= MAX_CARS)    return UINT32_MAX;
    if (roadPath.size() < 2)        return UINT32_MAX;

    // Spawn proximity check (use bus-sized distance)
    const float busSep = BUS_HL * 2.f + 2.0f;
    const float spawnX = roadPath[0].x;
    const float spawnZ = roadPath[0].y;
    for (uint32_t j = 0; j < activeCount; ++j) {
        if (state_[j] != State::DRIVING && state_[j] != State::ENTERING_PARKING) continue;
        float dx = posX_[j] - spawnX;
        float dz = posZ_[j] - spawnZ;
        if (dx*dx + dz*dz < busSep * busSep)
            return UINT32_MAX;
    }

    const uint32_t i = activeCount++;

    const uint8_t wc = (uint8_t)std::min(roadPath.size(), (size_t)MAX_WP);
    XMFLOAT2* wp = &wpBuf_[(size_t)i * MAX_WP];
    for (uint8_t w = 0; w < wc; ++w) wp[w] = roadPath[w];
    wpCount_[i] = wc;
    SimplifyPath(wp, wpCount_[i]);
    wpCurr_[i]  = 1;
    carDir_[i]  = 0;

    // Buses always use the outermost lane
    {
        float lo[3]; int lc;
        GetLaneOffsets(6, lo, lc);
        laneIdx_[i] = (int8_t)(lc - 1);
        laneOff_[i] = lo[lc - 1];
        laneTarget_[i] = laneOff_[i];
    }

    XMFLOAT2 d0 = DirTo(wp[0], wp[1]);
    float busRX = d0.y, busRZ = -d0.x;
    posX_[i]    = wp[0].x + busRX * laneOff_[i];
    posZ_[i]    = wp[0].y + busRZ * laneOff_[i];
    speed_[i]   = 0.f;
    heading_[i] = std::atan2(d0.x, d0.y);

    parkCell_[i]  = UINT32_MAX;
    myParkSlot_[i]= 0xFF;

    // Bus: start driving immediately, no parking
    state_[i]     = State::DRIVING;
    parkTimer_[i] = 0.f;

    // Yellow bus colour
    carColor_[i] = {0.92f, 0.72f, 0.08f, 1.f};
    halfLen_[i]  = BUS_HL;
    isBus_[i]    = true;

    wage_[i]  = 0.f;
    money_[i] = 0.f;
    schedDepart_[i] = 0.f;
    schedReturn_[i] = 0.f;
    lastDayTrip_[i] = UINT32_MAX; // never triggers schedule logic

    return i;
}

// ============================================================
//  SpawnRouteBus – spawn a bus that follows a specific route
// ============================================================
uint32_t CarSystem::SpawnRouteBus(int routeIdx, CityLayout& city, int direction)
{
    if (routeIdx < 0 || routeIdx >= (int)busRoutes_.size()) return UINT32_MAX;
    auto& route = busRoutes_[routeIdx];
    if (!route.active) return UINT32_MAX;

    const auto& path = (direction == 0) ? route.fullPathAB : route.fullPathBA;
    if (path.size() < 2) return UINT32_MAX;

    // Use existing SpawnBus to handle the actual spawning
    uint32_t i = SpawnBus(path, (uint32_t)(routeIdx * 1337 + direction * 7919));
    if (i == UINT32_MAX) return UINT32_MAX;

    // Tag with route info
    busRouteIdx_[i] = (int8_t)routeIdx;
    busDirection_[i] = (int8_t)direction;
    busNextStop_[i] = 0;
    busStopTimer_[i] = 0.f;
    busPassengers_[i] = 0;

    return i;
}

// ============================================================
//  UpdateBusRoutes – spawn buses for active routes, handle stops
// ============================================================
void CarSystem::UpdateBusRoutes(float dt, CityLayout& city)
{
    // For each active route, ensure at least one bus exists in each direction
    for (int ri = 0; ri < (int)busRoutes_.size(); ++ri)
    {
        auto& route = busRoutes_[ri];
        if (!route.active) continue;

        // Count buses on this route per direction
        int countAB = 0, countBA = 0;
        for (uint32_t j = 0; j < activeCount; ++j)
        {
            if (!isBus_[j]) continue;
            if (busRouteIdx_[j] != ri) continue;
            if (busDirection_[j] == 0) ++countAB;
            else ++countBA;
        }

        // Spawn if needed (1 bus per direction per route)
        if (countAB == 0) SpawnRouteBus(ri, city, 0);
        if (countBA == 0) SpawnRouteBus(ri, city, 1);
    }

    // Handle bus stop detection and timers (position-based)
    for (uint32_t i = 0; i < activeCount; ++i)
    {
        if (!isBus_[i] || busRouteIdx_[i] < 0) continue;
        if (state_[i] != State::DRIVING) continue;

        int ri = busRouteIdx_[i];
        if (ri >= (int)busRoutes_.size()) continue;
        auto& route = busRoutes_[ri];

        int dir = busDirection_[i];
        int nextStop = busNextStop_[i];

        // Stop timer active — bus is paused at a stop
        if (busStopTimer_[i] > 0.f)
        {
            busStopTimer_[i] -= dt;
            speed_[i] = 0.f; // hold still
            if (busStopTimer_[i] <= 0.f)
            {
                busStopTimer_[i] = 0.f;
                busNextStop_[i]++;
            }
            continue;
        }

        // Determine which stop list and index to use for current direction
        const auto& stopList = (dir == 0) ? route.stops :
            (route.circular && !route.stopsBA.empty()) ? route.stopsBA : route.stops;
        int totalStops = (int)stopList.size();

        // For non-circular B→A, reverse the index
        int stopListIdx;
        if (dir == 0) {
            stopListIdx = nextStop;
        } else if (route.circular && !route.stopsBA.empty()) {
            stopListIdx = nextStop;
        } else {
            stopListIdx = totalStops - 1 - nextStop;
        }

        if (nextStop >= 0 && nextStop < totalStops && stopListIdx >= 0 && stopListIdx < totalStops)
        {
            XMFLOAT2 stopPos = city.GridCellCenter(stopList[stopListIdx].gx, stopList[stopListIdx].gz);
            float dx = posX_[i] - stopPos.x;
            float dz = posZ_[i] - stopPos.y;
            float dist2 = dx*dx + dz*dz;

            if (dist2 < 12.0f * 12.0f)
            {
                // Arrived at stop — pause for 5 seconds
                busStopTimer_[i] = 5.0f;
                speed_[i] = 0.f;
                // Passengers board/exit (simple: random 0-3 board, 0-2 exit)
                uint8_t boarding = (uint8_t)(rand() % 4);
                uint8_t exiting = std::min(busPassengers_[i], (uint8_t)(rand() % 3));
                busPassengers_[i] = (uint8_t)std::min(40, (int)busPassengers_[i] + boarding - exiting);
            }
            else if (dist2 < 40.0f * 40.0f)
            {
                // Decelerate when approaching stop
                float dist = std::sqrt(dist2);
                float approach = dist / 40.0f;
                float maxSpd = MAX_SPEED * approach;
                if (speed_[i] > maxSpd) speed_[i] = maxSpd;
            }
        }
    }
}

// ============================================================
//  Update  –  COMPLETE REWRITE: pure kinematic integration
//
//  CORE RULE: Position is ONLY changed via pos += forward * speed * dt.
//  No snapping, no teleporting. Ever.
//
//  Turning: Circular arc through intersection. The car steers
//  toward a rolling target on the arc. Waypoint advance happens
//  when the car's forward projection crosses the exit line.
//
//  Straight: Stanley lateral controller for lane tracking.
//  heading ≈ segDir + atan(k·e / (v + 0.5))
//
//  Traffic lights: virtual stopped car injected at stop line.
// ============================================================
void CarSystem::Update(float dt, CityLayout& city, const TrafficLightSystem& lights,
                       float timeOfDay, float dayDuration,
                       const PedestrianView* peds)
{
    // ---- DT clamping: substep when simSpeed pushes dt beyond safe limit ----
    constexpr float MAX_PHYSICS_DT = 0.05f;          // 50 ms max per substep
    const int subSteps = (dt > MAX_PHYSICS_DT) ? (int)std::ceil(dt / MAX_PHYSICS_DT) : 1;
    const float subDt  = dt / (float)subSteps;

 for (int sub = 0; sub < subSteps; ++sub) {

    constexpr int GS = CityLayout::GRID_SIZE;
    constexpr float CS = CityLayout::CELL_SIZE;
    constexpr float HALF_CS = CS * 0.5f;
    constexpr float SIDEWALK_MID = HALF_CS - CityLayout::SIDEWALK_W + CityLayout::SIDEWALK_W * 0.5f; // 9.0 m

    // Track day transitions for trip-per-day tracking (only on first substep)
    if (sub == 0) {
        if (timeOfDay < prevTimeOfDay_)
            ++dayCounter_;
        prevTimeOfDay_ = timeOfDay;
    }

    float currentHour = timeOfDay * 24.0f;
    dt = subDt;  // use clamped substep dt for all physics below

    // ---- Rebuild per-cell linked list of DRIVING + ENTERING_PARKING cars ----
    std::fill(driveHead_.begin(), driveHead_.end(), UINT32_MAX);
    std::fill(driveNext_.begin(), driveNext_.end(), UINT32_MAX);
    for (uint32_t i = 0; i < activeCount; ++i)
    {
        if (state_[i] != State::DRIVING && state_[i] != State::ENTERING_PARKING) continue;
        int gx, gz;
        if (city.WorldToGrid(posX_[i], posZ_[i], gx, gz))
        {
            int key = gz * GS + gx;
            driveNext_[i]   = driveHead_[key];
            driveHead_[key] = i;
        }
    }

    // ---- Per-car update ----
    for (uint32_t i = 0; i < activeCount; ++i)
    {
        // ---- PARKED ----
        if (state_[i] == State::PARKED)
        {
            if (parkTimer_[i] > 0.0f)
            {
                parkTimer_[i] -= dt;
                continue;
            }

            // Check if it's time to depart based on schedule
            bool shouldDepart = wandering_[i]; // wandering cars depart immediately

            if (!shouldDepart && carDir_[i] == 0)
            {
                float dep = schedDepart_[i];
                float diff = currentHour - dep;
                if (diff < 0.0f) diff += 24.0f;
                if (diff >= 0.0f && diff < 0.5f && lastDayTrip_[i] != dayCounter_ * 2 + 1)
                {
                    shouldDepart = true;
                    lastDayTrip_[i] = dayCounter_ * 2 + 1;
                }

                if (!shouldDepart && lastDayTrip_[i] != dayCounter_ * 2 + 1)
                {
                    uint32_t jHash = (uint32_t)(i * 2654435761u + dayCounter_ * 31u +
                                     (uint32_t)(currentHour * 100.f));
                    float jRand = (float)(jHash & 0xFFFF) / 65535.f;
                    float hourFrac = dt / (dayDuration / 24.0f);
                    if (jRand < JOYRIDE_CHANCE * hourFrac)
                    {
                        shouldDepart = true;
                        wandering_[i] = true;  // mark as wandering trip
                    }
                }
            }
            else if (!shouldDepart && carDir_[i] == 1)
            {
                float ret = schedReturn_[i];
                float diff = currentHour - ret;
                if (diff < 0.0f) diff += 24.0f;
                if (diff >= 0.0f && diff < 0.5f && lastDayTrip_[i] != dayCounter_ * 2 + 2)
                {
                    shouldDepart = true;
                    lastDayTrip_[i] = dayCounter_ * 2 + 2;
                }
            }

            if (!shouldDepart)
                continue;

            // Departure proximity check: delay if a driving car is too close
            {
                bool blocked = false;
                int cgx, cgz;
                if (city.WorldToGrid(posX_[i], posZ_[i], cgx, cgz))
                {
                    for (int dg = -1; dg <= 1 && !blocked; ++dg)
                    for (int dh = -1; dh <= 1 && !blocked; ++dh)
                    {
                        int nx = cgx+dh, nz = cgz+dg;
                        if (nx < 0 || nx >= GS || nz < 0 || nz >= GS) continue;
                        int nkey = nz * GS + nx;
                        for (uint32_t j = driveHead_[nkey]; j != UINT32_MAX; j = driveNext_[j])
                        {
                            float ddx = posX_[j] - posX_[i];
                            float ddz = posZ_[j] - posZ_[i];
                            float sep = halfLen_[i] + halfLen_[j] + 1.5f;
                            if (ddx*ddx + ddz*ddz < sep * sep)
                            { blocked = true; break; }
                        }
                    }
                }
                if (blocked) { parkTimer_[i] = 0.5f; continue; }
            }

            // Release parking slot
            if (parkCell_[i] != UINT32_MAX && myParkSlot_[i] != 0xFF)
            {
                int key = (int)parkCell_[i];
                int pgx = key % GS, pgz = key / GS;
                bool isBuildingSlot = (city.GetCellType(pgx, pgz) == CityLayout::CellType::PARKING);
                if (isBuildingSlot)
                {
                    int sl   = (int)myParkSlot_[i];
                    int fl   = sl / CityLayout::PARKING_SPOTS_PER_FLOOR;
                    if (fl >= 0 && fl < CityLayout::PARKING_FLOORS)
                        city.parkingFloorOcc_[key][fl] = std::max(0, city.parkingFloorOcc_[key][fl] - 1);
                    city.AddParkOcc(pgx, pgz, -1);

                    // Place car on adjacent road (necessary transition from building)
                    int rx, rz;
                    if (city.ParkingAdjacentRoad(pgx, pgz, rx, rz))
                    {
                        XMFLOAT2 roadC = city.GridCellCenter(rx, rz);
                        float dirX = city.GridCellCenter(pgx, pgz).x - roadC.x;
                        float dirZ = city.GridCellCenter(pgx, pgz).y - roadC.y;
                        float dirLen = std::sqrt(dirX * dirX + dirZ * dirZ);
                        if (dirLen > 0.1f) { dirX /= dirLen; dirZ /= dirLen; }
                        posX_[i] = roadC.x + dirX * 3.f;
                        posZ_[i] = roadC.y + dirZ * 3.f;
                    }
                }
            }
            parkCell_[i]   = UINT32_MAX;
            myParkSlot_[i] = 0xFF;

            // Re-route using live traffic data
            XMFLOAT2* wp = &wpBuf_[(size_t)i * MAX_WP];
            uint8_t   wc = wpCount_[i];

            int srcGX, srcGZ, dstGX, dstGZ;
            bool haveSrc = city.WorldToGrid(posX_[i], posZ_[i], srcGX, srcGZ);

            bool rerouted = false;

            // Wandering cars pick a random road cell as destination
            if (wandering_[i] && haveSrc)
            {
                constexpr int GS2 = CityLayout::GRID_SIZE;
                // Hash-based random road cell search
                uint32_t rh = (uint32_t)(i * 2654435761u + dayCounter_ * 997u +
                              (uint32_t)(currentHour * 1000.f));
                for (int attempt = 0; attempt < 20; ++attempt)
                {
                    rh = rh * 1664525u + 1013904223u;
                    int rx = (int)(rh % GS2);
                    rh = rh * 1664525u + 1013904223u;
                    int rz = (int)(rh % GS2);
                    auto ct = city.GetCellType(rx, rz);
                    if (ct != CityLayout::CellType::ROAD && ct != CityLayout::CellType::BUS_STOP)
                        continue;
                    // Must be at least 5 cells away for a meaningful trip
                    if (std::abs(rx - srcGX) + std::abs(rz - srcGZ) < 5)
                        continue;
                    auto newPath = city.FindPathRoad(srcGX, srcGZ, rx, rz);
                    if (newPath.size() >= 2)
                    {
                        uint8_t nwc = (uint8_t)std::min(newPath.size(), (size_t)MAX_WP);
                        for (uint8_t w = 0; w < nwc; ++w) wp[w] = newPath[w];
                        wpCount_[i] = nwc;
                        SimplifyPath(wp, wpCount_[i]);
                        rerouted = true;
                        break;
                    }
                }
            }

            // Normal commute: re-route to original home/work destination
            if (!rerouted && haveSrc)
            {
                float d0sq = (posX_[i] - wp[0].x) * (posX_[i] - wp[0].x)
                           + (posZ_[i] - wp[0].y) * (posZ_[i] - wp[0].y);
                float dNsq = (posX_[i] - wp[wc-1].x) * (posX_[i] - wp[wc-1].x)
                           + (posZ_[i] - wp[wc-1].y) * (posZ_[i] - wp[wc-1].y);
                bool nearStart = (d0sq <= dNsq);
                XMFLOAT2 dstPt = nearStart ? wp[wc-1] : wp[0];

                bool haveDst = city.WorldToGrid(dstPt.x, dstPt.y, dstGX, dstGZ);
                if (haveDst && !(srcGX == dstGX && srcGZ == dstGZ))
                {
                    auto newPath = city.FindPathRoad(srcGX, srcGZ, dstGX, dstGZ);
                    if (newPath.size() >= 2)
                    {
                        uint8_t nwc = (uint8_t)std::min(newPath.size(), (size_t)MAX_WP);
                        for (uint8_t w = 0; w < nwc; ++w) wp[w] = newPath[w];
                        wpCount_[i] = nwc;
                        SimplifyPath(wp, wpCount_[i]);
                        rerouted = true;
                    }
                }

                if (!rerouted)
                {
                    if (!nearStart)
                    {
                        for (uint8_t a = 0, b = wc-1; a < b; ++a, --b)
                            std::swap(wp[a], wp[b]);
                    }
                }
            }

            wpCurr_[i] = 1;
            carDir_[i] ^= 1;

            // Set heading toward wp[1] — but do NOT snap position to lane.
            // The car is already on the road from the parking exit above.
            // The Stanley controller will smoothly guide it to the lane centre.
            wc = wpCount_[i];
            if (wc >= 2)
            {
                XMFLOAT2 d0dir = DirTo(wp[0], wp[1]);
                heading_[i] = std::atan2(d0dir.x, d0dir.y);
            }
            speed_[i]  = 0.f;
            state_[i]  = State::DRIVING;

            // Insert into drive linked list so subsequent parked cars see us
            {
                int dgx, dgz;
                if (city.WorldToGrid(posX_[i], posZ_[i], dgx, dgz))
                {
                    int dkey = dgz * GS + dgx;
                    driveNext_[i]    = driveHead_[dkey];
                    driveHead_[dkey] = i;
                }
            }
            continue;
        }

        // ================================================================
        //  DRIVING  –  pure kinematic integration, NO position snaps
        // ================================================================
        XMFLOAT2* wp = &wpBuf_[(size_t)i * MAX_WP];
        uint8_t   wc = wpCount_[i];
        uint8_t&  wn = wpCurr_[i];

        // Reached last waypoint → buses re-route, cars find parking
        if (wn >= wc)
        {
            // ---- Bus re-route: route buses reverse, random buses pick new destination ----
            if (isBus_[i])
            {
                // Route bus: reverse direction
                if (busRouteIdx_[i] >= 0 && busRouteIdx_[i] < (int)busRoutes_.size())
                {
                    auto& route = busRoutes_[busRouteIdx_[i]];
                    int newDir = (busDirection_[i] == 0) ? 1 : 0;
                    busDirection_[i] = (int8_t)newDir;
                    busNextStop_[i] = 0;
                    busStopTimer_[i] = 0.f;
                    const auto& newPath = (newDir == 0) ? route.fullPathAB : route.fullPathBA;
                    if ((int)newPath.size() >= 2) {
                        uint8_t nc = (uint8_t)std::min(newPath.size(), (size_t)MAX_WP);
                        for (uint8_t w = 0; w < nc; ++w) wp[w] = newPath[w];
                        wpCount_[i] = nc;
                        SimplifyPath(wp, wpCount_[i]);
                        wpCurr_[i] = 1;
                        {
                            float lo[3]; int lc;
                            GetLaneOffsets(6, lo, lc);
                            laneIdx_[i] = (int8_t)(lc - 1);
                            laneOff_[i] = lo[lc - 1];
                            laneTarget_[i] = laneOff_[i];
                        }
                        continue;
                    }
                }

                // Fallback: reverse path
                for (uint8_t a = 0, b = wc - 1; a < b; ++a, --b)
                    std::swap(wp[a], wp[b]);
                wpCurr_[i] = 1;
                carDir_[i] ^= 1;
                continue;
            }

            bool parked = false;

            {
                constexpr int PARK_BLDG_RADIUS = 12;
                int destGX, destGZ;
                if (city.WorldToGrid(wp[wc-1].x, wp[wc-1].y, destGX, destGZ))
                {
                    float bestDist2 = 1e9f;
                    int bestPGX = -1, bestPGZ = -1;
                    for (int dz = -PARK_BLDG_RADIUS; dz <= PARK_BLDG_RADIUS && !parked; ++dz)
                    for (int dx = -PARK_BLDG_RADIUS; dx <= PARK_BLDG_RADIUS && !parked; ++dx)
                    {
                        int pgx = destGX + dx, pgz = destGZ + dz;
                        if (!city.ParkingHasRoom(pgx, pgz)) continue;
                        float d2 = (float)(dx*dx + dz*dz);
                        if (d2 < bestDist2) { bestDist2 = d2; bestPGX = pgx; bestPGZ = pgz; }
                    }
                    if (bestPGX >= 0)
                    {
                        XMFLOAT2 pkCenter = city.GridCellCenter(bestPGX, bestPGZ);
                        int freeFl = 0;
                        int pidx = bestPGZ * GS + bestPGX;
                        for (int f = 0; f < CityLayout::PARKING_FLOORS; ++f)
                        {
                            if (city.parkingFloorOcc_[pidx][f] < CityLayout::PARKING_SPOTS_PER_FLOOR)
                            { freeFl = f; break; }
                        }

                        int slotN  = city.parkingFloorOcc_[pidx][freeFl] % CityLayout::PARKING_SPOTS_PER_FLOOR;
                        int col = slotN % 3;
                        int row = slotN / 3;
                        float xOff = (col - 1) * 5.0f;
                        float zOff = (row == 0) ? -3.0f : 3.0f;

                        // Parking is the ONE place we set position directly
                        // (car disappears into building)
                        posX_[i]       = pkCenter.x + xOff;
                        posZ_[i]       = pkCenter.y + zOff;
                        speed_[i]      = 0.f;
                        heading_[i]    = XM_PIDIV2;
                        state_[i]      = State::PARKED;
                        parkTimer_[i]  = PARK_PAUSE;

                        city.parkingFloorOcc_[pidx][freeFl]++;
                        city.AddParkOcc(bestPGX, bestPGZ, 1);
                        parkCell_[i]   = (uint32_t)pidx;
                        myParkSlot_[i] = (uint8_t)(freeFl * CityLayout::PARKING_SPOTS_PER_FLOOR + slotN);
                        parked = true;

                        // Wandering cars: short stop, then maybe keep wandering
                        if (wandering_[i])
                        {
                            parkTimer_[i] = 3.0f + (float)(i % 5) * 2.0f; // 3-13s stop
                            // Chance to go home
                            uint32_t wh = (uint32_t)(i * 2654435761u + dayCounter_ * 53u);
                            float goHome = (float)(wh & 0xFFFF) / 65535.f;
                            if (goHome < WANDER_HOME_CHANCE)
                                wandering_[i] = false; // will re-route home normally
                        }
                    }
                }
            }

            if (!parked)
            {
                for (uint8_t a = 0, b = wc - 1; a < b; ++a, --b)
                    std::swap(wp[a], wp[b]);
                wpCurr_[i] = 1;
                carDir_[i] ^= 1;
            }
            continue;
        }

        // ---- Segment geometry ----
        XMFLOAT2 prev_raw   = (wn > 0) ? wp[wn-1] : wp[0];
        XMFLOAT2 target_raw = wp[wn];
        XMFLOAT2 segDir     = DirTo(prev_raw, target_raw);
        float segLen = std::sqrt((target_raw.x - prev_raw.x) * (target_raw.x - prev_raw.x) +
                                 (target_raw.y - prev_raw.y) * (target_raw.y - prev_raw.y));

        float myFwdX = std::sin(heading_[i]);
        float myFwdZ = std::cos(heading_[i]);

        int myGX = 0, myGZ = 0;
        bool onGrid = city.WorldToGrid(posX_[i], posZ_[i], myGX, myGZ);

        // ---- Turn detection ----
        XMFLOAT2 nextSeg = segDir;
        float cornerCos = 1.0f;
        float turnCross = 0.0f;
        if (wn + 1 < wc)
        {
            nextSeg = DirTo(target_raw, wp[wn + 1]);
            cornerCos = segDir.x * nextSeg.x + segDir.y * nextSeg.y;
            turnCross = segDir.x * nextSeg.y - segDir.y * nextSeg.x;
        }
        bool isTurn = (cornerCos < 0.3f);

        // ---- Dynamic lane offsets ----
        // At intersections where two roads with different lane counts cross,
        // use the lane count from the road the car is actually traveling on
        // (look at the adjacent cell behind us along the segment direction).
        float cellLaneOffs[3]; int cellLaneCount = 0;
        {
            int totalLanes = 6;
            if (onGrid) {
                totalLanes = city.GetRoadLanes(myGX, myGZ);
                // If at an intersection, resolve lane count from the road
                // behind us so we keep the same lane config through the crossing
                if (lights.IsIntersection(myGX, myGZ)) {
                    int stepX = (int)std::round(segDir.x);
                    int stepZ = (int)std::round(segDir.y);
                    for (int s = 1; s <= 3; ++s) {
                        int backGX = myGX - stepX * s;
                        int backGZ = myGZ - stepZ * s;
                        auto bct = city.GetCellType(backGX, backGZ);
                        if (bct == CityLayout::CellType::ROAD &&
                            !lights.IsIntersection(backGX, backGZ)) {
                            totalLanes = city.GetRoadLanes(backGX, backGZ);
                            break;
                        }
                    }
                }
            }
            GetLaneOffsets(totalLanes, cellLaneOffs, cellLaneCount);
        }

        // Clamp lane index to valid range for this cell
        if (laneIdx_[i] >= cellLaneCount) laneIdx_[i] = (int8_t)(cellLaneCount - 1);
        if (laneIdx_[i] < 0) laneIdx_[i] = 0;

        // Derive lane target from lane index
        laneTarget_[i] = cellLaneOffs[laneIdx_[i]];

        // Smooth lane transition: blend laneOff toward laneTarget
        {
            float laneDiff = laneTarget_[i] - laneOff_[i];
            float maxLateral = 5.0f * dt;
            if (std::abs(laneDiff) > maxLateral)
                laneOff_[i] += (laneDiff > 0.0f ? maxLateral : -maxLateral);
            else
                laneOff_[i] = laneTarget_[i];
        }

        // No lane changes inside intersection cells
        bool inIntersection = onGrid && lights.IsIntersection(myGX, myGZ);
        bool laneSettled = std::abs(laneOff_[i] - laneTarget_[i]) < 0.1f;

        // Forward projection along segment (needed early for turn approach check)
        float segFwd = (posX_[i] - prev_raw.x) * segDir.x + (posZ_[i] - prev_raw.y) * segDir.y;

        // Allow lane changes on the straight approach portion of a turn segment
        // (before the arc entry point), so cars can get into the correct lane
        float arcEntryDist = isTurn ? std::max(0.0f, segLen - HALF_CS) : 1e6f;
        bool onTurnApproach = isTurn && segFwd < arcEntryDist - 5.0f;
        bool canChangeLane = (!isTurn || onTurnApproach) && !inIntersection && onGrid && laneSettled;

        // Helper: check if a target lane is clear of nearby same-direction cars
        auto isLaneClear = [&](float targetLane) -> bool {
            for (int dg = -1; dg <= 1; ++dg)
            for (int dh2 = -1; dh2 <= 1; ++dh2)
            {
                int cgx = myGX + dh2, cgz = myGZ + dg;
                if (cgx < 0 || cgx >= GS || cgz < 0 || cgz >= GS) continue;
                int cellKey = cgz * GS + cgx;
                for (uint32_t j = driveHead_[cellKey]; j != UINT32_MAX; j = driveNext_[j])
                {
                    if (j == i) continue;
                    float jFwdX = std::sin(heading_[j]);
                    float jFwdZ = std::cos(heading_[j]);
                    float hcos  = myFwdX * jFwdX + myFwdZ * jFwdZ;
                    if (hcos < 0.5f) continue;
                    if (std::abs(laneOff_[j] - targetLane) > 1.0f) continue;
                    float odx = posX_[j] - posX_[i];
                    float odz = posZ_[j] - posZ_[i];
                    float fwd = odx * myFwdX + odz * myFwdZ;
                    if (fwd > -8.f && fwd < 15.f) return false;
                }
            }
            return true;
        };

        // ---- Turn lane preparation (uses lane index) ----
        // Left turn → leftmost lane (0), right turn → rightmost lane (N-1),
        // straight → any lane (pick least busy)
        int8_t requiredIdx = laneIdx_[i];
        bool turnPrepNeeded = false;
        float distToTurnWp = 1e6f;

        // Current turn: left → inner (0), right → outer (N-1)
        if (isTurn)
        {
            requiredIdx = (turnCross > 0.0f) ? 0 : (int8_t)(cellLaneCount - 1);
            turnPrepNeeded = true;
            float tdx = target_raw.x - posX_[i], tdz = target_raw.y - posZ_[i];
            distToTurnWp = std::sqrt(tdx * tdx + tdz * tdz);
        }

        // Look ahead for upcoming turns (8 segments for early lane change)
        if (!turnPrepNeeded)
        {
            float tdx0 = target_raw.x - posX_[i], tdz0 = target_raw.y - posZ_[i];
            float accumDist = std::sqrt(tdx0 * tdx0 + tdz0 * tdz0);
            for (uint8_t look = 1; look <= 8 && (wn + look) < wc; ++look)
            {
                uint8_t futIdx = wn + look;
                if (futIdx + 1 >= wc) break;
                XMFLOAT2 fSeg = DirTo(wp[futIdx - 1], wp[futIdx]);
                XMFLOAT2 fNext = DirTo(wp[futIdx], wp[futIdx + 1]);
                float fCos = fSeg.x * fNext.x + fSeg.y * fNext.y;
                float fCross = fSeg.x * fNext.y - fSeg.y * fNext.x;
                if (fCos < 0.3f)
                {
                    requiredIdx = (fCross > 0.0f) ? 0 : (int8_t)(cellLaneCount - 1);
                    turnPrepNeeded = true;
                    distToTurnWp = accumDist;
                    break;
                }
                float sdx = wp[futIdx].x - wp[futIdx - 1].x;
                float sdz = wp[futIdx].y - wp[futIdx - 1].y;
                accumDist += std::sqrt(sdx * sdx + sdz * sdz);
            }
        }

        // Execute lane change for turn preparation (only on safe straights)
        if (turnPrepNeeded && laneIdx_[i] != requiredIdx
            && canChangeLane && isLaneClear(cellLaneOffs[requiredIdx]))
        {
            laneIdx_[i] = requiredIdx;
            laneTarget_[i] = cellLaneOffs[laneIdx_[i]];
        }

        // Wrong lane for upcoming turn: car must slow down to change before intersection
        bool wrongLaneForTurn = turnPrepNeeded && (laneIdx_[i] != requiredIdx);

        // Balance lane distribution: no turn ahead → prefer less crowded lane
        if (!turnPrepNeeded && canChangeLane && cellLaneCount > 1)
        {
            int laneCounts[3] = {0, 0, 0};
            for (int dg = -1; dg <= 1; ++dg)
            for (int dh2 = -1; dh2 <= 1; ++dh2)
            {
                int cgx = myGX + dh2, cgz = myGZ + dg;
                if (cgx < 0 || cgx >= GS || cgz < 0 || cgz >= GS) continue;
                int cellKey = cgz * GS + cgx;
                for (uint32_t j = driveHead_[cellKey]; j != UINT32_MAX; j = driveNext_[j])
                {
                    if (j == i) continue;
                    float jFwdX = std::sin(heading_[j]);
                    float jFwdZ = std::cos(heading_[j]);
                    float hcos = myFwdX * jFwdX + myFwdZ * jFwdZ;
                    if (hcos < 0.5f) continue;
                    float odx = posX_[j] - posX_[i];
                    float odz = posZ_[j] - posZ_[i];
                    float fwd = odx * myFwdX + odz * myFwdZ;
                    if (fwd < 0.f || fwd > 40.f) continue;
                    for (int l = 0; l < cellLaneCount; ++l)
                    {
                        if (std::abs(laneOff_[j] - cellLaneOffs[l]) < 1.5f)
                        { laneCounts[l]++; break; }
                    }
                }
            }
            int myCount = laneCounts[laneIdx_[i]];
            int bestIdx = laneIdx_[i];
            int bestCount = myCount;
            for (int l = 0; l < cellLaneCount; ++l)
            {
                if (l != laneIdx_[i] && laneCounts[l] < bestCount - 1)
                { bestCount = laneCounts[l]; bestIdx = l; }
            }
            if (bestIdx != laneIdx_[i] && isLaneClear(cellLaneOffs[bestIdx]))
            {
                laneIdx_[i] = (int8_t)bestIdx;
                laneTarget_[i] = cellLaneOffs[laneIdx_[i]];
            }
        }

        // ---- Compute target point for steering ----
        //   Two-phase turn model:
        //     Phase 1 (approach): drive straight with Stanley until the
        //       arc entry point (HALF_CS before the turn waypoint).
        //     Phase 2 (arc): follow circular arc through the intersection.
        //   Straight segments use Stanley controller throughout.
        XMFLOAT2 steerTarget;
        float exitX = 0, exitZ = 0;
        float exitNX = 0, exitNZ = 0;
        bool inArc = false;

        if (isTurn)
        {
            float inRX =  segDir.y, inRZ = -segDir.x;
            float outRX = nextSeg.y, outRZ = -nextSeg.x;

            // Use laneTarget_ (stable) for arc geometry — NOT laneOff_ which
            // may be mid-transition, causing the arc to wobble frame-to-frame.
            XMFLOAT2 entryPt = {
                target_raw.x - segDir.x * HALF_CS + inRX * laneTarget_[i],
                target_raw.y - segDir.y * HALF_CS + inRZ * laneTarget_[i]
            };
            XMFLOAT2 exitPt = {
                target_raw.x + nextSeg.x * HALF_CS + outRX * laneTarget_[i],
                target_raw.y + nextSeg.y * HALF_CS + outRZ * laneTarget_[i]
            };

            // Distance along segment where the arc entry sits
            float entryDist = std::max(0.0f, segLen - HALF_CS);

            if (segFwd >= entryDist - 1.0f)
            {
                // ---- ARC PHASE: car is at/past the intersection entry ----
                inArc = true;
                // Commit lane offset — no more smooth transition during the arc
                laneOff_[i] = laneTarget_[i];

                float entToExit_x = exitPt.x - entryPt.x;
                float entToExit_z = exitPt.y - entryPt.y;
                float chordLen = std::sqrt(entToExit_x * entToExit_x + entToExit_z * entToExit_z);
                float R = std::max(1.0f, chordLen * 0.7071f);

                float perpX, perpZ;
                // Right turn: center is to the right of travel (inside of turn)
                // Left turn: center is to the left of travel (inside of turn)
                if (turnCross < 0.0f) { perpX =  inRX; perpZ =  inRZ; }
                else                  { perpX = -inRX; perpZ = -inRZ; }
                float arcCX = entryPt.x + perpX * R;
                float arcCZ = entryPt.y + perpZ * R;

                float carAngle = std::atan2(posX_[i] - arcCX, posZ_[i] - arcCZ);
                float entAngle = std::atan2(entryPt.x - arcCX, entryPt.y - arcCZ);
                float extAngle = std::atan2(exitPt.x  - arcCX, exitPt.y  - arcCZ);

                auto normAngle = [](float a) {
                    while (a >  XM_PI) a -= XM_2PI;
                    while (a < -XM_PI) a += XM_2PI;
                    return a;
                };
                float arcSpan = normAngle(extAngle - entAngle);
                float carSpan = normAngle(carAngle - entAngle);
                float tArc = (std::abs(arcSpan) > 0.01f)
                               ? std::clamp(carSpan / arcSpan, 0.0f, 1.0f)
                               : 0.0f;

                float tLook = std::min(1.0f, tArc + 0.30f);
                float lookAngle = entAngle + arcSpan * tLook;
                steerTarget.x = arcCX + R * std::sin(lookAngle);
                steerTarget.y = arcCZ + R * std::cos(lookAngle);

                exitX  = exitPt.x;  exitZ  = exitPt.y;
                exitNX = nextSeg.x; exitNZ = nextSeg.y;

                float crossDot = (posX_[i] - exitX) * exitNX + (posZ_[i] - exitZ) * exitNZ;
                if (crossDot >= -0.5f && tArc > 0.75f)
                {
                    heading_[i] = std::atan2(nextSeg.x, nextSeg.y);
                    ++wn;
                    continue;
                }
            }
            else
            {
                // ---- APPROACH PHASE: drive straight toward the arc entry ----
                float rightX = segDir.y, rightZ = -segDir.x;
                float lookAhead = std::clamp(speed_[i] * 0.8f, 3.0f, 12.0f);
                float tgtFwd = std::min(segFwd + lookAhead, entryDist);
                steerTarget.x = prev_raw.x + segDir.x * tgtFwd + rightX * laneOff_[i];
                steerTarget.y = prev_raw.y + segDir.y * tgtFwd + rightZ * laneOff_[i];
                // No waypoint advance in approach phase
            }
        }
        else
        {
            // ---- STRAIGHT SEGMENT ----
            float rightX = segDir.y, rightZ = -segDir.x;
            float lookAhead = std::clamp(speed_[i] * 0.8f, 3.0f, 12.0f);
            float tgtFwd = std::min(segFwd + lookAhead, segLen);
            steerTarget.x = prev_raw.x + segDir.x * tgtFwd + rightX * laneOff_[i];
            steerTarget.y = prev_raw.y + segDir.y * tgtFwd + rightZ * laneOff_[i];

            exitX  = target_raw.x; exitZ  = target_raw.y;
            exitNX = segDir.x;     exitNZ = segDir.y;

            float crossDot = (posX_[i] - exitX) * exitNX + (posZ_[i] - exitZ) * exitNZ;
            if (crossDot >= -0.3f && segFwd >= segLen - 1.0f)
            {
                ++wn;
                continue;
            }
        }

        // ---- Distance to steer target ----
        float stDx   = steerTarget.x - posX_[i];
        float stDz   = steerTarget.y - posZ_[i];
        float stDist = std::sqrt(stDx * stDx + stDz * stDz);

        // ---- Distance to raw waypoint (for traffic light distance calc) ----
        float rawDx = target_raw.x - posX_[i];
        float rawDz = target_raw.y - posZ_[i];
        float rawDist = std::sqrt(rawDx * rawDx + rawDz * rawDz);

        // ---- Find nearest car ahead (3x3 neighbourhood, lane-aware) ----
        float bestGap = 1e6f;
        float bestSpd = MAX_SPEED;

        if (onGrid)
        {
            for (int dg = -1; dg <= 1; ++dg)
            for (int dh = -1; dh <= 1; ++dh)
            {
                int cgx = myGX + dh;
                int cgz = myGZ + dg;
                if (cgx < 0 || cgx >= GS || cgz < 0 || cgz >= GS) continue;
                int cellKey = cgz * GS + cgx;

                for (uint32_t j = driveHead_[cellKey]; j != UINT32_MAX; j = driveNext_[j])
                {
                    if (j == i) continue;

                    float odx = posX_[j] - posX_[i];
                    float odz = posZ_[j] - posZ_[i];

                    float fwd = odx * myFwdX + odz * myFwdZ;
                    if (fwd <= 0.f) continue;

                    float lat = std::abs(odx * myFwdZ - odz * myFwdX);
                    float latBand = LAT_BAND + (isBus_[i] ? 0.5f : 0.f) + (isBus_[j] ? 0.5f : 0.f);
                    if (lat > latBand) continue;

                    // Same-direction check: skip perpendicular & opposing cars
                    float jFwdX = std::sin(heading_[j]);
                    float jFwdZ = std::cos(heading_[j]);
                    float hcos  = myFwdX * jFwdX + myFwdZ * jFwdZ;
                    if (hcos < 0.3f) continue;

                    // Lane-aware: only follow cars in the SAME lane (or during arc/turn)
                    if (!inArc && !inIntersection && hcos > 0.5f)
                    {
                        float laneDiff = std::abs(laneOff_[j] - laneOff_[i]);
                        if (laneDiff > 2.0f) continue;
                    }

                    float gap = fwd - (halfLen_[i] + halfLen_[j] + 1.5f);
                    if (gap < bestGap)
                    {
                        bestGap = gap;
                        bestSpd = speed_[j];
                    }
                }
            }
        }

        // ---- Traffic light stopping ----
        // Virtual stopped car at the stop line feeds into IDM gap.
        // Scan cells along the segment direction to catch ALL intersections
        // (SimplifyPath merges collinear waypoints, so intermediate
        //  intersection cells may not be explicit waypoints).
        if (onGrid)
        {
            // Determine turn intent at the current target waypoint
            using TI = TrafficLightSystem::TurnIntent;
            TI turnIntent = TI::STRAIGHT;
            int turnCellGX = -1, turnCellGZ = -1;
            if (isTurn)
            {
                turnIntent = (turnCross > 0.0f) ? TI::LEFT : TI::RIGHT;
                city.WorldToGrid(target_raw.x, target_raw.y, turnCellGX, turnCellGZ);
            }

            auto checkLight = [&](int cgx, int cgz, float dirX, float dirZ) {
                if (!lights.IsIntersection(cgx, cgz)) return;

                // Use turn intent only for the intersection at the turn waypoint;
                // all other intersections on this segment are traversed straight
                TI localIntent = (cgx == turnCellGX && cgz == turnCellGZ)
                                 ? turnIntent : TI::STRAIGHT;

                auto color = lights.GetLight(cgx, cgz, dirX, dirZ, localIntent);
                if (color == TrafficLightSystem::LightColor::GREEN) return;

                XMFLOAT2 intC = city.GridCellCenter(cgx, cgz);
                float toCX = intC.x - posX_[i];
                float toCZ = intC.y - posZ_[i];
                float fwdDist = toCX * dirX + toCZ * dirZ;
                if (fwdDist < 0.0f) return;
                // Stop at the traffic light pole (13 m from intersection centre)
                // Account for vehicle half-length so the front stops at the line
                float stopLine = fwdDist - HALF_CS - 3.0f - halfLen_[i];
                if (stopLine < -1.0f) return; // already well past stop line, committed to crossing
                float gap = std::max(0.0f, stopLine);

                if (color == TrafficLightSystem::LightColor::RED)
                {
                    if (gap < bestGap) { bestGap = gap; bestSpd = 0.0f; }
                }
                else if (color == TrafficLightSystem::LightColor::YELLOW)
                {
                    if (gap > 3.0f && gap < bestGap) { bestGap = gap; bestSpd = 0.0f; }
                }
            };

            // Don't brake if already inside the intersection
            if (!lights.IsIntersection(myGX, myGZ))
            {
                // Scan cells ahead along the current segment direction
                int prevScanGX = myGX, prevScanGZ = myGZ;
                float scanMax = std::min(80.0f, segLen - segFwd + 20.0f);
                for (float d = HALF_CS; d < scanMax; d += HALF_CS)
                {
                    float scanX = posX_[i] + segDir.x * d;
                    float scanZ = posZ_[i] + segDir.y * d;
                    int sgx, sgz;
                    if (!city.WorldToGrid(scanX, scanZ, sgx, sgz)) continue;
                    if (sgx == prevScanGX && sgz == prevScanGZ) continue;
                    prevScanGX = sgx; prevScanGZ = sgz;
                    checkLight(sgx, sgz, segDir.x, segDir.y);
                }
            }

            // ---- Gridlock prevention: don't enter intersection if exit is blocked ----
            // "Don't block the box" — treat a jammed exit as a virtual red light
            if (!inArc && !lights.IsIntersection(myGX, myGZ))
            {
                int prevScanGX2 = myGX, prevScanGZ2 = myGZ;
                for (float d = HALF_CS; d < 60.0f; d += HALF_CS)
                {
                    float scanX = posX_[i] + segDir.x * d;
                    float scanZ = posZ_[i] + segDir.y * d;
                    int sgx, sgz;
                    if (!city.WorldToGrid(scanX, scanZ, sgx, sgz)) break;
                    if (sgx == prevScanGX2 && sgz == prevScanGZ2) continue;
                    prevScanGX2 = sgx; prevScanGZ2 = sgz;

                    if (!lights.IsIntersection(sgx, sgz)) continue;

                    // Found an intersection ahead — check the exit cell beyond it
                    int exitCX = sgx + (int)std::round(segDir.x);
                    int exitCZ = sgz + (int)std::round(segDir.y);
                    if (isTurn) {
                        exitCX = sgx + (int)std::round(nextSeg.x);
                        exitCZ = sgz + (int)std::round(nextSeg.y);
                    }
                    if (exitCX < 0 || exitCX >= GS || exitCZ < 0 || exitCZ >= GS) break;

                    // Count stopped/slow cars in exit cell heading same direction
                    int exitKey = exitCZ * GS + exitCX;
                    int jamCount = 0;
                    for (uint32_t j2 = driveHead_[exitKey]; j2 != UINT32_MAX; j2 = driveNext_[j2])
                    {
                        if (speed_[j2] >= 1.0f) continue;
                        float j2Fx = std::sin(heading_[j2]);
                        float j2Fz = std::cos(heading_[j2]);
                        float dirDot = segDir.x * j2Fx + segDir.y * j2Fz;
                        if (dirDot > 0.3f) jamCount++;
                    }
                    // Scale threshold by lane count: need real blockage, not just 2 cars
                    int exitLanes = city.GetRoadLanes(exitCX, exitCZ);
                    int jamThreshold = exitLanes * 2;
                    if (jamCount >= jamThreshold)
                    {
                        // Treat as virtual stop before the intersection
                        XMFLOAT2 intC2 = city.GridCellCenter(sgx, sgz);
                        float toCX2 = intC2.x - posX_[i];
                        float toCZ2 = intC2.y - posZ_[i];
                        float fwdDist2 = toCX2 * segDir.x + toCZ2 * segDir.y;
                        if (fwdDist2 > 0.0f) {
                            float stopDist = fwdDist2 - HALF_CS - 3.0f;
                            if (stopDist > -1.0f && stopDist < bestGap) { bestGap = std::max(0.0f, stopDist); bestSpd = 0.0f; }
                        }
                    }
                    break; // only check nearest intersection
                }
            }
        }

        // ---- Pedestrian safety brake: yield to peds in our path ----
        if (peds && peds->count > 0 && onGrid) {
            float scanAhead = std::min(40.0f, speed_[i] * 3.5f + 8.0f);
            float fwdX = std::sin(heading_[i]);
            float fwdZ = std::cos(heading_[i]);
            for (int dg = -2; dg <= 2; dg++)
            for (int dh = -2; dh <= 2; dh++) {
                int cgx = myGX + dh, cgz = myGZ + dg;
                if (cgx < 0 || cgx >= GS || cgz < 0 || cgz >= GS) continue;
                int cellKey = cgz * GS + cgx;
                for (uint32_t p = peds->cellHead[cellKey]; p != UINT32_MAX; p = peds->cellNext[p]) {
                    if (peds->state[p] == 0) continue; // IDLE
                    // Skip peds safely on the sidewalk — only brake for peds on road/crosswalk
                    if (city.IsOnSidewalk(peds->posX[p], peds->posZ[p])) continue;
                    float pdx = peds->posX[p] - posX_[i];
                    float pdz = peds->posZ[p] - posZ_[i];

                    // Method 1: Forward projection
                    float fwd = pdx * fwdX + pdz * fwdZ;
                    float lat = std::abs(pdx * fwdZ - pdz * fwdX);
                    float latThresh = (inIntersection || isTurn || inArc)
                        ? (CAR_HW + 0.25f + 4.0f)
                        : (CAR_HW + 0.25f + 1.0f);
                    if (fwd > 0.3f && fwd <= scanAhead && lat <= latThresh) {
                        float stopDist = std::max(0.0f, fwd - halfLen_[i] - 0.25f - 2.0f);
                        if (stopDist < bestGap) { bestGap = stopDist; bestSpd = 0.0f; }
                    }

                    // Method 2: Radius check at intersections/turns
                    if (inIntersection || inArc) {
                        float d2 = pdx * pdx + pdz * pdz;
                        float safeR = halfLen_[i] + 0.25f + 4.5f;
                        if (d2 < safeR * safeR && d2 > 0.01f) {
                            float dd = std::sqrt(d2);
                            float stopDist = std::max(0.0f, dd - halfLen_[i] - 0.25f - 1.0f);
                            if (stopDist < bestGap) { bestGap = stopDist; bestSpd = 0.0f; }
                        }
                    }

                    // Method 3: Approaching turn — detect peds at next intersection
                    if (!inIntersection && isTurn && fwd > -1.0f) {
                        float pedToDst = std::sqrt(
                            (peds->posX[p] - target_raw.x) * (peds->posX[p] - target_raw.x) +
                            (peds->posZ[p] - target_raw.y) * (peds->posZ[p] - target_raw.y));
                        if (pedToDst < HALF_CS + SIDEWALK_MID) {
                            float myToDst = std::sqrt(
                                (posX_[i] - target_raw.x) * (posX_[i] - target_raw.x) +
                                (posZ_[i] - target_raw.y) * (posZ_[i] - target_raw.y));
                            float stopDist = std::max(0.0f, myToDst - HALF_CS - 2.0f);
                            if (stopDist < bestGap) { bestGap = stopDist; bestSpd = 0.0f; }
                        }
                    }
                }
            }
        }

        // ---- IDM (Intelligent Driver Model) acceleration ----
        float v = speed_[i];
        float vRatio = v / MAX_SPEED;
        float vR4 = vRatio * vRatio * vRatio * vRatio;
        float idmAccel;

        if (bestGap < 1e5f)
        {
            float idm_2ab = 2.f * std::sqrt(ACCEL * IDM_B);
            float dv     = v - bestSpd;
            float s_star = IDM_S0 + std::max(0.f, v * IDM_T + v * dv / idm_2ab);
            float gap    = std::max(bestGap, 0.01f);
            idmAccel     = ACCEL * (1.f - vR4 - (s_star / gap) * (s_star / gap));
        }
        else
        {
            idmAccel = ACCEL * (1.f - vR4);
        }

        // ---- Corner speed limit ----
        float desiredMax = MAX_SPEED;
        if (inArc)
            desiredMax = MAX_SPEED * 0.30f;
        else if (isTurn)
        {
            // Approaching a turn: smoothly ramp speed down over the last 25m
            float distToEntry = std::max(0.0f, segLen - segFwd - HALF_CS);
            float approachFactor = std::clamp(distToEntry / 25.0f, 0.0f, 1.0f);
            desiredMax = MAX_SPEED * (0.30f + 0.70f * approachFactor);
        }

        // Wrong lane for turn: slow down and stop ONLY if the correct lane is available
        if (wrongLaneForTurn && !inArc)
        {
            float urgency = 1.0f - std::clamp(distToTurnWp / 60.0f, 0.0f, 1.0f);
            float laneChangeMax = MAX_SPEED * (1.0f - urgency * 0.85f);
            if (distToTurnWp < 25.0f)
                laneChangeMax = std::min(laneChangeMax, 2.0f);
            desiredMax = std::min(desiredMax, laneChangeMax);

            // Near the arc: stop and force lane change IF the target lane has room.
            // If the lane is blocked, don't stop — just proceed at reduced speed.
            if (isTurn)
            {
                float distToArc = arcEntryDist - segFwd;
                if (distToArc < 15.0f && isLaneClear(cellLaneOffs[requiredIdx]))
                {
                    float stopDist = std::max(0.0f, distToArc - halfLen_[i] - 2.0f);
                    if (stopDist < bestGap) { bestGap = stopDist; bestSpd = 0.0f; }
                    laneIdx_[i] = requiredIdx;
                    laneTarget_[i] = cellLaneOffs[requiredIdx];
                }
            }
        }

        if (v > desiredMax)
            idmAccel = std::min(idmAccel, -IDM_B * 1.5f);

        // Hard brake for very small gaps — only when lead vehicle is
        // truly stopped, and allow creep once gap opens a little
        if (bestGap < 0.5f && bestSpd < 0.1f)
            idmAccel = -DECEL;
        else if (bestGap < 3.0f && bestSpd < 0.5f)
            idmAccel = std::min(idmAccel, -IDM_B * 2.0f);

        // Apply acceleration
        speed_[i] = std::clamp(v + idmAccel * dt, 0.0f, desiredMax);

        // Allow slow creep instead of permanent hard-stop
        if (bestGap <= -0.5f && bestSpd < 0.1f)
            speed_[i] = std::min(speed_[i], 0.5f);

        // Cars inside an intersection creep through to clear the box, but only if room ahead
        if (inIntersection && speed_[i] < 2.0f && bestGap > 5.0f)
            speed_[i] = std::min(2.0f, desiredMax);

        // ---- Proactive lane switching (congestion avoidance) ----
        if (!isTurn && !inArc && !inIntersection && speed_[i] < MAX_SPEED * 0.25f
            && bestGap < 10.f && onGrid && !turnPrepNeeded
            && std::abs(laneOff_[i] - laneTarget_[i]) < 0.1f)
        {
            int8_t otherIdx = (laneIdx_[i] == 0) ? (int8_t)(cellLaneCount - 1) : (int8_t)0;
            if (isLaneClear(cellLaneOffs[otherIdx]))
            {
                laneIdx_[i] = otherIdx;
                laneTarget_[i] = cellLaneOffs[otherIdx];
            }
        }

        // ================================================================
        //  STEERING + MOVEMENT — the ONLY place position changes
        //  pos += forward(heading) * speed * dt
        // ================================================================
        if (speed_[i] > 0.001f && stDist > 0.01f)
        {
            float step = speed_[i] * dt;

            if (!inArc)
            {
                // ---------- STANLEY LATERAL CONTROLLER ----------
                float segH = std::atan2(segDir.x, segDir.y);
                float rightX = segDir.y, rightZ = -segDir.x;

                float segFwd2 = (posX_[i] - prev_raw.x) * segDir.x
                              + (posZ_[i] - prev_raw.y) * segDir.y;
                float laneX = prev_raw.x + segDir.x * segFwd2 + rightX * laneOff_[i];
                float laneZ = prev_raw.y + segDir.y * segFwd2 + rightZ * laneOff_[i];
                float cte = (posX_[i] - laneX) * rightX + (posZ_[i] - laneZ) * rightZ;

                constexpr float K_STANLEY = 3.5f;
                float steerCorr = -std::atan2(K_STANLEY * cte, speed_[i] + 0.5f);
                // Stronger correction when off-center or changing lanes
                float maxCorr;
                if (std::abs(cte) > 1.0f)
                    maxCorr = 0.40f;  // far off-center: strong correction
                else if (std::abs(laneTarget_[i] - laneOff_[i]) > 0.3f)
                    maxCorr = 0.30f;  // lane change in progress
                else
                    maxCorr = 0.18f;  // normal tracking
                steerCorr = std::clamp(steerCorr, -maxCorr, maxCorr);

                heading_[i] = segH + steerCorr;

                float fwdX = std::sin(heading_[i]);
                float fwdZ = std::cos(heading_[i]);
                posX_[i] += fwdX * step;
                posZ_[i] += fwdZ * step;
            }
            else
            {
                // ---------- TURN: smooth pursuit steering ----------
                // Desired heading = direction toward steer target
                float desiredH = std::atan2(stDx, stDz);
                float dh = desiredH - heading_[i];
                while (dh >  XM_PI) dh -= XM_2PI;
                while (dh < -XM_PI) dh += XM_2PI;

                // Smooth steering: blend toward desired heading.
                // Rate scales with speed to allow tight turns at low speed.
                float maxSteerRate = 5.0f;  // rad/s
                float steerDelta = std::clamp(dh, -maxSteerRate * dt, maxSteerRate * dt);
                // Also allow a proportion-based blend for responsiveness
                float blendDelta = dh * std::min(1.0f, 8.0f * dt);
                // Use whichever is larger in magnitude (but capped)
                if (std::abs(blendDelta) > std::abs(steerDelta))
                    steerDelta = std::clamp(blendDelta, -maxSteerRate * dt * 1.5f, maxSteerRate * dt * 1.5f);

                heading_[i] += steerDelta;

                float fwdX = std::sin(heading_[i]);
                float fwdZ = std::cos(heading_[i]);
                posX_[i] += fwdX * step;
                posZ_[i] += fwdZ * step;
            }
        }

        // ---- Economy ----
        if (carDir_[i] == 1)
        {
            float earned  = wage_[i] * dt;
            money_[i]    += earned;
            taxAccum_    += earned * TAX_RATE;
        }
    } // end per-car

    // ==========================================================
    //  Soft collision resolution — speed-only + gentle nudge
    //  No hard position snaps. Only slow down overlapping cars
    //  and apply a tiny separation force (max 0.5m/frame).
    // ==========================================================
    std::fill(driveHead_.begin(), driveHead_.end(), UINT32_MAX);
    std::fill(driveNext_.begin(), driveNext_.end(), UINT32_MAX);
    for (uint32_t i = 0; i < activeCount; ++i)
    {
        if (state_[i] != State::DRIVING && state_[i] != State::ENTERING_PARKING) continue;
        int gx, gz;
        if (city.WorldToGrid(posX_[i], posZ_[i], gx, gz))
        {
            int key = gz * GS + gx;
            driveNext_[i] = driveHead_[key];
            driveHead_[key] = i;
        }
    }

    auto resolvePair = [&](uint32_t a, uint32_t b) {
        float ddx = posX_[b] - posX_[a];
        float ddz = posZ_[b] - posZ_[a];
        float dd2 = ddx * ddx + ddz * ddz;
        float sep = halfLen_[a] + halfLen_[b] + 1.5f;
        if (dd2 >= sep * sep || dd2 < 0.001f) return;

        float hcos = std::sin(heading_[a]) * std::sin(heading_[b])
                   + std::cos(heading_[a]) * std::cos(heading_[b]);
        bool bothDriving = (state_[a] == State::DRIVING && state_[b] == State::DRIVING);
        if (bothDriving && hcos < 0.3f) return;

        if (bothDriving)
        {
            float sinH = std::sin(heading_[a]), cosH = std::cos(heading_[a]);
            float lat  = std::abs(ddx * cosH - ddz * sinH);
            float pairLatBand = LAT_BAND + (isBus_[a] ? 0.5f : 0.f) + (isBus_[b] ? 0.5f : 0.f);
            if (lat > pairLatBand) return;
            // Also skip if clearly in different lanes (lane offset difference > 2.0)
            float laneDiff = std::abs(laneOff_[a] - laneOff_[b]);
            if (hcos > 0.5f && laneDiff > 2.0f) return;
        }

        float dd = std::sqrt(dd2);
        float overlap = sep - dd;
        float nx = ddx / dd, nz = ddz / dd;

        // Project push onto the FORWARD axis of car A (longitudinal only)
        // This prevents cars from being shoved sideways out of their lane
        float sinH = std::sin(heading_[a]), cosH = std::cos(heading_[a]);
        float pushFwd = nx * sinH + nz * cosH;
        float pushMag = std::min(overlap * 0.3f, 0.3f);
        float pushX = sinH * pushFwd * pushMag;
        float pushZ = cosH * pushFwd * pushMag;
        // Only push if magnitude is meaningful
        if (std::abs(pushFwd) > 0.1f) {
            posX_[a] -= pushX;
            posZ_[a] -= pushZ;
            posX_[b] += pushX;
            posZ_[b] += pushZ;
        }

        // Slow down the car that's behind
        float fwd = ddx * sinH + ddz * cosH;
        if (fwd > 0.f)
            speed_[a] = std::min(speed_[a], speed_[b] * 0.5f);
        else
            speed_[b] = std::min(speed_[b], speed_[a] * 0.5f);
    };

    for (int cy = 0; cy < GS; ++cy)
    for (int cx = 0; cx < GS; ++cx)
    {
        int key = cy * GS + cx;
        if (driveHead_[key] == UINT32_MAX) continue;

        for (uint32_t a = driveHead_[key]; a != UINT32_MAX; a = driveNext_[a])
            for (uint32_t b = driveNext_[a]; b != UINT32_MAX; b = driveNext_[b])
                resolvePair(a, b);

        static const int NBR[][2] = {{1,0}, {0,1}, {1,1}, {-1,1}};
        for (const auto& nb : NBR)
        {
            int nx2 = cx + nb[0], nz2 = cy + nb[1];
            if (nx2 < 0 || nx2 >= GS || nz2 < 0 || nz2 >= GS) continue;
            int nkey = nz2 * GS + nx2;
            if (driveHead_[nkey] == UINT32_MAX) continue;

            for (uint32_t a = driveHead_[key]; a != UINT32_MAX; a = driveNext_[a])
                for (uint32_t b = driveHead_[nkey]; b != UINT32_MAX; b = driveNext_[b])
                    resolvePair(a, b);
        }
    }

 } // end substep loop
}

// ============================================================
//  CreateInstPool
// ============================================================
void CarSystem::CreateInstPool()
{
    auto& scene = wi::scene::GetScene();

    // Shared mesh (white so obj->color becomes the car colour)
    meshEnt_ = scene.Entity_CreateCube("car_mesh");
    scene.materials.GetComponent(meshEnt_)->SetBaseColor(
        XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f));
    auto* mt = scene.transforms.GetComponent(meshEnt_);
    mt->Scale(XMFLOAT3(0.f, 0.f, 0.f));
    mt->UpdateTransform();

    instPool_.resize(MAX_VISIBLE);
    for (uint32_t j = 0; j < MAX_VISIBLE; ++j)
    {
        wi::ecs::Entity e = wi::ecs::CreateEntity();
        scene.layers.Create(e);
        auto& tr   = scene.transforms.Create(e);
        // Scale is constant — set once here, never touched in the render loop
        tr.scale_local = XMFLOAT3(CAR_HW * 2.f, CAR_HH * 2.f, CAR_HL * 2.f);
        tr.translation_local = XMFLOAT3(0.f, -1000.f, 0.f);
        tr.SetDirty();
        auto& obj  = scene.objects.Create(e);
        obj.meshID = meshEnt_;
        obj.color  = XMFLOAT4(0.82f, 0.82f, 0.82f, 1.f);
        instPool_[j] = e;
    }
    prevVisCount_ = 0;
}

// ============================================================
//  GetCarView — return lightweight snapshot for CrowdSystem
// ============================================================
CarView CarSystem::GetCarView() const
{
    CarView v;
    v.posX     = posX_.data();
    v.posZ     = posZ_.data();
    v.speed    = speed_.data();
    v.heading  = heading_.data();
    v.count    = activeCount;
    v.cellHead = driveHead_.data();
    v.cellNext = driveNext_.data();
    return v;
}

// ============================================================
//  RenderCars — place visible car instances with position + yaw
// ============================================================
void CarSystem::RenderCars(const XMFLOAT3& cameraPos, const CityLayout& city)
{
    auto& scene = wi::scene::GetScene();
    constexpr int GS = CityLayout::GRID_SIZE;

    const float camX  = cameraPos.x,  camZ = cameraPos.z;
    const float farSq = 800.f * 800.f;
    const float midSq = 350.f * 350.f;

    uint32_t count = 0;
    for (uint32_t i = 0; i < activeCount && count < MAX_VISIBLE; ++i)
    {
        // Don't render parked cars that aren't in a parking building
        if (state_[i] == State::PARKED && parkCell_[i] == UINT32_MAX)
            continue;

        float dx = posX_[i] - camX, dz = posZ_[i] - camZ;
        float d2 = dx*dx + dz*dz;
        if (d2 >= farSq) continue;
        if (d2 >= midSq && (i & 1u) != 0) continue;
        visIdx_[count++] = i;
    }
    visCount_ = count;

    // Position visible cars
    for (uint32_t s = 0; s < count; ++s)
    {
        const uint32_t i = visIdx_[s];
        auto* tr = scene.transforms.GetComponent(instPool_[s]);
        if (!tr) continue;

        // Determine Y position:
        // - PARKED in building (myParkSlot_ encodes floor): use floor Y
        // - Otherwise: ground level
        float carY = (isBus_[i] ? BUS_HH : CAR_HH) + 0.15f;
        if (state_[i] == State::PARKED &&
                 parkCell_[i] != UINT32_MAX &&
                 myParkSlot_[i] != 0xFF)
        {
            int key = (int)parkCell_[i];
            int pgx = key % GS, pgz = key / GS;
            if (city.GetCellType(pgx, pgz) == CityLayout::CellType::PARKING)
            {
                int fl = (int)myParkSlot_[i] / CityLayout::PARKING_SPOTS_PER_FLOOR;
                carY = CityLayout::PARKING_FLOOR_H * (float)fl + CAR_HH + 0.55f; // buses don't park in buildings
            }
        }

        // Per-vehicle scale
        if (isBus_[i])
            tr->scale_local = XMFLOAT3(BUS_HW * 2.f, BUS_HH * 2.f, BUS_HL * 2.f);
        else
            tr->scale_local = XMFLOAT3(CAR_HW * 2.f, CAR_HH * 2.f, CAR_HL * 2.f);

        // Direct SRT write — scale is already set in CreateInstPool and never changes
        float h2 = heading_[i] * 0.5f;
        tr->rotation_local    = XMFLOAT4(0.f, sinf(h2), 0.f, cosf(h2));
        tr->translation_local = XMFLOAT3(posX_[i], carY, posZ_[i]);
        tr->SetDirty();

        // Per-car colour via ObjectComponent tint
        auto* obj = scene.objects.GetComponent(instPool_[s]);
        if (obj) obj->color = carColor_[i];
    }

    // Only un-hide the slots that were visible last frame but are hidden this frame
    for (uint32_t j = count; j < prevVisCount_; ++j)
    {
        auto* tr = scene.transforms.GetComponent(instPool_[j]);
        if (tr) { tr->translation_local = XMFLOAT3(0.f, -1000.f, 0.f); tr->SetDirty(); }
    }
    prevVisCount_ = count;
}
