#pragma once
#include "WickedEngine.h"
#include "CityLayout.h"
#include "TrafficLightSystem.h"
#include <vector>
#include <unordered_map>
#include <atomic>

// Forward-declared lightweight ped view (defined in CrowdSystem.h)
struct PedestrianView;
struct CarView;

// ============================================================
//  BusRoute — defines a route between two depots with stops
// ============================================================
struct BusRoute {
    int lineNumber = 0;                  // bus line number (user-assigned)
    int depotA_gx = 0, depotA_gz = 0;   // origin depot
    int depotB_gx = 0, depotB_gz = 0;   // destination depot (same as A for circular)
    bool circular = false;               // true = loop back to same depot
    struct Stop { int gx = 0, gz = 0; char name[32] = {}; };
    std::vector<Stop> stops;             // ordered stops A→B (outbound)
    std::vector<Stop> stopsBA;           // ordered stops B→A (return, circular only)
    std::vector<XMFLOAT2> fullPathAB;   // precomputed waypoints A→B
    std::vector<XMFLOAT2> fullPathBA;   // precomputed waypoints B→A
    std::vector<int> stopWpIdxAB;       // waypoint index for each stop (A→B path)
    std::vector<int> stopWpIdxBA;       // waypoint index for each stop (B→A path, reversed order)
    int maxBuses = 5;                   // max total buses on this route
    bool active = false;
};

// ============================================================
//  CarSystem  –  city traffic simulation
//
//  Roads are 20 m wide (CELL_SIZE).  Right-hand drive.
//  Lane count per cell is variable: 2, 4, or 6 total lanes.
//  Sidewalks at ±9 m.
//
//  "Right of travel" 2D formula (XZ plane, right-hand drive):
//      right = (dz, -dx)  [normalised]
// ============================================================
class CarSystem
{
public:
    static constexpr uint32_t MAX_CARS     = 10'000;
    static constexpr uint8_t  MAX_WP       = 64;
    static constexpr uint32_t MAX_VISIBLE  = 5'000;
    static constexpr uint32_t MAX_LIT_CARS = 500;  // cars with headlights + brake lights

    // Physics
    static constexpr float MAX_SPEED   = 14.0f;   // m/s ≈ 50 km/h
    static constexpr float ACCEL       =  6.0f;   // m/s²
    static constexpr float DECEL       = 14.0f;   // m/s²

    // Geometry – variable lanes (2, 4 or 6 per cell)
    static constexpr float PARK_PAUSE  =  2.0f;   // s idle at destination
    static constexpr float CAR_HW      =  0.45f;  // half-width  (~0.9 m)
    static constexpr float CAR_HH      =  0.35f;  // half-height (~0.7 m)
    static constexpr float CAR_HL      =  1.00f;  // half-length (~2.0 m)
    static constexpr float BUS_HL      =  3.00f;  // half-length (~6.0 m)
    static constexpr float BUS_HW      =  0.50f;  // half-width  (~1.0 m) – same as car
    static constexpr float BUS_HH      =  0.70f;  // half-height (~1.4 m) – twice car height
    static constexpr float MIN_SEP     =  CAR_HL * 2.f + 1.5f; // bumper-to-bumper (3.5 m)
    static constexpr float LAT_BAND    =  1.5f;   // lateral collision band (< inter-lane gap of 3m)

    // Lane offsets for each road mode (per-direction, from centreline)
    // Drivable half-width = HALF_CS - SIDEWALK_W = 10 - 2 = 8 m per side.
    // 2-lane (1 per dir): centre at 4.0 m  (midpoint of 0..8)
    // 6-lane (3 per dir): inner 1.33, middle 4.0, outer 6.67
    static void GetLaneOffsets(int totalLanes, float* offsets, int& count)
    {
        if (totalLanes >= 6) { count = 3; offsets[0] = 1.33f; offsets[1] = 4.0f; offsets[2] = 6.67f; }
        else { count = 1; offsets[0] = 4.0f; }
    }

    // IDM (Intelligent Driver Model) following parameters
    static constexpr float IDM_S0    =  2.0f;   // minimum gap (jam distance), m
    static constexpr float IDM_T    =  1.5f;   // desired time headway, s
    static constexpr float IDM_B    =  3.0f;   // comfortable deceleration, m/s²

    // Economy
    static constexpr float TAX_RATE    =  0.10f;  // default; overridable via taxRateOverride
    float taxRateOverride = 0.10f;  // set by president menu

    // Schedule
    static constexpr float JOYRIDE_CHANCE = 0.15f;  // 15% chance per parked-at-home cycle
    static constexpr float WANDER_HOME_CHANCE = 0.30f; // 30% chance wanderer goes home after stop
    static constexpr float STAY_HOME_CHANCE = 0.25f;   // 25% of cars stay home and wander

    enum class State : uint8_t { PARKED, DRIVING, ENTERING_PARKING, EXITING_PARKING };

    void     Initialize();
    // Returns car slot, or UINT32_MAX if at capacity / bad path
    uint32_t SpawnCar(const std::vector<XMFLOAT2>& roadPath, uint32_t colorSeed, int spawnLanes = 6);
    // Spawn a bus on a random loop through the city
    uint32_t SpawnBus(const std::vector<XMFLOAT2>& roadPath, uint32_t seed);
    void     Update(float dt, CityLayout& city, const TrafficLightSystem& lights,
                    float timeOfDay = 0.5f, float dayDuration = 86400.0f,
                    const PedestrianView* peds = nullptr);
    void     RenderCars(const XMFLOAT3& cameraPos, const CityLayout& city);

    uint32_t GetCarCount()        const { return activeCount; }
    CarView  GetCarView()        const;
    float    GetPosX(uint32_t i)  const { return i < posX_.size() ? posX_[i] : 0.f; }
    float    GetPosZ(uint32_t i)  const { return i < posZ_.size() ? posZ_[i] : 0.f; }
    // Drain tax accumulated since last call (main loop adds to treasury)
    float    DrainTax()                 { return taxAccum_.exchange(0.f); }
    // For HUD
    float    GetMoney(uint32_t i) const { return i < money_.size() ? money_[i] : 0.f; }

    bool IsBus(uint32_t i) const { return i < isBus_.size() && isBus_[i]; }
    float GetHalfLen(uint32_t i) const { return i < halfLen_.size() ? halfLen_[i] : CAR_HL; }

    // Shadow casting toggle — applies to all visible car instances
    void SetShadowCasting(bool value);

    // Bus route system
    std::vector<BusRoute> busRoutes_;
    uint32_t SpawnRouteBus(int routeIdx, CityLayout& city, int direction = 0);
    void     UpdateBusRoutes(float dt, CityLayout& city);
    int8_t   GetBusRouteIdx(uint32_t i) const { return i < busRouteIdx_.size() ? busRouteIdx_[i] : -1; }
    int8_t   GetBusDirection(uint32_t i) const { return i < busDirection_.size() ? busDirection_[i] : 0; }
    uint8_t  GetBusPassengers(uint32_t i) const { return i < busPassengers_.size() ? busPassengers_[i] : 0; }
    bool     IsBusAtStop(uint32_t i) const { return i < busStopTimer_.size() && busStopTimer_[i] > 0.f; }
    int8_t   GetBusNextStop(uint32_t i) const { return i < busNextStop_.size() ? busNextStop_[i] : -1; }
    void     SetBusRouteIdx(uint32_t i, int8_t val) { if (i < busRouteIdx_.size()) busRouteIdx_[i] = val; }

    // For click-on-car inspection
    State    GetState(uint32_t i)   const { return i < state_.size() ? state_[i] : State::PARKED; }
    float    GetSpeed(uint32_t i)   const { return i < speed_.size() ? speed_[i] : 0.f; }
    float    GetHeading(uint32_t i) const { return i < heading_.size() ? heading_[i] : 0.f; }
    uint8_t  GetWpCount(uint32_t i) const { return i < wpCount_.size() ? wpCount_[i] : 0; }
    uint8_t  GetWpCurr(uint32_t i)  const { return i < wpCurr_.size() ? wpCurr_[i] : 0; }
    XMFLOAT2 GetWaypoint(uint32_t i, uint8_t w) const {
        if (i >= wpCount_.size() || w >= wpCount_[i]) return {0,0};
        return wpBuf_[(size_t)i * MAX_WP + w];
    }
    uint8_t  GetCarDir(uint32_t i) const { return i < carDir_.size() ? carDir_[i] : 0; }
    float    GetLaneOff(uint32_t i) const { return i < laneOff_.size() ? laneOff_[i] : 0.f; }
    uint32_t GetVisibleCarIndex(uint32_t slot) const { return slot < visCount_ ? visIdx_[slot] : UINT32_MAX; }
    uint32_t GetVisibleCarCount() const { return visCount_; }
    static const char* StateStr(State s) {
        switch(s) {
        case State::PARKED: return "Parked";
        case State::DRIVING: return "Driving";
        case State::ENTERING_PARKING: return "Entering parking";
        case State::EXITING_PARKING: return "Exiting parking";
        }
        return "Unknown";
    }

    // ---------- inline geometry helpers (used in .cpp and potentially main.cpp) ----------
    // Right-hand-drive lane offset (right of travel)
    static XMFLOAT2 LanedPos(XMFLOAT2 raw, float dx, float dz, float offset)
    {
        float len = std::sqrt(dx*dx + dz*dz);
        if (len < 1e-6f) return raw;
        // right2D = (dz, -dx)
        float rx = dz / len,  rz = -dx / len;
        return { raw.x + rx * offset, raw.y + rz * offset };
    }
    static XMFLOAT2 DirTo(XMFLOAT2 from, XMFLOAT2 to)
    {
        float dx = to.x - from.x,  dz = to.y - from.y;
        float len = std::sqrt(dx*dx + dz*dz);
        if (len < 1e-6f) return {1.f, 0.f};
        return {dx/len, dz/len};
    }

    // Merge consecutive collinear waypoints, keeping only turn points and endpoints
    static void SimplifyPath(XMFLOAT2* wp, uint8_t& count);

    // Reset all cars (for save loading)
    void ResetAll() { activeCount = 0; busRoutes_.clear(); }

private:
    uint32_t activeCount = 0;

    // SoA car data
    std::vector<float>   posX_, posZ_;    // actual (laned) world position
    std::vector<float>   speed_;          // m/s
    std::vector<float>   heading_;        // radians (Y-axis rotation)
    std::vector<State>   state_;
    std::vector<float>   parkTimer_;
    std::vector<XMFLOAT4>carColor_;
    std::vector<float>   halfLen_;         // per-vehicle half-length
    std::vector<bool>    isBus_;           // true for buses
    std::vector<bool>    braking_;         // true when decelerating (for brake lights)

    // Per-bus route data (-1 = legacy random bus)
    std::vector<int8_t>  busRouteIdx_;     // which route this bus follows
    std::vector<int8_t>  busDirection_;    // 0 = A→B, 1 = B→A
    std::vector<int8_t>  busNextStop_;     // next stop index to visit
    std::vector<float>   busStopTimer_;    // countdown timer at stops (seconds)
    std::vector<uint8_t> busPassengers_;   // current passenger count

    // Economy
    std::vector<float>   wage_;           // $/game-sec when returning from work
    std::vector<float>   money_;
    std::atomic<float>   taxAccum_{0.f};

    // Waypoints – raw road-cell centres  [i * MAX_WP + j]
    std::vector<XMFLOAT2> wpBuf_;
    std::vector<uint8_t>  wpCount_;
    std::vector<uint8_t>  wpCurr_;
    std::vector<uint8_t>  carDir_;   // 0 = to work, 1 = to home
    std::vector<bool>     wandering_; // true = car is wandering (random destination)
    std::vector<bool>     stayHome_;  // true = car never commutes, just wanders
    std::vector<float>    laneOff_;  // current lane offset from centreline
    std::vector<float>    laneTarget_; // target lane for smooth transitions
    std::vector<int8_t>   laneIdx_;    // lane index (0=inner, N-1=outer)
    std::vector<int8_t>   wantsLaneDir_; // gap-creation signal (-1/0/+1) for cooperative lane changes

    // Stuck/deadlock detection
    std::vector<float>   stuckTimer_;     // seconds car has been near-stationary
    std::vector<uint8_t> stuckStalls_;    // consecutive stall count (for escalating fix)

    // Parking-animation data
    struct ParkAnim {
        XMFLOAT3 entry;     // world position at building entry
        XMFLOAT3 rampTop;   // world position at top of ramp (floor level)
        XMFLOAT3 slot;      // final perpendicular slot position
        float    slotAngle; // heading when parked in slot
        int      pgx, pgz;  // grid cell of the building
        uint8_t  phase;     // 0=approaching entry, 1=climbing ramp, 2=approaching slot
    };
    std::vector<ParkAnim> parkAnim_;    // per-car parking animation state

    // Workday schedule
    std::vector<float>    schedDepart_; // hour (0-24) to leave home for work
    std::vector<float>    schedReturn_; // hour (0-24) to leave work for home
    std::vector<uint32_t> lastDayTrip_; // day counter to prevent multiple trips per day
    uint32_t              dayCounter_ = 0;
    float                 prevTimeOfDay_ = 0.0f;

    // Following-distance: per-cell linked list of DRIVING cars (rebuilt each frame)
    std::vector<uint32_t> driveHead_;    // head per grid cell     (UINT32_MAX = empty)
    std::vector<uint32_t> driveNext_;    // next car in same cell  (UINT32_MAX = end)

    // Scratch buffer for parallel dispatch of DRIVING cars
    std::vector<uint32_t> drivingIdx_;

    // Parking occupancy (building slots only)
    std::vector<uint32_t> parkCell_;     // which grid cell car i is parked in
    std::vector<uint8_t>  myParkSlot_;   // which slot this car occupies, 0xFF = none

    // Rendering
    std::vector<uint32_t>        visIdx_;
    uint32_t                     visCount_    = 0;
    uint32_t                     prevVisCount_ = 0;   // slots visible last frame
    wi::ecs::Entity              meshEnt_  = wi::ecs::INVALID_ENTITY;
    std::vector<wi::ecs::Entity> instPool_;

    // Car lights: 2 headlights + 2 brake lights per visible slot
    std::vector<wi::ecs::Entity> headlightL_;   // left headlight
    std::vector<wi::ecs::Entity> headlightR_;   // right headlight
    std::vector<wi::ecs::Entity> brakelightL_;  // left brake light
    std::vector<wi::ecs::Entity> brakelightR_;  // right brake light

    void CreateInstPool();
};
