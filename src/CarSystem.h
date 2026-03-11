#pragma once
#include "WickedEngine.h"
#include "CityLayout.h"
#include "TrafficLightSystem.h"
#include <vector>
#include <unordered_map>

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
    static constexpr uint32_t MAX_CARS     = 2'000;
    static constexpr uint8_t  MAX_WP       = 64;
    static constexpr uint32_t MAX_VISIBLE  = 1'500;

    // Physics
    static constexpr float MAX_SPEED   = 14.0f;   // m/s ≈ 50 km/h
    static constexpr float ACCEL       =  6.0f;   // m/s²
    static constexpr float DECEL       = 14.0f;   // m/s²

    // Geometry – variable lanes (2, 4 or 6 per cell)
    static constexpr float PARK_PAUSE  =  2.0f;   // s idle at destination
    static constexpr float CAR_HW      =  0.50f;  // half-width  (~1.0 m)
    static constexpr float CAR_HH      =  0.35f;  // half-height (~0.7 m)
    static constexpr float CAR_HL      =  1.00f;  // half-length (~2.0 m)
    static constexpr float MIN_SEP     =  CAR_HL * 2.f + 0.5f; // bumper-to-bumper (2.5 m)
    static constexpr float LAT_BAND    =  2.6f;   // lateral collision band (must be < inter-lane gap)

    // Lane offsets for each road mode (per-direction, from centreline)
    // 2-lane (1 per dir): centre at 2.5 m
    // 4-lane (2 per dir): inner 1.5 m, outer 4.5 m
    // 6-lane (3 per dir): inner 1.5 m, middle 4.0 m, outer 6.5 m
    static void GetLaneOffsets(int totalLanes, float* offsets, int& count)
    {
        if (totalLanes >= 6) { count = 3; offsets[0] = 1.5f; offsets[1] = 4.0f; offsets[2] = 6.5f; }
        else if (totalLanes >= 4) { count = 2; offsets[0] = 1.5f; offsets[1] = 4.5f; }
        else { count = 1; offsets[0] = 2.5f; }
    }

    // IDM (Intelligent Driver Model) following parameters
    static constexpr float IDM_S0    =  2.0f;   // minimum gap (jam distance), m
    static constexpr float IDM_T    =  1.5f;   // desired time headway, s
    static constexpr float IDM_B    =  3.0f;   // comfortable deceleration, m/s²

    // Economy
    static constexpr float TAX_RATE    =  0.10f;

    // Schedule
    static constexpr float JOYRIDE_CHANCE = 0.03f;  // 3% chance per parked-at-home cycle

    enum class State : uint8_t { PARKED, DRIVING, ENTERING_PARKING, EXITING_PARKING };

    void     Initialize();
    // Returns car slot, or UINT32_MAX if at capacity / bad path
    uint32_t SpawnCar(const std::vector<XMFLOAT2>& roadPath, uint32_t colorSeed);
    void     Update(float dt, CityLayout& city, const TrafficLightSystem& lights,
                    float timeOfDay = 0.5f, float dayDuration = 86400.0f);
    void     RenderCars(const XMFLOAT3& cameraPos, const CityLayout& city);

    uint32_t GetCarCount()        const { return activeCount; }
    float    GetPosX(uint32_t i)  const { return i < posX_.size() ? posX_[i] : 0.f; }
    float    GetPosZ(uint32_t i)  const { return i < posZ_.size() ? posZ_[i] : 0.f; }
    // Drain tax accumulated since last call (main loop adds to treasury)
    float    DrainTax()                 { float t = taxAccum_; taxAccum_ = 0.f; return t; }
    // For HUD
    float    GetMoney(uint32_t i) const { return i < money_.size() ? money_[i] : 0.f; }

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

private:
    uint32_t activeCount = 0;

    // SoA car data
    std::vector<float>   posX_, posZ_;    // actual (laned) world position
    std::vector<float>   speed_;          // m/s
    std::vector<float>   heading_;        // radians (Y-axis rotation)
    std::vector<State>   state_;
    std::vector<float>   parkTimer_;
    std::vector<XMFLOAT4>carColor_;

    // Economy
    std::vector<float>   wage_;           // $/game-sec when returning from work
    std::vector<float>   money_;
    float                taxAccum_ = 0.f;

    // Waypoints – raw road-cell centres  [i * MAX_WP + j]
    std::vector<XMFLOAT2> wpBuf_;
    std::vector<uint8_t>  wpCount_;
    std::vector<uint8_t>  wpCurr_;
    std::vector<uint8_t>  carDir_;   // 0 = to work, 1 = to home
    std::vector<float>    laneOff_;  // current lane offset from centreline

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

    // Parking occupancy (building slots only)
    std::vector<uint32_t> parkCell_;     // which grid cell car i is parked in
    std::vector<uint8_t>  myParkSlot_;   // which slot this car occupies, 0xFF = none

    // Rendering
    std::vector<uint32_t>        visIdx_;
    uint32_t                     visCount_ = 0;
    wi::ecs::Entity              meshEnt_  = wi::ecs::INVALID_ENTITY;
    std::vector<wi::ecs::Entity> instPool_;

    void CreateInstPool();
};
