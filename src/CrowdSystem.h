#pragma once
#include "WickedEngine.h"
#include "CityLayout.h"
#include "TrafficLightSystem.h"
#include <vector>
#include <cstdint>

// ============================================================
//  Lightweight ped-data view — passed to CarSystem for safety brake
//  (avoids circular header dependency)
// ============================================================
struct PedestrianView {
    const float*    posX   = nullptr;
    const float*    posZ   = nullptr;
    const uint8_t*  state  = nullptr;   // 0=idle, nonzero=active
    uint32_t        count  = 0;
    const uint32_t* cellHead = nullptr; // per-grid-cell linked-list head
    const uint32_t* cellNext = nullptr; // per-ped next pointer
};

// Lightweight car-data view — passed to CrowdSystem so peds yield to cars
struct CarView {
    const float*    posX     = nullptr;
    const float*    posZ     = nullptr;
    const float*    speed    = nullptr;
    const float*    heading  = nullptr;
    uint32_t        count    = 0;
    const uint32_t* cellHead = nullptr;
    const uint32_t* cellNext = nullptr;
};

// ============================================================
//  CrowdSystem — sidewalk-only pedestrian simulation
//
//  Pedestrians walk on sidewalks (±9 m from road centreline).
//  They cross roads only at crosswalks and follow ped traffic lights.
//  Pathfinding uses Dijkstra on road cells → sidewalk waypoints.
// ============================================================
class CrowdSystem
{
public:
    static constexpr uint32_t MAX_PEDS      = 5000;
    static constexpr uint16_t MAX_WP        = 256;

    static constexpr float PED_SPEED_MIN    = 1.2f;   // m/s
    static constexpr float PED_SPEED_MAX    = 1.8f;   // m/s
    static constexpr float PED_RADIUS       = 0.25f;
    static constexpr float PED_MIN_SEP      = 0.6f;
    static constexpr float PED_AVOID_DIST   = 2.0f;

    // Sidewalk geometry (must match CityLayout)
    static constexpr float SIDEWALK_W       = CityLayout::SIDEWALK_W;          // 2.0 m
    static constexpr float SIDEWALK_INNER   = CityLayout::CELL_SIZE * 0.5f - SIDEWALK_W; // 8.0 m
    static constexpr float SIDEWALK_MID     = SIDEWALK_INNER + SIDEWALK_W * 0.5f;        // 9.0 m

    enum class State : uint8_t { IDLE = 0, WALKING = 1, WAITING_CROSS = 2, ON_BUS = 3 };

    void     Initialize();
    uint32_t SpawnPed(int homeGX, int homeGZ, int workGX, int workGZ,
                      const CityLayout& city);
    void     Update(float dt, const CityLayout& city,
                    const TrafficLightSystem& lights,
                    const CarView* carView = nullptr);
    void     Render(const XMFLOAT3& cameraPos);

    uint32_t GetPedCount() const { return activeCount_; }
    PedestrianView GetView() const;

    // For HUD
    uint32_t GetWalkingCount() const;
    uint32_t GetWaitingCount() const;

    // Reset all peds (for save loading)
    void ResetAll() { activeCount_ = 0; }

    // Shadow casting toggle — applies to all visible ped instances
    void SetShadowCasting(bool value);

private:
    uint32_t activeCount_ = 0;

    // SoA ped data
    std::vector<float>    posX_, posZ_;
    std::vector<float>    speed_;
    std::vector<float>    heading_;
    std::vector<State>    state_;
    std::vector<uint8_t>  dir_;           // 0=to work, 1=to home

    // Bus riding data (valid when state == ON_BUS)
    std::vector<uint32_t> ridingBusIdx_;  // which bus this ped is on (UINT32_MAX = none)
    std::vector<int>      exitStopGX_;    // grid cell where ped wants to exit
    std::vector<int>      exitStopGZ_;

    // Waypoints per ped
    std::vector<std::vector<XMFLOAT2>> wpBuf_;
    std::vector<uint16_t> wpCurr_;

    // Spatial hash for avoidance
    std::vector<uint32_t> cellHead_;      // per grid cell
    std::vector<uint32_t> cellNext_;      // per ped

    // Rendering
    static constexpr uint32_t MAX_VISIBLE = 2000;

    // Pedestrian mesh half-extents (used in CreateInstPool and Render)
    static constexpr float PED_HW = 0.15f;  // half-width  (30 cm)
    static constexpr float PED_HH = 0.45f;  // half-height (90 cm → 1.8m tall)
    static constexpr float PED_HD = 0.10f;  // half-depth  (20 cm)
    std::vector<uint32_t>        visIdx_;
    uint32_t                     visCount_    = 0;
    uint32_t                     prevVisCount_ = 0;   // slots visible last frame
    wi::ecs::Entity              meshEnt_ = wi::ecs::INVALID_ENTITY;
    std::vector<wi::ecs::Entity> instPool_;

    void RebuildSpatialHash();
    void CreateInstPool();

    // Pathfinding: road Dijkstra → sidewalk waypoints
    static std::vector<XMFLOAT2> FindPedPath(
        int srcGX, int srcGZ, int dstGX, int dstGZ,
        const CityLayout& city);

    // Traffic light query for crosswalk
    static bool CanPedCross(int gx, int gz,
                            const CityLayout& city,
                            const TrafficLightSystem& lights);
};