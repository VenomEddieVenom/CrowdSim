#pragma once
#include "WickedEngine.h"
#include "CityLayout.h"
#include <vector>
#include <cmath>

// ============================================================
//  TrafficLightSystem  –  8-phase protected-left-turn lights
//
//  Auto-detects intersections (road cells with 3+ road neighbors).
//  8-phase cycle (32 s total):
//    Phase 0 – NS through  (NS straight+right GREEN)                  8 s
//    Phase 1 – NS yellow                                               2 s
//    Phase 2 – NS left     (NS left GREEN only)                        4 s
//    Phase 3 – ALL RED clearance                                       2 s
//    Phase 4 – EW through  (EW straight+right GREEN)                   8 s
//    Phase 5 – EW yellow                                               2 s
//    Phase 6 – EW left     (EW left GREEN only)                        4 s
//    Phase 7 – ALL RED clearance                                       2 s
//
//  Cars pass TurnIntent (STRAIGHT/RIGHT/LEFT) so the system
//  can grant or deny passage based on conflict analysis.
// ============================================================
class TrafficLightSystem
{
public:
    static constexpr int GS = CityLayout::GRID_SIZE;
    static constexpr float CS = CityLayout::CELL_SIZE;

    // Phase durations (seconds)
    static constexpr float THROUGH_DUR  = 8.0f;
    static constexpr float YELLOW_DUR   = 2.0f;
    static constexpr float LEFT_DUR     = 4.0f;
    static constexpr float ALL_RED_DUR  = 2.0f;
    static constexpr float CYCLE_DUR    = (THROUGH_DUR + YELLOW_DUR + LEFT_DUR + ALL_RED_DUR) * 2.0f; // 32 s
    static constexpr int   NUM_PHASES   = 8;

    // Phase identifiers
    enum class Phase : uint8_t {
        NS_THROUGH  = 0,
        NS_YELLOW   = 1,
        NS_LEFT     = 2,
        ALL_RED_1   = 3,
        EW_THROUGH  = 4,
        EW_YELLOW   = 5,
        EW_LEFT     = 6,
        ALL_RED_2   = 7
    };

    // Traffic direction (axis of travel)
    enum class Axis : uint8_t { NS, EW };

    // Car's intended movement at the intersection
    enum class TurnIntent : uint8_t { STRAIGHT, RIGHT, LEFT };

    // Traffic state seen by cars/pedestrians
    enum class LightColor : uint8_t { GREEN, YELLOW, RED };

    void Initialize();
    void RebuildIntersections(const CityLayout& city);
    void Update(float dt);
    void UpdateVisuals();

    bool IsIntersection(int gx, int gz) const
    {
        if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) return false;
        return isIntersection_[gz * GS + gx];
    }

    // Query light color for a car with given approach direction and turn intent.
    // dx,dz = normalised segment direction (approach heading).
    LightColor GetLight(int gx, int gz, float dx, float dz,
                        TurnIntent intent = TurnIntent::STRAIGHT) const
    {
        if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) return LightColor::RED;
        int key = gz * GS + gx;
        if (!isIntersection_[key]) return LightColor::GREEN;

        Axis carAxis = (std::abs(dz) > std::abs(dx)) ? Axis::NS : Axis::EW;
        Phase ph = phase_[key];

        switch (ph)
        {
        case Phase::NS_THROUGH:
            if (carAxis == Axis::NS)
                return (intent == TurnIntent::LEFT) ? LightColor::RED : LightColor::GREEN;
            return LightColor::RED;

        case Phase::NS_YELLOW:
            if (carAxis == Axis::NS && intent != TurnIntent::LEFT)
                return LightColor::YELLOW;
            return LightColor::RED;

        case Phase::NS_LEFT:
            if (carAxis == Axis::NS && intent == TurnIntent::LEFT)
                return LightColor::GREEN;
            return LightColor::RED;

        case Phase::ALL_RED_1:
            return LightColor::RED;

        case Phase::EW_THROUGH:
            if (carAxis == Axis::EW)
                return (intent == TurnIntent::LEFT) ? LightColor::RED : LightColor::GREEN;
            return LightColor::RED;

        case Phase::EW_YELLOW:
            if (carAxis == Axis::EW && intent != TurnIntent::LEFT)
                return LightColor::YELLOW;
            return LightColor::RED;

        case Phase::EW_LEFT:
            if (carAxis == Axis::EW && intent == TurnIntent::LEFT)
                return LightColor::GREEN;
            return LightColor::RED;

        case Phase::ALL_RED_2:
            return LightColor::RED;
        }
        return LightColor::RED;
    }

    bool CanPedestrianCross(int gx, int gz, Axis crossAxis) const
    {
        if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) return true;
        int key = gz * GS + gx;
        if (!isIntersection_[key]) return true;

        Phase ph = phase_[key];
        if (crossAxis == Axis::NS)
            return (ph == Phase::EW_THROUGH || ph == Phase::EW_YELLOW ||
                    ph == Phase::EW_LEFT    || ph == Phase::ALL_RED_1 || ph == Phase::ALL_RED_2);
        else
            return (ph == Phase::NS_THROUGH || ph == Phase::NS_YELLOW ||
                    ph == Phase::NS_LEFT    || ph == Phase::ALL_RED_1 || ph == Phase::ALL_RED_2);
    }

private:
    std::vector<bool>    isIntersection_;
    std::vector<Phase>   phase_;
    std::vector<float>   timer_;

    struct PoleVisuals {
        wi::ecs::Entity pole          = wi::ecs::INVALID_ENTITY;
        wi::ecs::Entity arm           = wi::ecs::INVALID_ENTITY;
        wi::ecs::Entity lightLeft     = wi::ecs::INVALID_ENTITY;
        wi::ecs::Entity lightStraight = wi::ecs::INVALID_ENTITY;
        wi::ecs::Entity lightRight    = wi::ecs::INVALID_ENTITY;
        wi::ecs::Entity arrowLeft     = wi::ecs::INVALID_ENTITY;
        wi::ecs::Entity arrowStraight = wi::ecs::INVALID_ENTITY;
        wi::ecs::Entity arrowRight    = wi::ecs::INVALID_ENTITY;
        bool hasLeft = false, hasStraight = false, hasRight = false;
    };
    std::vector<std::vector<PoleVisuals>> poleVisuals_;

    void CreatePolesForIntersection(int gx, int gz, const CityLayout& city);
    void RemovePolesForCell(int key);

    static Phase AdvancePhase(Phase p)
    {
        return static_cast<Phase>((static_cast<int>(p) + 1) % NUM_PHASES);
    }
    static float PhaseDuration(Phase p)
    {
        switch (p)
        {
        case Phase::NS_THROUGH: return THROUGH_DUR;
        case Phase::NS_YELLOW:  return YELLOW_DUR;
        case Phase::NS_LEFT:    return LEFT_DUR;
        case Phase::ALL_RED_1:  return ALL_RED_DUR;
        case Phase::EW_THROUGH: return THROUGH_DUR;
        case Phase::EW_YELLOW:  return YELLOW_DUR;
        case Phase::EW_LEFT:    return LEFT_DUR;
        case Phase::ALL_RED_2:  return ALL_RED_DUR;
        }
        return THROUGH_DUR;
    }
};
