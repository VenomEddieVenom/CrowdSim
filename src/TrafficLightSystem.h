#pragma once
#include "WickedEngine.h"
#include "CityLayout.h"
#include <vector>
#include <cmath>

// ============================================================
//  TrafficLightSystem  –  Collision-free 4×(green+yellow) + ped phase
//
//  Phase 0 (N_GREEN):  N all turns,  W right-only          8 s
//  Phase 1 (N_YELLOW): yellow for above                    3.5 s
//  Phase 2 (E_GREEN):  E all turns,  N right-only          8 s
//  Phase 3 (E_YELLOW): yellow for above                    3.5 s
//  Phase 4 (S_GREEN):  S all turns,  E right-only          8 s
//  Phase 5 (S_YELLOW): yellow for above                    3.5 s
//  Phase 6 (W_GREEN):  W all turns,  S right-only          8 s
//  Phase 7 (W_YELLOW): yellow for above                    3.5 s
//  Phase 8 (PED_WALK): all vehicles RED, peds GREEN        20 s
//  Phase 9 (PED_CLEAR): ALL red (cars+peds) clearance      5 s
//  Total cycle: 71 s
// ============================================================
class TrafficLightSystem
{
public:
    static constexpr int GS = CityLayout::GRID_SIZE;
    static constexpr float CS = CityLayout::CELL_SIZE;

    // Phase durations (seconds)
    static constexpr float GREEN_DUR     = 8.0f;
    static constexpr float YELLOW_DUR    = 3.5f;   // 2s yellow + 1.5s all-red clearance
    static constexpr float PED_DUR       = 20.0f;  // pedestrian crossing phase (enough for slow peds to cross 20m cell)
    static constexpr float PED_CLEAR_DUR = 5.0f;   // all-red clearance after ped walk
    static constexpr float CYCLE_DUR     = (GREEN_DUR + YELLOW_DUR) * 4.0f + PED_DUR + PED_CLEAR_DUR; // 71 s
    static constexpr int   NUM_PHASES    = 10;

    enum class Phase : uint8_t {
        N_GREEN  = 0,   // N dominant + W right-turn
        N_YELLOW = 1,
        E_GREEN  = 2,   // E dominant + N right-turn
        E_YELLOW = 3,
        S_GREEN  = 4,   // S dominant + E right-turn
        S_YELLOW = 5,
        W_GREEN  = 6,   // W dominant + S right-turn
        W_YELLOW = 7,
        PED_WALK  = 8,  // all vehicles RED, pedestrians GREEN
        PED_CLEAR = 9   // all RED (cars+peds) — clearance for peds still crossing
    };

    // Approach direction as seen by the car
    enum class ApproachDir : uint8_t { N, E, S, W };

    // Traffic direction (axis of travel) – kept for pedestrian API
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

    // Derive approach direction from movement vector.
    // A car moving in +Z is approaching from the NORTH (heading south).
    static ApproachDir GetApproachDir(float dx, float dz)
    {
        if (std::abs(dz) >= std::abs(dx))
            return (dz > 0.f) ? ApproachDir::N : ApproachDir::S;
        else
            return (dx > 0.f) ? ApproachDir::W : ApproachDir::E;
    }

    LightColor GetLight(int gx, int gz, float dx, float dz,
                        TurnIntent intent = TurnIntent::STRAIGHT) const
    {
        if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) return LightColor::RED;
        int key = gz * GS + gx;
        if (!isIntersection_[key]) return LightColor::GREEN;

        Phase ph = phase_[key];

        // PED_WALK / PED_CLEAR: all vehicles RED
        if (ph == Phase::PED_WALK || ph == Phase::PED_CLEAR) return LightColor::RED;

        int phIdx = static_cast<int>(ph);
        bool isYellow = (phIdx & 1) != 0;
        int greenIdx = isYellow ? (phIdx - 1) : phIdx;

        ApproachDir dominant   = static_cast<ApproachDir>(greenIdx / 2);
        ApproachDir companion  = static_cast<ApproachDir>((static_cast<int>(dominant) + 3) % 4);

        ApproachDir ad = GetApproachDir(dx, dz);
        if (ad == dominant) {
            return isYellow ? LightColor::YELLOW : LightColor::GREEN;
        }
        if (ad == companion && intent == TurnIntent::RIGHT) {
            return isYellow ? LightColor::YELLOW : LightColor::GREEN;
        }
        return LightColor::RED;
    }

    // ---- Inspection helpers ----
    Phase GetPhase(int gx, int gz) const
    {
        if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) return Phase::N_GREEN;
        return phase_[gz * GS + gx];
    }
    float GetPhaseTimer(int gx, int gz) const
    {
        if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) return 0.f;
        return timer_[gz * GS + gx];
    }
    float GetPhaseTotalDuration(int gx, int gz) const
    {
        return PhaseDuration(GetPhase(gx, gz));
    }
    static const char* PhaseName(Phase p)
    {
        switch (p) {
        case Phase::N_GREEN:  return "N Green (+W right)";
        case Phase::N_YELLOW: return "N Yellow";
        case Phase::E_GREEN:  return "E Green (+N right)";
        case Phase::E_YELLOW: return "E Yellow";
        case Phase::S_GREEN:  return "S Green (+E right)";
        case Phase::S_YELLOW: return "S Yellow";
        case Phase::W_GREEN:  return "W Green (+S right)";
        case Phase::W_YELLOW: return "W Yellow";
        case Phase::PED_WALK:  return "Pedestrian Walk";
        case Phase::PED_CLEAR: return "Ped Clearance (All Red)";
        }
        return "?";
    }
    static const char* LightStr(LightColor lc)
    {
        switch (lc) {
        case LightColor::GREEN:  return "GREEN";
        case LightColor::YELLOW: return "YELLOW";
        case LightColor::RED:    return "RED";
        }
        return "?";
    }

    bool CanPedestrianCross(int gx, int gz, Axis /*crossAxis*/) const
    {
        if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) return true;
        int key = gz * GS + gx;
        if (!isIntersection_[key]) return true;

        // Pedestrians may ONLY start crossing during the dedicated PED_WALK phase.
        // During PED_CLEAR all lights are red (grace period), but no new crossings.
        return phase_[key] == Phase::PED_WALK;
    }

private:
    std::vector<bool>    isIntersection_;
    std::vector<Phase>   phase_;
    std::vector<Phase>   prevPhase_;    // phase at last visual update; sentinel 0xFF forces first draw
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
        wi::ecs::Entity pedSignal     = wi::ecs::INVALID_ENTITY;   // walk / don't-walk
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
        if (p == Phase::PED_WALK)  return PED_DUR;
        if (p == Phase::PED_CLEAR) return PED_CLEAR_DUR;
        int phIdx = static_cast<int>(p);
        return (phIdx & 1) ? YELLOW_DUR : GREEN_DUR;
    }
};
