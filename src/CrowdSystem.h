#pragma once

#include "WickedEngine.h"

#include <vector>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <string>

class CityLayout; // forward declaration

// ============================================================
//  CrowdSystem  –  waypoint-following pedestrian crowd
//
//  Agents are spawned with a world-space path (std::vector<XMFLOAT2>).
//  They follow the path, then reverse direction to loop home↔work.
//  Visible agents are rendered as solid cube instances.
// ============================================================
class CrowdSystem
{
public:
    static constexpr uint32_t MAX_AGENTS     = 50'000;
    static constexpr uint32_t MAX_WAYPOINTS  = 64;
    static constexpr float    LOD_NEAR       = 100.0f;
    static constexpr float    LOD_MID        = 200.0f;
    static constexpr float    LOD_FAR        = 600.0f;
    static constexpr uint32_t JOB_GROUP_SIZE = 256;
    static constexpr uint32_t MAX_VISIBLE    = 8'000;

    void Initialize();

    // Spawn `count` agents walking along `path` (road cells, built by CityLayout::FindPath)
    void SpawnAgents(const std::vector<XMFLOAT2>& path,
                     int homeGX, int homeGZ, int workGX, int workGZ,
                     uint32_t count = 5);

    uint32_t GetAgentCount() const { return activeCount; }

    void Update(float dt, const XMFLOAT3& cameraPos, const CityLayout& city);
    void RenderSolidAgents(const XMFLOAT3& cameraPos);

    float    GetPosX(uint32_t i)    const { return posX[i]; }
    float    GetPosZ(uint32_t i)    const { return posZ[i]; }
    float    GetTargetX(uint32_t i) const;
    float    GetTargetZ(uint32_t i) const;
    float    GetSpeed(uint32_t i)   const { return speed[i]; }
    const char* GetName(uint32_t i) const { return i < agentName.size() ? agentName[i].c_str() : ""; }
    uint8_t     GetAge(uint32_t i)  const { return i < agentAge.size()  ? agentAge[i]  : 0; }
    const char* GetActivity(uint32_t i) const
    {
        if (i >= activeCount) return "";
        return agentDir[i] == 0 ? "Going to work" : "Going home";
    }
    float    GetMoney(uint32_t i)    const { return i < agentMoney.size() ? agentMoney[i] : 0.0f; }
    float    GetWage(uint32_t i)     const { return i < agentWage.size()  ? agentWage[i]  : 0.0f; }
    // Returns tax collected this frame (call once per Update tick)
    float    DrainTax()                    { float t = lastFrameTax_; lastFrameTax_ = 0.0f; return t; }

    uint32_t GetVisibleAgentIndex(uint32_t slot) const
        { return slot < visCount_ ? visIndices[slot] : UINT32_MAX; }
    uint32_t GetVisibleCount()  const { return visCount_; }
    uint32_t GetThreadCount()   const { return wi::jobsystem::GetThreadCount(); }
    size_t   GetRenderedCount() const { return renderedCount.load(std::memory_order_relaxed); }

private:
    uint32_t activeCount = 0;

    // Position + speed (SoA)
    std::vector<float> posX, posZ;
    std::vector<float> speed;

    // Agent metadata
    std::vector<std::string> agentName;
    std::vector<uint8_t>     agentAge;
    std::vector<uint8_t>     agentDir;   // 0 = going to work, 1 = going home
    std::vector<float>       agentMoney; // lifetime savings ($)
    std::vector<float>       agentWage;  // $/game-second while at work
    std::vector<float>       agentSidewalkOff_; // lateral sidewalk offset ±9 m
    float taxRate_     = 0.10f; // 10% city tax
    float lastFrameTax_ = 0.0f;

    // Per-agent waypoint paths
    // Agent i's path: waypointBuf[i*MAX_WAYPOINTS .. (i+1)*MAX_WAYPOINTS)
    std::vector<XMFLOAT2> waypointBuf;
    std::vector<uint8_t>  waypointCount_;  // valid entries per agent
    std::vector<uint8_t>  waypointCurr_;   // current target index

    // Rendering
    std::vector<uint32_t>        visIndices;
    uint32_t                     visCount_ = 0;
    wi::ecs::Entity              meshEntity = wi::ecs::INVALID_ENTITY;
    std::vector<wi::ecs::Entity> instancePool;
    mutable std::atomic<size_t>  renderedCount{ 0 };

    wi::jobsystem::context jobCtx;

    void CreateInstancePool();
};
