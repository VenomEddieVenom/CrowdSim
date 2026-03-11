#include "TrafficLightSystem.h"
#include <algorithm>

// ============================================================
//  Initialize
// ============================================================
void TrafficLightSystem::Initialize()
{
    const int N = GS * GS;
    isIntersection_.assign(N, false);
    phase_.assign(N, Phase::NS_THROUGH);
    timer_.assign(N, 0.0f);
    poleVisuals_.resize(N);

    wi::backlog::post("[TrafficLightSystem] Ready", wi::backlog::LogLevel::Default);
}

// ============================================================
//  RebuildIntersections – scan grid for road cells with 3+ road neighbours
// ============================================================
void TrafficLightSystem::RebuildIntersections(const CityLayout& city)
{
    const int ddx[] = { 1, -1,  0,  0 };
    const int ddz[] = { 0,  0,  1, -1 };
    const int N = GS * GS;

    for (int gz = 0; gz < GS; ++gz)
    for (int gx = 0; gx < GS; ++gx)
    {
        int key = gz * GS + gx;
        bool wasIntersection = isIntersection_[key];

        if (city.GetCellType(gx, gz) != CityLayout::CellType::ROAD)
        {
            if (wasIntersection) RemovePolesForCell(key);
            isIntersection_[key] = false;
            continue;
        }

        int rn = 0;
        for (int d = 0; d < 4; ++d)
        {
            auto ct = city.GetCellType(gx + ddx[d], gz + ddz[d]);
            if (ct == CityLayout::CellType::ROAD ||
                ct == CityLayout::CellType::CROSSWALK)
                ++rn;
        }

        bool isNow = (rn >= 3);
        isIntersection_[key] = isNow;

        if (isNow && !wasIntersection)
        {
            // Stagger start phase by position for visual variety
            int startPhase = (gx + gz) % NUM_PHASES;
            phase_[key] = static_cast<Phase>(startPhase);
            timer_[key] = 0.0f;
            CreatePolesForIntersection(gx, gz, city);
        }
        else if (!isNow && wasIntersection)
        {
            RemovePolesForCell(key);
        }
    }
}

// ============================================================
//  Update – advance phase timers
// ============================================================
void TrafficLightSystem::Update(float dt)
{
    const int N = GS * GS;
    for (int key = 0; key < N; ++key)
    {
        if (!isIntersection_[key]) continue;
        timer_[key] += dt;
        float dur = PhaseDuration(phase_[key]);
        while (timer_[key] >= dur)
        {
            timer_[key] -= dur;
            phase_[key] = AdvancePhase(phase_[key]);
            dur = PhaseDuration(phase_[key]);
        }
    }
}

// ============================================================
//  UpdateVisuals – set traffic light colors on pole entities
// ============================================================
void TrafficLightSystem::UpdateVisuals()
{
    auto& scene = wi::scene::GetScene();
    const int N = GS * GS;

    for (int key = 0; key < N; ++key)
    {
        if (!isIntersection_[key]) continue;
        auto& poles = poleVisuals_[key];
        if (poles.empty()) continue;

        Phase ph = phase_[key];

        // Determine NS and EW light colors for poles
        XMFLOAT4 nsColor, ewColor;
        switch (ph)
        {
        case Phase::NS_THROUGH:
            nsColor = {0.1f, 0.9f, 0.1f, 1.0f};  // green
            ewColor = {0.9f, 0.1f, 0.1f, 1.0f};  // red
            break;
        case Phase::NS_YELLOW:
            nsColor = {0.9f, 0.8f, 0.1f, 1.0f};  // yellow
            ewColor = {0.9f, 0.1f, 0.1f, 1.0f};  // red
            break;
        case Phase::NS_LEFT:
            nsColor = {0.1f, 0.9f, 0.4f, 1.0f};  // green-ish (left arrow)
            ewColor = {0.9f, 0.1f, 0.1f, 1.0f};  // red
            break;
        case Phase::ALL_RED_1:
        case Phase::ALL_RED_2:
            nsColor = {0.9f, 0.1f, 0.1f, 1.0f};  // red
            ewColor = {0.9f, 0.1f, 0.1f, 1.0f};  // red
            break;
        case Phase::EW_THROUGH:
            nsColor = {0.9f, 0.1f, 0.1f, 1.0f};  // red
            ewColor = {0.1f, 0.9f, 0.1f, 1.0f};  // green
            break;
        case Phase::EW_YELLOW:
            nsColor = {0.9f, 0.1f, 0.1f, 1.0f};  // red
            ewColor = {0.9f, 0.8f, 0.1f, 1.0f};  // yellow
            break;
        case Phase::EW_LEFT:
            nsColor = {0.9f, 0.1f, 0.1f, 1.0f};  // red
            ewColor = {0.1f, 0.9f, 0.4f, 1.0f};  // green-ish (left arrow)
            break;
        }

        // Poles: 0=NE, 1=NW, 2=SE, 3=SW
        // NE and SW face NS traffic; NW and SE face EW traffic
        // Actually: poles on the SIDE of the road face the approaching traffic.
        // NE corner: faces South (NS axis) and West (EW axis)
        // For simplicity: poles 0,3 (NE,SW diagonal) show NS color; poles 1,2 (NW,SE) show EW color
        // This makes NS traffic (travelling along Z) see the NS light on poles at their right
        for (int p = 0; p < 4 && p < (int)poles.size(); ++p)
        {
            auto* mat = scene.materials.GetComponent(poles[p].light);
            if (!mat) continue;
            // Poles 0(NE), 3(SW) → NS light; Poles 1(NW), 2(SE) → EW light
            XMFLOAT4 color = (p == 0 || p == 3) ? nsColor : ewColor;
            mat->SetBaseColor(color);
            mat->SetEmissiveColor(XMFLOAT4(color.x, color.y, color.z, 4.0f));
        }
    }
}

// ============================================================
//  CreatePolesForIntersection – 4 poles at intersection corners
// ============================================================
void TrafficLightSystem::CreatePolesForIntersection(int gx, int gz,
                                                     const CityLayout& city)
{
    auto& scene = wi::scene::GetScene();
    int key = gz * GS + gx;
    auto& poles = poleVisuals_[key];

    // Remove old if any
    RemovePolesForCell(key);
    poles.resize(4);

    XMFLOAT2 center = city.GridCellCenter(gx, gz);
    float h = CS * 0.5f;

    // 4 corners: NE, NW, SE, SW  (offset from center)
    const float offX[4] = {  h - 1.5f, -(h - 1.5f),   h - 1.5f, -(h - 1.5f) };
    const float offZ[4] = { -(h - 1.5f), -(h - 1.5f),  h - 1.5f,   h - 1.5f };

    // Mast arm directions: horizontal arm extends from pole over the road
    // Pole 0 (NE, NS light): arm → -X (west over northbound lanes)
    // Pole 1 (NW, EW light): arm → +Z (south over eastbound lanes)
    // Pole 2 (SE, EW light): arm → -Z (north over westbound lanes)
    // Pole 3 (SW, NS light): arm → +X (east over southbound lanes)
    const float armDX[4] = { -1.f,  0.f,  0.f,  1.f };
    const float armDZ[4] = {  0.f,  1.f, -1.f,  0.f };
    const float armLen = 5.5f;    // arm length over road
    const float poleH  = 7.0f;   // pole height

    for (int p = 0; p < 4; ++p)
    {
        float px = center.x + offX[p];
        float pz = center.y + offZ[p];

        // Vertical pole (tall gray)
        auto poleEnt = scene.Entity_CreateCube("tl_pole");
        auto* pt = scene.transforms.GetComponent(poleEnt);
        pt->Scale(XMFLOAT3(0.15f, poleH, 0.15f));
        pt->Translate(XMFLOAT3(px, poleH * 0.5f, pz));
        pt->UpdateTransform();
        scene.materials.GetComponent(poleEnt)->SetBaseColor(
            XMFLOAT4(0.25f, 0.25f, 0.25f, 1.0f));

        // Horizontal mast arm (extends from pole top over road)
        float armCX = px + armDX[p] * armLen * 0.5f;
        float armCZ = pz + armDZ[p] * armLen * 0.5f;
        // Scale: arm is long in one axis, thin in others
        float armSX = (armDX[p] != 0.f) ? armLen : 0.12f;
        float armSZ = (armDZ[p] != 0.f) ? armLen : 0.12f;

        auto armEnt = scene.Entity_CreateCube("tl_arm");
        auto* at = scene.transforms.GetComponent(armEnt);
        at->Scale(XMFLOAT3(armSX, 0.12f, armSZ));
        at->Translate(XMFLOAT3(armCX, poleH, armCZ));
        at->UpdateTransform();
        scene.materials.GetComponent(armEnt)->SetBaseColor(
            XMFLOAT4(0.25f, 0.25f, 0.25f, 1.0f));

        // Traffic light head (hanging from arm tip, slightly below)
        float lightX = px + armDX[p] * armLen;
        float lightZ = pz + armDZ[p] * armLen;

        auto lightEnt = scene.Entity_CreateCube("tl_light");
        auto* lt = scene.transforms.GetComponent(lightEnt);
        lt->Scale(XMFLOAT3(0.4f, 0.7f, 0.4f));
        lt->Translate(XMFLOAT3(lightX, poleH - 0.55f, lightZ));
        lt->UpdateTransform();
        // Default red with emissive glow
        auto* lightMat = scene.materials.GetComponent(lightEnt);
        lightMat->SetBaseColor(XMFLOAT4(0.9f, 0.1f, 0.1f, 1.0f));
        lightMat->SetEmissiveColor(XMFLOAT4(0.9f, 0.1f, 0.1f, 4.0f));

        poles[p].pole  = poleEnt;
        poles[p].arm   = armEnt;
        poles[p].light = lightEnt;
    }
}

// ============================================================
//  RemovePolesForCell
// ============================================================
void TrafficLightSystem::RemovePolesForCell(int key)
{
    auto& scene = wi::scene::GetScene();
    auto& poles = poleVisuals_[key];
    for (auto& pv : poles)
    {
        if (pv.pole  != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.pole);
        if (pv.arm   != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.arm);
        if (pv.light != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.light);
        pv.pole  = wi::ecs::INVALID_ENTITY;
        pv.arm   = wi::ecs::INVALID_ENTITY;
        pv.light = wi::ecs::INVALID_ENTITY;
    }
    poles.clear();
}
