#include "TrafficLightSystem.h"
#include <algorithm>

// ============================================================
//  Initialize
// ============================================================
void TrafficLightSystem::Initialize()
{
    const int N = GS * GS;
    isIntersection_.assign(N, false);
    phase_.assign(N, Phase::N_GREEN);
    prevPhase_.assign(N, static_cast<Phase>(0xFF));
    timer_.assign(N, 0.0f);
    poleVisuals_.resize(N);

    wi::backlog::post("[TrafficLightSystem] Ready (4-phase collision-free)", wi::backlog::LogLevel::Default);
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
            prevPhase_[key] = static_cast<Phase>(0xFF);  // force visual update on first frame
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

    const XMFLOAT4 RED    = {0.9f, 0.1f, 0.1f, 1.0f};
    const XMFLOAT4 GREEN  = {0.1f, 0.9f, 0.1f, 1.0f};
    const XMFLOAT4 YELLOW = {0.9f, 0.8f, 0.1f, 1.0f};
    const XMFLOAT4 DIM    = {0.15f, 0.15f, 0.15f, 1.0f};

    auto setLight = [&](wi::ecs::Entity ent, const XMFLOAT4& color) {
        if (ent == wi::ecs::INVALID_ENTITY) return;
        auto* mat = scene.materials.GetComponent(ent);
        if (!mat) return;
        mat->SetBaseColor(color);
        float emStr = (color.x > 0.5f || color.y > 0.5f) ? 4.0f : 0.2f;
        mat->SetEmissiveColor(XMFLOAT4(color.x, color.y, color.z, emStr));
    };

    // Pole → ApproachDir mapping: pole 0=S, 1=E, 2=W, 3=N
    const ApproachDir poleDir[4] = {
        ApproachDir::S, ApproachDir::E, ApproachDir::W, ApproachDir::N
    };

    for (int key = 0; key < N; ++key)
    {
        if (!isIntersection_[key]) continue;
        if (phase_[key] == prevPhase_[key]) continue;
        prevPhase_[key] = phase_[key];
        auto& poles = poleVisuals_[key];
        if (poles.empty()) continue;

        Phase ph = phase_[key];
        int phIdx = static_cast<int>(ph);
        bool isYellow = (phIdx & 1) != 0;
        int greenIdx = isYellow ? (phIdx - 1) : phIdx;
        ApproachDir dominant  = static_cast<ApproachDir>(greenIdx / 2);
        ApproachDir companion = static_cast<ApproachDir>((static_cast<int>(dominant) + 3) % 4);

        for (int p = 0; p < (int)poles.size(); ++p)
        {
            if (poles[p].pole == wi::ecs::INVALID_ENTITY) continue;

            ApproachDir ad = poleDir[p];
            XMFLOAT4 leftC = RED, straightC = RED, rightC = RED;

            if (ad == dominant) {
                // All turns green/yellow
                XMFLOAT4 c = isYellow ? YELLOW : GREEN;
                leftC = c; straightC = c; rightC = c;
            } else if (ad == companion) {
                // Right turn only
                XMFLOAT4 c = isYellow ? YELLOW : GREEN;
                rightC = c;
            }
            // else: all RED (default)

            if (poles[p].hasLeft)     setLight(poles[p].lightLeft, leftC);
            if (poles[p].hasStraight) setLight(poles[p].lightStraight, straightC);
            if (poles[p].hasRight)    setLight(poles[p].lightRight, rightC);

            auto arrowCol = [&](const XMFLOAT4& lc) -> XMFLOAT4 {
                return (lc.y > 0.5f || lc.x > 0.5f) ? XMFLOAT4(1.f, 1.f, 1.f, 1.f) : DIM;
            };
            if (poles[p].hasLeft)     setLight(poles[p].arrowLeft, arrowCol(leftC));
            if (poles[p].hasStraight) setLight(poles[p].arrowStraight, arrowCol(straightC));
            if (poles[p].hasRight)    setLight(poles[p].arrowRight, arrowCol(rightC));
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

    // Near-side signal placement: poles on the LEFT side of the road,
    // arm extends RIGHT over the road so lights hang on driver's right.
    // Order seen by driver: traffic light → crosswalk → intersection.
    const float pOff  = h - 1.5f;   // 8.5  perpendicular offset
    const float cwOff = h + 3.0f;   // 13.0 along approach
    const float offX[4] = { -pOff,  cwOff, -cwOff,  pOff };
    const float offZ[4] = { cwOff,  pOff,  -pOff, -cwOff };

    // Mast arm directions: from pole toward driver's RIGHT (across road)
    // Pole 0 (NW): arm → +X   Pole 1 (SE): arm → -Z
    // Pole 2 (NE): arm → +Z   Pole 3 (SW): arm → -X
    const float armDX[4] = {  1.f,  0.f,  0.f, -1.f };
    const float armDZ[4] = {  0.f, -1.f,  1.f,  0.f };
    const float armLen = 5.5f;
    const float poleH  = 7.0f;

    // Direction tables: approach-from, straight-exit, left-exit, right-exit
    const int aDX[4] = {  0,  1, -1,  0 }, aDZ[4] = {  1,  0,  0, -1 };
    const int sDX[4] = {  0, -1,  1,  0 }, sDZ[4] = { -1,  0,  0,  1 };
    const int lDX[4] = { -1,  0,  0,  1 }, lDZ[4] = {  0,  1, -1,  0 };
    const int rDX[4] = {  1,  0,  0, -1 }, rDZ[4] = {  0, -1,  1,  0 };

    // Light arrangement: arm extends toward driver's right
    //   Right light = further along arm (driver's right)
    //   Straight = center
    //   Left light = closer to pole (driver's left)
    const float spacing = 0.80f;

    for (int p = 0; p < 4; ++p)
    {
        // Only create pole + lights if traffic approaches from this direction
        bool hasApproach = city.IsRoadLike(gx + aDX[p], gz + aDZ[p]);
        if (!hasApproach)
        {
            poles[p] = {};
            continue;
        }

        float px = center.x + offX[p];
        float pz = center.y + offZ[p];

        // Vertical pole
        auto poleEnt = scene.Entity_CreateCube("tl_pole");
        auto* pt = scene.transforms.GetComponent(poleEnt);
        pt->Scale(XMFLOAT3(0.15f, poleH, 0.15f));
        pt->Translate(XMFLOAT3(px, poleH * 0.5f, pz));
        pt->UpdateTransform();
        scene.materials.GetComponent(poleEnt)->SetBaseColor(
            XMFLOAT4(0.25f, 0.25f, 0.25f, 1.0f));

        // Horizontal mast arm
        float armCX = px + armDX[p] * armLen * 0.5f;
        float armCZ = pz + armDZ[p] * armLen * 0.5f;
        float armSX = (armDX[p] != 0.f) ? armLen : 0.12f;
        float armSZ = (armDZ[p] != 0.f) ? armLen : 0.12f;

        auto armEnt = scene.Entity_CreateCube("tl_arm");
        auto* at = scene.transforms.GetComponent(armEnt);
        at->Scale(XMFLOAT3(armSX, 0.12f, armSZ));
        at->Translate(XMFLOAT3(armCX, poleH, armCZ));
        at->UpdateTransform();
        scene.materials.GetComponent(armEnt)->SetBaseColor(
            XMFLOAT4(0.25f, 0.25f, 0.25f, 1.0f));

        // Detect available exit directions
        bool hasStraight = city.IsRoadLike(gx + sDX[p], gz + sDZ[p]);
        bool hasLeft     = city.IsRoadLike(gx + lDX[p], gz + lDZ[p]);
        bool hasRight    = city.IsRoadLike(gx + rDX[p], gz + rDZ[p]);

        poles[p].hasStraight = hasStraight;
        poles[p].hasLeft     = hasLeft;
        poles[p].hasRight    = hasRight;

        // Light and arrow positions
        float lightBaseX = px + armDX[p] * armLen;
        float lightBaseZ = pz + armDZ[p] * armLen;
        float lightY = poleH - 0.55f;
        float arrowY = lightY - 0.45f;

        // Offset directions for left/right lights relative to driver
        float leftOX  =  armDX[p] * spacing, leftOZ  =  armDZ[p] * spacing;
        float rightOX = -armDX[p] * spacing, rightOZ = -armDZ[p] * spacing;

        // Arrow shape: horizontal arrows elongated in leftDir (= armDir),
        // straight arrow elongated vertically
        bool armAlongX = std::abs(armDX[p]) > 0.5f;
        XMFLOAT3 hArrowScale = armAlongX
            ? XMFLOAT3(0.28f, 0.04f, 0.06f)
            : XMFLOAT3(0.06f, 0.04f, 0.28f);
        XMFLOAT3 vArrowScale(0.06f, 0.28f, 0.06f);

        // Light shapes: horizontal arrows = wide in armDir, straight = tall
        XMFLOAT3 hLightScale = armAlongX
            ? XMFLOAT3(0.40f, 0.30f, 0.18f)
            : XMFLOAT3(0.18f, 0.30f, 0.40f);
        XMFLOAT3 vLightScale(0.18f, 0.45f, 0.18f);

        auto makeLight = [&](const char* name, float ox, float oz, const XMFLOAT3& sc) {
            auto e = scene.Entity_CreateCube(name);
            auto* t = scene.transforms.GetComponent(e);
            t->Scale(sc);
            t->Translate(XMFLOAT3(lightBaseX + ox, lightY, lightBaseZ + oz));
            t->UpdateTransform();
            auto* m = scene.materials.GetComponent(e);
            m->SetBaseColor(XMFLOAT4(0.9f, 0.1f, 0.1f, 1.0f));
            m->SetEmissiveColor(XMFLOAT4(0.9f, 0.1f, 0.1f, 4.0f));
            return e;
        };

        auto makeArrow = [&](const char* name, float ox, float oz, const XMFLOAT3& sc) {
            auto e = scene.Entity_CreateCube(name);
            auto* t = scene.transforms.GetComponent(e);
            t->Scale(sc);
            t->Translate(XMFLOAT3(lightBaseX + ox, arrowY, lightBaseZ + oz));
            t->UpdateTransform();
            auto* m = scene.materials.GetComponent(e);
            m->SetBaseColor(XMFLOAT4(0.15f, 0.15f, 0.15f, 1.0f));
            m->SetEmissiveColor(XMFLOAT4(0.15f, 0.15f, 0.15f, 0.2f));
            return e;
        };

        if (hasLeft)
        {
            poles[p].lightLeft = makeLight("tl_left", leftOX, leftOZ, hLightScale);
            poles[p].arrowLeft = makeArrow("tl_arr_left", leftOX, leftOZ, hArrowScale);
        }
        if (hasStraight)
        {
            poles[p].lightStraight = makeLight("tl_straight", 0.f, 0.f, vLightScale);
            poles[p].arrowStraight = makeArrow("tl_arr_straight", 0.f, 0.f, vArrowScale);
        }
        if (hasRight)
        {
            poles[p].lightRight = makeLight("tl_right", rightOX, rightOZ, hLightScale);
            poles[p].arrowRight = makeArrow("tl_arr_right", rightOX, rightOZ, hArrowScale);
        }

        poles[p].pole = poleEnt;
        poles[p].arm  = armEnt;
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
        if (pv.pole          != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.pole);
        if (pv.arm           != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.arm);
        if (pv.lightLeft     != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.lightLeft);
        if (pv.lightStraight != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.lightStraight);
        if (pv.lightRight    != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.lightRight);
        if (pv.arrowLeft     != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.arrowLeft);
        if (pv.arrowStraight != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.arrowStraight);
        if (pv.arrowRight    != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(pv.arrowRight);
        pv = {};
    }
    poles.clear();
}
