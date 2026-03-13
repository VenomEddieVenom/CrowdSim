 // ============================================================
//  main.cpp  –  CrowdSim entry point (Win32 + WickedEngine)
//
//  Architecture:
//    CrowdApp (wi::Application)
//      └─ CrowdRenderPath (wi::RenderPath3D)
//            ├─ Start()      : set up camera & crowd (replaces Initialize)
//            ├─ Update(dt)   : tick crowd + queue DrawBox calls
//            └─ Compose(cmd) : HUD overlay (FPS, threads, counts)
// ============================================================

#include "WickedEngine.h"
#include "CityLayout.h"
#include "CarSystem.h"
#include "CrowdSystem.h"
#include "TrafficLightSystem.h"

#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <DbgHelp.h>
#pragma comment(lib, "DbgHelp.lib")

// ---- Crash logger ----
static void WriteCrashLog(EXCEPTION_POINTERS* ep)
{
    std::ofstream f("crash.log", std::ios::trunc);
    if (!f) return;
    std::time_t t = std::time(nullptr);
    char tbuf[64]; std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    f << "=== CRASH " << tbuf << " ===\n";
    if (ep && ep->ExceptionRecord) {
        f << "ExceptionCode : 0x" << std::hex << ep->ExceptionRecord->ExceptionCode << "\n";
        f << "ExceptionAddr : 0x" << std::hex << (uintptr_t)ep->ExceptionRecord->ExceptionAddress << "\n";
    }
    // Walk stack
    HANDLE proc = GetCurrentProcess();
    HANDLE thrd = GetCurrentThread();
    SymInitialize(proc, nullptr, TRUE);
    CONTEXT ctx = ep ? *ep->ContextRecord : CONTEXT{};
    if (!ep) { ctx.ContextFlags = CONTEXT_FULL; RtlCaptureContext(&ctx); }
    STACKFRAME64 sf{};
    sf.AddrPC.Mode = sf.AddrFrame.Mode = sf.AddrStack.Mode = AddrModeFlat;
#ifdef _M_X64
    sf.AddrPC.Offset    = ctx.Rip;
    sf.AddrFrame.Offset = ctx.Rbp;
    sf.AddrStack.Offset = ctx.Rsp;
    DWORD mtype = IMAGE_FILE_MACHINE_AMD64;
#else
    sf.AddrPC.Offset    = ctx.Eip;
    sf.AddrFrame.Offset = ctx.Ebp;
    sf.AddrStack.Offset = ctx.Esp;
    DWORD mtype = IMAGE_FILE_MACHINE_I386;
#endif
    char symBuf[sizeof(SYMBOL_INFO) + MAX_SYM_NAME] = {};
    SYMBOL_INFO* sym = reinterpret_cast<SYMBOL_INFO*>(symBuf);
    sym->SizeOfStruct = sizeof(SYMBOL_INFO); sym->MaxNameLen = MAX_SYM_NAME;
    IMAGEHLP_LINE64 line{}; line.SizeOfStruct = sizeof(line); DWORD disp32 = 0;
    f << "Stack trace:\n";
    for (int frame = 0; frame < 64; ++frame) {
        if (!StackWalk64(mtype, proc, thrd, &sf, &ctx, nullptr,
                         SymFunctionTableAccess64, SymGetModuleBase64, nullptr)) break;
        if (!sf.AddrPC.Offset) break;
        DWORD64 disp64 = 0;
        f << "  [" << frame << "] 0x" << std::hex << sf.AddrPC.Offset;
        if (SymFromAddr(proc, sf.AddrPC.Offset, &disp64, sym))
            f << "  " << sym->Name << " +0x" << std::hex << disp64;
        if (SymGetLineFromAddr64(proc, sf.AddrPC.Offset, &disp32, &line))
            f << "  (" << line.FileName << ":" << std::dec << line.LineNumber << ")";
        f << "\n";
    }
    SymCleanup(proc);
    f.flush();
}
// ---- Event log (step-by-step trace, flushed each write) ----
static std::ofstream g_evlog;
static void EvLog(const std::string& msg) {
    if (!g_evlog.is_open()) g_evlog.open("evlog.txt", std::ios::trunc);
    g_evlog << msg << "\n"; g_evlog.flush();
}
static LONG WINAPI UnhandledExFilter(EXCEPTION_POINTERS* ep) {
    WriteCrashLog(ep);
    return EXCEPTION_CONTINUE_SEARCH;
}



// ============================================================
//  CrowdRenderPath
// ============================================================
class CrowdRenderPath : public wi::RenderPath3D
{
public:
    CityLayout  city;
    CarSystem   cars;
    CrowdSystem crowd;
    TrafficLightSystem trafficLights;

    // distance threshold: >CAR_HOP_THRESHOLD road cells → use car
    static constexpr int CAR_HOP_THRESHOLD = 3;

    // FPS-camera state
    wi::scene::TransformComponent camera_transform;
    float moveSpeed = 50.0f;  // units/second (scroll wheel adjusts)
    float simSpeed  = 1.0f;   // simulation time scale

    // Day/night cycle
    wi::ecs::Entity sunEntity = wi::ecs::INVALID_ENTITY;
    float timeOfDay   = 0.30f;  // 0..1 (0.25=sunrise, 0.5=noon, 0.75=sunset)
    float dayDuration = 86400.0f; // real seconds for a full day at 1x (1:1 real time)

    mutable float displayFPS = 0.0f;

    // Economy
    float townTreasury  = 0.0f;
    static constexpr float TAX_RATE = 0.10f;

    // Spawn tracking – prevents double-spawning per house cell
    bool houseHasSpawned[CityLayout::GRID_SIZE * CityLayout::GRID_SIZE] = {};

    // Reproduction timer
    float reproTimer_ = 0.0f;
    static constexpr float REPRO_INTERVAL = 30.0f; // every 30 game-seconds

    // Bus spawn timer
    float busTimer_ = 0.0f;
    static constexpr float BUS_INTERVAL = 20.0f; // try every 20 game-seconds
    static constexpr int   MAX_BUSES    = 5;      // bus cap
    uint32_t busRng_ = 12345;

    // Agent / car / building selection
    int32_t selectedAgent = -1;  // agent index, or -1 for none
    int32_t selectedCar   = -1;  // car index, or -1 for none
    int32_t inspectGX = -1, inspectGZ = -1;  // inspected building cell, or -1 for none

    // Shadow cascade sliders
    float cascadeDist1_     = 30.0f;    // near cascade distance (m)
    float cascadeDist2_     = 200.0f;   // far cascade distance (m)
    bool  draggingCascade1_ = false;
    bool  draggingCascade2_ = false;

    // Build mode
    using CellType = CityLayout::CellType;
    // Deferred scene-modification queue: actions are pushed during input handling
    // (which runs AFTER wi::RenderPath3D::Update submits render jobs) and drained
    // at the TOP of the next Update(), BEFORE any render jobs are submitted.
    // This prevents the render thread from reading scene data that the main thread
    // is simultaneously mutating (ACCESS_VIOLATION in ComputeObjectLODForView).
    std::vector<std::function<void()>> pendingPlacementActions_;
    bool     buildModeEnabled = false;
    int      placeTool        = 0;    // 0=road 1=house 2=workplace 3=tree 4=crosswalk 5=parking 6=lake 7=power_plant 8=water_pump 9=police 10=fire_station 11=hospital
    int      hoverGX = 0, hoverGZ = 0;
    bool     hoverValid = false;
    bool     clickConsumedByPanel = false;  // prevent world-placement when clicking UI

    // Road click-start / click-end state
    bool     roadPlaceActive  = false;  // true = waiting for end point
    int      roadStartGX = 0, roadStartGZ = 0;
    std::vector<std::pair<int,int>> roadPreviewPath; // cells of the preview path

    // Saves menu
    bool     savesMenuOpen = false;
    bool     showSidewalkDebug = false;  // F8 toggle: visualise sidewalk layer
    float    hoverWorldX = 0.f, hoverWorldZ = 0.f;  // world-space hit position
    bool     swDebugClickActive = false;
    float    swDebugClickX = 0.f, swDebugClickZ = 0.f;
    int      swDebugClickSX = 0, swDebugClickSZ = 0;
    bool     swDebugClickOnSW = false;
    std::vector<std::string> saveFiles;     // filenames in saves/
    int      savesMenuScroll = 0;
    bool     savesMenuSaveMode = false;     // true = saving, false = loading
    std::string pendingSaveName;            // for new save typing
    bool     typingSaveName = false;

    // UI scale factor: all pixel sizes are authored for 1080p reference.
    // Scale proportionally to actual screen height so everything looks right
    // at any resolution / DPI.
    float uiScale() const { return (float)height / 1080.0f; }

    void RefreshSaveList() {
        saveFiles.clear();
        namespace fs = std::filesystem;
        fs::create_directories("saves");
        for (auto& entry : fs::directory_iterator("saves")) {
            if (entry.is_regular_file() && entry.path().extension() == ".city")
                saveFiles.push_back(entry.path().stem().string());
        }
        std::sort(saveFiles.begin(), saveFiles.end());
    }

    void DoLoadSave(const std::string& name) {
        std::string path = "saves/" + name + ".city";
        auto sd = CityLayout::LoadFromFile(path.c_str());
        if (!sd.valid) {
            wi::backlog::post("[Load] Failed to load: " + path, wi::backlog::LogLevel::Warning);
            return;
        }
        trafficLights.RebuildIntersections(city);
        for (int gz2 = 0; gz2 < CityLayout::GRID_SIZE; ++gz2)
        for (int gx2 = 0; gx2 < CityLayout::GRID_SIZE; ++gx2)
            city.ClearCell(gx2, gz2);
        std::fill(std::begin(houseHasSpawned), std::end(houseHasSpawned), false);
        // Three passes: roads first, buildings second, crosswalks last.
        // Crosswalks must be placed after all road/intersection cells are in the grid
        // so that BuildCrosswalk can correctly detect which neighbor is the intersection
        // and shift the zebra stripes to the right edge (not leave them at cell center).
        for (int pass = 0; pass < 3; ++pass)
        for (int gz2 = 0; gz2 < CityLayout::GRID_SIZE; ++gz2)
        for (int gx2 = 0; gx2 < CityLayout::GRID_SIZE; ++gx2) {
            auto ct = sd.cells[gz2 * CityLayout::GRID_SIZE + gx2];
            if (ct == CellType::EMPTY) continue;
            if (pass == 0 && ct != CellType::ROAD) continue;
            if (pass == 1 && (ct == CellType::ROAD || ct == CellType::CROSSWALK)) continue;
            if (pass == 2 && ct != CellType::CROSSWALK) continue;
            city.PlaceCell(gx2, gz2, ct);
        }
        trafficLights.RebuildIntersections(city);
        PlaceCrosswalksAroundIntersections();
        city.BuildSidewalkLayer([&](int gx, int gz){ return trafficLights.IsIntersection(gx, gz); });
        city.RebuildAllLaneMarkings();
        for (int gz2 = 0; gz2 < CityLayout::GRID_SIZE; ++gz2)
        for (int gx2 = 0; gx2 < CityLayout::GRID_SIZE; ++gx2) {
            int lanes = sd.roadLanes[gz2 * CityLayout::GRID_SIZE + gx2];
            if (lanes == 2 || lanes == 4 || lanes == 6)
                city.SetRoadLanes(gx2, gz2, lanes);
        }
        for (int gz2 = 0; gz2 < CityLayout::GRID_SIZE; ++gz2)
        for (int gx2 = 0; gx2 < CityLayout::GRID_SIZE; ++gx2) {
            int idx2 = gz2 * CityLayout::GRID_SIZE + gx2;
            if (sd.cells[idx2] == CellType::HOUSE) {
                city.SetHousePop(gx2, gz2, 0);
                if (TrySpawnFromHouse(gx2, gz2))
                    houseHasSpawned[idx2] = true;
                // Restore saved population (TrySpawnFromHouse adds pop as side-effect)
                city.SetHousePop(gx2, gz2, sd.housePop[idx2]);
                city.UpdateHouseHeight(gx2, gz2);
            }
        }
        wi::backlog::post("[Load] Loaded: " + path, wi::backlog::LogLevel::Default);
        savesMenuOpen = false;
    }

    void Start() override
    {
        city.Initialize();
        cars.Initialize();
        crowd.Initialize();
        trafficLights.Initialize();

        // ---- Camera: oblique overview of city centre ----
        auto& cam = wi::scene::GetCamera();
        cam.zNearP = 0.5f;
        cam.zFarP  = 2500.0f;
        cam.fov    = XM_PI / 3.5f;

        camera_transform.ClearTransform();
        camera_transform.Translate(XMFLOAT3(0.0f, 200.0f, -300.0f));
        camera_transform.RotateRollPitchYaw(XMFLOAT3(-0.55f, 0.0f, 0.0f));
        camera_transform.UpdateTransform();
        cam.TransformCamera(camera_transform);
        cam.UpdateCamera();

        // ---- Sky (initial values, updated by day cycle each frame) ----
        auto& scene = wi::scene::GetScene();
        scene.weather.fogStart   = 600.0f;
        scene.weather.fogDensity = 0.002f;

        // ---- Sun (directional light with shadow cascades) ----
        sunEntity = scene.Entity_CreateLight(
            "Sun",
            XMFLOAT3(0.0f, 0.0f, 0.0f),
            XMFLOAT3(1.0f, 0.95f, 0.82f),
            10.0f, 10000.0f,
            wi::scene::LightComponent::DIRECTIONAL
        );
        auto* sunLight = scene.lights.GetComponent(sunEntity);
        if (sunLight)
        {
            sunLight->SetCastShadow(true);
            sunLight->cascade_distances = { 30.0f, 200.0f };  // 2 cascades; enough for overhead view
        }
        // Set initial sun rotation matching timeOfDay
        {
            float sunAngle = (timeOfDay - 0.5f) * XM_2PI;
            auto* sunTr = scene.transforms.GetComponent(sunEntity);
            sunTr->ClearTransform();
            sunTr->RotateRollPitchYaw(XMFLOAT3(sunAngle, 0.4f, 0.0f));
            sunTr->SetDirty();
        }

        // ---- Post-process ----
        setExposure(1.3f);
        setBloomThreshold(0.6f);
        setAO(AO_MSAO);           // MSAO is much cheaper than HBAO
        setAOPower(1.5f);
        setShadowsEnabled(true);
        setScreenSpaceShadowSampleCount(4);  // 16 was excessive

        wi::renderer::SetToDrawGridHelper(false);
    }

    void Render() const override
    {
        wi::RenderPath3D::Render();
    }

    void Update(float dt) override
    {
        // Drain deferred scene modifications BEFORE submitting render jobs.
        // All PlaceCell / RebuildIntersections calls happen here, before the
        // renderer touches scene object arrays on job threads.
        if (!pendingPlacementActions_.empty()) {
            for (auto& action : pendingPlacementActions_) action();
            pendingPlacementActions_.clear();
        }
        if (city.laneMarksDirty_) city.RebuildAllLaneMarkings();

        wi::RenderPath3D::Update(dt);

        auto& cam  = wi::scene::GetCamera();
        auto  ms   = wi::input::GetMouseState();

        // ---- Mouse-look: hold RMB (WickedEngine native input) ----
        const XMFLOAT4 originalMouse = wi::input::GetPointer();
        float xDif = 0.0f, yDif = 0.0f;

        if (wi::input::Down(wi::input::MOUSE_BUTTON_RIGHT))
        {
            wi::input::HidePointer(true);
            wi::input::SetPointer(originalMouse);
            xDif = ms.delta_position.x * 0.1f * (1.0f / 60.0f);
            yDif = ms.delta_position.y * 0.1f * (1.0f / 60.0f);
        }
        else
        {
            wi::input::HidePointer(false);
        }

        // ---- Speed (scroll to adjust, Shift to sprint) ----
        moveSpeed += ms.delta_wheel * 8.0f;
        moveSpeed  = std::clamp(moveSpeed, 5.0f, 500.0f);

        float spd = moveSpeed * dt;
        if (wi::input::Down(wi::input::KEYBOARD_BUTTON_LSHIFT) ||
            wi::input::Down(wi::input::KEYBOARD_BUTTON_RSHIFT))
            spd *= 4.0f;

        // ---- WASD movement in camera-local space ----
        XMVECTOR moveVec = XMVectorZero();
        if (wi::input::Down((wi::input::BUTTON)'W')) moveVec = XMVectorAdd(moveVec, XMVectorSet(0,0,1,0));
        if (wi::input::Down((wi::input::BUTTON)'S')) moveVec = XMVectorAdd(moveVec, XMVectorSet(0,0,-1,0));
        if (wi::input::Down((wi::input::BUTTON)'A')) moveVec = XMVectorAdd(moveVec, XMVectorSet(-1,0,0,0));
        if (wi::input::Down((wi::input::BUTTON)'D')) moveVec = XMVectorAdd(moveVec, XMVectorSet(1,0,0,0));
        if (wi::input::Down((wi::input::BUTTON)'Q')) moveVec = XMVectorAdd(moveVec, XMVectorSet(0,-1,0,0));
        if (wi::input::Down((wi::input::BUTTON)'E')) moveVec = XMVectorAdd(moveVec, XMVectorSet(0,1,0,0));

        XMMATRIX camRot = XMMatrixRotationQuaternion(XMLoadFloat4(&camera_transform.rotation_local));
        XMVECTOR movWorld = XMVector3TransformNormal(moveVec * spd, camRot);
        XMFLOAT3 _move; XMStoreFloat3(&_move, movWorld);

        if (abs(xDif) + abs(yDif) > 0.0f ||
            XMVectorGetX(XMVector3LengthSq(movWorld)) > 0.00001f)
        {
            camera_transform.Translate(_move);
            camera_transform.RotateRollPitchYaw(XMFLOAT3(yDif, xDif, 0.0f));
            cam.SetDirty();
        }
        camera_transform.UpdateTransform();
        cam.TransformCamera(camera_transform);
        cam.UpdateCamera();

        // ---- Day/night cycle ----
        {
            auto& scene = wi::scene::GetScene();
            timeOfDay += (dt * simSpeed) / dayDuration;
            timeOfDay = fmodf(timeOfDay, 1.0f);

            // sunAngle: 0 at noon (timeOfDay=0.5), PI at midnight
            float sunAngle = (timeOfDay - 0.5f) * XM_2PI;
            float sunElev = cosf(sunAngle); // +1=noon, -1=midnight

            // Rotate sun transform (engine derives direction from Y-axis)
            auto* sunTr = scene.transforms.GetComponent(sunEntity);
            if (sunTr)
            {
                sunTr->ClearTransform();
                sunTr->RotateRollPitchYaw(XMFLOAT3(sunAngle, 0.4f, 0.0f));
                sunTr->SetDirty();
            }

            // Light intensity + warm color at low elevation
            auto* sunLight = scene.lights.GetComponent(sunEntity);
            if (sunLight)
            {
                if (sunElev > 0.0f)
                {
                    sunLight->intensity = 5.0f * sunElev;
                    sunLight->color = XMFLOAT3(
                        1.0f,
                        0.75f + 0.20f * sunElev,
                        0.50f + 0.40f * sunElev);
                    sunLight->SetCastShadow(true);   // sun is up — enable shadows
                }
                else
                {
                    sunLight->intensity = 0.0f;
                    sunLight->SetCastShadow(false);  // sun below horizon — skip shadow maps
                }
            }

            // Sky and ambient based on sun elevation
            float dayFactor = std::clamp(sunElev * 3.0f + 0.3f, 0.0f, 1.0f);
            float sunsetGlow = std::max(0.0f, 1.0f - std::abs(sunElev) * 4.0f);

            scene.weather.horizon = XMFLOAT3(
                0.15f + 0.40f * dayFactor + 0.50f * sunsetGlow,
                0.12f + 0.53f * dayFactor + 0.20f * sunsetGlow,
                0.10f + 0.72f * dayFactor);
            scene.weather.zenith = XMFLOAT3(
                0.01f + 0.07f * dayFactor,
                0.02f + 0.18f * dayFactor,
                0.05f + 0.50f * dayFactor);
            scene.weather.ambient = XMFLOAT3(
                0.03f + 0.15f * dayFactor,
                0.03f + 0.13f * dayFactor,
                0.05f + 0.18f * dayFactor);
            scene.weather.sunColor = XMFLOAT3(
                1.0f * dayFactor + 0.8f * sunsetGlow,
                0.85f * dayFactor + 0.3f * sunsetGlow,
                0.65f * dayFactor);

            setExposure(1.3f + 0.5f * (1.0f - dayFactor));
        }

        // ---- Agent count: Numpad+ / Numpad- ----
        // (agent count now driven by placed houses)

        // ---- Shadow cascade sliders (bottom-left panel) ----
        {
            const float S = uiScale();
            XMFLOAT4 ptr  = wi::input::GetPointer();
            bool lmbDown  = wi::input::Down(wi::input::MOUSE_BUTTON_LEFT);
            bool lmbPress = wi::input::Press(wi::input::MOUSE_BUTTON_LEFT);

            const float sliderX = 15.f*S, sliderW = 185.f*S;
            const float track1Y = (float)cam.height - 85.f*S;
            const float track2Y = (float)cam.height - 48.f*S;
            const float trackH  = 12.f*S, hitExtra = 8.f*S;

            auto hitTrack = [&](float ty) {
                return ptr.x >= sliderX && ptr.x <= sliderX + sliderW
                    && ptr.y >= ty - hitExtra && ptr.y <= ty + trackH + hitExtra;
            };

            if (lmbPress && hitTrack(track1Y)) { draggingCascade1_ = true; clickConsumedByPanel = true; }
            if (lmbPress && hitTrack(track2Y)) { draggingCascade2_ = true; clickConsumedByPanel = true; }
            if (!lmbDown) { draggingCascade1_ = false; draggingCascade2_ = false; }

            if (draggingCascade1_) {
                float t = std::clamp((ptr.x - sliderX) / sliderW, 0.f, 1.f);
                cascadeDist1_ = 5.f + t * 145.f;  // 5 – 150 m
                cascadeDist2_ = std::max(cascadeDist2_, cascadeDist1_ + 10.f);
            }
            if (draggingCascade2_) {
                float t = std::clamp((ptr.x - sliderX) / sliderW, 0.f, 1.f);
                cascadeDist2_ = 30.f + t * 770.f;  // 30 – 800 m
                cascadeDist1_ = std::min(cascadeDist1_, cascadeDist2_ - 10.f);
            }

            // Apply current slider values to the sun light every frame
            if (auto* sl = wi::scene::GetScene().lights.GetComponent(sunEntity))
                sl->cascade_distances = { cascadeDist1_, cascadeDist2_ };
        }

        // ---- Build mode toggle ----
        if (wi::input::Press((wi::input::BUTTON)'B'))
            buildModeEnabled = !buildModeEnabled;

        if (buildModeEnabled)
        {
            const float S = uiScale();
            // Panel click detection (right side of screen, 12 tool buttons)
            const float panelX  = (float)cam.width - 215.0f*S;
            const float btnH    = 46.0f*S;
            const float btnStart = 145.0f*S;
            const float btnStep  = 55.0f*S;

            XMFLOAT4 ptr = wi::input::GetPointer();
            bool inPanel = ptr.x >= panelX;
            clickConsumedByPanel = false;

            if (inPanel && wi::input::Press(wi::input::MOUSE_BUTTON_LEFT))
            {
                clickConsumedByPanel = true;
                for (int k = 0; k < 12; ++k) {
                    float by = btnStart + k * btnStep;
                    if (ptr.y >= by && ptr.y < by + btnH)
                        placeTool = k;
                }
            }

            // Ray → ground plane to find hovered grid cell
            hoverValid = false;
            XMMATRIX invVP = XMMatrixInverse(nullptr, cam.GetViewProjection());
            float ndcX =  (ptr.x / static_cast<float>(cam.width))  * 2.0f - 1.0f;
            float ndcY = 1.0f - (ptr.y / static_cast<float>(cam.height)) * 2.0f;
            XMVECTOR nearPt = XMVector3TransformCoord(XMVectorSet(ndcX, ndcY, 0.f, 1.f), invVP);
            XMVECTOR farPt  = XMVector3TransformCoord(XMVectorSet(ndcX, ndcY, 1.f, 1.f), invVP);
            XMVECTOR rDir   = XMVector3Normalize(farPt - nearPt);
            float rDirY = XMVectorGetY(rDir);
            if (fabsf(rDirY) > 0.001f)
            {
                float t = -XMVectorGetY(nearPt) / rDirY;
                if (t > 0.0f)
                {
                    float hx = XMVectorGetX(nearPt) + XMVectorGetX(rDir) * t;
                    float hz = XMVectorGetZ(nearPt) + XMVectorGetZ(rDir) * t;
                    hoverWorldX = hx; hoverWorldZ = hz;
                    hoverValid = city.WorldToGrid(hx, hz, hoverGX, hoverGZ);
                }
            }

            // LMB interactions (not while RMB-looking, not on panel)
            bool lmbDown = wi::input::Down(wi::input::MOUSE_BUTTON_LEFT) &&
                           !wi::input::Down(wi::input::MOUSE_BUTTON_RIGHT) &&
                           !clickConsumedByPanel && !inPanel;
            bool lmbPress = wi::input::Press(wi::input::MOUSE_BUTTON_LEFT) &&
                            !wi::input::Down(wi::input::MOUSE_BUTTON_RIGHT) &&
                            !clickConsumedByPanel && !inPanel;

            // Cancel road placement on RMB press
            if (roadPlaceActive && wi::input::Press(wi::input::MOUSE_BUTTON_RIGHT))
            {
                roadPlaceActive = false;
                roadPreviewPath.clear();
            }

            // When tool changes away from road, cancel active road placement
            if (placeTool != 0 && roadPlaceActive)
            {
                roadPlaceActive = false;
                roadPreviewPath.clear();
            }

            // Update road preview path when hovering in road-start mode
            if (placeTool == 0 && roadPlaceActive && hoverValid)
            {
                roadPreviewPath = ComputeRoadPreviewPath(
                    roadStartGX, roadStartGZ, hoverGX, hoverGZ, city);
            }
            else if (!roadPlaceActive)
            {
                roadPreviewPath.clear();
            }

            // Middle-click (or Shift+LMB) on a road cell: cycle lane count
            if (hoverValid && wi::input::Press(wi::input::MOUSE_BUTTON_MIDDLE))
            {
                auto ct = city.GetCellType(hoverGX, hoverGZ);
                if (ct == CellType::ROAD || ct == CellType::CROSSWALK)
                    city.CycleRoadLanes(hoverGX, hoverGZ);
            }

            if (hoverValid && lmbPress)
            {
                if (placeTool == 0) // ROAD — click-start / click-end
                {
                    if (!roadPlaceActive)
                    {
                        // First click: record start
                        roadPlaceActive = true;
                        roadStartGX     = hoverGX;
                        roadStartGZ     = hoverGZ;
                        // Defer PlaceCell to next frame so render jobs finish first
                        int cgx = hoverGX, cgz = hoverGZ;
                        EvLog("[ROAD-1st] queuing PlaceCell gx=" + std::to_string(cgx) + " gz=" + std::to_string(cgz));
                        pendingPlacementActions_.push_back([this, cgx, cgz](){
                            EvLog("[ROAD-1st-def] PlaceCell gx=" + std::to_string(cgx) + " gz=" + std::to_string(cgz));
                            city.PlaceCell(cgx, cgz, CellType::ROAD);
                            EvLog("[ROAD-1st-def] PlaceCell done");
                            PlaceCrosswalksNearCells({{cgx, cgz}});
                            TrySpawnAllUnspawnedHouses();
                        });
                        roadPreviewPath.clear();
                    }
                    else
                    {
                        // Second click: place entire preview path
                        // Defer path placement to next frame
                        auto capturedPath = roadPreviewPath;
                        EvLog("[ROAD-2nd] queuing path size=" + std::to_string(capturedPath.size()));
                        pendingPlacementActions_.push_back([this, capturedPath](){
                            bool anyPlaced = false;
                            for (auto& [pgx, pgz] : capturedPath)
                            {
                                if (city.GetCellType(pgx, pgz) == CellType::EMPTY)
                                {
                                    EvLog("[ROAD-2nd-def] PlaceCell gx=" + std::to_string(pgx) + " gz=" + std::to_string(pgz));
                                    city.PlaceCell(pgx, pgz, CellType::ROAD);
                                    anyPlaced = true;
                                }
                            }
                            if (anyPlaced)
                            {
                                EvLog("[ROAD-2nd-def] calling PlaceCrosswalksNearCells");
                                PlaceCrosswalksNearCells(capturedPath);
                                EvLog("[ROAD-2nd-def] done");
                                TrySpawnAllUnspawnedHouses();
                            }
                        });
                        roadPlaceActive = false;
                        roadPreviewPath.clear();
                    }
                }
                else if (placeTool == 3) // TREE
                {
                    if (lmbDown)
                    {
                        XMFLOAT2 c = city.GridCellCenter(hoverGX, hoverGZ);
                        wi::scene::Scene tmp;
                        wi::scene::LoadModel(tmp, "models/drzewo.wiscene",
                            XMMatrixTranslation(c.x, 0.0f, c.y));
                        wi::scene::GetScene().Merge(tmp);
                    }
                }
                else if (placeTool == 4) // CROSSWALK — single click only
                {
                    // Crosswalk can only be placed on existing ROAD cells (not intersections)
                    CellType ct = city.GetCellType(hoverGX, hoverGZ);
                    EvLog("[CW-manual] click gx=" + std::to_string(hoverGX) + " gz=" + std::to_string(hoverGZ) + " ct=" + std::to_string((int)ct));
                    if (ct == CellType::ROAD)
                    {
                        // Prevent placing on intersection cells (3+ road neighbours)
                        const int ddx2[] = { 1,-1, 0, 0 };
                        const int ddz2[] = { 0, 0, 1,-1 };
                        int roadNeighbors = 0;
                        for (int d = 0; d < 4; ++d)
                        {
                            auto nct = city.GetCellType(hoverGX + ddx2[d], hoverGZ + ddz2[d]);
                            if (nct == CellType::ROAD || nct == CellType::CROSSWALK)
                                ++roadNeighbors;
                        }
                        EvLog("[CW-manual] roadNeighbors=" + std::to_string(roadNeighbors));
                        if (roadNeighbors <= 2)
                        {
                            EvLog("[CW-manual] queuing PlaceCell");
                        int cgx = hoverGX, cgz = hoverGZ;
                        pendingPlacementActions_.push_back([this, cgx, cgz](){
                            EvLog("[CW-manual-def] PlaceCell gx=" + std::to_string(cgx) + " gz=" + std::to_string(cgz));
                            city.PlaceCell(cgx, cgz, CellType::CROSSWALK);
                            EvLog("[CW-manual-def] PlaceCell done, calling RebuildIntersections");
                            trafficLights.RebuildIntersections(city);
                            EvLog("[CW-manual-def] RebuildIntersections done, calling BuildSidewalkLayer");
                            city.BuildSidewalkLayer([&](int gx2, int gz2){ return trafficLights.IsIntersection(gx2, gz2); });
                            EvLog("[CW-manual-def] done");
                        });
                        }
                    }
                }
                else
                {
                    // Tools 1=house, 2=workplace, 5=parking, 6=lake, 7=power_plant, 8=water_pump, 9=police, 10=fire_station, 11=hospital
                    CellType ct;
                    if (placeTool == 5)      ct = CellType::PARKING;
                    else if (placeTool == 6) ct = CellType::LAKE;
                    else if (placeTool == 7) ct = CellType::POWER_PLANT;
                    else if (placeTool == 8) {
                        // Water pump requires adjacent lake
                        if (!city.HasAdjacentLake(hoverGX, hoverGZ)) {
                            // Silently refuse — no lake nearby
                            ct = CellType::EMPTY; // will be skipped
                        } else {
                            ct = CellType::WATER_PUMP;
                        }
                    }
                    else if (placeTool == 9)  ct = CellType::POLICE;
                    else if (placeTool == 10) ct = CellType::FIRE_STATION;
                    else if (placeTool == 11) ct = CellType::HOSPITAL;
                    else
                        ct = static_cast<CellType>(placeTool + 1);
                    if (ct != CellType::EMPTY) {
                    bool placed = city.PlaceCell(hoverGX, hoverGZ, ct);
                    if (placed)
                    {
                        int cellIdx = hoverGZ * CityLayout::GRID_SIZE + hoverGX;
                        if (ct == CellType::HOUSE)
                        {
                            houseHasSpawned[cellIdx] = false;
                            if (TrySpawnFromHouse(hoverGX, hoverGZ))
                                houseHasSpawned[cellIdx] = true;
                        }
                        else
                        {
                            TrySpawnAllUnspawnedHouses();
                        }
                    }
                    }
                }
            }
            else if (hoverValid && lmbDown && placeTool >= 1 && placeTool <= 2)
            {
                // Allow dragging for house/workplace placement
                CellType ct = static_cast<CellType>(placeTool + 1);
                bool placed = city.PlaceCell(hoverGX, hoverGZ, ct);
                if (placed)
                {
                    int cellIdx = hoverGZ * CityLayout::GRID_SIZE + hoverGX;
                    if (ct == CellType::HOUSE)
                    {
                        houseHasSpawned[cellIdx] = false;
                        if (TrySpawnFromHouse(hoverGX, hoverGZ))
                            houseHasSpawned[cellIdx] = true;
                    }
                    else
                    {
                        TrySpawnAllUnspawnedHouses();
                    }
                }
            }

            DrawBuildGrid();
        }
        else
        {
            // ---- Simulation speed: 1/2/3/4/5 keys ----
            if (wi::input::Press((wi::input::BUTTON)'1')) simSpeed = 1.0f;
            if (wi::input::Press((wi::input::BUTTON)'2')) simSpeed = 2.0f;
            if (wi::input::Press((wi::input::BUTTON)'3')) simSpeed = 5.0f;
            if (wi::input::Press((wi::input::BUTTON)'4')) simSpeed = 10.0f;
            if (wi::input::Press((wi::input::BUTTON)'5')) simSpeed = 100.0f;
            if (wi::input::Press((wi::input::BUTTON)'6')) simSpeed = 1000.0f;
            if (wi::input::Press((wi::input::BUTTON)'7')) simSpeed = 1000.0f;
        }

        // ---- Save / Load (F5 / F9) ----
        if (wi::input::Press(wi::input::KEYBOARD_BUTTON_F5))
        {
            savesMenuSaveMode = true;
            typingSaveName = false;
            pendingSaveName.clear();
            RefreshSaveList();
            savesMenuOpen = true;
        }
        // ---- Sidewalk debug layer (F8) ----
        if (wi::input::Press(wi::input::KEYBOARD_BUTTON_F8))
        {
            showSidewalkDebug = !showSidewalkDebug;
            swDebugClickActive = false;
        }
        if (wi::input::Press(wi::input::KEYBOARD_BUTTON_F9))
        {
            savesMenuSaveMode = false;
            typingSaveName = false;
            RefreshSaveList();
            savesMenuOpen = !savesMenuOpen;
        }
        if (wi::input::Press(wi::input::KEYBOARD_BUTTON_ESCAPE) && savesMenuOpen)
        {
            savesMenuOpen = false;
            typingSaveName = false;
        }

        // Handle saves menu interaction
        if (savesMenuOpen)
        {
            const float S = uiScale();
            float mx = (float)wi::input::GetPointer().x;
            float my = (float)wi::input::GetPointer().y;
            float menuW = 400.0f*S;
            float menuX = this->width * 0.5f - menuW * 0.5f;
            float menuY = 80.0f*S;
            float entryH = 40.0f*S;

            if (wi::input::Press(wi::input::MOUSE_BUTTON_LEFT))
            {
                // "New Save" button at top (only in save mode)
                if (savesMenuSaveMode)
                {
                    float newBtnY = menuY + 40.0f*S;
                    if (mx >= menuX && mx <= menuX + menuW && my >= newBtnY && my <= newBtnY + entryH)
                    {
                        // Generate auto-name
                        namespace fs = std::filesystem;
                        fs::create_directories("saves");
                        int num = 1;
                        while (fs::exists("saves/save_" + std::to_string(num) + ".city")) num++;
                        std::string autoName = "save_" + std::to_string(num);
                        std::string path = "saves/" + autoName + ".city";
                        if (city.SaveToFile(path.c_str()))
                            wi::backlog::post("[Save] Saved to " + path, wi::backlog::LogLevel::Default);
                        RefreshSaveList();
                        savesMenuOpen = false;
                    }
                }

                // Save entries
                float listStartY = menuY + (savesMenuSaveMode ? 90.0f*S : 40.0f*S);
                int maxShow = 12;
                for (int si = 0; si < (int)saveFiles.size() && si < maxShow; ++si)
                {
                    float ey = listStartY + (float)si * entryH;
                    if (mx >= menuX && mx <= menuX + menuW && my >= ey && my <= ey + entryH)
                    {
                        if (savesMenuSaveMode)
                        {
                            // Overwrite this save
                            std::string path = "saves/" + saveFiles[si] + ".city";
                            if (city.SaveToFile(path.c_str()))
                                wi::backlog::post("[Save] Overwrote " + path, wi::backlog::LogLevel::Default);
                            savesMenuOpen = false;
                        }
                        else
                        {
                            DoLoadSave(saveFiles[si]);
                        }
                        break;
                    }
                }
            }
        }

        // ---- Update traffic density on city grid for traffic-aware pathfinding ----
        {
            constexpr int GS = CityLayout::GRID_SIZE;
            for (int z = 0; z < GS; ++z)
                for (int x = 0; x < GS; ++x)
                    city.SetTrafficCount(x, z, 0);
            for (uint32_t c = 0; c < cars.GetCarCount(); ++c)
            {
                int gx, gz;
                if (city.WorldToGrid(cars.GetPosX(c), cars.GetPosZ(c), gx, gz))
                    city.SetTrafficCount(gx, gz, city.GetTrafficCount(gx, gz) + 1);
            }
        }

        // ---- Simulate cars + traffic lights + pedestrians + render ----
        trafficLights.Update(dt * simSpeed);
        trafficLights.UpdateVisuals();
        auto carView = cars.GetCarView();
        crowd.Update(dt * simSpeed, city, trafficLights, &carView);
        auto pedView = crowd.GetView();
        cars.Update(dt * simSpeed, city, trafficLights, timeOfDay, dayDuration, &pedView);
        townTreasury += cars.DrainTax();
        cars.RenderCars(cam.Eye, city);
        crowd.Render(cam.Eye);

        // ---- Reproduction: houses grow population over time ----
        reproTimer_ += dt * simSpeed;
        if (reproTimer_ >= REPRO_INTERVAL)
        {
            reproTimer_ -= REPRO_INTERVAL;
            constexpr int GS = CityLayout::GRID_SIZE;
            for (int gz = 0; gz < GS; ++gz)
            for (int gx = 0; gx < GS; ++gx)
            {
                if (city.GetCellType(gx, gz) != CellType::HOUSE) continue;
                int pop = city.GetHousePop(gx, gz);
                if (pop < 2) continue; // need at least 2 to reproduce
                if (!city.HasFullServiceCoverage(gx, gz)) continue; // needs police, fire, hospital
                // One new person born (cap at 50 per house)
                if (pop >= 50) continue;
                city.AddHousePop(gx, gz, 1);
                city.UpdateHouseHeight(gx, gz);

                // Spawn the new person with a job
                TrySpawnOneWorker(gx, gz);
            }
        }

        // ---- Bus spawning: periodically add buses on roads ----
        busTimer_ += dt * simSpeed;
        if (busTimer_ >= BUS_INTERVAL)
        {
            busTimer_ -= BUS_INTERVAL;
            // Count current buses
            int busCount = 0;
            for (uint32_t bi = 0; bi < cars.GetCarCount(); ++bi)
                if (cars.IsBus(bi)) ++busCount;
            if (busCount < MAX_BUSES)
            {
                constexpr int GS = CityLayout::GRID_SIZE;
                // Collect road cells
                std::vector<int> roadCells;
                for (int c = 0; c < GS * GS; ++c)
                    if (city.GetCellType(c % GS, c / GS) == CellType::ROAD)
                        roadCells.push_back(c);
                if (roadCells.size() >= 2)
                {
                    // Pick two distinct random road cells
                    busRng_ ^= busRng_ << 13; busRng_ ^= busRng_ >> 17; busRng_ ^= busRng_ << 5;
                    int srcIdx = (int)(busRng_ % (uint32_t)roadCells.size());
                    busRng_ ^= busRng_ << 13; busRng_ ^= busRng_ >> 17; busRng_ ^= busRng_ << 5;
                    int dstIdx = (int)(busRng_ % (uint32_t)roadCells.size());
                    if (srcIdx != dstIdx)
                    {
                        int sx = roadCells[srcIdx] % GS, sz = roadCells[srcIdx] / GS;
                        int dx = roadCells[dstIdx] % GS, dz = roadCells[dstIdx] / GS;
                        auto path = city.FindPathRoad(sx, sz, dx, dz);
                        if ((int)path.size() >= 2)
                            cars.SpawnBus(path, busRng_);
                    }
                }
            }
        }

        // ---- Agent / car picking (LMB click, not in build mode, not RMB look) ----
        if (!buildModeEnabled &&
            wi::input::Press(wi::input::MOUSE_BUTTON_LEFT) &&
            !wi::input::Down(wi::input::MOUSE_BUTTON_RIGHT))
        {
            XMFLOAT4 pointer = wi::input::GetPointer();
            // Create a ray from screen coordinates through the camera
            XMVECTOR rayOrigin, rayDir;
            {
                XMMATRIX VP = cam.GetViewProjection();
                XMMATRIX invVP = XMMatrixInverse(nullptr, VP);
                float screenW = static_cast<float>(cam.width);
                float screenH = static_cast<float>(cam.height);
                float ndcX = (pointer.x / screenW) * 2.0f - 1.0f;
                float ndcY = 1.0f - (pointer.y / screenH) * 2.0f;
                XMVECTOR nearPt = XMVector3TransformCoord(XMVectorSet(ndcX, ndcY, 0.0f, 1.0f), invVP);
                XMVECTOR farPt  = XMVector3TransformCoord(XMVectorSet(ndcX, ndcY, 1.0f, 1.0f), invVP);
                rayOrigin = nearPt;
                rayDir = XMVector3Normalize(farPt - nearPt);
            }

            // Check cars (larger targets, pick first)
            float bestCarDist = 5.0f;
            int32_t bestCarIdx = -1;
            {
                uint32_t carVisCount = cars.GetVisibleCarCount();
                for (uint32_t s = 0; s < carVisCount; ++s)
                {
                    uint32_t ci = cars.GetVisibleCarIndex(s);
                    if (ci == UINT32_MAX) continue;
                    XMVECTOR carPos = XMVectorSet(cars.GetPosX(ci), 0.5f, cars.GetPosZ(ci), 1.0f);
                    XMVECTOR toCar = carPos - rayOrigin;
                    float t = XMVectorGetX(XMVector3Dot(toCar, rayDir));
                    if (t < 0.0f) continue;
                    XMVECTOR closest = rayOrigin + rayDir * t;
                    float dist = XMVectorGetX(XMVector3Length(closest - carPos));
                    if (dist < bestCarDist)
                    {
                        bestCarDist = dist;
                        bestCarIdx = static_cast<int32_t>(ci);
                    }
                }
            }

            // Check agents
            float bestAgentDist = 5.0f;
            int32_t bestAgentIdx = -1;

            // Prefer cars (bigger), then agents, then buildings
            if (bestCarIdx >= 0 && bestCarDist <= bestAgentDist)
            {
                selectedCar = bestCarIdx;
                selectedAgent = -1;
                inspectGX = inspectGZ = -1;
            }
            else if (bestAgentIdx >= 0)
            {
                selectedAgent = bestAgentIdx;
                selectedCar = -1;
                inspectGX = inspectGZ = -1;
            }
            else
            {
                selectedCar = -1;
                selectedAgent = -1;
                // Try ground-plane hit for building inspection
                float rDirY3 = XMVectorGetY(rayDir);
                if (fabsf(rDirY3) > 0.001f) {
                    float t3 = -XMVectorGetY(rayOrigin) / rDirY3;
                    if (t3 > 0.0f) {
                        float hx3 = XMVectorGetX(rayOrigin) + XMVectorGetX(rayDir) * t3;
                        float hz3 = XMVectorGetZ(rayOrigin) + XMVectorGetZ(rayDir) * t3;
                        int gx3, gz3;
                        if (city.WorldToGrid(hx3, hz3, gx3, gz3)) {
                            auto ct3 = city.GetCellType(gx3, gz3);
                            bool isIntersection = (ct3 == CellType::ROAD || ct3 == CellType::CROSSWALK)
                                                  && trafficLights.IsIntersection(gx3, gz3);
                            if (isIntersection || (ct3 != CellType::EMPTY && ct3 != CellType::ROAD && ct3 != CellType::CROSSWALK)) {
                                inspectGX = gx3;
                                inspectGZ = gz3;
                            } else {
                                inspectGX = inspectGZ = -1;
                            }
                        } else {
                            inspectGX = inspectGZ = -1;
                        }
                    }
                }
            }
        }

        // ---- Draw car selection visuals ----
        if (selectedCar >= 0 && static_cast<uint32_t>(selectedCar) < cars.GetCarCount())
        {
            uint32_t ci = static_cast<uint32_t>(selectedCar);
            float cx = cars.GetPosX(ci);
            float cz = cars.GetPosZ(ci);

            // Draw route: lines through remaining waypoints
            uint8_t wpCurr  = cars.GetWpCurr(ci);
            uint8_t wpCount = cars.GetWpCount(ci);
            float prevX = cx, prevZ = cz;
            for (uint8_t w = wpCurr; w < wpCount; ++w)
            {
                XMFLOAT2 wp = cars.GetWaypoint(ci, w);
                wi::renderer::RenderableLine rl;
                rl.start = XMFLOAT3(prevX, 1.5f, prevZ);
                rl.end   = XMFLOAT3(wp.x, 1.5f, wp.y);
                rl.color_start = XMFLOAT4(0.0f, 1.0f, 0.5f, 1.0f);
                rl.color_end   = XMFLOAT4(1.0f, 0.5f, 0.0f, 1.0f);
                wi::renderer::DrawLine(rl);
                prevX = wp.x;
                prevZ = wp.y;
            }

            // Destination marker
            if (wpCount > 0)
            {
                XMFLOAT2 dest = cars.GetWaypoint(ci, wpCount - 1);
                wi::renderer::RenderablePoint dp;
                dp.position = XMFLOAT3(dest.x, 1.5f, dest.y);
                dp.size = 10.0f;
                dp.color = XMFLOAT4(1.0f, 0.5f, 0.0f, 1.0f);
                wi::renderer::DrawPoint(dp);
            }

            // Highlight around car
            wi::renderer::RenderablePoint cp;
            cp.position = XMFLOAT3(cx, 1.0f, cz);
            cp.size = 10.0f;
            cp.color = XMFLOAT4(0.0f, 1.0f, 0.5f, 1.0f);
            wi::renderer::DrawPoint(cp);

            // ---- DEBUG: traffic light awareness ----
            // Compute the car's current segment direction and scan for intersections
            if (wpCurr < wpCount && cars.GetState(ci) == CarSystem::State::DRIVING)
            {
                XMFLOAT2 prev2 = (wpCurr > 0)
                    ? cars.GetWaypoint(ci, wpCurr - 1)
                    : XMFLOAT2{cx, cz};
                XMFLOAT2 tgt   = cars.GetWaypoint(ci, wpCurr);
                float sdx = tgt.x - prev2.x, sdz = tgt.y - prev2.y;
                float slen = std::sqrt(sdx * sdx + sdz * sdz);
                if (slen > 0.01f) { sdx /= slen; sdz /= slen; }

                // Draw segment direction line (cyan arrow, 30m ahead)
                wi::renderer::RenderableLine dirLine;
                dirLine.start = XMFLOAT3(cx, 2.5f, cz);
                dirLine.end   = XMFLOAT3(cx + sdx * 30.f, 2.5f, cz + sdz * 30.f);
                dirLine.color_start = XMFLOAT4(0.f, 1.f, 1.f, 1.f);
                dirLine.color_end   = XMFLOAT4(0.f, 0.5f, 1.f, 0.5f);
                wi::renderer::DrawLine(dirLine);

                // Check if car is currently in an intersection
                int carGX, carGZ;
                bool carOnGrid = city.WorldToGrid(cx, cz, carGX, carGZ);
                bool carInIntersection = carOnGrid && trafficLights.IsIntersection(carGX, carGZ);

                // Mark car's own cell
                if (carOnGrid)
                {
                    XMFLOAT2 cellC = city.GridCellCenter(carGX, carGZ);
                    float hcs = CityLayout::CELL_SIZE * 0.5f;
                    XMFLOAT4 cellCol = carInIntersection
                        ? XMFLOAT4(1.f, 0.f, 1.f, 0.6f)   // magenta = in intersection
                        : XMFLOAT4(0.f, 0.6f, 0.6f, 0.4f); // teal = normal cell
                    wi::renderer::RenderableLine cl;
                    cl.color_start = cl.color_end = cellCol;
                    float ax0 = cellC.x-hcs, ax1 = cellC.x+hcs;
                    float az0 = cellC.y-hcs, az1 = cellC.y+hcs;
                    cl.start={ax0,0.5f,az0}; cl.end={ax1,0.5f,az0}; wi::renderer::DrawLine(cl);
                    cl.start={ax1,0.5f,az0}; cl.end={ax1,0.5f,az1}; wi::renderer::DrawLine(cl);
                    cl.start={ax1,0.5f,az1}; cl.end={ax0,0.5f,az1}; wi::renderer::DrawLine(cl);
                    cl.start={ax0,0.5f,az1}; cl.end={ax0,0.5f,az0}; wi::renderer::DrawLine(cl);
                }

                // Scan cells ahead and mark intersections with light color
                if (carOnGrid && !carInIntersection)
                {
                    constexpr float HALF_CS = CityLayout::CELL_SIZE * 0.5f;
                    int prevSGX = carGX, prevSGZ = carGZ;
                    float segFwd2 = (cx - prev2.x)*sdx + (cz - prev2.y)*sdz;
                    float scanMax2 = std::min(80.f, slen - segFwd2 + 20.f);
                    for (float dd = HALF_CS; dd < scanMax2; dd += HALF_CS)
                    {
                        float sx = cx + sdx * dd;
                        float sz = cz + sdz * dd;
                        int sgx, sgz;
                        if (!city.WorldToGrid(sx, sz, sgx, sgz)) continue;
                        if (sgx == prevSGX && sgz == prevSGZ) continue;
                        prevSGX = sgx; prevSGZ = sgz;

                        if (!trafficLights.IsIntersection(sgx, sgz)) continue;

                        auto lc = trafficLights.GetLight(sgx, sgz, sdx, sdz);
                        XMFLOAT4 lcol;
                        switch (lc)
                        {
                        case TrafficLightSystem::LightColor::GREEN:
                            lcol = {0.f, 1.f, 0.f, 0.8f}; break;
                        case TrafficLightSystem::LightColor::YELLOW:
                            lcol = {1.f, 1.f, 0.f, 0.8f}; break;
                        case TrafficLightSystem::LightColor::RED:
                            lcol = {1.f, 0.f, 0.f, 0.8f}; break;
                        }

                        // Draw thick outline around detected intersection cell
                        XMFLOAT2 ic = city.GridCellCenter(sgx, sgz);
                        float hcs = CityLayout::CELL_SIZE * 0.5f;
                        wi::renderer::RenderableLine il;
                        il.color_start = il.color_end = lcol;
                        float ix0 = ic.x-hcs, ix1 = ic.x+hcs;
                        float iz0 = ic.y-hcs, iz1 = ic.y+hcs;
                        // Bottom
                        il.start={ix0,0.3f,iz0}; il.end={ix1,0.3f,iz0}; wi::renderer::DrawLine(il);
                        il.start={ix1,0.3f,iz0}; il.end={ix1,0.3f,iz1}; wi::renderer::DrawLine(il);
                        il.start={ix1,0.3f,iz1}; il.end={ix0,0.3f,iz1}; wi::renderer::DrawLine(il);
                        il.start={ix0,0.3f,iz1}; il.end={ix0,0.3f,iz0}; wi::renderer::DrawLine(il);
                        // Top
                        il.start={ix0,3.f,iz0}; il.end={ix1,3.f,iz0}; wi::renderer::DrawLine(il);
                        il.start={ix1,3.f,iz0}; il.end={ix1,3.f,iz1}; wi::renderer::DrawLine(il);
                        il.start={ix1,3.f,iz1}; il.end={ix0,3.f,iz1}; wi::renderer::DrawLine(il);
                        il.start={ix0,3.f,iz1}; il.end={ix0,3.f,iz0}; wi::renderer::DrawLine(il);
                        // Verticals
                        il.start={ix0,0.3f,iz0}; il.end={ix0,3.f,iz0}; wi::renderer::DrawLine(il);
                        il.start={ix1,0.3f,iz0}; il.end={ix1,3.f,iz0}; wi::renderer::DrawLine(il);
                        il.start={ix1,0.3f,iz1}; il.end={ix1,3.f,iz1}; wi::renderer::DrawLine(il);
                        il.start={ix0,0.3f,iz1}; il.end={ix0,3.f,iz1}; wi::renderer::DrawLine(il);
                    }
                }
            }
        }

        // ---- Sidewalk debug: click to inspect sub-cell ----
        if (showSidewalkDebug && !buildModeEnabled &&
            wi::input::Press(wi::input::MOUSE_BUTTON_LEFT) &&
            !wi::input::Down(wi::input::MOUSE_BUTTON_RIGHT))
        {
            // Compute ground-plane hit from mouse
            XMFLOAT4 ptr2 = wi::input::GetPointer();
            XMMATRIX invVP2 = XMMatrixInverse(nullptr, cam.GetViewProjection());
            float ndcX2 =  (ptr2.x / static_cast<float>(cam.width))  * 2.0f - 1.0f;
            float ndcY2 = 1.0f - (ptr2.y / static_cast<float>(cam.height)) * 2.0f;
            XMVECTOR nearPt2 = XMVector3TransformCoord(XMVectorSet(ndcX2, ndcY2, 0.f, 1.f), invVP2);
            XMVECTOR farPt2  = XMVector3TransformCoord(XMVectorSet(ndcX2, ndcY2, 1.f, 1.f), invVP2);
            XMVECTOR rDir2   = XMVector3Normalize(farPt2 - nearPt2);
            float rDirY2 = XMVectorGetY(rDir2);
            if (fabsf(rDirY2) > 0.001f) {
                float t2 = -XMVectorGetY(nearPt2) / rDirY2;
                if (t2 > 0.0f) {
                    float hitX = XMVectorGetX(nearPt2) + XMVectorGetX(rDir2) * t2;
                    float hitZ = XMVectorGetZ(nearPt2) + XMVectorGetZ(rDir2) * t2;
                    swDebugClickX = hitX;
                    swDebugClickZ = hitZ;
                    constexpr float HW2 = CityLayout::HALF_WORLD;
                    constexpr float SS2 = CityLayout::SW_SUB_SIZE;
                    swDebugClickSX = (int)std::floor((hitX + HW2) / SS2);
                    swDebugClickSZ = (int)std::floor((hitZ + HW2) / SS2);
                    swDebugClickOnSW = city.IsOnSidewalk(hitX, hitZ);
                    swDebugClickActive = true;
                }
            }
        }

        // ---- Sidewalk debug overlay (F8) ----
        if (showSidewalkDebug)
        {
            constexpr int   SD = CityLayout::SW_DIM;
            constexpr float SS = CityLayout::SW_SUB_SIZE;
            constexpr float HW = CityLayout::HALF_WORLD;
            const float debugY = 0.30f;
            const XMFLOAT4 swColor(0.0f, 0.8f, 1.0f, 0.45f);
            // Only draw near camera to avoid flooding
            XMFLOAT3 camPos;
            XMStoreFloat3(&camPos, wi::scene::GetCamera().GetEye());
            const float drawRadius = 120.0f;
            for (int sz = 0; sz < SD; sz++)
            for (int sx = 0; sx < SD; sx++) {
                if (!city.sidewalkLayer[sz * SD + sx]) continue;
                float cx = -HW + (sx + 0.5f) * SS;
                float cz = -HW + (sz + 0.5f) * SS;
                if (std::abs(cx - camPos.x) > drawRadius || std::abs(cz - camPos.z) > drawRadius) continue;
                float hs = SS * 0.48f;
                wi::renderer::RenderableLine sl;
                sl.color_start = sl.color_end = swColor;
                sl.start={cx-hs,debugY,cz-hs}; sl.end={cx+hs,debugY,cz-hs}; wi::renderer::DrawLine(sl);
                sl.start={cx+hs,debugY,cz-hs}; sl.end={cx+hs,debugY,cz+hs}; wi::renderer::DrawLine(sl);
                sl.start={cx+hs,debugY,cz+hs}; sl.end={cx-hs,debugY,cz+hs}; wi::renderer::DrawLine(sl);
                sl.start={cx-hs,debugY,cz+hs}; sl.end={cx-hs,debugY,cz-hs}; wi::renderer::DrawLine(sl);
            }
        }

        if (dt > 1e-6f)
            displayFPS = displayFPS * 0.9f + (1.0f / dt) * 0.1f;
    }

    // -- Build helpers --
    // Returns true if people were spawned/assigned for this house
    bool TrySpawnFromHouse(int hGX, int hGZ)
    {
        for (int wz = 0; wz < CityLayout::GRID_SIZE; ++wz)
        for (int wx = 0; wx < CityLayout::GRID_SIZE; ++wx)
        {
            if (city.GetCellType(wx, wz) == CityLayout::CellType::WORKPLACE)
            {
                if (!city.WorkplaceHasRoom(wx, wz)) continue;

                auto path = city.FindPath(hGX, hGZ, wx, wz);
                if ((int)path.size() < 2) continue;

                int workers = 0;
                int toSpawn = ((int)path.size() > CAR_HOP_THRESHOLD) ? 3 : 5;
                uint32_t seed = (uint32_t)(hGX * 37 + hGZ * 19 + wx * 7 + wz * 3);
                for (int c = 0; c < toSpawn; ++c) {
                    if (cars.SpawnCar(path, seed + (uint32_t)c) != UINT32_MAX)
                        ++workers;
                }
                if (workers == 0) continue;
                city.AddWorkers(wx, wz, workers);
                city.AddHousePop(hGX, hGZ, workers);

                // Spawn pedestrians (2 per house-workplace pair)
                for (int p = 0; p < 2; ++p)
                    crowd.SpawnPed(hGX, hGZ, wx, wz, city);

                return true;
            }
        }
        return false;
    }

    // Spawn a single new worker from a house (for reproduction)
    void TrySpawnOneWorker(int hGX, int hGZ)
    {
        for (int wz = 0; wz < CityLayout::GRID_SIZE; ++wz)
        for (int wx = 0; wx < CityLayout::GRID_SIZE; ++wx)
        {
            if (city.GetCellType(wx, wz) != CityLayout::CellType::WORKPLACE) continue;
            if (!city.WorkplaceHasRoom(wx, wz)) continue;

            auto path = city.FindPath(hGX, hGZ, wx, wz);
            if ((int)path.size() < 2) continue;

            {
                uint32_t seed = (uint32_t)(hGX * 37 + hGZ * 19 + wx * 7 + wz * 3 + city.GetHousePop(hGX, hGZ));
                if (cars.SpawnCar(path, seed) == UINT32_MAX) continue;
            }
            city.AddWorkers(wx, wz, 1);
            return;
        }
    }

    // Targeted helper: after placing road cell(s), check only the immediate
    // neighbourhood (the placed cells + their 4 neighbours) for newly-formed
    // intersections and add crosswalks on their straight arms.
    // Avoids scanning the whole grid during interactive placement (which can
    // disturb running simulation state and cause crashes).
    void PlaceCrosswalksNearCells(const std::vector<std::pair<int,int>>& cells)
    {
        constexpr int GS = CityLayout::GRID_SIZE;
        const int ddx[] = { 1, -1, 0, 0 };
        const int ddz[] = { 0, 0, 1, -1 };

        auto isIntersectionAt = [&](int gx, int gz) -> bool {
            if (gx < 0 || gx >= GS || gz < 0 || gz >= GS) return false;
            if (city.GetCellType(gx, gz) != CellType::ROAD) return false;
            int rn = 0;
            for (int d = 0; d < 4; ++d)
                if (city.IsRoadLike(gx + ddx[d], gz + ddz[d])) ++rn;
            return rn >= 3;
        };

        bool anyChanged = false;
        for (auto [cx, cz] : cells) {
            EvLog("[PCNC] checking cell gx=" + std::to_string(cx) + " gz=" + std::to_string(cz));
            // Check the placed cell itself, and each of its 4 neighbours,
            // as potential new intersections.
            for (int cand = -1; cand < 4; ++cand) {
                int ix = cx + (cand < 0 ? 0 : ddx[cand]);
                int iz = cz + (cand < 0 ? 0 : ddz[cand]);
                if (!isIntersectionAt(ix, iz)) continue;
                EvLog("[PCNC] found intersection at ix=" + std::to_string(ix) + " iz=" + std::to_string(iz));

                for (int d = 0; d < 4; ++d) {
                    int nx = ix + ddx[d], nz = iz + ddz[d];
                    if (nx < 0 || nx >= GS || nz < 0 || nz >= GS) continue;
                    if (city.GetCellType(nx, nz) != CellType::ROAD) continue;
                    if (isIntersectionAt(nx, nz)) continue;
                    int rn2 = 0;
                    for (int d2 = 0; d2 < 4; ++d2)
                        if (city.IsRoadLike(nx + ddx[d2], nz + ddz[d2])) ++rn2;
                    if (rn2 == 2) {
                        EvLog("[PCNC] PlaceCell CROSSWALK nx=" + std::to_string(nx) + " nz=" + std::to_string(nz));
                        city.PlaceCell(nx, nz, CellType::CROSSWALK);
                        EvLog("[PCNC] PlaceCell CROSSWALK done");
                        anyChanged = true;
                    }
                }
            }
        }
        EvLog("[PCNC] calling RebuildIntersections");
        trafficLights.RebuildIntersections(city);
        EvLog("[PCNC] calling BuildSidewalkLayer");
        city.BuildSidewalkLayer([&](int gx2, int gz2){ return trafficLights.IsIntersection(gx2, gz2); });
        EvLog("[PCNC] done");
    }

    // Automatically place crosswalks on road cells adjacent to intersections.
    // A crosswalk is placed 1 cell away from the intersection on each arm,
    // but only on straight road segments (exactly 2 road-like neighbors).
    // Used during batch load/init only — not for interactive placement.
    void PlaceCrosswalksAroundIntersections()
    {
        constexpr int GS = CityLayout::GRID_SIZE;
        const int ddx[] = { 1, -1, 0, 0 };
        const int ddz[] = { 0, 0, 1, -1 };

        // First pass: find intersections (road cells with 3+ road-like neighbors)
        std::vector<bool> isIntersection(GS * GS, false);
        for (int gz = 0; gz < GS; ++gz)
        for (int gx = 0; gx < GS; ++gx) {
            if (city.GetCellType(gx, gz) != CellType::ROAD) continue;
            int rn = 0;
            for (int d = 0; d < 4; ++d) {
                if (city.IsRoadLike(gx + ddx[d], gz + ddz[d]))
                    ++rn;
            }
            if (rn >= 3) isIntersection[gz * GS + gx] = true;
        }

        // Second pass: for each intersection, place crosswalks 1 cell out on each arm
        for (int gz = 0; gz < GS; ++gz)
        for (int gx = 0; gx < GS; ++gx) {
            if (!isIntersection[gz * GS + gx]) continue;
            for (int d = 0; d < 4; ++d) {
                int nx = gx + ddx[d], nz = gz + ddz[d];
                if (nx < 0 || nx >= GS || nz < 0 || nz >= GS) continue;
                if (city.GetCellType(nx, nz) != CellType::ROAD) continue;
                if (isIntersection[nz * GS + nx]) continue; // skip adjacent intersections
                // Check it's a straight segment (exactly 2 road-like neighbors)
                int rn2 = 0;
                for (int d2 = 0; d2 < 4; ++d2)
                    if (city.IsRoadLike(nx + ddx[d2], nz + ddz[d2]))
                        ++rn2;
                if (rn2 == 2)
                    city.PlaceCell(nx, nz, CellType::CROSSWALK);
            }
        }
        trafficLights.RebuildIntersections(city);
        city.BuildSidewalkLayer([&](int gx, int gz){ return trafficLights.IsIntersection(gx, gz); });
    }

    // Called when roads or a workplace is placed – try all unspawned houses
    void TrySpawnAllUnspawnedHouses()
    {
        for (int gz = 0; gz < CityLayout::GRID_SIZE; ++gz)
        for (int gx = 0; gx < CityLayout::GRID_SIZE; ++gx)
        {
            int idx = gz * CityLayout::GRID_SIZE + gx;
            if (!houseHasSpawned[idx] &&
                city.GetCellType(gx, gz) == CityLayout::CellType::HOUSE)
            {
                if (TrySpawnFromHouse(gx, gz))
                    houseHasSpawned[idx] = true;
            }
        }
    }

    // Compute shortest road path between two grid cells using Dijkstra.
    // Prefers existing road cells (cost 1) over empty cells (cost 3).
    // Blocked by building cells. Returns grid (gx,gz) pairs inclusive.
    static std::vector<std::pair<int,int>> ComputeRoadPreviewPath(
        int x0, int z0, int x1, int z1, const CityLayout& layout)
    {
        constexpr int GS = CityLayout::GRID_SIZE;
        const int ddx[] = { 1,-1, 0, 0 };
        const int ddz[] = { 0, 0, 1,-1 };

        std::vector<int>  dist(GS * GS, INT32_MAX);
        std::vector<int>  from(GS * GS, -1);
        using PQ = std::priority_queue<std::pair<int,int>,
                   std::vector<std::pair<int,int>>,
                   std::greater<std::pair<int,int>>>;
        PQ pq;
        int srcId = z0 * GS + x0;
        int dstId = z1 * GS + x1;
        dist[srcId] = 0;
        from[srcId] = srcId;
        pq.push({0, srcId});
        bool found = (srcId == dstId);

        while (!pq.empty() && !found)
        {
            auto [d, cur] = pq.top(); pq.pop();
            if (d > dist[cur]) continue;
            int cx = cur % GS, cz = cur / GS;
            for (int dir = 0; dir < 4; ++dir)
            {
                int nx = cx + ddx[dir], nz = cz + ddz[dir];
                if (nx < 0 || nx >= GS || nz < 0 || nz >= GS) continue;
                int nid = nz * GS + nx;
                auto ct = layout.GetCellType(nx, nz);
                bool blocked = (ct == CityLayout::CellType::HOUSE  ||
                                ct == CityLayout::CellType::WORKPLACE ||
                                ct == CityLayout::CellType::PARKING);
                if (blocked) continue;
                int cost = (ct == CityLayout::CellType::ROAD ||
                            ct == CityLayout::CellType::CROSSWALK) ? 1 : 3;
                int nd = dist[cur] + cost;
                if (nd < dist[nid])
                {
                    dist[nid] = nd;
                    from[nid] = cur;
                    if (nid == dstId) { found = true; break; }
                    pq.push({nd, nid});
                }
            }
        }

        if (!found) return {};

        std::vector<std::pair<int,int>> path;
        for (int cur = dstId; cur != srcId; cur = from[cur])
            path.push_back({cur % GS, cur / GS});
        path.push_back({x0, z0});
        std::reverse(path.begin(), path.end());
        return path;
    }

    void DrawBuildGrid()
    {
        static constexpr float hw = CityLayout::HALF_WORLD;
        static constexpr float cs = CityLayout::CELL_SIZE;
        static constexpr float Y  = 0.30f;

        wi::renderer::RenderableLine ln;
        ln.color_start = ln.color_end = XMFLOAT4(0.30f, 0.80f, 0.30f, 0.28f);
        for (int i = 0; i <= CityLayout::GRID_SIZE; ++i)
        {
            float w = -hw + i * cs;
            ln.start = XMFLOAT3(w, Y, -hw); ln.end = XMFLOAT3(w, Y,  hw);
            wi::renderer::DrawLine(ln);
            ln.start = XMFLOAT3(-hw, Y, w);  ln.end = XMFLOAT3( hw, Y,  w);
            wi::renderer::DrawLine(ln);
        }
        if (hoverValid)
        {
            XMFLOAT2 c = city.GridCellCenter(hoverGX, hoverGZ);
            float h = cs * 0.5f;
            XMFLOAT4 col;
            switch (placeTool)
            {
            case 0:  col = {0.4f, 0.4f, 1.0f, 1.0f}; break; // road
            case 1:  col = {1.0f, 0.5f, 0.1f, 1.0f}; break; // house
            case 2:  col = {0.2f, 0.5f, 1.0f, 1.0f}; break; // workplace
            case 3:  col = {0.2f, 0.8f, 0.2f, 1.0f}; break; // tree
            case 4:  col = {0.9f, 0.9f, 0.4f, 1.0f}; break; // crosswalk
            case 5:  col = {0.55f, 0.53f, 0.75f, 1.0f}; break; // parking
            default: col = {1.0f, 1.0f, 1.0f, 1.0f}; break;
            }
            wi::renderer::RenderableLine bl;
            bl.color_start = bl.color_end = col;
            float fx0 = c.x-h, fx1 = c.x+h, fz0 = c.y-h, fz1 = c.y+h;

            // --- Road tool: draw semi-transparent preview path ---
            if (placeTool == 0 && roadPlaceActive && !roadPreviewPath.empty())
            {
                // Draw start cell marker (bright)
                XMFLOAT2 sc = city.GridCellCenter(roadStartGX, roadStartGZ);
                wi::renderer::RenderableLine sl;
                sl.color_start = sl.color_end = XMFLOAT4(0.2f, 1.0f, 0.2f, 1.0f);
                float sx0 = sc.x-h, sx1 = sc.x+h, sz0 = sc.y-h, sz1 = sc.y+h;
                sl.start={sx0,Y+.15f,sz0}; sl.end={sx1,Y+.15f,sz0}; wi::renderer::DrawLine(sl);
                sl.start={sx1,Y+.15f,sz0}; sl.end={sx1,Y+.15f,sz1}; wi::renderer::DrawLine(sl);
                sl.start={sx1,Y+.15f,sz1}; sl.end={sx0,Y+.15f,sz1}; wi::renderer::DrawLine(sl);
                sl.start={sx0,Y+.15f,sz1}; sl.end={sx0,Y+.15f,sz0}; wi::renderer::DrawLine(sl);

                // Draw each preview cell as semi-transparent outline
                for (auto& [pgx, pgz] : roadPreviewPath)
                {
                    XMFLOAT2 pc = city.GridCellCenter(pgx, pgz);
                    bool existing = (city.GetCellType(pgx, pgz) == CellType::ROAD ||
                                     city.GetCellType(pgx, pgz) == CellType::CROSSWALK);
                    // Existing road: cyan; new road: semi-transparent blue
                    XMFLOAT4 pcol = existing
                        ? XMFLOAT4(0.2f, 0.9f, 0.9f, 0.65f)
                        : XMFLOAT4(0.35f, 0.45f, 1.0f, 0.55f);
                    wi::renderer::RenderableLine pl;
                    pl.color_start = pl.color_end = pcol;
                    float px0 = pc.x-h, px1 = pc.x+h, pz0 = pc.y-h, pz1 = pc.y+h;
                    pl.start={px0,Y+.12f,pz0}; pl.end={px1,Y+.12f,pz0}; wi::renderer::DrawLine(pl);
                    pl.start={px1,Y+.12f,pz0}; pl.end={px1,Y+.12f,pz1}; wi::renderer::DrawLine(pl);
                    pl.start={px1,Y+.12f,pz1}; pl.end={px0,Y+.12f,pz1}; wi::renderer::DrawLine(pl);
                    pl.start={px0,Y+.12f,pz1}; pl.end={px0,Y+.12f,pz0}; wi::renderer::DrawLine(pl);
                }
            }
            else if (placeTool == 4 && city.GetCellType(hoverGX, hoverGZ) == CellType::ROAD)
            {
                // Crosswalk preview: thin band (1/3 of tile)
                float bandH = h / 3.0f;
                bool hasE = city.IsRoadLike(hoverGX+1, hoverGZ);
                bool hasW = city.IsRoadLike(hoverGX-1, hoverGZ);
                bool hasN = city.IsRoadLike(hoverGX, hoverGZ-1);
                bool hasS = city.IsRoadLike(hoverGX, hoverGZ+1);
                bool goesEW = (hasE || hasW) && !(hasN || hasS);

                float bx0, bx1, bz0, bz1;
                if (goesEW) {
                    bx0 = c.x - bandH; bx1 = c.x + bandH;
                    bz0 = c.y - h;     bz1 = c.y + h;
                } else {
                    bx0 = c.x - h;     bx1 = c.x + h;
                    bz0 = c.y - bandH; bz1 = c.y + bandH;
                }
                bl.start={bx0,Y+.10f,bz0}; bl.end={bx1,Y+.10f,bz0}; wi::renderer::DrawLine(bl);
                bl.start={bx1,Y+.10f,bz0}; bl.end={bx1,Y+.10f,bz1}; wi::renderer::DrawLine(bl);
                bl.start={bx1,Y+.10f,bz1}; bl.end={bx0,Y+.10f,bz1}; wi::renderer::DrawLine(bl);
                bl.start={bx0,Y+.10f,bz1}; bl.end={bx0,Y+.10f,bz0}; wi::renderer::DrawLine(bl);
            }
            else
            {
                // Default: single cell outline at hover
                bl.start={fx0,Y+.05f,fz0}; bl.end={fx1,Y+.05f,fz0}; wi::renderer::DrawLine(bl);
                bl.start={fx1,Y+.05f,fz0}; bl.end={fx1,Y+.05f,fz1}; wi::renderer::DrawLine(bl);
                bl.start={fx1,Y+.05f,fz1}; bl.end={fx0,Y+.05f,fz1}; wi::renderer::DrawLine(bl);
                bl.start={fx0,Y+.05f,fz1}; bl.end={fx0,Y+.05f,fz0}; wi::renderer::DrawLine(bl);
            }
        }
    }

    void Compose(wi::graphics::CommandList cmd) const override
    {
        wi::RenderPath3D::Compose(cmd);

        const uint32_t threads  = wi::jobsystem::GetThreadCount();

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1)
            << "FPS          : " << displayFPS << "\n"
            << "CPU Threads  : " << threads    << "\n"
            << "Car drivers  : " << cars.GetCarCount() << "\n"
            << "Pedestrians  : " << crowd.GetPedCount()
               << " (walk:" << crowd.GetWalkingCount() << " wait:" << crowd.GetWaitingCount() << ")\n"
            << "Treasury  $  : " << static_cast<int>(townTreasury) << "\n"
            << "Time         : " << std::setfill('0') << std::setw(2) << (int)(timeOfDay * 24.0f) << ":"
            << std::setw(2) << (int)(std::fmod(timeOfDay * 24.0f * 60.0f, 60.0f)) << std::setfill(' ') << "\n"
            << "\n"
            << "[WASD]        Move\n"
            << "[RMB + Mouse] Look around\n"
            << "[Q / E]       Down / Up\n"
            << "[Shift]       Sprint\n"
            << "[Scroll]      Speed: " << static_cast<int>(moveSpeed) << " u/s\n"
            << "[B]           Build Mode: " << (buildModeEnabled ? "ON" : "OFF") << "\n"
            << "[F5]          Save  |  [F9] Load / Saves Menu\n"
            << "[F8]          Sidewalk Debug: " << (showSidewalkDebug ? "ON" : "OFF");
        if (showSidewalkDebug && swDebugClickActive)
        {
            oss << "\n  Click: world(" << std::fixed << std::setprecision(1)
                << swDebugClickX << ", " << swDebugClickZ << ") sub("
                << swDebugClickSX << "," << swDebugClickSZ << ") "
                << (swDebugClickOnSW ? "SIDEWALK" : "NOT sidewalk");
        }
        if (buildModeEnabled)
        {
            if (placeTool == 0)
            {
                if (roadPlaceActive)
                    oss << "\n[LMB]         Click END point  |  [RMB] Cancel";
                else
                    oss << "\n[LMB]         Click START point";
            }
            else
                oss << "\n[LMB drag]    Place selected tile";
            oss << "\n[1-7]         Sim speed: " << simSpeed << "x";

            // ---- Build panel (right side) ----
            const float S = uiScale();
            const float px   = (float)this->width - 215.0f*S;
            const float btnH = 46.0f*S, btnW = 205.0f*S;
            const float btnStart = 145.0f*S;
            const float btnStep  = 55.0f*S;
            struct BtnDef { const char* label; XMFLOAT4 col; };
            BtnDef btns[12] = {
                { "  Road",         {0.40f,0.40f,1.00f,1.0f} },
                { "  House",        {1.00f,0.50f,0.10f,1.0f} },
                { "  Workplace",    {0.20f,0.55f,1.00f,1.0f} },
                { "  Tree",         {0.20f,0.80f,0.20f,1.0f} },
                { "  Crosswalk",    {0.90f,0.90f,0.40f,1.0f} },
                { "  Parking",      {0.55f,0.53f,0.75f,1.0f} },
                { "  Lake",         {0.10f,0.45f,0.85f,1.0f} },
                { "  Power Plant",  {0.85f,0.65f,0.10f,1.0f} },
                { "  Water Pump",   {0.15f,0.60f,0.80f,1.0f} },
                { "  Police",       {0.15f,0.20f,0.55f,1.0f} },
                { "  Fire Station", {0.75f,0.15f,0.10f,1.0f} },
                { "  Hospital",     {0.90f,0.90f,0.95f,1.0f} },
            };

            // Panel title
            wi::font::Params title;
            title.posX  = px;
            title.posY  = 108.0f*S;
            title.size  = (int)(22*S);
            title.color = wi::Color(220, 220, 220, 230);
            wi::font::Draw("BUILD TOOLS", title, cmd);

            for (int k = 0; k < 12; ++k)
            {
                bool sel = (placeTool == k);
                float by = btnStart + k * btnStep;

                // Button background
                wi::image::Params bg;
                bg.pos  = XMFLOAT3(px - 5.0f*S, by, 0.0f);
                bg.siz  = XMFLOAT2(btnW, btnH);
                bg.color = sel
                    ? XMFLOAT4(btns[k].col.x*0.6f, btns[k].col.y*0.6f, btns[k].col.z*0.6f, 0.90f)
                    : XMFLOAT4(0.08f, 0.08f, 0.12f, 0.75f);
                wi::image::Draw(nullptr, bg, cmd);

                // Selection bar on left edge
                if (sel)
                {
                    wi::image::Params bar;
                    bar.pos  = XMFLOAT3(px - 5.0f*S, by, 0.0f);
                    bar.siz  = XMFLOAT2(5.0f*S, btnH);
                    bar.color = btns[k].col;
                    wi::image::Draw(nullptr, bar, cmd);
                }

                // Label
                wi::font::Params fp;
                fp.posX  = px + 12.0f*S;
                fp.posY  = by + 13.0f*S;
                fp.size  = (int)(26*S);
                fp.color = sel
                    ? wi::Color(255, 240, 80,  255)
                    : wi::Color(
                        static_cast<uint8_t>(btns[k].col.x * 220 + 35),
                        static_cast<uint8_t>(btns[k].col.y * 220 + 35),
                        static_cast<uint8_t>(btns[k].col.z * 220 + 35),
                        200);
                wi::font::Draw(btns[k].label, fp, cmd);
            }

            // Hover cell info below buttons
            wi::font::Params hfp;
            hfp.posX  = px;
            hfp.posY  = btnStart + 12 * btnStep + 10.0f*S;
            hfp.size  = (int)(18*S);
            hfp.color = wi::Color(180, 180, 180, 180);
            if (hoverValid)
            {
                std::ostringstream hov;
                hov << "Cell (" << hoverGX << ", " << hoverGZ << ")";
                auto hct = city.GetCellType(hoverGX, hoverGZ);
                if (hct == CellType::ROAD || hct == CellType::CROSSWALK)
                    hov << "  [" << city.GetRoadLanes(hoverGX, hoverGZ) << "-lane]  MMB cycle";
                wi::font::Draw(hov.str(), hfp, cmd);
            }
            else
            {
                wi::font::Draw("Hover over world", hfp, cmd);
            }
        }
        else
        {
            oss << "\n[1-6]         Sim speed: " << simSpeed << "x";
        }

        // ---- Shadow cascade slider panel (bottom-left, always visible) ----
        {
            const float S = uiScale();
            const float panX = 10.f*S, panW = 215.f*S, panH = 105.f*S;
            const float panY = (float)height - panH - 10.f*S;
            const float sliderX = panX + 5.f*S, sliderW = 185.f*S, trackH = 12.f*S;
            const float track1Y = panY + 30.f*S;
            const float track2Y = panY + 67.f*S;

            // Panel background
            wi::image::Params bg;
            bg.pos   = XMFLOAT3(panX, panY, 0.f);
            bg.siz   = XMFLOAT2(panW, panH);
            bg.color = XMFLOAT4(0.05f, 0.05f, 0.10f, 0.80f);
            wi::image::Draw(nullptr, bg, cmd);

            // Title
            wi::font::Params tp;
            tp.posX  = panX + 6.f*S; tp.posY = panY + 6.f*S;
            tp.size  = (int)(16*S); tp.color = wi::Color(160, 180, 255, 220);
            wi::font::Draw("Shadow Cascades", tp, cmd);

            auto drawSlider = [&](float trackY, float val, float minV, float maxV,
                                  const char* label, wi::Color fillCol) {
                float t = std::clamp((val - minV) / (maxV - minV), 0.f, 1.f);

                // Track background
                wi::image::Params bg2;
                bg2.pos   = XMFLOAT3(sliderX, trackY, 0.f);
                bg2.siz   = XMFLOAT2(sliderW, trackH);
                bg2.color = XMFLOAT4(0.18f, 0.18f, 0.22f, 1.f);
                wi::image::Draw(nullptr, bg2, cmd);

                // Filled portion
                wi::image::Params fill;
                fill.pos   = XMFLOAT3(sliderX, trackY, 0.f);
                fill.siz   = XMFLOAT2(sliderW * t, trackH);
                fill.color = XMFLOAT4(fillCol.getR()/255.f, fillCol.getG()/255.f,
                                      fillCol.getB()/255.f, 0.85f);
                wi::image::Draw(nullptr, fill, cmd);

                // Thumb
                wi::image::Params thumb;
                thumb.pos   = XMFLOAT3(sliderX + sliderW * t - 5.f*S, trackY - 3.f*S, 0.f);
                thumb.siz   = XMFLOAT2(10.f*S, trackH + 6.f*S);
                thumb.color = XMFLOAT4(1.f, 1.f, 1.f, 0.95f);
                wi::image::Draw(nullptr, thumb, cmd);

                // Label
                std::ostringstream los;
                los << label << ": " << std::fixed << std::setprecision(0) << val << " m";
                wi::font::Params lp;
                lp.posX  = sliderX; lp.posY = trackY + trackH + 2.f*S;
                lp.size  = (int)(14*S); lp.color = wi::Color(190, 190, 190, 200);
                wi::font::Draw(los.str(), lp, cmd);
            };

            drawSlider(track1Y, cascadeDist1_,   5.f, 150.f, "Near", wi::Color(120, 160, 255, 255));
            drawSlider(track2Y, cascadeDist2_,  30.f, 800.f, "Far",  wi::Color( 80, 210, 200, 255));
        }

        // Selected entity info panel
        if (selectedCar >= 0 && static_cast<uint32_t>(selectedCar) < cars.GetCarCount())
        {
            uint32_t ci = static_cast<uint32_t>(selectedCar);
            auto st = cars.GetState(ci);
            float spd = cars.GetSpeed(ci);
            uint8_t wpCurr  = cars.GetWpCurr(ci);
            uint8_t wpCount = cars.GetWpCount(ci);

            std::ostringstream sel;
            sel << std::fixed << std::setprecision(1)
                << "\n\n┌────── CAR " << ci << " ──────┐\n"
                << "| State:    " << CarSystem::StateStr(st) << "\n"
                << "| Dir:      " << (cars.GetCarDir(ci) == 0 ? "To work" : "To home") << "\n"
                << "| Speed:    " << spd << " m/s (" << std::setprecision(0) << (spd * 3.6f) << " km/h)\n"
                << "| Waypoint: " << (int)wpCurr << "/" << (int)wpCount << "\n"
                << "| Lane:     " << std::setprecision(1) << cars.GetLaneOff(ci) << "\n"
                << "| Savings:  $" << std::setprecision(0) << cars.GetMoney(ci) << "\n";

            // Debug: show traffic light status for next intersection
            if (st == CarSystem::State::DRIVING && wpCurr < wpCount)
            {
                float px = cars.GetPosX(ci), pz = cars.GetPosZ(ci);
                XMFLOAT2 prev2 = (wpCurr > 0)
                    ? cars.GetWaypoint(ci, wpCurr - 1)
                    : XMFLOAT2{px, pz};
                XMFLOAT2 tgt = cars.GetWaypoint(ci, wpCurr);
                float sdx2 = tgt.x - prev2.x, sdz2 = tgt.y - prev2.y;
                float slen2 = std::sqrt(sdx2*sdx2 + sdz2*sdz2);
                if (slen2 > 0.01f) { sdx2 /= slen2; sdz2 /= slen2; }

                int cGX, cGZ;
                bool cGrid = city.WorldToGrid(px, pz, cGX, cGZ);
                bool cInInt = cGrid && trafficLights.IsIntersection(cGX, cGZ);

                sel << "| Cell:     (" << (cGrid ? cGX : -1) << "," << (cGrid ? cGZ : -1) << ")"
                    << (cInInt ? " [IN ISEC]" : "") << "\n";
                sel << "| SegDir:   (" << std::setprecision(2) << sdx2 << "," << sdz2 << ")\n";

                // Report nearest intersection ahead
                if (cGrid && !cInInt)
                {
                    constexpr float HCS = CityLayout::CELL_SIZE * 0.5f;
                    int psgx = cGX, psgz = cGZ;
                    float sf = (px - prev2.x)*sdx2 + (pz - prev2.y)*sdz2;
                    float sm = std::min(80.f, slen2 - sf + 20.f);
                    for (float dd2 = HCS; dd2 < sm; dd2 += HCS)
                    {
                        int sx2, sz2;
                        if (!city.WorldToGrid(px+sdx2*dd2, pz+sdz2*dd2, sx2, sz2)) continue;
                        if (sx2 == psgx && sz2 == psgz) continue;
                        psgx = sx2; psgz = sz2;
                        if (!trafficLights.IsIntersection(sx2, sz2)) continue;
                        auto lc2 = trafficLights.GetLight(sx2, sz2, sdx2, sdz2);
                        const char* lcStr = (lc2 == TrafficLightSystem::LightColor::GREEN) ? "GREEN"
                                          : (lc2 == TrafficLightSystem::LightColor::YELLOW) ? "YELLOW"
                                          : "RED";
                        sel << "| Light@(" << sx2 << "," << sz2 << "): "
                            << lcStr << " d=" << std::setprecision(0) << dd2 << "m\n";
                        break; // show nearest only
                    }
                }
                else if (cInInt)
                {
                    sel << "| (inside intersection)\n";
                }
                else
                {
                    sel << "| (off-grid)\n";
                }
            }
            sel << "└──────────────────┘";
            oss << sel.str();
        }
        else if (inspectGX >= 0 && inspectGZ >= 0)
        {
            auto ict = city.GetCellType(inspectGX, inspectGZ);
            std::ostringstream ip;
            ip << std::fixed << std::setprecision(0);
            ip << "\n\n┌── INSPECT (" << inspectGX << "," << inspectGZ << ") ──┐\n";
            switch (ict) {
            case CellType::HOUSE:
                ip << "| Type:     House\n"
                   << "| Pop:      " << city.GetHousePop(inspectGX, inspectGZ) << "\n"
                   << "| Services: " << (city.HasFullServiceCoverage(inspectGX, inspectGZ) ? "OK" : "MISSING") << "\n"
                   << "| Power:    " << CityLayout::POWER_PER_HOUSE << " needed\n"
                   << "| Water:    " << CityLayout::WATER_PER_HOUSE << " needed\n";
                break;
            case CellType::WORKPLACE:
                ip << "| Type:     Workplace\n"
                   << "| Workers:  " << city.GetWorkCount(inspectGX, inspectGZ)
                   << "/" << city.GetWorkCap(inspectGX, inspectGZ) << "\n"
                   << "| Power:    " << CityLayout::POWER_PER_WORKPLACE << " needed\n"
                   << "| Water:    " << CityLayout::WATER_PER_WORKPLACE << " needed\n";
                break;
            case CellType::PARKING:
                ip << "| Type:     Parking\n"
                   << "| Cars:     " << city.GetParkOcc(inspectGX, inspectGZ)
                   << "/" << city.GetParkCap(inspectGX, inspectGZ) << "\n";
                break;
            case CellType::LAKE:
                ip << "| Type:     Lake\n";
                break;
            case CellType::POWER_PLANT:
                ip << "| Type:     Power Plant\n"
                   << "| Output:   " << CityLayout::POWER_PER_PLANT << " power\n";
                break;
            case CellType::WATER_PUMP:
                ip << "| Type:     Water Pump\n"
                   << "| Output:   " << CityLayout::WATER_PER_PUMP << " water\n";
                break;
            case CellType::POLICE:
                ip << "| Type:     Police Station\n"
                   << "| Range:    " << CityLayout::SERVICE_RADIUS << " cells\n"
                   << "| Power:    " << CityLayout::POWER_PER_SERVICE << " needed\n"
                   << "| Water:    " << CityLayout::WATER_PER_SERVICE << " needed\n";
                break;
            case CellType::FIRE_STATION:
                ip << "| Type:     Fire Station\n"
                   << "| Range:    " << CityLayout::SERVICE_RADIUS << " cells\n"
                   << "| Power:    " << CityLayout::POWER_PER_SERVICE << " needed\n"
                   << "| Water:    " << CityLayout::WATER_PER_SERVICE << " needed\n";
                break;
            case CellType::HOSPITAL:
                ip << "| Type:     Hospital\n"
                   << "| Range:    " << CityLayout::SERVICE_RADIUS << " cells\n"
                   << "| Power:    " << CityLayout::POWER_PER_SERVICE << " needed\n"
                   << "| Water:    " << CityLayout::WATER_PER_SERVICE << " needed\n";
                break;
            case CellType::ROAD:
                ip << "| Type:     Intersection\n"
                   << "| Lanes:    " << city.GetRoadLanes(inspectGX, inspectGZ) << "\n";
                break;
            case CellType::CROSSWALK:
                ip << "| Type:     Crosswalk\n";
                break;
            default:
                ip << "| Type:     " << (int)ict << "\n";
                break;
            }

            // ── Intersection info (if this cell is a traffic-light intersection) ──
            if (trafficLights.IsIntersection(inspectGX, inspectGZ))
            {
                using TLS = TrafficLightSystem;
                using TI  = TLS::TurnIntent;
                auto ph = trafficLights.GetPhase(inspectGX, inspectGZ);
                float timer = trafficLights.GetPhaseTimer(inspectGX, inspectGZ);
                float dur   = trafficLights.GetPhaseTotalDuration(inspectGX, inspectGZ);
                ip << "|\n"
                   << "| Phase: " << TLS::PhaseName(ph)
                   << " (" << std::setprecision(1) << timer << "/" << dur << "s)\n";

                // Show per-direction light state (N/E/S/W approach)
                auto lStr = [&](float dx, float dz, TI ti) {
                    return TLS::LightStr(trafficLights.GetLight(inspectGX, inspectGZ, dx, dz, ti));
                };
                ip << "|\n"
                   << "| From N: S=" << lStr(0,1,TI::STRAIGHT)
                   << " R=" << lStr(0,1,TI::RIGHT)
                   << " L=" << lStr(0,1,TI::LEFT) << "\n"
                   << "| From E: S=" << lStr(-1,0,TI::STRAIGHT)
                   << " R=" << lStr(-1,0,TI::RIGHT)
                   << " L=" << lStr(-1,0,TI::LEFT) << "\n"
                   << "| From S: S=" << lStr(0,-1,TI::STRAIGHT)
                   << " R=" << lStr(0,-1,TI::RIGHT)
                   << " L=" << lStr(0,-1,TI::LEFT) << "\n"
                   << "| From W: S=" << lStr(1,0,TI::STRAIGHT)
                   << " R=" << lStr(1,0,TI::RIGHT)
                   << " L=" << lStr(1,0,TI::LEFT) << "\n";
            }

            ip << "└──────────────────────┘";
            oss << ip.str();
        }
        else
        {
            oss << "\n\n[LMB] Click anything to inspect";
        }

        {
            const float S = uiScale();
            wi::font::Params shadow;
            shadow.posX  = 12.5f*S;  shadow.posY  = 12.5f*S;
            shadow.size  = (int)(22*S);
            shadow.color = wi::Color(0, 0, 0, 180);
            wi::font::Draw(oss.str(), shadow, cmd);
        }
        {
            const float S = uiScale();
            wi::font::Params fp;
            fp.posX  = 10.0f*S;  fp.posY  = 10.0f*S;
            fp.size  = (int)(22*S);
            fp.color = wi::Color(240, 240, 60, 255);
            wi::font::Draw(oss.str(), fp, cmd);
        }

        // ---- Utility supply/demand HUD (top-right, always visible) ----
        {
            int pSup, pDem, wSup, wDem;
            city.ComputeUtilities(pSup, pDem, wSup, wDem);

            const float S = uiScale();
            float uhX = (float)this->width - 260.0f*S;
            float uhY = 10.0f*S;

            // Background panel
            wi::image::Params ubg;
            ubg.pos   = XMFLOAT3(uhX - 5.f*S, uhY, 0.f);
            ubg.siz   = XMFLOAT2(255.f*S, 60.f*S);
            ubg.color = XMFLOAT4(0.05f, 0.05f, 0.10f, 0.80f);
            wi::image::Draw(nullptr, ubg, cmd);

            // Power line
            {
                std::ostringstream ps;
                ps << "Power: " << pSup << " / " << pDem;
                wi::font::Params pfp;
                pfp.posX = uhX; pfp.posY = uhY + 6.f*S;
                pfp.size = (int)(20*S);
                pfp.color = (pSup >= pDem) ? wi::Color(80, 255, 80, 240) : wi::Color(255, 80, 80, 240);
                wi::font::Draw(ps.str(), pfp, cmd);
            }
            // Water line
            {
                std::ostringstream ws;
                ws << "Water: " << wSup << " / " << wDem;
                wi::font::Params wfp;
                wfp.posX = uhX; wfp.posY = uhY + 32.f*S;
                wfp.size = (int)(20*S);
                wfp.color = (wSup >= wDem) ? wi::Color(80, 200, 255, 240) : wi::Color(255, 80, 80, 240);
                wi::font::Draw(ws.str(), wfp, cmd);
            }
        }

        // ---- Saves Menu Overlay ----
        if (savesMenuOpen)
        {
            const float S = uiScale();
            float menuW = 400.0f*S;
            float menuX = (float)this->width * 0.5f - menuW * 0.5f;
            float menuY = 80.0f*S;
            float entryH = 40.0f*S;
            int maxShow = 12;

            // Dim background
            {
                wi::image::Params dim;
                dim.pos  = XMFLOAT3(0, 0, 0);
                dim.siz  = XMFLOAT2((float)this->width, (float)this->height);
                dim.color = XMFLOAT4(0, 0, 0, 0.5f);
                wi::image::Draw(nullptr, dim, cmd);
            }

            // Title
            {
                wi::font::Params tp;
                tp.posX = menuX; tp.posY = menuY;
                tp.size = (int)(28*S);
                tp.color = wi::Color(255, 255, 255, 255);
                wi::font::Draw(savesMenuSaveMode ? "SAVE GAME" : "LOAD GAME", tp, cmd);
            }

            float listStartY = menuY + 40.0f*S;

            // "New Save" button (save mode only)
            if (savesMenuSaveMode)
            {
                wi::image::Params bg;
                bg.pos  = XMFLOAT3(menuX, listStartY, 0);
                bg.siz  = XMFLOAT2(menuW, entryH);
                bg.color = XMFLOAT4(0.15f, 0.55f, 0.15f, 0.85f);
                wi::image::Draw(nullptr, bg, cmd);

                wi::font::Params fp2;
                fp2.posX = menuX + 12.0f*S; fp2.posY = listStartY + 8.0f*S;
                fp2.size = (int)(22*S);
                fp2.color = wi::Color(255, 255, 255, 255);
                wi::font::Draw("+ New Save", fp2, cmd);
                listStartY += entryH + 10.0f*S;
            }

            // List existing saves
            for (int si = 0; si < (int)saveFiles.size() && si < maxShow; ++si)
            {
                float ey = listStartY + (float)si * entryH;
                bool hover = false;
                float mx2 = (float)wi::input::GetPointer().x;
                float my2 = (float)wi::input::GetPointer().y;
                if (mx2 >= menuX && mx2 <= menuX + menuW && my2 >= ey && my2 <= ey + entryH)
                    hover = true;

                wi::image::Params bg;
                bg.pos = XMFLOAT3(menuX, ey, 0);
                bg.siz = XMFLOAT2(menuW, entryH - 2.0f*S);
                bg.color = hover ? XMFLOAT4(0.3f, 0.3f, 0.6f, 0.9f) : XMFLOAT4(0.1f, 0.1f, 0.15f, 0.85f);
                wi::image::Draw(nullptr, bg, cmd);

                wi::font::Params fp2;
                fp2.posX = menuX + 12.0f*S; fp2.posY = ey + 8.0f*S;
                fp2.size = (int)(22*S);
                fp2.color = hover ? wi::Color(255, 255, 80, 255) : wi::Color(220, 220, 220, 230);
                wi::font::Draw(saveFiles[si], fp2, cmd);

                if (savesMenuSaveMode) {
                    wi::font::Params ov;
                    ov.posX = menuX + menuW - 100.0f*S; ov.posY = ey + 8.0f*S;
                    ov.size = (int)(18*S);
                    ov.color = wi::Color(255, 120, 120, 200);
                    wi::font::Draw("overwrite", ov, cmd);
                }
            }

            // Footer
            {
                float footY = listStartY + (float)std::min((int)saveFiles.size(), maxShow) * entryH + 10.0f*S;
                wi::font::Params fp2;
                fp2.posX = menuX; fp2.posY = footY;
                fp2.size = (int)(18*S);
                fp2.color = wi::Color(180, 180, 180, 180);
                wi::font::Draw("[ESC] Close", fp2, cmd);
            }
        }
    }

};


// ============================================================
//  CrowdApp
// ============================================================
class CrowdApp : public wi::Application
{
    CrowdRenderPath renderPath;

public:
    void Initialize() override
    {
        wi::Application::Initialize();
        setFrameRateLock(false);
        wi::eventhandler::SetVSync(false);  // disable VSync for uncapped FPS
        ActivatePath(&renderPath);
    }
};

// ============================================================
//  Win32 plumbing
// ============================================================
static CrowdApp g_app;

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    // Resize / DPI: re-supply the window so WickedEngine rebuilds the swapchain
    case WM_SIZE:
    case WM_DPICHANGED:
        if (g_app.is_window_active)
            g_app.SetWindow(hWnd);
        break;

    // Text input forwarding (text fields in WickedEngine GUI)
    case WM_CHAR:
        switch (wParam)
        {
        case VK_BACK:
            wi::gui::TextInputField::DeleteFromInput();
            break;
        default:
        {
            const wchar_t c = static_cast<wchar_t>(wParam);
            wi::gui::TextInputField::AddInput(c);
        }
        break;
        }
        break;

    // Raw input: required for wi::input key/mouse polling
    case WM_INPUT:
        wi::input::rawinput::ParseMessage(reinterpret_cast<void*>(lParam));
        break;

    case WM_SETFOCUS:
        g_app.is_window_active = true;
        break;
    case WM_KILLFOCUS:
        g_app.is_window_active = false;
        wi::input::HidePointer(false);  // show cursor when focus lost
        break;

    case WM_DESTROY:
        PostQuitMessage(0);
        break;

    default:
        return DefWindowProc(hWnd, msg, wParam, lParam);
    }
    return 0;
}

int WINAPI wWinMain(
    HINSTANCE hInstance,
    HINSTANCE /*hPrevInstance*/,
    LPWSTR    lpCmdLine,
    int       nCmdShow)
{
    SetUnhandledExceptionFilter(UnhandledExFilter);
    wi::arguments::Parse(lpCmdLine);

    // Opt in to per-monitor DPI scaling (matches the template)
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);

    // ---- Register window class ----
    WNDCLASSEXW wc{};
    wc.cbSize        = sizeof(wc);
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = hInstance;
    wc.hIcon         = LoadIcon(nullptr, IDI_APPLICATION);
    wc.hCursor       = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOWFRAME);
    wc.lpszClassName = L"CrowdSimClass";
    RegisterClassExW(&wc);

    // ---- Create window (80% of primary monitor work area) ----
    RECT workArea;
    SystemParametersInfoW(SPI_GETWORKAREA, 0, &workArea, 0);
    int monW = workArea.right  - workArea.left;
    int monH = workArea.bottom - workArea.top;
    int winW = (int)(monW * 0.8);
    int winH = (int)(monH * 0.8);
    RECT rc{ 0, 0, winW, winH };
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);

    HWND hWnd = CreateWindowExW(
        0,
        L"CrowdSimClass",
        L"Crowd Simulation  \u2013  Wicked Engine  (100k Agents | C++20)",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rc.right  - rc.left,
        rc.bottom - rc.top,
        nullptr, nullptr,
        hInstance, nullptr
    );

    if (!hWnd)
    {
        MessageBoxW(nullptr, L"Failed to create window.", L"Error", MB_ICONERROR);
        return -1;
    }

    // SetWindow creates the swap-chain; call it before Show so the device exists.
    // Do NOT call Initialize() here – let Run() call it on the first iteration
    // AFTER ShowWindow has triggered WM_SIZE with the final window dimensions.
    g_app.SetWindow(hWnd);

    ShowWindow(hWnd, nCmdShow);
    UpdateWindow(hWnd);

    // ---- Message / render loop ----
    MSG msg{};
    while (msg.message != WM_QUIT)
    {
        if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        else
        {
            g_app.Run();
        }
    }

    wi::jobsystem::ShutDown();  // wait for outstanding jobs before DLL unload
    return static_cast<int>(msg.wParam);
}
