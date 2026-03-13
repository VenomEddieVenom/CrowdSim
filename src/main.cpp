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

    // Agent / car selection
    int32_t selectedAgent = -1;  // agent index, or -1 for none
    int32_t selectedCar   = -1;  // car index, or -1 for none

    // Shadow cascade sliders
    float cascadeDist1_     = 30.0f;    // near cascade distance (m)
    float cascadeDist2_     = 200.0f;   // far cascade distance (m)
    bool  draggingCascade1_ = false;
    bool  draggingCascade2_ = false;

    // Build mode
    using CellType = CityLayout::CellType;
    bool     buildModeEnabled = false;
    int      placeTool        = 0;    // 0=road 1=house 2=workplace 3=tree 4=crosswalk 5=parking
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
        for (int pass = 0; pass < 2; ++pass)
        for (int gz2 = 0; gz2 < CityLayout::GRID_SIZE; ++gz2)
        for (int gx2 = 0; gx2 < CityLayout::GRID_SIZE; ++gx2) {
            auto ct = sd.cells[gz2 * CityLayout::GRID_SIZE + gx2];
            bool isRoad = (ct == CellType::ROAD || ct == CellType::CROSSWALK);
            if (pass == 0 && !isRoad) continue;
            if (pass == 1 && isRoad) continue;
            if (ct == CellType::EMPTY) continue;
            city.PlaceCell(gx2, gz2, ct);
        }
        trafficLights.RebuildIntersections(city);
        PlaceCrosswalksAroundIntersections();
        city.BuildSidewalkLayer([&](int gx, int gz){ return trafficLights.IsIntersection(gx, gz); });
        for (int gz2 = 0; gz2 < CityLayout::GRID_SIZE; ++gz2)
        for (int gx2 = 0; gx2 < CityLayout::GRID_SIZE; ++gx2) {
            int lanes = sd.roadLanes[gz2 * CityLayout::GRID_SIZE + gx2];
            if (lanes == 2 || lanes == 6)
                city.SetRoadLanes(gx2, gz2, lanes);
        }
        for (int gz2 = 0; gz2 < CityLayout::GRID_SIZE; ++gz2)
        for (int gx2 = 0; gx2 < CityLayout::GRID_SIZE; ++gx2) {
            int idx2 = gz2 * CityLayout::GRID_SIZE + gx2;
            if (sd.cells[idx2] == CellType::HOUSE) {
                city.SetHousePop(gx2, gz2, 0);
                if (TrySpawnFromHouse(gx2, gz2))
                    houseHasSpawned[idx2] = true;
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
            XMFLOAT4 ptr  = wi::input::GetPointer();
            bool lmbDown  = wi::input::Down(wi::input::MOUSE_BUTTON_LEFT);
            bool lmbPress = wi::input::Press(wi::input::MOUSE_BUTTON_LEFT);

            const float sliderX = 15.f, sliderW = 185.f;
            const float track1Y = (float)cam.height - 85.f;
            const float track2Y = (float)cam.height - 48.f;
            const float trackH  = 12.f, hitExtra = 8.f;

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
            // Panel click detection (right side of screen, 6 tool buttons)
            const float panelX  = (float)cam.width - 215.0f;
            const float btnY[6] = { 145.0f, 215.0f, 285.0f, 355.0f, 425.0f, 495.0f };
            const float btnH    = 60.0f;

            XMFLOAT4 ptr = wi::input::GetPointer();
            bool inPanel = ptr.x >= panelX;
            clickConsumedByPanel = false;

            if (inPanel && wi::input::Press(wi::input::MOUSE_BUTTON_LEFT))
            {
                clickConsumedByPanel = true;
                for (int k = 0; k < 6; ++k)
                    if (ptr.y >= btnY[k] && ptr.y < btnY[k] + btnH)
                        placeTool = k; // 0=road 1=house 2=workplace 3=tree 4=crosswalk 5=parking
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
                        // Place start cell immediately
                        city.PlaceCell(hoverGX, hoverGZ, CellType::ROAD);
                        trafficLights.RebuildIntersections(city);
                        city.BuildSidewalkLayer([&](int gx2, int gz2){ return trafficLights.IsIntersection(gx2, gz2); });
                        TrySpawnAllUnspawnedHouses();
                        roadPreviewPath.clear();
                    }
                    else
                    {
                        // Second click: place entire preview path
                        bool anyPlaced = false;
                        for (auto& [pgx, pgz] : roadPreviewPath)
                        {
                            if (city.GetCellType(pgx, pgz) == CellType::EMPTY)
                            {
                                city.PlaceCell(pgx, pgz, CellType::ROAD);
                                anyPlaced = true;
                            }
                        }
                        if (anyPlaced)
                        {
                            trafficLights.RebuildIntersections(city);
                            city.BuildSidewalkLayer([&](int gx2, int gz2){ return trafficLights.IsIntersection(gx2, gz2); });
                            TrySpawnAllUnspawnedHouses();
                        }
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
                        if (roadNeighbors <= 2)
                        {
                            city.PlaceCell(hoverGX, hoverGZ, CellType::CROSSWALK);
                            trafficLights.RebuildIntersections(city);
                            city.BuildSidewalkLayer([&](int gx2, int gz2){ return trafficLights.IsIntersection(gx2, gz2); });
                        }
                    }
                }
                else
                {
                    // Tools 1=house, 2=workplace, 5=parking (lmbDown drag)
                    CellType ct;
                    if (placeTool == 5)
                        ct = CellType::PARKING;
                    else
                        ct = static_cast<CellType>(placeTool + 1);
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
            float mx = (float)wi::input::GetPointer().x;
            float my = (float)wi::input::GetPointer().y;
            float menuX = this->width * 0.5f - 200.0f;
            float menuY = 80.0f;
            float entryH = 40.0f;
            float menuW = 400.0f;

            if (wi::input::Press(wi::input::MOUSE_BUTTON_LEFT))
            {
                // "New Save" button at top (only in save mode)
                if (savesMenuSaveMode)
                {
                    float newBtnY = menuY + 40.0f;
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
                float listStartY = menuY + (savesMenuSaveMode ? 90.0f : 40.0f);
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
                // One new person born (cap at 50 per house)
                if (pop >= 50) continue;
                city.AddHousePop(gx, gz, 1);
                city.UpdateHouseHeight(gx, gz);

                // Spawn the new person with a job
                TrySpawnOneWorker(gx, gz);
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

            // Prefer cars (bigger), then agents
            if (bestCarIdx >= 0 && bestCarDist <= bestAgentDist)
            {
                selectedCar = bestCarIdx;
                selectedAgent = -1;
            }
            else if (bestAgentIdx >= 0)
            {
                selectedAgent = bestAgentIdx;
                selectedCar = -1;
            }
            else
            {
                selectedCar = -1;
                selectedAgent = -1;
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

    // Automatically place crosswalks on road cells adjacent to intersections.
    // A crosswalk is placed 1 cell away from the intersection on each arm,
    // but only on straight road segments (exactly 2 road-like neighbors).
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
            const float px   = (float)this->width - 215.0f;
            const float btnY[6] = { 145.0f, 215.0f, 285.0f, 355.0f, 425.0f, 495.0f };
            const float btnH = 55.0f, btnW = 205.0f;
            struct BtnDef { const char* label; XMFLOAT4 col; };
            BtnDef btns[6] = {
                { "  Road",       {0.40f,0.40f,1.00f,1.0f} },
                { "  House",      {1.00f,0.50f,0.10f,1.0f} },
                { "  Workplace",  {0.20f,0.55f,1.00f,1.0f} },
                { "  Tree",       {0.20f,0.80f,0.20f,1.0f} },
                { "  Crosswalk",  {0.90f,0.90f,0.40f,1.0f} },
                { "  Parking",    {0.55f,0.53f,0.75f,1.0f} },
            };

            // Panel title
            wi::font::Params title;
            title.posX  = px;
            title.posY  = 108.0f;
            title.size  = 22;
            title.color = wi::Color(220, 220, 220, 230);
            wi::font::Draw("BUILD TOOLS", title, cmd);

            for (int k = 0; k < 6; ++k)
            {
                bool sel = (placeTool == k);

                // Button background
                wi::image::Params bg;
                bg.pos  = XMFLOAT3(px - 5.0f, btnY[k], 0.0f);
                bg.siz  = XMFLOAT2(btnW, btnH);
                bg.color = sel
                    ? XMFLOAT4(btns[k].col.x*0.6f, btns[k].col.y*0.6f, btns[k].col.z*0.6f, 0.90f)
                    : XMFLOAT4(0.08f, 0.08f, 0.12f, 0.75f);
                wi::image::Draw(nullptr, bg, cmd);

                // Selection bar on left edge
                if (sel)
                {
                    wi::image::Params bar;
                    bar.pos  = XMFLOAT3(px - 5.0f, btnY[k], 0.0f);
                    bar.siz  = XMFLOAT2(5.0f, btnH);
                    bar.color = btns[k].col;
                    wi::image::Draw(nullptr, bar, cmd);
                }

                // Label
                wi::font::Params fp;
                fp.posX  = px + 12.0f;
                fp.posY  = btnY[k] + 13.0f;
                fp.size  = 26;
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
            hfp.posY  = 500.0f + 65.0f;
            hfp.size  = 18;
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
            const float panX = 10.f, panW = 215.f, panH = 105.f;
            const float panY = (float)height - panH - 10.f;
            const float sliderX = panX + 5.f, sliderW = 185.f, trackH = 12.f;
            const float track1Y = panY + 30.f;
            const float track2Y = panY + 67.f;

            // Panel background
            wi::image::Params bg;
            bg.pos   = XMFLOAT3(panX, panY, 0.f);
            bg.siz   = XMFLOAT2(panW, panH);
            bg.color = XMFLOAT4(0.05f, 0.05f, 0.10f, 0.80f);
            wi::image::Draw(nullptr, bg, cmd);

            // Title
            wi::font::Params tp;
            tp.posX  = panX + 6.f; tp.posY = panY + 6.f;
            tp.size  = 16; tp.color = wi::Color(160, 180, 255, 220);
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
                thumb.pos   = XMFLOAT3(sliderX + sliderW * t - 5.f, trackY - 3.f, 0.f);
                thumb.siz   = XMFLOAT2(10.f, trackH + 6.f);
                thumb.color = XMFLOAT4(1.f, 1.f, 1.f, 0.95f);
                wi::image::Draw(nullptr, thumb, cmd);

                // Label
                std::ostringstream los;
                los << label << ": " << std::fixed << std::setprecision(0) << val << " m";
                wi::font::Params lp;
                lp.posX  = sliderX; lp.posY = trackY + trackH + 2.f;
                lp.size  = 14; lp.color = wi::Color(190, 190, 190, 200);
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
        else
        {
            oss << "\n\n[LMB] Click any agent or car to inspect";
        }

        {
            wi::font::Params shadow;
            shadow.posX  = 12.5f;  shadow.posY  = 12.5f;
            shadow.size  = 22;
            shadow.color = wi::Color(0, 0, 0, 180);
            wi::font::Draw(oss.str(), shadow, cmd);
        }
        {
            wi::font::Params fp;
            fp.posX  = 10.0f;  fp.posY  = 10.0f;
            fp.size  = 22;
            fp.color = wi::Color(240, 240, 60, 255);
            wi::font::Draw(oss.str(), fp, cmd);
        }

        // ---- Saves Menu Overlay ----
        if (savesMenuOpen)
        {
            float menuW = 400.0f;
            float menuX = (float)this->width * 0.5f - menuW * 0.5f;
            float menuY = 80.0f;
            float entryH = 40.0f;
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
                tp.size = 28;
                tp.color = wi::Color(255, 255, 255, 255);
                wi::font::Draw(savesMenuSaveMode ? "SAVE GAME" : "LOAD GAME", tp, cmd);
            }

            float listStartY = menuY + 40.0f;

            // "New Save" button (save mode only)
            if (savesMenuSaveMode)
            {
                wi::image::Params bg;
                bg.pos  = XMFLOAT3(menuX, listStartY, 0);
                bg.siz  = XMFLOAT2(menuW, entryH);
                bg.color = XMFLOAT4(0.15f, 0.55f, 0.15f, 0.85f);
                wi::image::Draw(nullptr, bg, cmd);

                wi::font::Params fp2;
                fp2.posX = menuX + 12.0f; fp2.posY = listStartY + 8.0f;
                fp2.size = 22;
                fp2.color = wi::Color(255, 255, 255, 255);
                wi::font::Draw("+ New Save", fp2, cmd);
                listStartY += entryH + 10.0f;
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
                bg.siz = XMFLOAT2(menuW, entryH - 2.0f);
                bg.color = hover ? XMFLOAT4(0.3f, 0.3f, 0.6f, 0.9f) : XMFLOAT4(0.1f, 0.1f, 0.15f, 0.85f);
                wi::image::Draw(nullptr, bg, cmd);

                wi::font::Params fp2;
                fp2.posX = menuX + 12.0f; fp2.posY = ey + 8.0f;
                fp2.size = 22;
                fp2.color = hover ? wi::Color(255, 255, 80, 255) : wi::Color(220, 220, 220, 230);
                wi::font::Draw(saveFiles[si], fp2, cmd);

                if (savesMenuSaveMode) {
                    wi::font::Params ov;
                    ov.posX = menuX + menuW - 100.0f; ov.posY = ey + 8.0f;
                    ov.size = 18;
                    ov.color = wi::Color(255, 120, 120, 200);
                    wi::font::Draw("overwrite", ov, cmd);
                }
            }

            // Footer
            {
                float footY = listStartY + (float)std::min((int)saveFiles.size(), maxShow) * entryH + 10.0f;
                wi::font::Params fp2;
                fp2.posX = menuX; fp2.posY = footY;
                fp2.size = 18;
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

    // ---- Create window (1920 × 1080) ----
    RECT rc{ 0, 0, 1920, 1080 };
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
