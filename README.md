# CrowdSim — City Traffic & Crowd Simulation

A real-time city traffic and pedestrian simulation built with [WickedEngine](https://github.com/turanszkij/WickedEngine). Place roads, houses, and workplaces on a grid to watch cars and pedestrians commute with realistic traffic light control, lane-based driving, and intelligent driver behaviour.

![C++20](https://img.shields.io/badge/C%2B%2B-20-blue) ![WickedEngine](https://img.shields.io/badge/engine-WickedEngine-orange) ![Platform](https://img.shields.io/badge/platform-Windows-lightgrey)

---

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Running](#running)
- [Controls](#controls)
- [Architecture](#architecture)
  - [Project Structure](#project-structure)
  - [CityLayout](#citylayout)
  - [CarSystem](#carsystem)
  - [CrowdSystem](#crowdsystem)
  - [TrafficLightSystem](#trafficlightsystem)
  - [Main Loop](#main-loop)
- [How It Works](#how-it-works)
  - [City Grid](#city-grid)
  - [Pathfinding](#pathfinding)
  - [Car Driving Model](#car-driving-model)
  - [Traffic Lights — 8-Phase Protected Left Turn](#traffic-lights--8-phase-protected-left-turn)
  - [Day/Night Cycle & Schedule](#daynight-cycle--schedule)
  - [Economy](#economy)
  - [Save / Load](#save--load)
- [Configuration Constants](#configuration-constants)
- [License](#license)

---

## Features

- **Editable city grid** — 40×40 cells, 20 m each (800 m × 800 m world)
- **Cell types** — Road, House, Workplace, Crosswalk, Parking, Trees
- **Click-to-draw road builder** — click start → click end, L-shaped auto-path
- **Variable lane roads** — 2, 4, or 6 lanes per cell (middle-click to cycle)
- **Up to 2,000 cars** with SoA (Structure-of-Arrays) data layout
- **Up to 50,000 pedestrians** with multi-threaded waypoint walking
- **Intelligent Driver Model (IDM)** — realistic car-following with gap control
- **Stanley lateral controller** — smooth lane tracking and cornering
- **8-phase protected left-turn traffic lights** — conflict-free intersection management
- **Dijkstra pathfinding** with real-time traffic-weighted costs
- **Day/night cycle** with dynamic sun, sky colour, and ambient lighting
- **Agent economy** — wages, savings, city tax
- **Save/Load** — binary city file format (F5 / F9)
- **First-person free camera** — WASD + RMB look, scroll to adjust speed
- **Debug HUD** — FPS, car count, selected agent/car info, traffic light status

---

## Prerequisites

### Required Software

| Software | Version | Download |
|---|---|---|
| **Visual Studio 2022** | 17.x (with C++ Desktop workload) | [Download](https://visualstudio.microsoft.com/downloads/) |
| **CMake** | 3.21 or newer | [Download](https://cmake.org/download/) |
| **Git** | Any recent version | [Download](https://git-scm.com/downloads) |
| **Windows SDK** | 10.0.19041.0+ (included with VS) | Installed via VS Installer |

### Visual Studio Workloads

In the Visual Studio Installer, ensure these are selected:

1. **Desktop development with C++**
   - MSVC v143 build tools (x64/x86)
   - Windows 10/11 SDK
   - C++ CMake tools for Windows

### Libraries (Auto-Fetched)

You do **not** need to manually download any libraries. CMake's `FetchContent` automatically clones and builds:

| Library | Source | Purpose |
|---|---|---|
| **WickedEngine** | [GitHub](https://github.com/turanszkij/WickedEngine) | 3D rendering, scene management, physics, input, shaders |

WickedEngine itself bundles its own dependencies (DirectX 12, Jolt Physics, LUA, etc.) — all handled automatically during the CMake configure step.

### Hardware Requirements

- **GPU**: DirectX 12 capable (any modern GPU from ~2015+)
- **CPU**: AVX2 support required (Intel Haswell+ / AMD Excavator+)
- **RAM**: 4 GB minimum, 8 GB recommended
- **OS**: Windows 10 64-bit or newer

---

## Building

### Option A: Command Line (Recommended)

```powershell
# Clone the repository
git clone https://github.com/VenomEddieVenom/CrowdSim.git
cd CrowdSim

# Configure (first time takes a while — downloads WickedEngine)
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 17 2022" -A x64

# Build
cmake --build build --config Release --parallel
```

### Option B: VS Code with Tasks

The project includes VS Code tasks in `.vscode/tasks.json`:

- **Build and Run** (default build task, `Ctrl+Shift+B`) — configures + builds Release
- **CMake: Configure (Release)** — configure only
- **CMake: Configure (Debug)** — configure debug build  
- **CMake: Build Release** — build Release only
- **CMake: Build Debug** — build Debug only

### Build Output

After building, the executable and all required files are in:

```
build/Release/
├── CrowdSim.exe          # Main executable
├── dxcompiler.dll         # DirectX shader compiler
├── shaders/               # WickedEngine shaders (copied automatically)
├── Content/               # WickedEngine content (copied automatically)
└── models/                # Custom models (drzewo.wiscene tree model)
```

### First Build Note

The first `cmake -B build` will clone WickedEngine (~1 GB) and build all its libraries. This can take 10-30 minutes depending on your internet and CPU. Subsequent builds only recompile changed CrowdSim source files and are very fast.

---

## Running

```powershell
# From the project root
.\build\Release\CrowdSim.exe
```

Or press `Ctrl+Shift+B` in VS Code (the "Build and Run" task).

The application starts with an empty grass field. Press **B** to enter Build Mode, place roads to form a network, add houses and workplaces, and watch the city come alive.

---

## Controls

### Camera

| Key | Action |
|---|---|
| **W/A/S/D** | Move forward/left/back/right |
| **Q/E** | Move down/up |
| **RMB + Mouse** | Look around (first-person) |
| **Scroll Wheel** | Adjust camera speed (5–1200 m/s) |
| **Shift** | Sprint (4× speed) |

### Build Mode

| Key | Action |
|---|---|
| **B** | Toggle build mode on/off |
| **LMB** | Place selected cell type / Road: click start → click end |
| **RMB** | Cancel road placement |
| **Middle-click** | Cycle road lanes (2 → 4 → 6) on hovered road cell |

**Tool Panel** (right side of screen in build mode):

| Tool | Cell Type |
|---|---|
| Road | Creates drivable road surface with sidewalks |
| House | Residential building — spawns cars (and pedestrians when enabled) |
| Workplace | Destination for commuters |
| Tree | Decorative tree model (drzewo.wiscene) |
| Crosswalk | Pedestrian crossing on existing road cell |
| Parking | Parking area |

### Simulation

| Key | Action |
|---|---|
| **1** | 1× speed (real-time) |
| **2** | 2× speed |
| **3** | 5× speed |
| **4** | 10× speed |
| **5** | 100× speed |
| **6/7** | 1000× speed |
| **F5** | Save city layout |
| **F9** | Load city layout |
| **LMB** (outside build mode) | Click on a car to inspect it |

---

## Architecture

### Project Structure

```
CrowdSim/
├── CMakeLists.txt              # Build system — fetches WickedEngine, defines targets
├── .gitignore
├── README.md                   # This file
├── .vscode/
│   └── tasks.json              # VS Code build tasks
├── src/
│   ├── main.cpp                # Entry point, CrowdApp, CrowdRenderPath, UI, camera, day cycle
│   ├── CityLayout.h            # Grid city data, cell placement, pathfinding, save/load
│   ├── CrowdSystem.h           # Pedestrian crowd header
│   ├── CrowdSystem.cpp         # Pedestrian waypoint walking, sidewalk constraint, rendering
│   ├── CarSystem.h             # Car simulation header, SoA layout, IDM constants
│   ├── CarSystem.cpp           # Car driving, IDM following, Stanley controller, turning
│   ├── TrafficLightSystem.h    # 8-phase traffic light header, GetLight() logic
│   └── TrafficLightSystem.cpp  # Light pole visuals, phase timer update
└── include/
    └── README.md               # Placeholder for third-party headers
```

### CityLayout

**File**: `src/CityLayout.h` (header-only, ~500 lines)

The city is a flat 40×40 grid. Each cell is 20 m × 20 m. The world coordinate system ranges from −400 m to +400 m on both X and Z axes.

**Cell types**:
- `EMPTY` — grass (default)
- `ROAD` — drivable surface with auto-generated sidewalks on unconnected edges
- `HOUSE` — residential building, spawns cars
- `WORKPLACE` — destination for commuter cars
- `CROSSWALK` — pedestrian crossing (placed on existing road cells)
- `PARKING` — parking area

**Key features**:
- **Auto-sidewalks**: Road cells automatically generate sidewalk geometry on edges not adjacent to other road cells. Corner fill pieces appear where two perpendicular sidewalks meet.
- **Variable lanes**: Each road cell stores a lane count (2, 4, or 6). This affects how cars distribute across the road width.
- **Dijkstra pathfinding**: `FindPath(srcGX, srcGZ, dstGX, dstGZ)` returns a path of road cell centres. Costs are weighted by real-time traffic density (`1.0 + 0.5 × carsInCell`), so cars naturally avoid congested routes.
- **Binary save format**: Magic `0x43495459` ("CITY"), version 2, stores cell types + house population + road lane counts.

### CarSystem

**File**: `src/CarSystem.h` + `src/CarSystem.cpp` (~800 lines)

Manages up to **2,000 cars** using a Structure-of-Arrays (SoA) memory layout for cache-friendly iteration.

**Car states**: `PARKED` → `EXITING_PARKING` → `DRIVING` → `ENTERING_PARKING` → `PARKED`

**Driving model**:
- **Intelligent Driver Model (IDM)** for longitudinal control (speed, following distance, braking)
- **Stanley lateral controller** for lane tracking (keeps car centred in its lane, smooth cornering)
- **Circular arc turning** at intersections (two-phase: straight approach → arc → straight exit)
- **Path simplification** (`SimplifyPath`) merges collinear waypoints to reduce overhead

**Lane system**:
- Right-hand drive (cars drive on the right side of the road centre line)
- Lane offset formula: `right2D = (dz, -dx)` applied to road cell centres
- Lane offsets per config: 2-lane (2.5 m), 4-lane (1.5 m inner, 4.5 m outer), 6-lane (1.5 / 4.0 / 6.5 m)

**Collision avoidance**:
- Per-frame per-cell linked list of driving cars
- Each car scans its lane for the nearest car ahead and applies IDM gap control
- Hard brake at gap < 0.5 m, full stop at gap ≤ −0.5 m

**Traffic light awareness**:
- Cars scan cells ahead along their segment direction (up to 80 m)
- At each intersection cell, they compute their **TurnIntent** (STRAIGHT, RIGHT, or LEFT) based on upcoming waypoint geometry
- The intent determines whether the current traffic phase grants them a green light

### CrowdSystem

**File**: `src/CrowdSystem.h` + `src/CrowdSystem.cpp` (~300 lines)

Manages up to **50,000 pedestrians** with multi-threaded update (WickedEngine's job system, groups of 256).

**Movement**:
- Waypoint following along road cell centres
- Offset to sidewalk corridor (±9 m from road centre, within 2 m sidewalk band)
- At turns: diagonal offset bisects the corner so pedestrians walk around the corner block
- At intersections: constrained to outer perimeter ring (crosswalk simulation)

**Rendering**: Cube instances, LOD-based visibility culling (NEAR 100 m, MID 200 m, FAR 600 m).

> **Note**: Pedestrians are currently disabled in `main.cpp` (spawn and update calls commented out). The system is fully functional but turned off to focus on car traffic.

### TrafficLightSystem

**File**: `src/TrafficLightSystem.h` + `src/TrafficLightSystem.cpp` (~300 lines)

Automatic intersection detection and 8-phase protected left-turn signal control.

**Intersection detection**: A road cell with 3 or more road/crosswalk neighbours is an intersection. Detected automatically when the road network changes.

**8 phases per intersection** (32-second total cycle):

| Phase | Duration | NS Traffic | EW Traffic |
|---|---|---|---|
| **NS Through** | 8 s | Straight + Right: GREEN | Right only: GREEN (merge yield) |
| **NS Yellow** | 2 s | Straight + Right: YELLOW | RED |
| **NS Left** | 4 s | Left only: GREEN | RED |
| **All Red 1** | 2 s | RED | RED |
| **EW Through** | 8 s | Right only: GREEN (merge yield) | Straight + Right: GREEN |
| **EW Yellow** | 2 s | RED | Straight + Right: YELLOW |
| **EW Left** | 4 s | RED | Left only: GREEN |
| **All Red 2** | 2 s | RED | RED |

**Key design**:
- **Protected left turns** — left-turning cars never conflict with oncoming traffic
- **Right-on-red from perpendicular** — during NS Through, EW right-turners get GREEN (they merge into traffic without crossing opposing lanes)
- **All-red clearance** — 2-second gap between direction changes for intersection clearing
- **Staggered start** — each intersection starts at a different phase based on grid position, preventing green waves of simultaneous switching

**Visual poles**: Each intersection gets 4 traffic light poles at corners with horizontal mast arms and emissive light heads. Colours update in real-time per phase.

### Main Loop

**File**: `src/main.cpp` (~1400 lines)

The entry point creates a Win32 `wi::Application` with a custom `CrowdRenderPath` that inherits from `wi::RenderPath3D`.

**Lifecycle**:
1. `Start()` — initialise city, systems, camera, sun, sky
2. `Update(dt)` — per-frame: camera, day cycle, build mode, agent/car update, traffic lights, economy, debug visuals
3. `Compose(cmd)` — HUD overlay: FPS, sim speed, population, treasury, build panel, selected car info

**Key update flow** (per frame):
```
Camera input → Day/night cycle → Build mode input → Save/Load
→ Traffic density update → cars.Update() → trafficLights.Update()
→ trafficLights.UpdateVisuals() → cars.RenderCars()
→ Debug visuals (waypoints, lights, segment arrows)
→ HUD Compose (FPS, info panel, build palette)
```

---

## How It Works

### City Grid

The world is centred at (0, 0, 0). Grid cell `(gx, gz)` maps to world position:

```
worldX = -400 + (gx + 0.5) × 20
worldZ = -400 + (gz + 0.5) × 20
```

Roads are the **asphalt surface** (dark grey, y = 0.10 m). Sidewalks are **raised kerbs** (light grey, y = 0.14 m) at the edges of road cells that don't connect to other roads. Buildings are placed as coloured cubes above the grid.

### Pathfinding

When a house is placed adjacent to a road, and at least one workplace exists with road access:

1. **Dijkstra** finds the shortest road-cell path from the house's adjacent road cell to a workplace's adjacent road cell
2. Edge cost = `1.0 + 0.5 × traffic_count[cell]` — congested roads are avoided
3. **SimplifyPath** merges consecutive collinear waypoints (e.g., a straight road of 10 cells becomes just 2 endpoints)
4. Cars receive the simplified waypoint list and apply **lane offsets** perpendicular to each segment

### Car Driving Model

**Longitudinal control — IDM** (Intelligent Driver Model):

$$a = A \left[ 1 - \left(\frac{v}{v_0}\right)^4 - \left(\frac{s^*(v, \Delta v)}{s}\right)^2 \right]$$

Where:
- $A = 6$ m/s² (max acceleration)
- $v_0 = 14$ m/s (desired speed, ~50 km/h)
- $s$ = actual gap to car ahead
- $s^* = s_0 + vT + \frac{v \Delta v}{2\sqrt{Ab}}$ (desired minimum gap)
- $s_0 = 2$ m (jam distance)
- $T = 1.5$ s (time headway)
- $b = 3$ m/s² (comfortable deceleration)

**Lateral control — Stanley controller**:

$$\delta = \theta_e + \arctan\left(\frac{K \cdot e}{v}\right)$$

Where:
- $\theta_e$ = heading error (angle between car heading and segment direction)
- $e$ = cross-track error (lateral distance from lane centre)
- $K = 2.5$ (gain parameter)

**Turning**: At intersections, cars detect upcoming turns by checking the angle between consecutive waypoint segments. Turns (cos < 0.3) use a **circular arc** model:
1. Straight approach to arc tangent point
2. Circular arc through the turn (radius ≈ inner lane offset)
3. Straight exit on the new segment

### Traffic Lights — 8-Phase Protected Left Turn

Each car approaching an intersection computes its **TurnIntent**:

```cpp
TurnIntent = STRAIGHT;  // default
if (isTurn) {
    // turnCross = cross product of current and next segment directions
    TurnIntent = (turnCross > 0) ? LEFT : RIGHT;
}
```

The system queries `GetLight(cellX, cellZ, dirX, dirZ, intent)` which:

1. Determines the car's **axis** (NS if |dz| > |dx|, else EW)
2. Looks up the intersection's current **phase**
3. Returns GREEN, YELLOW, or RED based on whether that axis + intent combination is allowed in the current phase

**Conflict-free guarantee**: Left turns only get GREEN during a dedicated phase when no other conflicting movements are active.

### Day/Night Cycle

Time of day ranges from 0.0 to 1.0:
- 0.25 = sunrise (6 AM)
- 0.50 = noon (12 PM)
- 0.75 = sunset (6 PM)
- 0.00 = midnight

The sun entity rotates around the X-axis. Light intensity, colour temperature, sky horizon/zenith colours, ambient light, and camera exposure all change smoothly based on sun elevation.

Cars have a **work schedule**: each car has a random departure time (6–9 AM) and return time (3–6 PM). Cars drive to work in the morning and back home in the evening. There's also a small chance (3%) of a joyride when parked at home.

### Economy

- Each car has a **wage** ($0.40–$1.80 per game-second while driving home from work)
- The city collects a **10% tax** on all earnings
- The **town treasury** accumulates tax revenue, displayed in the HUD
- Each car has lifetime **savings** displayed when selected

### Save / Load

- **F5**: Saves the current city layout to `save.city` (binary format)
- **F9**: Loads from `save.city`, rebuilds the entire city, respawns all agents and cars

File format (version 2):
```
[4 bytes] Magic: 0x43495459 ("CITY")
[4 bytes] Version: 2
[4 bytes] Grid size: 40
[1600 bytes] Cell types (1 byte each, 40×40)
[6400 bytes] House population (4 bytes each, 40×40)
[6400 bytes] Road lane counts (4 bytes each, 40×40)
```

---

## Configuration Constants

| Constant | Value | Location | Description |
|---|---|---|---|
| `GRID_SIZE` | 40 | CityLayout.h | Grid dimensions (40×40) |
| `CELL_SIZE` | 20.0 m | CityLayout.h | World size of each cell |
| `MAX_CARS` | 2,000 | CarSystem.h | Maximum simultaneous cars |
| `MAX_AGENTS` | 50,000 | CrowdSystem.h | Maximum pedestrians |
| `MAX_SPEED` | 14.0 m/s | CarSystem.h | Car speed limit (~50 km/h) |
| `IDM_S0` | 2.0 m | CarSystem.h | Minimum following gap |
| `IDM_T` | 1.5 s | CarSystem.h | Desired time headway |
| `IDM_B` | 3.0 m/s² | CarSystem.h | Comfortable deceleration |
| `THROUGH_DUR` | 8.0 s | TrafficLightSystem.h | Green phase for through traffic |
| `LEFT_DUR` | 4.0 s | TrafficLightSystem.h | Green phase for left turns |
| `YELLOW_DUR` | 2.0 s | TrafficLightSystem.h | Yellow clearance |
| `ALL_RED_DUR` | 2.0 s | TrafficLightSystem.h | All-red clearance |
| `CYCLE_DUR` | 32.0 s | TrafficLightSystem.h | Full traffic light cycle |
| `TAX_RATE` | 10% | CarSystem.h | City tax on earnings |

---

## License

This project is provided as-is for educational and personal use.  
[WickedEngine](https://github.com/turanszkij/WickedEngine) is licensed under the MIT License.
