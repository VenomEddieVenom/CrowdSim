"""
Generate a large, well-designed city save for CrowdSim.
All building types, bus lines, no trees. Big city with district variety.

Binary formats:
  .city: magic(4) + version(4) + gridsize(4) + cellType[1600](1600) + pop[1600](6400) + lanes[1600](6400)
  .routes: magic(4) + count(4) + per-route data
"""
import struct
import os

# ===========================================================================
# CONSTANTS
# ===========================================================================
GRID = 40
EMPTY = 0; ROAD = 1; HOUSE = 2; WORKPLACE = 3; CROSSWALK = 4
PARKING = 5; LAKE = 6; POWER_PLANT = 7; WATER_PUMP = 8
POLICE = 9; FIRE_STATION = 10; HOSPITAL = 11; BUS_DEPOT = 12; BUS_STOP = 13

grid  = [[EMPTY]*GRID for _ in range(GRID)]   # grid[z][x]
pop   = [[0]*GRID     for _ in range(GRID)]
lanes = [[0]*GRID     for _ in range(GRID)]

# ===========================================================================
# 1. ROAD NETWORK  (9 lines each axis → 8×8 city blocks)
# ===========================================================================
road_lines = [0, 5, 10, 15, 20, 25, 30, 35, 39]

for z in road_lines:
    for x in range(GRID):
        grid[z][x] = ROAD
        lanes[z][x] = 6
for x in road_lines:
    for z in range(GRID):
        grid[z][x] = ROAD
        lanes[z][x] = 6

# Arterials (6 lanes) – center cross + inner ring
for a in [15, 20, 25]:
    for x in range(GRID):
        lanes[a][x] = 6
    for z in range(GRID):
        lanes[z][a] = 6

# Boulevard ring (6 lanes)
for a in [5, 35]:
    for x in range(GRID):
        lanes[a][x] = 6
    for z in range(GRID):
        lanes[z][a] = 6

# Outer ring (2 lanes)
for x in range(GRID):
    lanes[0][x]  = 2
    lanes[39][x] = 2
for z in range(GRID):
    lanes[z][0]  = 2
    lanes[z][39] = 2

# ===========================================================================
# 2. LAKE  (SW area – irregular shape)
# ===========================================================================
lake_positions = set()
# Main body
for x in range(1, 5):
    for z in range(31, 35):
        lake_positions.add((x, z))
# Southern extension
for x in range(1, 4):
    for z in range(36, 39):
        lake_positions.add((x, z))
# Eastern arm
for x in range(6, 9):
    for z in range(31, 34):
        lake_positions.add((x, z))

for x, z in lake_positions:
    grid[z][x] = LAKE

# ===========================================================================
# 3. SPECIAL BUILDINGS  (placed before block-fill so they're not overwritten)
# ===========================================================================

# --- Bus Depots (5) ---
bus_depot_positions = [
    (1,  1),   # NW
    (38, 1),   # NE
    (18, 3),   # N-central
    (11, 38),  # S-central
    (34, 36),  # SE
]
for x, z in bus_depot_positions:
    grid[z][x] = BUS_DEPOT

# --- Police Stations (6) – Manhattan coverage ≈ 10 cells ---
police_pos = [(3,3), (33,3), (3,21), (33,21), (18,33), (21,11)]
for x, z in police_pos:
    grid[z][x] = POLICE

# --- Fire Stations (6) ---
fire_pos = [(8,8), (28,8), (8,28), (28,28), (18,16), (36,18)]
for x, z in fire_pos:
    grid[z][x] = FIRE_STATION

# --- Hospitals (5) ---
hosp_pos = [(13,6), (33,13), (13,28), (28,33), (6,17)]
for x, z in hosp_pos:
    grid[z][x] = HOSPITAL

# --- Power Plants (20) – spread across city ---
pp_pos = [
    # SE industrial cluster
    (36,31),(37,31),(38,31),(36,32),(37,32),(38,32),
    (36,33),(37,33),
    # SE corner
    (38,36),(37,37),(38,37),(36,37),
    # West side
    (1,13),(2,13),
    # East side
    (37,13),(38,13),
    # NW
    (1,6),(2,6),
    # NE
    (37,6),(38,6),
]
for x, z in pp_pos:
    if grid[z][x] == EMPTY:
        grid[z][x] = POWER_PLANT

# --- Water Pumps (adjacent to lake, auto-detect) ---
wp_placed = 0
for x in range(GRID):
    for z in range(GRID):
        if grid[z][x] != EMPTY:
            continue
        adj_lake = False
        for dx, dz in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, nz = x+dx, z+dz
            if 0 <= nx < GRID and 0 <= nz < GRID and (nx,nz) in lake_positions:
                adj_lake = True
                break
        if adj_lake and wp_placed < 12:
            grid[z][x] = WATER_PUMP
            wp_placed += 1

# --- Extra Parking Garages in key spots ---
extra_parking = [
    # Near CBD
    (17,17),(22,17),(17,22),(22,22),(19,17),(17,19),(22,19),(19,22),
    # Inner ring
    (24,3),(3,24),(31,16),(16,31),(26,26),(11,3),(3,11),(34,3),(3,34),
    # Near intersections across the city
    (6,6),(6,11),(6,16),(6,26),(6,31),(6,36),
    (11,6),(11,11),(11,16),(11,26),(11,31),
    (16,6),(16,11),(16,26),(16,36),
    (26,6),(26,11),(26,16),(26,31),(26,36),
    (31,6),(31,11),(31,26),(31,31),
    (36,6),(36,11),(36,16),(36,26),
    # Near residential clusters
    (2,2),(4,4),(37,2),(2,37),(37,38),(33,38),
    (8,14),(14,8),(27,14),(14,27),(27,27),
]
for x, z in extra_parking:
    if grid[z][x] == EMPTY:
        grid[z][x] = PARKING

# ===========================================================================
# 4. BUS STOPS  (on ROAD cells along major routes)
# ===========================================================================
bus_stop_positions = [
    # E-W along z=5
    (8,5),(13,5),(18,5),(23,5),(28,5),(33,5),
    # E-W along z=10
    (3,10),(18,10),(28,10),(37,10),
    # E-W along z=20
    (3,20),(8,20),(13,20),(23,20),(28,20),(33,20),
    # E-W along z=30
    (8,30),(18,30),(28,30),
    # E-W along z=35
    (8,35),(13,35),(23,35),(28,35),(33,35),
    # N-S along x=10
    (10,8),(10,18),(10,28),
    # N-S along x=20
    (20,3),(20,8),(20,13),(20,23),(20,28),(20,33),
    # N-S along x=30
    (30,8),(30,18),(30,28),
    # N-S along x=35
    (35,8),(35,18),(35,28),
]
for x, z in bus_stop_positions:
    if grid[z][x] == ROAD:
        grid[z][x] = BUS_STOP
        # keep lane width from road

# ===========================================================================
# 5. CROSSWALKS  (on ROAD cells with ≤2 road-like neighbors, near intersections)
# ===========================================================================
road_like = {ROAD, CROSSWALK, BUS_STOP}

def count_road_neighbors(cx, cz):
    n = 0
    for dx, dz in [(-1,0),(1,0),(0,-1),(0,1)]:
        nx, nz = cx+dx, cz+dz
        if 0 <= nx < GRID and 0 <= nz < GRID and grid[nz][nx] in road_like:
            n += 1
    return n

# Generate candidates: 1 cell away from inner intersections
inner_roads = [5, 10, 15, 20, 25, 30, 35]  # skip outer 0/39
cw_candidates = []
for rz in inner_roads:
    for rx in inner_roads:
        for dx, dz in [(-1,0),(1,0),(0,-1),(0,1)]:
            cx, cz = rx+dx, rz+dz
            if 0 <= cx < GRID and 0 <= cz < GRID:
                if grid[cz][cx] == ROAD and count_road_neighbors(cx, cz) <= 2:
                    cw_candidates.append((cx, cz))

# Deduplicate and pick up to 50
seen = set()
cw_unique = []
for c in cw_candidates:
    if c not in seen:
        seen.add(c)
        cw_unique.append(c)

# Place crosswalks – pick every other one for good distribution
cw_placed = 0
for i, (cx, cz) in enumerate(cw_unique):
    if cw_placed >= 50:
        break
    if grid[cz][cx] == ROAD and count_road_neighbors(cx, cz) <= 2:
        grid[cz][cx] = CROSSWALK
        cw_placed += 1

# ===========================================================================
# 6. BLOCK FILLS  (district-based)
# ===========================================================================
block_ranges = [
    (1,4), (6,9), (11,14), (16,19), (21,24), (26,29), (31,34), (36,38)
]

# District map  [bz][bx]  – 8×8 blocks
# R=Residential  W=Workplace  M=Mixed  C=CBD  P=Parking-heavy
# L=Lake/green   I=Industrial  U=Utility
district_map = [
  # bx: 0    1    2    3    4    5    6    7
  ['R', 'R', 'R', 'M', 'M', 'R', 'R', 'R'],  # bz0 (north edge)
  ['R', 'R', 'M', 'W', 'W', 'M', 'R', 'R'],  # bz1
  ['R', 'M', 'W', 'W', 'W', 'W', 'M', 'R'],  # bz2
  ['M', 'M', 'W', 'C', 'C', 'W', 'P', 'M'],  # bz3 (central)
  ['M', 'M', 'W', 'C', 'C', 'W', 'M', 'M'],  # bz4
  ['R', 'R', 'M', 'W', 'W', 'M', 'R', 'R'],  # bz5
  ['L', 'L', 'R', 'M', 'R', 'R', 'R', 'I'],  # bz6 (lake area)
  ['L', 'U', 'R', 'R', 'R', 'R', 'R', 'I'],  # bz7 (south edge)
]

def dist_from_center(x, z):
    return abs(x - 20) + abs(z - 20)

for bz in range(8):
    for bx in range(8):
        district = district_map[bz][bx]
        xs, xe = block_ranges[bx]
        zs, ze = block_ranges[bz]

        idx = 0
        for z in range(zs, ze+1):
            for x in range(xs, xe+1):
                if grid[z][x] != EMPTY:
                    idx += 1
                    continue

                d = dist_from_center(x, z)

                if district == 'R':
                    # Add parking every 7th cell in residential
                    if idx % 7 == 3:
                        grid[z][x] = PARKING
                    else:
                        grid[z][x] = HOUSE
                        if d < 15:
                            pop[z][x] = 30 + (x*17 + z*13) % 20
                        elif d < 25:
                            pop[z][x] = 15 + (x*11 + z*7) % 15
                        else:
                            pop[z][x] = 8 + (x*7 + z*3) % 12

                elif district == 'W':
                    # Parking every 6th cell in workplace districts
                    if idx % 6 == 2:
                        grid[z][x] = PARKING
                    else:
                        grid[z][x] = WORKPLACE

                elif district == 'M':
                    if idx % 8 == 5:
                        grid[z][x] = PARKING
                    elif (x + z) % 3 == 0:
                        grid[z][x] = WORKPLACE
                    else:
                        grid[z][x] = HOUSE
                        pop[z][x] = 10 + (x*11 + z*7) % 20

                elif district == 'C':
                    if idx % 5 == 0:
                        grid[z][x] = PARKING
                    else:
                        grid[z][x] = WORKPLACE

                elif district == 'P':
                    if idx % 3 == 0:
                        grid[z][x] = PARKING
                    else:
                        grid[z][x] = WORKPLACE

                elif district == 'I':
                    if idx % 6 == 0 and grid[z][x] == EMPTY:
                        grid[z][x] = POWER_PLANT
                    elif idx % 5 == 1:
                        grid[z][x] = PARKING
                    else:
                        grid[z][x] = WORKPLACE

                elif district == 'U':
                    # Utility near lake – fill gaps with small houses
                    grid[z][x] = HOUSE
                    pop[z][x] = 5 + (x*5 + z*3) % 10

                elif district == 'L':
                    pass  # Leave as EMPTY (green space near lake)

                idx += 1

# ===========================================================================
# 7. WRITE .city FILE
# ===========================================================================
save_dir = r"E:\Wicked\build\Release\saves"
os.makedirs(save_dir, exist_ok=True)

city_path = os.path.join(save_dir, "designed_city.city")
with open(city_path, 'wb') as f:
    f.write(struct.pack('<I', 0x43495459))  # Magic "CITY"
    f.write(struct.pack('<I', 2))            # Version 2
    f.write(struct.pack('<I', 40))           # Grid size

    # Cell types  (row-major: z outer, x inner)
    for z in range(GRID):
        for x in range(GRID):
            f.write(struct.pack('B', grid[z][x]))

    # House population
    for z in range(GRID):
        for x in range(GRID):
            f.write(struct.pack('<i', pop[z][x]))

    # Road lanes
    for z in range(GRID):
        for x in range(GRID):
            f.write(struct.pack('<i', lanes[z][x]))

# ===========================================================================
# 8. WRITE .routes FILE  (5 bus lines)
# ===========================================================================
routes_path = os.path.join(save_dir, "designed_city.routes")

def make_routes():
    routes = []

    # Route 1 (Line 101): East-West North
    routes.append({
        'line': 101,
        'depotA': (1, 1),
        'depotB': (38, 1),
        'circular': False,
        'stops': [
            (8, 5,  "Oak Avenue"),
            (13, 5, "Maple Street"),
            (18, 5, "Park Central N"),
            (23, 5, "Pine Road"),
            (28, 5, "Cedar Lane"),
            (33, 5, "Elm Drive"),
        ],
        'return_stops': [],
    })

    # Route 2 (Line 102): East-West South
    routes.append({
        'line': 102,
        'depotA': (11, 38),
        'depotB': (34, 36),
        'circular': False,
        'stops': [
            (13, 35, "South Market"),
            (23, 35, "Central Park S"),
            (28, 35, "Harbor View"),
            (33, 35, "Docklands"),
        ],
        'return_stops': [],
    })

    # Route 3 (Line 201): North-South Central
    routes.append({
        'line': 201,
        'depotA': (18, 3),
        'depotB': (11, 38),
        'circular': False,
        'stops': [
            (20, 8,  "University"),
            (20, 13, "City Hall"),
            (20, 23, "Downtown"),
            (20, 28, "Riverside"),
            (20, 33, "Lakeside"),
        ],
        'return_stops': [],
    })

    # Route 4 (Line 301): Downtown Circular
    routes.append({
        'line': 301,
        'depotA': (18, 3),
        'depotB': (18, 3),
        'circular': True,
        'stops': [
            (13, 20, "West Station"),
            (20, 13, "North Plaza"),
            (28, 20, "East Junction"),
            (20, 28, "South Gate"),
        ],
        'return_stops': [
            (20, 28, "South Gate R"),
            (28, 20, "East Junction R"),
            (20, 13, "North Plaza R"),
            (13, 20, "West Station R"),
        ],
    })

    # Route 5 (Line 401): North-South East
    routes.append({
        'line': 401,
        'depotA': (38, 1),
        'depotB': (34, 36),
        'circular': False,
        'stops': [
            (35, 8,  "East Village N"),
            (35, 18, "Tech Park"),
            (35, 28, "East Village S"),
        ],
        'return_stops': [],
    })

    return routes

routes = make_routes()

with open(routes_path, 'wb') as f:
    f.write(struct.pack('<I', 0x42555332))  # Magic "BUS2"
    f.write(struct.pack('<I', len(routes)))  # Route count

    for route in routes:
        f.write(struct.pack('<i', route['line']))
        ax, az = route['depotA']
        bx, bz = route['depotB']
        f.write(struct.pack('<ii', ax, az))
        f.write(struct.pack('<ii', bx, bz))
        f.write(struct.pack('B', 1 if route['circular'] else 0))

        # Outbound stops
        f.write(struct.pack('<I', len(route['stops'])))
        for gx, gz, name in route['stops']:
            f.write(struct.pack('<ii', gx, gz))
            nb = name.encode('utf-8')[:31]
            f.write(nb + b'\x00' * (32 - len(nb)))

        # Active
        f.write(struct.pack('B', 1))

        # Return stops
        f.write(struct.pack('<I', len(route['return_stops'])))
        for gx, gz, name in route['return_stops']:
            f.write(struct.pack('<ii', gx, gz))
            nb = name.encode('utf-8')[:31]
            f.write(nb + b'\x00' * (32 - len(nb)))

# ===========================================================================
# 9. STATISTICS
# ===========================================================================
names = {0:'Empty', 1:'Road', 2:'House', 3:'Workplace', 4:'Crosswalk',
         5:'Parking', 6:'Lake', 7:'PowerPlant', 8:'WaterPump', 9:'Police',
         10:'FireStation', 11:'Hospital', 12:'BusDepot', 13:'BusStop'}

counts = {}
total_pop = 0
for z in range(GRID):
    for x in range(GRID):
        ct = grid[z][x]
        counts[ct] = counts.get(ct, 0) + 1
        total_pop += pop[z][x]

power_supply = counts.get(POWER_PLANT, 0) * 50
water_supply = counts.get(WATER_PUMP, 0) * 50
power_demand = counts.get(HOUSE,0)*3 + counts.get(WORKPLACE,0)*5 + \
    (counts.get(POLICE,0)+counts.get(FIRE_STATION,0)+counts.get(HOSPITAL,0)+counts.get(BUS_DEPOT,0))*4
water_demand = counts.get(HOUSE,0)*2 + counts.get(WORKPLACE,0)*3 + \
    (counts.get(POLICE,0)+counts.get(FIRE_STATION,0)+counts.get(HOSPITAL,0)+counts.get(BUS_DEPOT,0))*3

print(f"City saved to: {city_path}")
print(f"Routes saved to: {routes_path}")
print(f"File sizes: .city={os.path.getsize(city_path)} bytes, .routes={os.path.getsize(routes_path)} bytes")
print(f"\n=== City Statistics ===")
for ct in sorted(counts.keys()):
    print(f"  {names.get(ct, f'Type {ct}'):14s}: {counts[ct]:4d}")
print(f"  {'Total cells':14s}: {sum(counts.values()):4d}")
print(f"\n  Total population:  {total_pop}")
print(f"  Power: {power_supply} supply / {power_demand} demand")
print(f"  Water: {water_supply} supply / {water_demand} demand")
print(f"  Bus routes: {len(routes)}")
print(f"  Crosswalks placed: {cw_placed}")

# Print ASCII map
print("\n=== City Map (top-down, z=0 at top) ===")
sym = {0:'.', 1:'=', 2:'H', 3:'W', 4:'X', 5:'P', 6:'~', 7:'$',
       8:'O', 9:'!', 10:'F', 11:'+', 12:'B', 13:'S'}
for z in range(GRID):
    row = ""
    for x in range(GRID):
        row += sym.get(grid[z][x], '?')
    print(f"  {z:2d} {row}")
print("     " + "".join(str(x%10) for x in range(GRID)))
