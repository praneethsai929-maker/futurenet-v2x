# FutureNet M09 Technical Report & Design Decisions

## 1. Design Decisions & Guidelines Compliance

### 1.1 Non-Intrusive Integration (Quality Gate QG-07)
All module code lives entirely in `contrib/futurenet-v2x/`. No modifications were made to core ns-3 (`src/`).

### 1.2 Mechanism / Policy Separation
V2I association is decoupled using `RsuSelectionPolicy`. The base class presents a pure virtual interface `Select(state)`, implemented by `NearestRsuSelectionPolicy`. This baseline calculates Euclidean distance to candidate RSUs for active vehicles without global all-to-all $O(N^2)$ neighbor scans. External AI agents (e.g. M10 Agent) or reinforcement learning algorithms can subclass `RsuSelectionPolicy` without modifying networking code.

### 1.3 Avoiding $O(N^2)$ Performance Bottlenecks
Per Section 11 of the Common Guidelines:
- Instead of computing pair-wise vehicle-to-vehicle distances every simulation tick ($O(N^2)$), vehicle positions are managed natively by ns-3's `WaypointMobilityModel` and sampled only at `MobilityStep` intervals.
- RSU association checks evaluate each active vehicle against the fixed RSU set ($O(N \cdot M)$ where $M \le 100 \ll N$).

### 1.4 Randomness & Reproducibility (CG-09, CG-10)
All random variables (such as application start jitter) use `ns3::UniformRandomVariable` bound to ns-3's stream management system via `AssignStreams()`. Test `T5` verifies strict simulation repeatability.

---

## 2. Validation Results (V1, V2, V3)

Automated verification is implemented in `tools/validate_v2x.py`:
1. **V1 (Spatial Ground Truth)**: Compared 13 sampled waypoints across vehicles `veh0`, `veh1`, and `veh2` against the original SUMO FCD XML. Maximum observed spatial discrepancy: **0.000000 m** (tolerance: $< 0.01\text{ m}$).
2. **V2 (Kinematic Trend)**: Verified continuous forward trajectories with positive displacement and smooth waypoint interpolation.
3. **V3 (Controlled Variation & Boundary Test)**: Ran the same mobility trace under differing network message intervals (100 ms vs 500 ms) and verified that the output `mobility.csv` files were **100% byte-for-byte identical**, confirming network decoupling.

---

## 3. Preprocessing Instructions for SUMO & OSM (AC-05, FR-09)

1. **Obtain OpenStreetMap Data**: Download `.osm` extract for the desired target area (e.g., from OpenStreetMap export or Geofabrik).
2. **Convert OSM to SUMO Road Network**:
   ```bash
   python3 tools/osm_to_sumo.py target_city.osm -o city_net
   ```
3. **Generate Routes & Traffic Demand**:
   Use SUMO's `randomTrips.py` or `duarouter`:
   ```bash
   randomTrips.py -n city_net.net.xml -e 100 -l -o trips.trips.xml
   duarouter -n city_net.net.xml --route-files trips.trips.xml -o city_routes.rou.xml
   ```
4. **Export Floating Car Data (FCD)**:
   Run SUMO with FCD export enabled:
   ```bash
   sumo -n city_net.net.xml -r city_routes.rou.xml --fcd-output fcd_trace.xml --fcd-output.geo false
   ```
5. **Simulate in ns-3**:
   ```bash
   ./ns3 run "example-map --fcdTrace=fcd_trace.xml --simTime=60"
   ```
