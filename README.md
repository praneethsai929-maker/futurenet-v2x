# FutureNet M09: V2X Mobility, Road-Network and SUMO Integration (`futurenet-v2x`)

Contributed module for **ns-3.46.1** implementing vehicular network scenarios with offline Floating Car Data (FCD) trajectory ingestion from Eclipse SUMO, deterministic vehicle-to-node mapping, Roadside Unit (RSU) infrastructure, mechanism/policy split for V2I association, and comprehensive CSV telemetry.

---

## 1. Project Overview & Architecture

```
 OpenStreetMap (.osm)
        │
   (netconvert)
        ▼
 SUMO .net.xml + routes
        │
     (SUMO)
        ▼
 FCD Trajectory XML (fcd-export)
        │
        ▼
 SumoFcdParser ──> VehicleNodeMapper (Deterministic ID mapping)
        │
        ▼
 V2xMobilityImporter ──> ns-3 WaypointMobilityModel
        │
        ▼
 V2xScenarioHelper ──> RsuPlacementHelper + NearestRsuSelectionPolicy
        │
        ▼
 V2V (802.11 ad-hoc) & V2I Communications + FlowMonitor + CSV Tracing
```

### Key Capabilities
- **Streaming XML FCD Parser (`SumoFcdParser`)**: Stream-parses SUMO FCD output with coordinate scaling (`CoordinateScale`) and translations (`CoordinateOffsetX/Y`).
- **Deterministic Mapper (`VehicleNodeMapper`)**: Guarantees reproducible, sorted 1-to-1 mapping between SUMO vehicle ID strings and ns-3 node IDs.
- **Dynamic Vehicle Lifecycle (`V2xMobilityImporter`)**: Automatically triggers `VehicleActivated` and `VehicleDeactivated` TraceSources based on departure and arrival times.
- **Roadside Unit Deployment (`RsuPlacementHelper`)**: Deploys fixed RSUs with explicit coordinates, regular grid spacing, or junction lists.
- **Mechanism/Policy Separation (`RsuSelectionPolicy`)**: Clean abstract interface with deterministic `NearestRsuSelectionPolicy` baseline.
- **Three-Tier CSV Telemetry**:
  1. **Mobility CSV**: `time_s,vehicle_id,node_id,x_m,y_m,z_m,speed_mps,active`
  2. **Network CSV**: `time_s,vehicle_id,peer_id,event,delay_ms,throughput_mbps,loss`
  3. **Common Event Log**: `time_s,module,event,node_id,peer_id,flow_id,value1,value2,unit,detail`

---

## 2. Team Workload Division & Git Attribution

| Developer | Git Identity | Module Responsibility | Key Components & Requirements |
| :--- | :--- | :--- | :--- |
| **Dev 1** | `Kris-2006`<br>`88359324+Kris-2006@users.noreply.github.com` | **Trace Ingestion & Mapping Engine** | • `SumoFcdParser`<br>• `VehicleNodeMapper`<br>• `V2xScenarioHelper` core API<br>• Tests T01, T05, T06, T1<br>• Validation V1 (Spatial alignment) |
| **Dev 2** | `pvinod33`<br>`palthyavathvinod33@gmail.com` | **Waypoint Mobility & Node Lifecycle** | • `V2xMobilityImporter`<br>• `VehicleLifecycleManager`<br>• Sampling & Mobility Tracing<br>• Tests T02, T03, T04, T4<br>• Validation V2 (Kinematics) |
| **Dev 3** | `praneethsai929-maker`<br>`praneethsai929@gmail.com` | **RSUs, Wireless V2X & SUMO Tools** | • `RsuPlacementHelper`<br>• `RsuSelectionPolicy` baseline<br>• Wireless & Network Traces<br>• Tests T3, T5, T6<br>• Validation V3 (Controlled variation & extremes) |

---

## 3. Build & Configuration

### Prerequisites
- ns-3.46.1 baseline
- CMake >= 3.15
- C++17 compliant compiler (Clang / GCC)
- Python 3.8+ (for validation scripts)

### Build Commands
From the `ns-3.46.1` directory:
```bash
# 1. Configure ns-3 with examples and tests enabled
./ns3 configure --enable-examples --enable-tests

# 2. Build futurenet-v2x and examples
./ns3 build futurenet-v2x example-grid example-map
```

---

## 4. Running the Tests & Validations

### Automated Unit Test Suite
To execute all 10 unit tests (T01–T06, T1, T3, T5, T6):
```bash
./ns3 run "test-runner --suite=futurenet-v2x"
```

### Full Validation Suite (V1, V2, V3)
```bash
python3 contrib/futurenet-v2x/tools/validate_v2x.py
```
This script validates:
- **V1 (Spatial Alignment)**: Compares ns-3 node coordinates against SUMO ground truth ($< 0.01$ m tolerance).
- **V2 (Kinematic Trend)**: Verifies continuous forward velocity and displacement over time.
- **V3 (Controlled Variation)**: Confirms mobility output remains 100% identical under differing network parameters.

---

## 5. Running the Example Scenarios

### Example 1: Grid Scenario (`example-grid`) [D4: `example-basic`]
Runs a standard scenario with configurable command-line arguments:
```bash
./ns3 run "example-grid --fcdTrace=contrib/futurenet-v2x/test/data/fcd-demo.xml --numRsus=4 --simTime=5"
```

### Example 2: Arbitrary Map Scenario (`example-map`) [D4: `example-advanced`]
Runs with FlowMonitor integration and loads arbitrary user-provided traces without recompilation:
```bash
./ns3 run "example-map --fcdTrace=contrib/futurenet-v2x/test/data/fcd-small.xml --scale=1.0 --simTime=3 --tracePrefix=my-run"
```

---

## 6. Demonstration Scale (SRS Section 10)
To generate and run the $\ge 100$-vehicle and 4-RSU demonstration scenario:
```bash
python3 contrib/futurenet-v2x/tools/fcd_generator.py --output contrib/futurenet-v2x/test/data/fcd-demo.xml --vehicles 100 --duration 10.0
./ns3 run "example-grid --fcdTrace=contrib/futurenet-v2x/test/data/fcd-demo.xml --numRsus=4 --simTime=5"
```

---

## 7. Known Limitations & Extension Points (QG-10)
- **Offline Trajectory Scope**: High-speed offline trajectory playback is supported via `WaypointMobilityModel`. Real-time bidirectional physics changes require live TraCI co-simulation (`tools/live_traci_adapter.py`).
- **Antenna Patterns**: Default wireless uses isotropic antenna propagation. 3-D directional antenna models (e.g. from Family A) can be plugged in via standard ns-3 helpers.
- **External AI Integration**: RSU association and offloading can be driven by M10 Agent or external reinforcement learning by subclassing `RsuSelectionPolicy`.
