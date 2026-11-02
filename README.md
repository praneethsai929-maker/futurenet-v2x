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
To execute all 16 automated unit tests:
```bash
./ns3 run "test-runner --suite=futurenet-v2x"
```
The test suite covers:
- **T01**: Parser vehicle count and IDs
- **T02**: Known FCD waypoint mapping to ns-3 position
- **T03**: FirstSeen / LastSeen timestamp accuracy
- **T04**: Coordinate scale and offset transformation
- **T05**: Missing / malformed XML error handling
- **T06**: Deterministic vehicle-to-node ID mapping
- **T1**: Object construction and default attribute values
- **T2**: Attribute bounds validation (CG-11)
- **T3**: NearestRsuSelectionPolicy baseline correctness
- **T4**: TraceSource firing and activation ordering
- **T5**: PRNG stream reproducibility (`AssignStreams`)
- **T6**: Wi-Fi 802.11p V2V/V2I packet delivery and reception
- **FR-04**: Dynamic vehicle lifecycle (departure and arrival)
- **FR-08**: Three-tier telemetry schema conformity
- **Late-Departing**: Initial position is first waypoint (no origin glide)
- **RSU-Placement**: Placement modes (coordinates, grid, junctions)

### Full Validation Suite (V1, V2, V3 & Boundaries)
```bash
python3 contrib/futurenet-v2x/tools/validate_v2x.py
```
This script validates:
- **V1 (Spatial Alignment)**: Compares ns-3 node coordinates against SUMO ground truth ($< 0.01$ m tolerance; observed: $0.000000$ m).
- **V2 (Kinematic Trend)**: Verifies continuous forward velocity and displacement over time.
- **V3 (Controlled Variation)**: Confirms mobility output remains 100% byte-for-byte identical across differing packet rates (100 ms vs 500 ms) while network records differ.
- **V4 (Boundary & Extreme Cases)**: Tests 10 ms (100 Hz), 2000 ms (0.5 Hz), and 0-RSU pure ad-hoc configurations.

---

## 5. Running the Example Scenarios

### Deliverable D4 Mapping Table
| Target Scenario Requirement | Module Example Binary | Source Location | Key Features Demonstrated |
| :--- | :--- | :--- | :--- |
| **`example-basic`** | `example-grid` | `examples/example-grid.cc` | Baseline grid scenario, configurable RSUs (`--rsuPositions`, `--gridSpacing`), periodic V2V/V2I traffic, and 3-tier CSV telemetry. |
| **`example-advanced`** | `example-map` | `examples/example-map.cc` | Arbitrary map trace ingestion, coordinate scaling/offsets, FlowMonitor integration, and controlled variation testing (`--v2xInterval`). |

### Example 1: Grid Scenario (`example-grid` / `example-basic`)
Runs a standard scenario with configurable command-line arguments:
```bash
./ns3 run "example-grid --fcdTrace=contrib/futurenet-v2x/test/data/fcd-small.xml --numRsus=4 --simTime=5"
```
Optional CLI parameters:
- `--v2xInterval=100`: Periodic message interval in ms (default: 100ms)
- `--v2xBytes=300`: Packet payload in bytes (default: 300B)
- `--rsuPositions="50,50;250,50;50,250;250,250"`: Semicolon-separated explicit 3D positions
- `--gridSpacing=200`: Lattice spacing between RSUs in meters

### Example 2: Arbitrary Map Scenario (`example-map` / `example-advanced`)
Runs with FlowMonitor integration and loads arbitrary user-provided traces without recompilation:
```bash
./ns3 run "example-map --fcdTrace=contrib/futurenet-v2x/test/data/fcd-small.xml --scale=1.0 --simTime=3 --tracePrefix=my-run"
```

---

## 6. Demonstration Scale (SRS Section 10)
To generate and execute the $\ge 100$-vehicle and 4-RSU demonstration scenario using native Eclipse SUMO tools (`netgenerate`, `randomTrips.py`, `duarouter`, and `sumo`):
```bash
# 1. Generate 100+ vehicle FCD trace using native SUMO tools (or fallback to fcd_generator.py)
python3 contrib/futurenet-v2x/tools/generate_sumo_scenario.py --vehicles 100 --duration 10.0 --output /tmp/fcd-demo-100.xml

# 2. Execute full 10-second 4-RSU simulation in ns-3
./ns3 run "example-grid --fcdTrace=/tmp/fcd-demo-100.xml --numRsus=4 --simTime=10.0 --tracePrefix=demo-run"
```

### Empirical Demonstration Results (112 Vehicles, 4 RSUs, 10.0s Duration, IEEE 802.11p)
- **Packets Transmitted**: **10,516 packets** (5,258 V2I unicast, 5,258 V2V broadcast)
- **Packets Received**: **29,529 packets** (1,492 V2I unicast, 28,037 V2V broadcast fan-out)
- **V2I Packet Delivery Ratio (PDR)**: **28.38%** (3,766 lost due to vehicle propagation range boundaries)
- **Average One-Way Delay**: **1.22 ms**
- **Average Throughput**: **0.0240 Mbps**
- **Sequence Gaps Tracked**: **416** gaps across active links during handovers and coverage transitions.
- **Trace Output Deliverables**: `demo-run-mobility.csv`, `demo-run-network.csv`, `demo-run-common-events.csv`, `demo-run-flowmon.xml`. Clean representative slices (< 150 KB) committed under `doc/raw-validation-*` per guideline 14.3.

---

## 7. Newly Added FutureNet C++ API & Functions Reference

The `futurenet-v2x` module introduces a set of specialized classes, methods, and data structures to integrate SUMO FCD trajectories, manage deterministic node lifecycles, deploy Roadside Units, and decouple V2I association policies.

### 7.1 `SumoFcdParser` (`model/sumo-fcd-parser.h`)
Streaming XML parser for SUMO Floating Car Data (`.fcd.xml`) files with coordinate scaling and spatial translation.

| Method Signature | Return Type | Description |
| :--- | :--- | :--- |
| `Load(const std::string& filename)` | `bool` | Stream-parses SUMO FCD XML trajectory file in $O(N)$ streaming fashion. Returns `true` on success, `false` on failure. |
| `GetVehicleIds() const` | `std::vector<std::string>` | Returns an alphabetically sorted list of all unique vehicle IDs discovered in the trace. |
| `GetTrajectory(const std::string& vehicleId) const` | `std::vector<VehicleWaypoint>` | Retrieves the time-ordered sequence of waypoints for the specified vehicle ID. |
| `GetFirstSeen(const std::string& vehicleId) const` | `Time` | Returns the earliest simulation timestamp when the specified vehicle appears. |
| `GetLastSeen(const std::string& vehicleId) const` | `Time` | Returns the latest simulation timestamp when the specified vehicle is recorded. |
| `HasVehicle(const std::string& vehicleId) const` | `bool` | Checks whether the specified vehicle ID exists in the parsed trajectory data. |
| `GetVehicleCount() const` | `uint32_t` | Returns the total count of unique vehicles parsed from the trace file. |
| `GetCoordinateScale() const` | `double` | Returns the coordinate scaling factor (default: `1.0`). |
| `SetCoordinateScale(double scale)` | `void` | Sets the coordinate scaling factor applied to raw SUMO coordinates. |
| `GetCoordinateOffsetX() const` | `double` | Returns the translation offset along the X-axis in meters (default: `0.0`). |
| `SetCoordinateOffsetX(double offsetX)` | `void` | Sets the translation offset added to X coordinates in meters. |
| `GetCoordinateOffsetY() const` | `double` | Returns the translation offset along the Y-axis in meters (default: `0.0`). |
| `SetCoordinateOffsetY(double offsetY)` | `void` | Sets the translation offset added to Y coordinates in meters. |
| `GetMinTime() const` | `Time` | Returns the minimum simulation timestamp present across all vehicle trajectories. |
| `GetMaxTime() const` | `Time` | Returns the maximum simulation timestamp present across all vehicle trajectories. |
| `GetLastError() const` | `std::string` | Returns a human-readable diagnostic error message if parsing fails. |

---

### 7.2 `VehicleNodeMapper` (`model/vehicle-node-mapper.h`)
Guarantees reproducible, deterministic 1-to-1 bidirectional mapping between SUMO vehicle ID strings and ns-3 node instances.

| Method Signature | Return Type | Description |
| :--- | :--- | :--- |
| `Register(const std::string& vehicleId, Ptr<Node> node)` | `void` | Registers a deterministic 1-to-1 binding between a SUMO vehicle ID and an ns-3 node pointer. |
| `GetNode(const std::string& vehicleId) const` | `Ptr<Node>` | Looks up and returns the ns-3 node associated with the vehicle ID, or `nullptr` if unmapped. |
| `GetVehicleId(uint32_t nodeId) const` | `std::string` | Looks up and returns the SUMO vehicle ID associated with the ns-3 node ID, or `""` if unmapped. |
| `GetNodeId(const std::string& vehicleId) const` | `uint32_t` | Looks up and returns the integer ns-3 node ID for a vehicle ID, or `UINT32_MAX` if unmapped. |
| `HasVehicle(const std::string& vehicleId) const` | `bool` | Returns `true` if the vehicle ID is registered in the mapper. |
| `HasNode(uint32_t nodeId) const` | `bool` | Returns `true` if the ns-3 node ID is registered in the mapper. |
| `GetSize() const` | `uint32_t` | Returns the total count of registered vehicle-to-node mappings. |
| `GetAllNodes() const` | `NodeContainer` | Returns an ns-3 `NodeContainer` populated with all mapped vehicle nodes. |
| `GetAllVehicleIds() const` | `std::vector<std::string>` | Returns an alphabetically sorted vector of all registered vehicle IDs. |
| `Clear()` | `void` | Clears all registered bidirectional mappings. |

---

### 7.3 `V2xMobilityImporter` (`model/v2x-mobility-importer.h`)
Converts parsed SUMO trajectories into native ns-3 `WaypointMobilityModel` trajectories, manages departure/arrival lifecycle, and triggers periodic position sampling.

| Method Signature | Return Type | Description |
| :--- | :--- | :--- |
| `Install(Ptr<SumoFcdParser> parser, Ptr<VehicleNodeMapper> mapper)` | `void` | Populates waypoints into `WaypointMobilityModel` for all mapped nodes and schedules departure/arrival lifecycle events. |
| `Install(const SumoFcdParser& parser, NodeContainer nodes)` | `void` | Convenience overload installing parsed waypoints directly onto an ordered `NodeContainer`. |
| `GetFirstSeen(const std::string& vehicleId) const` | `Time` | Returns the scheduled activation timestamp (departure time) for the vehicle. |
| `GetLastSeen(const std::string& vehicleId) const` | `Time` | Returns the scheduled deactivation timestamp (arrival time) for the vehicle. |
| `IsVehicleActive(const std::string& vehicleId) const` | `bool` | Returns `true` if the vehicle is currently active within the ongoing simulation timeframe. |
| `SetMobilityStep(Time step)` | `void` | Sets the periodic position sampling interval (supported range: 10 ms to 2 s). |
| `GetMobilityStep() const` | `Time` | Returns the configured periodic position sampling interval. |

#### Trace Sources
- **`VehicleActivated`**: `TracedCallback<std::string, uint32_t>` — Fired when a vehicle enters the road network (`vehicleId`, `nodeId`).
- **`VehicleDeactivated`**: `TracedCallback<std::string, uint32_t>` — Fired when a vehicle completes its route and departs (`vehicleId`, `nodeId`).
- **`VehiclePosition`**: `TracedCallback<std::string, uint32_t, Vector, double>` — Fired periodically at every `MobilityStep` with current 3D position and instantaneous speed (`vehicleId`, `nodeId`, `position`, `speedMps`).

---

### 7.4 `RsuPlacementHelper` (`helper/rsu-placement-helper.h`)
Helper class for allocating, positioning, and configuring fixed Roadside Unit (RSU) nodes across network topologies.

| Method Signature | Return Type | Description |
| :--- | :--- | :--- |
| `CreateRsus(uint32_t numRsus)` | `NodeContainer` | Instantiates and returns a `NodeContainer` containing `numRsus` roadside unit nodes. |
| `InstallPositions(NodeContainer rsus, const std::vector<Vector>& positions)` | `void` | Assigns fixed 3D coordinates to RSU nodes from an explicit vector of positions using `ConstantPositionMobilityModel`. |
| `InstallGridSpacing(NodeContainer rsus, double startX, double startY, double deltaX, double deltaY, uint32_t gridWidth)` | `void` | Deploys RSUs in a uniform 2D grid lattice pattern starting at `(startX, startY)` with column spacing `deltaX` and row spacing `deltaY`. |
| `InstallJunctionList(NodeContainer rsus, const std::vector<Vector>& junctions)` | `void` | Deploys RSUs at specified road junction / intersection coordinates extracted from SUMO `.net.xml`. |
| `SetRsuHeight(double heightM)` | `void` | Configures the elevated mast height in meters for deployed RSUs (default: 5.0 m). |
| `GetRsuHeight() const` | `double` | Returns the configured RSU mast height in meters. |

---

### 7.5 `RsuSelectionPolicy` & `NearestRsuSelectionPolicy` (`model/rsu-selection-policy.h`)
Mechanism / policy split interface for vehicle-to-infrastructure (V2I) association and handover.

| Class & Method Signature | Return Type | Description |
| :--- | :--- | :--- |
| `virtual RsuSelectionDecision RsuSelectionPolicy::Select(const RsuSelectionState& state) = 0` | `RsuSelectionDecision` | Pure virtual interface allowing pluggable V2I selection algorithms (e.g. RL, QoS, load-balanced). |
| `NearestRsuSelectionPolicy::NearestRsuSelectionPolicy()` | Constructor | Initializes deterministic nearest-RSU selection policy. |
| `RsuSelectionDecision NearestRsuSelectionPolicy::Select(const RsuSelectionState& state) override` | `RsuSelectionDecision` | Evaluates Euclidean distances to candidate RSUs in $O(M)$ time and selects the geometrically closest RSU. |

---

### 7.6 `V2xScenarioHelper` (`helper/v2x-scenario-helper.h`)
High-level scenario orchestrator integrating trace ingestion, node lifecycle, 802.11p ad-hoc networking, RSU association, and 3-tier CSV telemetry.

| Method Signature | Return Type | Description |
| :--- | :--- | :--- |
| `LoadFcd(const std::string& filename)` | `bool` | Loads and parses a SUMO FCD XML trajectory file using the internal parser. |
| `CreateVehicleNodes()` | `NodeContainer` | Creates ns-3 nodes for all vehicles in the loaded trace and establishes deterministic mapping. |
| `CreateRsuNodes(uint32_t numRsus)` | `NodeContainer` | Allocates and deploys fixed RSU nodes using configured placement strategy. |
| `SetRsuPositions(const std::vector<Vector>& positions)` | `void` | Sets explicit coordinates for RSU nodes. |
| `SetRsuGridPlacement(double startX, double startY, double deltaX, double deltaY, uint32_t gridWidth)` | `void` | Configures regular 2D grid spacing for RSU deployment. |
| `SetRsuJunctionPlacement(const std::vector<Vector>& junctions)` | `void` | Configures junction-based coordinates for RSU deployment. |
| `InstallMobility()` | `void` | Attaches `WaypointMobilityModel` to vehicle nodes and schedules lifecycle callbacks. |
| `InstallExampleNetworking()` | `void` | Installs 802.11p/adhoc Wi-Fi devices, IPv4 network stacks, and periodic V2V/V2I beacon applications. |
| `EnableTracing(const std::string& prefix)` | `void` | Opens and configures 3-tier CSV telemetry streams (`mobility.csv`, `network.csv`, `common.csv`). |
| `CloseTracing()` | `void` | Flushes and closes all telemetry file streams. |
| `SetRsuSelectionPolicy(Ptr<RsuSelectionPolicy> policy)` | `void` | Injects a custom RSU selection policy implementation. |
| `AssignStreams(int64_t stream)` | `int64_t` | Assigns deterministic stream numbers to internal random variable streams (e.g., application jitter). |
| `GetVehicleNodes() const` | `NodeContainer` | Returns the container of all instantiated vehicle nodes. |
| `GetRsuNodes() const` | `NodeContainer` | Returns the container of all instantiated RSU nodes. |
| `GetParser() const` | `Ptr<SumoFcdParser>` | Returns a pointer to the internal `SumoFcdParser` instance. |
| `GetMapper() const` | `Ptr<VehicleNodeMapper>` | Returns a pointer to the internal `VehicleNodeMapper` instance. |
| `GetImporter() const` | `Ptr<V2xMobilityImporter>` | Returns a pointer to the internal `V2xMobilityImporter` instance. |
| `LogMobilitySample(std::string vehicleId, uint32_t nodeId, Vector position)` | `void` | Trace sink logging periodic sampled position and speed into `mobility.csv`. |
| `LogVehicleActivated(std::string vehicleId, uint32_t nodeId)` | `void` | Trace sink logging vehicle departure events into common event logs. |
| `LogVehicleDeactivated(std::string vehicleId, uint32_t nodeId)` | `void` | Trace sink logging vehicle arrival/exit events into common event logs. |
| `LogRsuAssociation(std::string vehicleId, uint32_t oldRsuNodeId, uint32_t newRsuNodeId)` | `void` | Logs RSU association and handover events. |

---

### 7.7 Data Structures & Common Types
Specialized structures defined in `model/vehicle-waypoint.h`, `model/rsu-selection-policy.h`, and `model/v2x-common-types.h`:

- **`VehicleWaypoint`**: Single trajectory point from SUMO FCD (`timestamp`, `position` Vector, `speedMps`, `angleDeg`, `laneId`, `posOnLane`).
- **`RsuSelectionState`**: Input state passed to `RsuSelectionPolicy::Select` (`vehicleNodeId`, `vehiclePosition`, `candidateRsus`).
- **`RsuSelectionDecision`**: Association outcome returned by `RsuSelectionPolicy` (`selectedRsu`, `distanceM`, `valid`).
- **`NetworkState`**: FutureNet Common Guidelines 4.1 telemetry representation (`timestamp`, `nodeId`, `oneWayDelayMs`, `throughputMbps`, `packetLossRatio`, `availableBandwidthMbps`, `queuePackets`, `nodeLoad`, `energyJ`, `energyValid`).
- **`LinkState`**: FutureNet Common Guidelines 4.2 link state (`sourceNodeId`, `destinationNodeId`, `available`, `dataRateMbps`, `propagationDelayMs`, `lossRatio`).
- **`MobilityState`**: FutureNet Common Guidelines 4.3 kinematic state (`nodeId`, `positionM`, `velocityMps`, `timestamp`).
- **`FutureNetTrafficClass`**: FutureNet Common Guidelines 4.4 traffic classification (`priority`, `importance`, `deadline`, `flowId`).

---

### 7.8 Python Utilities & Tooling (`tools/`)

| Script | Primary Functionality |
| :--- | :--- |
| **`generate_sumo_scenario.py`** | Reproducibly generates 100+ vehicle road networks and FCD trajectories using native Eclipse SUMO tools (`netgenerate`, `randomTrips.py`, `duarouter`, `sumo`). |
| **`fcd_generator.py`** | Synthesizes standard-compliant SUMO FCD XML trajectories with configurable vehicle counts, durations, and spatial bounds for stress testing. |
| **`validate_v2x.py`** | Runs automated multi-criteria validation suite: **V1** (spatial alignment $< 0.01$ m), **V2** (kinematics & displacement), and **V3** (network decoupling byte-level identity). |
| **`osm_to_sumo.py`** | Automates conversion from OpenStreetMap (`.osm`) files to SUMO road networks (`.net.xml`) via `netconvert`. |
| **`live_traci_adapter.py`** | [Prototype] TraCI TCP socket adapter demonstrating co-simulation bridging and live synchronized vehicle stepping. |

---

## 8. Known Limitations & Extension Points (QG-10)
- **Offline Trajectory Scope**: High-speed offline trajectory playback is supported via `WaypointMobilityModel`. Real-time bidirectional physics changes require live TraCI co-simulation (`tools/live_traci_adapter.py`).
- **Antenna Patterns**: Default wireless uses isotropic antenna propagation. 3-D directional antenna models (e.g. from Family A) can be plugged in via standard ns-3 helpers.
- **External AI Integration**: RSU association and offloading can be driven by M10 Agent or external reinforcement learning by subclassing `RsuSelectionPolicy`.
