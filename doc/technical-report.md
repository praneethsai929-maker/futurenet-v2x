# FutureNet M09 Technical Report & Design Decisions

## 1. Design Decisions & Guidelines Compliance

### 1.1 Non-Intrusive Integration (Quality Gate QG-07)
All module code lives entirely in `contrib/futurenet-v2x/`. No modifications or additions were made to core ns-3 source files (`src/`), fulfilling the modularity and non-intrusiveness mandate of Quality Gate QG-07 and ns-3 contrib conventions.

### 1.2 Mechanism / Policy Separation (Common Guidelines 4.6)
Vehicle-to-Infrastructure (V2I) association and handover logic is strictly decoupled from the low-level wireless transmission pipeline via the `RsuSelectionPolicy` abstract interface.
- The base class presents a pure virtual interface `Select(const RsuSelectionState& state)`.
- The default implementation, `NearestRsuSelectionPolicy`, selects the geometrically closest RSU based on 3D Euclidean distance.
- This design guarantees that downstream modules (e.g., M10 Agent, RL-based edge offloading, or QoS handover managers) can inject custom policies via `V2xScenarioHelper::SetRsuSelectionPolicy()` without altering the scenario orchestrator or Wi-Fi stack.

### 1.3 Complexity Analysis of Nearest-RSU Association: $\mathcal{O}(N \times R)$
In vehicular deployments, active vehicles $N$ periodically determine their closest Roadside Unit among $R$ deployed infrastructure nodes. At each mobility sampling step:
$$\text{Per-tick Complexity} = \mathcal{O}(N \times R)$$
In real-world V2X deployments, $R$ is fixed and moderate ($R \in [1, 20]$), making the per-tick complexity strictly linear in active vehicle count: $\mathcal{O}(N)$.

In our benchmark tests with 112 vehicles and 4 RSUs:
- Total distance calculations per 100 ms tick: $112 \times 4 = 448$ evaluations.
- Execution time: $< 15\,\mu\text{s}$ per evaluation step on modern x86/ARM processors.
- **Why this design is optimal**: Introducing dynamic spatial partitioning trees (such as quadtrees, BVH, or $k$-d trees) introduces dynamic node pointer reallocation and rebalancing overhead on every vehicle movement, which incurs higher cache-miss penalties and memory overhead than a compact array scan when $R \le 50$. Hence, the direct $\mathcal{O}(N \times R)$ scan is both computationally negligible and robust.

### 1.4 Randomness, Seeding & Reproducibility (CG-09, CG-10)
All pseudo-random variables—specifically application start jitter to prevent synchronization collisions—utilize `ns3::UniformRandomVariable` bound to ns-3's global stream management system via `V2xScenarioHelper::AssignStreams()`. Test `T5` verifies that identical seeds and stream allocations generate bit-for-bit identical network transmission events and CSV logs.

### 1.5 Deactivation and Safe Channel Exit
When vehicles reach the end of their SUMO trajectory (`LastSeen`), abrupt transceiver shutdown (`WifiPhy::SetOffMode`) risks causing zero-pointer assertions if an MPDU transmission event is pending in the MAC event queue. To safely leave the channel:
- Sockets and scheduled transmission events are closed/canceled.
- The node's position is relocated far outside radio propagation range ($(-100000, -100000, -1000)$ m), safely ensuring deactivated nodes cease transmitting and receiving without disrupting active simulation events.

---

## 2. Telemetry, Loss Formulations & Network Metrics

### 2.1 Network Metrics & Loss Calculation
The FutureNet telemetry engine logs detailed per-event records into `network.csv` and summary statistics into FlowMonitor XML:
1. **Unicast V2I Packet Delivery Ratio (PDR)**:
   $$\text{PDR}_{\text{V2I}} = \frac{\text{Rx}_{\text{V2I}}}{\text{Tx}_{\text{V2I}}} \times 100\%$$
   $$\text{Lost Packets}_{\text{V2I}} = \max\left(0, \text{Tx}_{\text{V2I}} - \text{Rx}_{\text{V2I}}\right)$$
2. **Sequence Gap Loss Tracking**:
   Every packet embeds an ns-3 `SeqTsHeader` carrying a sequential message index and transmission timestamp. Receivers maintain the last observed sequence number $seq_{\text{last}}$ for each transmitter-receiver link $(u, v)$:
   $$\text{Gap Loss} = seq_{\text{current}} - (seq_{\text{last}} + 1) \quad (\text{if } seq_{\text{current}} > seq_{\text{last}} + 1)$$
   This captures in-transit drops across link handovers and fading events.
3. **V2V Broadcast Fan-Out**:
   V2V beacons are transmitted over the subnet directed broadcast address (`10.1.255.255` on port `9998`). Because each broadcast transmission is received by all nodes within wireless transmission range ($k$ neighbors), the aggregate receive count satisfies $\text{Rx}_{\text{V2V}} = \sum_{i=1}^{\text{Tx}} k_i$, demonstrating typical ad-hoc fan-out.

### 2.2 Network & Infrastructure Parameters
- **V2I Application Port**: `9999` (`V2xScenarioHelper::V2I_PORT`)
- **V2V Application Port**: `9998` (`V2xScenarioHelper::V2V_PORT`)
- **Subnet Configuration**: `10.1.0.0/16` with dynamically computed directed broadcast address
- **Default RSU Mast Height**: `5.0` meters (`V2xScenarioHelper::DEFAULT_RSU_HEIGHT_M`)
- **Default Wi-Fi TX Power**: `21.5` dBm (802.11p / 5.9 GHz DSRC ad-hoc)

---

## 3. Deliverable D4 Scenario Mapping Table

| Requirement Target | Module Binary | Source Code | Command-Line Execution | Description & Key Features |
| :--- | :--- | :--- | :--- | :--- |
| **`example-basic`** | `example-grid` | `examples/example-grid.cc` | `./ns3 run example-grid` | Baseline Manhattan grid scenario (16 vehicles, 4 RSUs), regular lattice placement, periodic V2V/V2I beaconing, and 3-tier CSV generation. |
| **`example-advanced`** | `example-map` | `examples/example-map.cc` | `./ns3 run example-map` | Ingests arbitrary user-supplied FCD traces without code changes (AC-01), supports coordinate scaling/offset, and integrates FlowMonitor XML serialization. |

### Command-Line Arguments Reference

Both example executables support rich command-line parameters:
- `--fcdTrace=<path>`: Path to SUMO Floating Car Data XML trajectory file.
- `--numRsus=<uint>`: Number of roadside units to instantiate (default: `4`).
- `--simTime=<double>`: Simulation duration in seconds.
- `--mobilityStep=<double>`: Kinematic sampling interval in milliseconds (default: `100.0` ms).
- `--v2xInterval=<double>`: Periodic V2X beacon message interval in milliseconds (default: `100.0` ms).
- `--v2xBytes=<uint>`: Periodic message payload in bytes (default: `300` B).
- `--txPower=<double>`: Transmission power in dBm (default: `21.5` dBm).
- `--rsuPositions="x,y;x,y"`: Explicit semicolon-separated RSU coordinate list.
- `--gridSpacing=<double>`: Grid lattice spacing in meters (default: `200.0` m).
- `--gridWidth=<uint>`: Number of RSUs per grid row (default: `2`).
- `--tracePrefix=<str>`: Prefix name for output CSV and XML telemetry files.

---

## 4. Multi-Criteria Validation Results (V1–V4)

The automated validation suite (`tools/validate_v2x.py`) executes tests across spatial, kinematic, network, and scale criteria:

```
==========================================================
 FutureNet M09 Multi-Criteria Validation Suite Run Log
==========================================================
 ns-3 Version : 3.46.1
 SUMO Version : Eclipse SUMO sumo 1.28.0
 RNG Seed     : Default deterministic (AssignStreams tested)
==========================================================
```

### 4.1 V1: Spatial Ground-Truth Alignment
- Evaluates coordinate alignment between ns-3 sampled node positions and SUMO ground truth.
- **Results**: Verified **2,852 sampled points** across vehicle trajectories against SUMO FCD records.
- **Discrepancy**: Maximum spatial discrepancy = **0.000000 m** (Tolerance: $< 0.01$ m). **Status: PASS**.

### 4.2 V2: Kinematic Trend Validation
- Evaluates continuous forward displacement and speed interpolation over time.
- **Results**: Verified monotonic continuous forward movement across **110/110 moving trajectories** (112 total vehicles). **Status: PASS**.

### 4.3 V3: Controlled Variation & Network Decoupling
- Simulates identical mobility traces under two differing network configurations over 8.0 seconds:
  - Config 1: 100 ms message interval
  - Config 2: 500 ms message interval
- **Mobility Invariance**: Mobility CSV output is **100% byte-for-byte identical** between Config 1 and Config 2.
- **Network Decoupling Metrics**:
  - Config 1 (100 ms): $\text{Tx} = 6,896$ (V2I=3,448, V2V=3,448), $\text{Rx} = 15,901$ (V2I=1,020, V2V=14,881), $\text{V2I PDR} = 29.58\%$ ($\text{Lost} = 2,428$), $\text{Delay} = 1.12\text{ ms}$, $\text{Total Loss Gaps} = 105$.
  - Config 2 (500 ms): $\text{Tx} = 1,456$ (V2I=728, V2V=728), $\text{Rx} = 3,434$ (V2I=220, V2V=3,214), $\text{V2I PDR} = 30.22\%$ ($\text{Lost} = 508$), $\text{Delay} = 3.29\text{ ms}$, $\text{Total Loss Gaps} = 6$.
- **Status: PASS**.

### 4.4 V3.1: Controlled Variation & TxPower Sweep (Coverage Sanity)
- Evaluates transmission power sensitivity on the 112-vehicle topology:
  - Config A (21.5 dBm): $\text{Tx} = 1,336$, $\text{Rx} = 400$, $\text{V2I PDR} = 29.94\%$
  - Config B (30.0 dBm): $\text{Tx} = 1,336$, $\text{Rx} = 966$, $\text{V2I PDR} = 72.31\%$
- **Range Loss Validation**: Confirms the physical wireless range model: increasing transmit power expands coverage area, raising V2I PDR from $29.94\%$ to $72.31\%$ while mobility records remain 100% identical. **Status: PASS**.

### 4.5 V4: Boundary & Scale Extreme Cases
1. **High-Frequency Sampling** (`MobilityStep = 10 ms`, 100 Hz): Successfully generated 613 telemetry records safely.
2. **Sparse Sampling** (`MobilityStep = 2000 ms`, 0.5 Hz): Successfully generated 55 sparse records without interpolation error.
3. **Zero RSUs** (`NumRsus = 0`): Pure ad-hoc V2V broadcast executed without null pointer exceptions.
4. **Scale Boundary** (100+ vehicles): Completed safely with 526 mobility records. **Status: PASS**.

---

## 5. End-to-End Demonstration Scenario ($\ge 100$ Vehicles, 4 RSUs)

The full-scale benchmark scenario evaluates 112 vehicles and 4 RSUs over a 10.0-second simulation:
- **Vehicle Count**: 112 vehicles (generated via native Eclipse SUMO 1.28.0 using `tools/generate_sumo_scenario.py`).
- **RSU Infrastructure**: 4 RSUs deployed in a $2 \times 2$ grid with 200 m lattice spacing.
- **Simulation Duration**: 10.0 seconds full execution.

### Empirical Network Performance
- **Total Packets Transmitted**: **10,516 packets** (5,258 V2I unicast, 5,258 V2V broadcast)
- **Total Packets Received**: **29,529 packets** (1,492 V2I unicast, 28,037 V2V broadcast fan-out)
- **V2I Packet Delivery Ratio (PDR)**: **28.38%** (3,766 lost due to vehicle positions outside 200 m RSU coverage range)
- **Average One-Way Delay**: **1.22 ms**
- **Average Throughput**: **0.0240 Mbps**
- **Total Sequence Gaps**: **416** gaps across link boundary transitions.
- **Trace Output Deliverables**: Representative trimmed samples (~45–80 KB) committed under `doc/raw-validation-*` per guideline 14.3. The full trace is generated on-demand via `tools/validate_v2x.py`.

---

## 6. Preprocessing Instructions for SUMO & OSM (AC-05, FR-09)

To convert arbitrary real-world road networks from OpenStreetMap into ns-3 simulations:

1. **Obtain OpenStreetMap Data**:
   Download `.osm` extract for your desired geographic area (e.g. `downtown.osm`).
2. **Convert OSM to SUMO Road Network via `osm_to_sumo.py`**:
   ```bash
   python3 contrib/futurenet-v2x/tools/osm_to_sumo.py downtown.osm -o downtown
   ```
   This invokes `netconvert` with topology cleanup, roundabout inference, and traffic light synthesis.
3. **Generate Traffic Demand & Routes**:
   ```bash
   randomTrips.py -n downtown.net.xml -e 100 -p 0.5 -o trips.trips.xml --validate
   duarouter -n downtown.net.xml --route-files trips.trips.xml -o downtown.rou.xml
   ```
4. **Export Floating Car Data (FCD)**:
   ```bash
   sumo -n downtown.net.xml -r downtown.rou.xml --begin 0 --end 60 --step-length 0.2 --fcd-output downtown-fcd.xml --fcd-output.geo false
   ```
5. **Run in ns-3 without Source Code Modification (AC-01)**:
   ```bash
   ./ns3 run "example-map --fcdTrace=downtown-fcd.xml --simTime=60 --tracePrefix=downtown-run"
   ```

---

## 7. Known Limitations & Scope (Quality Gate QG-10)

1. **Offline Playback vs Live Feedback**:
   `futurenet-v2x` ingests SUMO FCD trajectories offline into ns-3's `WaypointMobilityModel`. Dynamic closed-loop driver reactions (e.g., dynamic re-routing or emergency braking caused by V2X packet reception) require live TraCI co-simulation.
2. **Live TraCI Adapter Prototype**:
   `tools/live_traci_adapter.py` is included as a research prototype demonstrating live socket connectivity and vehicle stepping. Offline FCD ingestion remains the primary high-performance production engine.
3. **Propagation Range Boundaries**:
   Vehicles that travel beyond RSU wireless coverage experience expected propagation loss (reflected in V2I PDR and sequence gaps). Adding more RSUs or raising TX power expands coverage.
