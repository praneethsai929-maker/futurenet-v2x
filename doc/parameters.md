# FutureNet M09 Parameter & Interface Reference Table

Conforming to Section 5 of the SRS and Section 5 of the FutureNet Common Guidelines. All units are explicit.

| Parameter Name | C++ Type | Default Value | Supported Range | Unit | Class / Attribute Location | Description |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`MobilityStep`** | `Time` | `100 ms` | `10 ms` – `2 s` | ms / s | `V2xMobilityImporter`<br>`V2xScenarioHelper` | Sampling interval for periodic vehicle position tracing and association updates |
| **`NumRsus`** | `uint32_t` | `4` | `0` – `100` | integer | `V2xScenarioHelper` | Number of fixed Roadside Units to instantiate in the scenario |
| **`CoordinateScale`** | `double` | `1.0` | `1e-6` – `1e6` | unitless | `SumoFcdParser`<br>`V2xScenarioHelper` | Multiplier applied to raw SUMO coordinates |
| **`CoordinateOffsetX`** | `double` | `0.0` | $(-\infty, \infty)$ | meters | `SumoFcdParser`<br>`V2xScenarioHelper` | Translation offset added to X coordinates |
| **`CoordinateOffsetY`** | `double` | `0.0` | $(-\infty, \infty)$ | meters | `SumoFcdParser`<br>`V2xScenarioHelper` | Translation offset added to Y coordinates |
| **`V2xMessageBytes`** | `uint32_t` | `300 B` | `64 B` – `1500 B` | bytes | `V2xScenarioHelper` | Payload size of periodic V2X messages |
| **`V2xMessageInterval`** | `Time` | `100 ms` | `10 ms` – `10 s` | ms / s | `V2xScenarioHelper` | Transmission period between consecutive V2X beacons |
| **`ApplicationStartJitter`** | `Time` | `100 ms` | `0` – `10 s` | ms / s | `V2xScenarioHelper` | Maximum uniform random start jitter to desynchronize packet broadcasts |
| **`TraCIPort`** | `uint16_t` | `8813` | `1024` – `65535` | port # | `tools/live_traci_adapter.py` | TCP socket port for TraCI co-simulation |
| **`FcdInputFile`** | `std::string` | `external` | Valid file path | string | `CommandLine` / `LoadFcd` | Input SUMO Floating Car Data XML trajectory file |

---

## Telemetry Schemas

### 1. Mobility CSV
`time_s,vehicle_id,node_id,x_m,y_m,z_m,speed_mps,active`

### 2. Network CSV
`time_s,vehicle_id,peer_id,event,delay_ms,throughput_mbps,loss`

### 3. Common Guidelines Event Log (Section 4.5)
`time_s,module,event,node_id,peer_id,flow_id,value1,value2,unit,detail`
