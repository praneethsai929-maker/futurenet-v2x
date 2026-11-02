FutureNet V2X Mobility Integration Module
=========================================

.. include:: replace.txt
.. highlight:: cpp

The ``futurenet-v2x`` module for |ns3| provides comprehensive vehicular mobility and network simulation capabilities with offline Floating Car Data (FCD) trajectory ingestion from Eclipse SUMO, deterministic vehicle-to-node mapping, Roadside Unit (RSU) infrastructure, and a decoupled mechanism/policy architecture for Vehicle-to-Infrastructure (V2I) association.

The source code for this module lives entirely in ``contrib/futurenet-v2x`` and makes no modifications to core |ns3| source code (satisfying Quality Gate QG-07).

Scope and Limitations
---------------------

Scope
~~~~~
* Ingestion of microscopic vehicular mobility traces exported by Eclipse SUMO in Floating Car Data (FCD) XML format.
* Dynamic vehicle node lifecycle management with automated departure (activation) and arrival (deactivation) scheduling.
* Safe channel exit ensuring deactivated vehicles cease transmission and reception without disrupting active MAC/PHY queues.
* Deployment of stationary Roadside Units (RSUs) via explicit coordinates, regular grid spacing, or road junction coordinates.
* Decoupled V2I association policy interface allowing interchangeable algorithmic policies (e.g. nearest-RSU, QoS, load-balancing).
* High-performance periodic V2V ad-hoc broadcast (port 9998) and unicast V2I (port 9999) vehicular communications over IEEE 802.11p.
* Comprehensive three-tier CSV telemetry logging conforming to FutureNet Common Guidelines.

Limitations
~~~~~~~~~~~
* **Offline Trajectory Scope**: Mobility is ingested from pre-computed SUMO FCD traces into |ns3| ``WaypointMobilityModel``. Real-time bidirectional physics feedback into vehicle driver behavior is outside the scope of M09 (a live TraCI adapter prototype is provided in ``tools/live_traci_adapter.py``).
* **Propagation Range Boundaries**: Vehicles traveling beyond the radio communication range of RSUs experience physical wireless propagation loss, as naturally modeled by the wireless channel.
* **Isotropic Antennas**: Transceivers use isotropic antenna models by default; directional antenna patterns can be added via standard |ns3| antenna helpers.

Model Description
-----------------

The model is structured into modular components:

1. **SumoFcdParser**: An :math:`\mathcal{O}(N)` stream XML parser reading SUMO FCD output with coordinate scaling (``CoordinateScale``) and offsets (``CoordinateOffsetX``, ``CoordinateOffsetY``).
2. **VehicleNodeMapper**: Maintains a deterministic, reproducible bidirectional map between SUMO vehicle ID strings and |ns3| node IDs.
3. **V2xMobilityImporter**: Translates parsed waypoints into |ns3| ``WaypointMobilityModel`` instances with ``InitialPositionIsWaypoint=true``, managing departure and arrival events.
4. **RsuPlacementHelper**: Deploys RSU infrastructure nodes with mast height elevation.
5. **RsuSelectionPolicy**: Abstract policy interface with default ``NearestRsuSelectionPolicy`` calculating Euclidean proximity in :math:`\mathcal{O}(N \times R)` time without quadratic neighbor scans.
6. **V2xScenarioHelper**: High-level orchestrator connecting parser, mapper, mobility, 802.11p networking, and telemetry tracing.

Usage
-----

Helpers
~~~~~~~

The primary entry point is ``V2xScenarioHelper``:

.. code-block:: cpp

   Ptr<V2xScenarioHelper> helper = CreateObject<V2xScenarioHelper>();
   helper->SetAttribute("MobilityStep", TimeValue(MilliSeconds(100)));
   helper->SetAttribute("V2xMessageInterval", TimeValue(MilliSeconds(100)));
   helper->SetAttribute("TxPower", DoubleValue(21.5));
   helper->SetRsuGridPlacement(50.0, 50.0, 200.0, 200.0, 2);

   helper->LoadFcd("path/to/trace.fcd.xml");
   NodeContainer vehicles = helper->CreateVehicleNodes();
   NodeContainer rsus = helper->CreateRsuNodes(4);

   helper->InstallMobility();
   helper->InstallExampleNetworking();
   helper->EnableTracing("output-prefix");

   Simulator::Stop(Seconds(10.0));
   Simulator::Run();
   Simulator::Destroy();

Attributes
~~~~~~~~~~

* **MobilityStep**: Kinematic position sampling interval (supported: 10 ms to 2 s; default: 100 ms).
* **NumRsus**: Number of roadside unit infrastructure nodes to instantiate (default: 4).
* **CoordinateScale**: Scale multiplier applied to raw SUMO coordinates (default: 1.0).
* **CoordinateOffsetX**: Offset in meters added to vehicle X coordinates (default: 0.0 m).
* **CoordinateOffsetY**: Offset in meters added to vehicle Y coordinates (default: 0.0 m).
* **V2xMessageBytes**: Application payload size for periodic V2X messages (default: 300 bytes).
* **V2xMessageInterval**: Transmission period between periodic beacons (default: 100 ms).
* **ApplicationStartJitter**: Maximum uniform random delay to desynchronize initial broadcasts (default: 100 ms).
* **TxPower**: IEEE 802.11p transmission power in dBm (default: 21.5 dBm).
* **RsuHeight**: Mast elevation height for RSU nodes (default: 5.0 m).

Traces
~~~~~~

* **VehicleActivated**: Fired when a vehicle enters the road network (``vehicleId``, ``nodeId``).
* **VehicleDeactivated**: Fired when a vehicle completes its trajectory and exits (``vehicleId``, ``nodeId``).
* **VehiclePosition**: Fired periodically at every ``MobilityStep`` with position and instantaneous speed.

Examples and Tests
------------------

Examples
~~~~~~~~

* **example-grid** (Deliverable D4 ``example-basic``): Baseline grid scenario with 16 vehicles and 4 RSUs. Demonstrates periodic V2V/V2I beaconing and 3-tier CSV generation.
* **example-map** (Deliverable D4 ``example-advanced``): Advanced scenario ingesting arbitrary FCD traces without code modifications (AC-01), with FlowMonitor serialization and controlled variation.

Tests
~~~~~

The module includes 16 automated unit tests covering all functional requirements:

.. code-block:: bash

   ./test.py -s futurenet-v2x

Validation
----------

The multi-criteria validation suite (``tools/validate_v2x.py``) verifies:

1. **V1 (Spatial Alignment)**: Validates sampled positions against SUMO ground truth (0.000000 m error observed, < 0.01 m tolerance).
2. **V2 (Kinematic Trend)**: Verifies continuous forward displacement across all moving trajectories.
3. **V3 (Controlled Variation)**: Verifies 100% byte-for-byte identical mobility CSV across differing packet intervals (100 ms vs 500 ms) while network metrics decouple.
4. **V4 (Boundary & Extreme Cases)**: Evaluates high sampling rates (10 ms / 100 Hz), sparse sampling (2000 ms), 0-RSU ad-hoc mode, and 100+ vehicle scale.

References
----------

1. IEEE Std 802.11p-2010: Wireless Access in Vehicular Environments (WAVE).
2. Eclipse SUMO: Simulation of Urban MObility (https://eclipse.dev/sumo/).
3. FutureNet M09 Software Requirements Specification (SRS).
