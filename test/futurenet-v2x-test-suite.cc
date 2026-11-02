#include "ns3/test.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "../model/sumo-fcd-parser.h"
#include "../model/vehicle-node-mapper.h"
#include "../model/v2x-mobility-importer.h"
#include "../model/rsu-selection-policy.h"
#include "../helper/rsu-placement-helper.h"
#include "../helper/v2x-scenario-helper.h"

#include <cmath>
#include <fstream>
#include <string>

using namespace ns3;

/**
 * @defgroup futurenet-v2x-tests Tests for futurenet-v2x
 * @ingroup futurenet-v2x
 */

namespace
{

std::string CreateTestFcdXml(const std::string& filename)
{
    std::ofstream out(filename);
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<fcd-export xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">\n"
        << "  <timestep time=\"0.00\">\n"
        << "    <vehicle id=\"veh0\" x=\"10.00\" y=\"20.00\" z=\"0.00\" speed=\"10.00\" angle=\"90.00\" pos=\"10.00\" lane=\"lane0\"/>\n"
        << "  </timestep>\n"
        << "  <timestep time=\"0.50\">\n"
        << "    <vehicle id=\"veh0\" x=\"15.00\" y=\"20.00\" z=\"0.00\" speed=\"10.00\" angle=\"90.00\" pos=\"15.00\" lane=\"lane0\"/>\n"
        << "    <vehicle id=\"veh1\" x=\"50.00\" y=\"30.00\" z=\"0.00\" speed=\"15.00\" angle=\"0.00\" pos=\"0.00\" lane=\"lane1\"/>\n"
        << "  </timestep>\n"
        << "  <timestep time=\"1.00\">\n"
        << "    <vehicle id=\"veh0\" x=\"20.00\" y=\"20.00\" z=\"0.00\" speed=\"10.00\" angle=\"90.00\" pos=\"20.00\" lane=\"lane0\"/>\n"
        << "    <vehicle id=\"veh1\" x=\"50.00\" y=\"37.50\" z=\"0.00\" speed=\"15.00\" angle=\"0.00\" pos=\"7.50\" lane=\"lane1\"/>\n"
        << "    <vehicle id=\"veh2\" x=\"100.00\" y=\"100.00\" z=\"0.00\" speed=\"20.00\" angle=\"45.00\" pos=\"5.00\" lane=\"lane2\"/>\n"
        << "  </timestep>\n"
        << "  <timestep time=\"2.00\">\n"
        << "    <vehicle id=\"veh0\" x=\"30.00\" y=\"20.00\" z=\"0.00\" speed=\"10.00\" angle=\"90.00\" pos=\"30.00\" lane=\"lane0\"/>\n"
        << "    <vehicle id=\"veh1\" x=\"50.00\" y=\"52.50\" z=\"0.00\" speed=\"15.00\" angle=\"0.00\" pos=\"22.50\" lane=\"lane1\"/>\n"
        << "  </timestep>\n"
        << "</fcd-export>\n";
    out.close();
    return filename;
}

} // namespace

// ===========================================================================
// T01: Parser reads known vehicle count and IDs
// ===========================================================================
class V2xTestT01ParserCountIds : public TestCase
{
public:
    V2xTestT01ParserCountIds() : TestCase("T01: Parser reads known vehicle count and IDs") {}
    void DoRun() override
    {
        std::string testFile = CreateTempDirFilename("v2x-t01.xml");
        CreateTestFcdXml(testFile);

        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();
        bool ok = parser->Load(testFile);
        NS_TEST_ASSERT_MSG_EQ(ok, true, "Parser failed to load valid test FCD XML");

        NS_TEST_ASSERT_MSG_EQ(parser->GetVehicleCount(), 3, "Expected exactly 3 vehicles");

        std::vector<std::string> ids = parser->GetVehicleIds();
        NS_TEST_ASSERT_MSG_EQ(ids.size(), 3, "Vector size must be 3");
        NS_TEST_ASSERT_MSG_EQ(ids[0], "veh0", "First vehicle must be veh0");
        NS_TEST_ASSERT_MSG_EQ(ids[1], "veh1", "Second vehicle must be veh1");
        NS_TEST_ASSERT_MSG_EQ(ids[2], "veh2", "Third vehicle must be veh2");
    }
};

// ===========================================================================
// T02: Known FCD waypoint maps to expected ns-3 position
// ===========================================================================
class V2xTestT02WaypointPosition : public TestCase
{
public:
    V2xTestT02WaypointPosition() : TestCase("T02: Known FCD waypoint maps to expected ns-3 position") {}
    void DoRun() override
    {
        std::string testFile = CreateTempDirFilename("v2x-t02.xml");
        CreateTestFcdXml(testFile);

        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();
        parser->Load(testFile);

        std::vector<VehicleWaypoint> traj = parser->GetTrajectory("veh0");
        NS_TEST_ASSERT_MSG_EQ(traj.size(), 4, "veh0 should have 4 waypoints");

        // Verify initial waypoint coordinates: (10.0, 20.0, 0.0)
        NS_TEST_ASSERT_MSG_EQ_TOL(traj[0].position.x, 10.0, 1e-4, "veh0 wp0 X mismatch");
        NS_TEST_ASSERT_MSG_EQ_TOL(traj[0].position.y, 20.0, 1e-4, "veh0 wp0 Y mismatch");
        NS_TEST_ASSERT_MSG_EQ_TOL(traj[0].speedMps, 10.0, 1e-4, "veh0 wp0 speed mismatch");

        // Verify final waypoint coordinates at t=2.0: (30.0, 20.0, 0.0)
        NS_TEST_ASSERT_MSG_EQ_TOL(traj[3].position.x, 30.0, 1e-4, "veh0 wp3 X mismatch");
        NS_TEST_ASSERT_MSG_EQ_TOL(traj[3].position.y, 20.0, 1e-4, "veh0 wp3 Y mismatch");
    }
};

// ===========================================================================
// T03: Vehicle first/last-seen times are correct
// ===========================================================================
class V2xTestT03FirstLastSeen : public TestCase
{
public:
    V2xTestT03FirstLastSeen() : TestCase("T03: Vehicle first and last seen times are correct") {}
    void DoRun() override
    {
        std::string testFile = CreateTempDirFilename("v2x-t03.xml");
        CreateTestFcdXml(testFile);

        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();
        parser->Load(testFile);

        // veh0: 0.0s to 2.0s
        NS_TEST_ASSERT_MSG_EQ(parser->GetFirstSeen("veh0"), Seconds(0.0), "veh0 first seen incorrect");
        NS_TEST_ASSERT_MSG_EQ(parser->GetLastSeen("veh0"), Seconds(2.0), "veh0 last seen incorrect");

        // veh1: 0.5s to 2.0s
        NS_TEST_ASSERT_MSG_EQ(parser->GetFirstSeen("veh1"), Seconds(0.5), "veh1 first seen incorrect");
        NS_TEST_ASSERT_MSG_EQ(parser->GetLastSeen("veh1"), Seconds(2.0), "veh1 last seen incorrect");

        // veh2: 1.0s to 1.0s
        NS_TEST_ASSERT_MSG_EQ(parser->GetFirstSeen("veh2"), Seconds(1.0), "veh2 first seen incorrect");
        NS_TEST_ASSERT_MSG_EQ(parser->GetLastSeen("veh2"), Seconds(1.0), "veh2 last seen incorrect");
    }
};

// ===========================================================================
// T04: Coordinate scaling and offset works
// ===========================================================================
class V2xTestT04ScaleAndOffset : public TestCase
{
public:
    V2xTestT04ScaleAndOffset() : TestCase("T04: Coordinate scaling and offset works") {}
    void DoRun() override
    {
        std::string testFile = CreateTempDirFilename("v2x-t04.xml");
        CreateTestFcdXml(testFile);

        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();
        parser->SetCoordinateScale(2.5);
        parser->SetCoordinateOffsetX(100.0);
        parser->SetCoordinateOffsetY(-50.0);

        parser->Load(testFile);

        std::vector<VehicleWaypoint> traj = parser->GetTrajectory("veh0");
        // raw X = 10.0 -> scaled = 10.0 * 2.5 + 100.0 = 125.0
        // raw Y = 20.0 -> scaled = 20.0 * 2.5 - 50.0  = 0.0
        NS_TEST_ASSERT_MSG_EQ_TOL(traj[0].position.x, 125.0, 1e-4, "Scaled X mismatch");
        NS_TEST_ASSERT_MSG_EQ_TOL(traj[0].position.y, 0.0, 1e-4, "Scaled Y mismatch");
    }
};

// ===========================================================================
// T05: Missing or malformed input generates clear failure
// ===========================================================================
class V2xTestT05MissingMalformedInput : public TestCase
{
public:
    V2xTestT05MissingMalformedInput() : TestCase("T05: Missing or malformed input generates clear failure") {}
    void DoRun() override
    {
        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();

        // Case 1: Non-existent file
        bool ok = parser->Load("/path/to/definitely_non_existent_fcd_file.xml");
        NS_TEST_ASSERT_MSG_EQ(ok, false, "Parser must return false for missing file");
        NS_TEST_ASSERT_MSG_NE(parser->GetLastError(), "", "Error message must not be empty on missing file");

        // Case 2: Malformed XML (missing root tag)
        std::string malformedFile = CreateTempDirFilename("malformed.xml");
        std::ofstream out(malformedFile);
        out << "<not-an-fcd-file><some-tag/></not-an-fcd-file>\n";
        out.close();

        ok = parser->Load(malformedFile);
        NS_TEST_ASSERT_MSG_EQ(ok, false, "Parser must return false for invalid XML structure");
    }
};

// ===========================================================================
// T06: Deterministic vehicle-to-node mapping
// ===========================================================================
class V2xTestT06DeterministicMapping : public TestCase
{
public:
    V2xTestT06DeterministicMapping() : TestCase("T06: Deterministic vehicle-to-node mapping") {}
    void DoRun() override
    {
        Ptr<VehicleNodeMapper> mapper = CreateObject<VehicleNodeMapper>();

        NodeContainer nodes;
        nodes.Create(3);

        mapper->Register("vehB", nodes.Get(1));
        mapper->Register("vehA", nodes.Get(0));
        mapper->Register("vehC", nodes.Get(2));

        NS_TEST_ASSERT_MSG_EQ(mapper->GetSize(), 3, "Mapper size mismatch");
        NS_TEST_ASSERT_MSG_EQ(mapper->GetVehicleId(nodes.Get(0)->GetId()), "vehA", "Node 0 must map to vehA");
        NS_TEST_ASSERT_MSG_EQ(mapper->GetVehicleId(nodes.Get(1)->GetId()), "vehB", "Node 1 must map to vehB");
        NS_TEST_ASSERT_MSG_EQ(mapper->GetVehicleId(nodes.Get(2)->GetId()), "vehC", "Node 2 must map to vehC");

        NS_TEST_ASSERT_MSG_EQ(mapper->GetNode("vehA"), nodes.Get(0), "vehA must map to Node 0");

        // Verify sorted deterministic vehicle IDs list
        std::vector<std::string> ids = mapper->GetAllVehicleIds();
        NS_TEST_ASSERT_MSG_EQ(ids[0], "vehA", "First alphabetical vehicle must be vehA");
        NS_TEST_ASSERT_MSG_EQ(ids[1], "vehB", "Second alphabetical vehicle must be vehB");
        NS_TEST_ASSERT_MSG_EQ(ids[2], "vehC", "Third alphabetical vehicle must be vehC");
    }
};

// ===========================================================================
// T1: Construction and default values
// ===========================================================================
class V2xTestT1ConstructionDefaults : public TestCase
{
public:
    V2xTestT1ConstructionDefaults() : TestCase("T1: Construction and default values") {}
    void DoRun() override
    {
        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();
        NS_TEST_ASSERT_MSG_EQ_TOL(parser->GetCoordinateScale(), 1.0, 1e-6, "Default scale must be 1.0");
        NS_TEST_ASSERT_MSG_EQ_TOL(parser->GetCoordinateOffsetX(), 0.0, 1e-6, "Default offset X must be 0.0");
        NS_TEST_ASSERT_MSG_EQ_TOL(parser->GetCoordinateOffsetY(), 0.0, 1e-6, "Default offset Y must be 0.0");

        Ptr<V2xMobilityImporter> importer = CreateObject<V2xMobilityImporter>();
        NS_TEST_ASSERT_MSG_EQ(importer->GetMobilityStep(), MilliSeconds(100), "Default step must be 100ms");
    }
};

// ===========================================================================
// T3: RSU Selection Policy (Nearest-RSU baseline)
// ===========================================================================
class V2xTestT3RsuSelectionPolicy : public TestCase
{
public:
    V2xTestT3RsuSelectionPolicy() : TestCase("T3: RSU Selection Policy (Nearest-RSU baseline)") {}
    void DoRun() override
    {
        RsuPlacementHelper rsuHelper;
        NodeContainer rsus = rsuHelper.CreateRsus(2);

        std::vector<Vector> rsuPositions = {
            Vector(10.0, 0.0, 0.0),  // RSU 0 at (10, 0)
            Vector(100.0, 0.0, 0.0)  // RSU 1 at (100, 0)
        };
        rsuHelper.InstallPositions(rsus, rsuPositions);

        Ptr<NearestRsuSelectionPolicy> policy = CreateObject<NearestRsuSelectionPolicy>();

        // Vehicle at (15, 0) is closer to RSU 0
        RsuSelectionState state1;
        state1.vehicleNodeId = 99;
        state1.vehiclePosition = Vector(15.0, 0.0, 0.0);
        state1.candidateRsus = rsus;

        RsuSelectionDecision decision1 = policy->Select(state1);
        NS_TEST_ASSERT_MSG_EQ(decision1.valid, true, "Decision should be valid");
        NS_TEST_ASSERT_MSG_EQ(decision1.selectedRsu, rsus.Get(0), "Vehicle at (15,0) should select RSU 0");
        NS_TEST_ASSERT_MSG_EQ_TOL(decision1.distanceM, 5.0, 1e-4, "Distance to RSU 0 should be 5m");

        // Vehicle moves to (90, 0), closer to RSU 1
        RsuSelectionState state2;
        state2.vehicleNodeId = 99;
        state2.vehiclePosition = Vector(90.0, 0.0, 0.0);
        state2.candidateRsus = rsus;

        RsuSelectionDecision decision2 = policy->Select(state2);
        NS_TEST_ASSERT_MSG_EQ(decision2.selectedRsu, rsus.Get(1), "Vehicle at (90,0) should select RSU 1");
        NS_TEST_ASSERT_MSG_EQ_TOL(decision2.distanceM, 10.0, 1e-4, "Distance to RSU 1 should be 10m");
    }
};

// ===========================================================================
// T2: Attribute bounds and validation
// ===========================================================================
class V2xTestT2AttributeBounds : public TestCase
{
public:
    V2xTestT2AttributeBounds() : TestCase("T2: Attribute bounds and validation") {}
    void DoRun() override
    {
        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();
        BooleanValue ok = parser->SetAttributeFailSafe("CoordinateScale", DoubleValue(2.5));
        NS_TEST_ASSERT_MSG_EQ(ok.Get(), true, "Setting valid CoordinateScale must succeed");
        BooleanValue badScale = parser->SetAttributeFailSafe("CoordinateScale", DoubleValue(-1.0));
        NS_TEST_ASSERT_MSG_EQ(badScale.Get(), false, "Negative CoordinateScale must fail checker");

        Ptr<V2xMobilityImporter> importer = CreateObject<V2xMobilityImporter>();
        BooleanValue okStep = importer->SetAttributeFailSafe("MobilityStep", TimeValue(MilliSeconds(50)));
        NS_TEST_ASSERT_MSG_EQ(okStep.Get(), true, "Setting valid MobilityStep (50ms) must succeed");
        BooleanValue badStep = importer->SetAttributeFailSafe("MobilityStep", TimeValue(MilliSeconds(5)));
        NS_TEST_ASSERT_MSG_EQ(badStep.Get(), false, "MobilityStep below 10ms must fail checker");

        Ptr<V2xScenarioHelper> helper = CreateObject<V2xScenarioHelper>();
        BooleanValue okBytes = helper->SetAttributeFailSafe("V2xMessageBytes", UintegerValue(500));
        NS_TEST_ASSERT_MSG_EQ(okBytes.Get(), true, "Setting valid V2xMessageBytes (500) must succeed");
        BooleanValue badBytes = helper->SetAttributeFailSafe("V2xMessageBytes", UintegerValue(10));
        NS_TEST_ASSERT_MSG_EQ(badBytes.Get(), false, "V2xMessageBytes below 64 must fail checker");

        BooleanValue okRsus = helper->SetAttributeFailSafe("NumRsus", UintegerValue(10));
        NS_TEST_ASSERT_MSG_EQ(okRsus.Get(), true, "Setting valid NumRsus (10) must succeed");
        BooleanValue badRsus = helper->SetAttributeFailSafe("NumRsus", UintegerValue(150));
        NS_TEST_ASSERT_MSG_EQ(badRsus.Get(), false, "NumRsus above 100 must fail checker");
    }
};

// ===========================================================================
// T4: TraceSource transitions and callbacks
// ===========================================================================
class V2xTestT4TraceSources : public TestCase
{
public:
    V2xTestT4TraceSources() : TestCase("T4: TraceSource transitions and callbacks") {}

    uint32_t m_activatedCount{0};
    uint32_t m_deactivatedCount{0};
    uint32_t m_positionCount{0};
    std::vector<std::string> m_activatedVehicles;

    void OnActivated(std::string vehicleId, uint32_t nodeId)
    {
        m_activatedCount++;
        m_activatedVehicles.push_back(vehicleId);
    }

    void OnDeactivated(std::string vehicleId, uint32_t nodeId)
    {
        m_deactivatedCount++;
    }

    void OnPosition(std::string vehicleId, uint32_t nodeId, Vector position, double speedMps)
    {
        m_positionCount++;
    }

    void DoRun() override
    {
        std::string testFile = CreateTempDirFilename("v2x-t4.xml");
        CreateTestFcdXml(testFile);

        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();
        parser->Load(testFile);

        Ptr<VehicleNodeMapper> mapper = CreateObject<VehicleNodeMapper>();
        std::vector<std::string> vids = parser->GetVehicleIds();
        NodeContainer nodes;
        nodes.Create(vids.size());
        for (size_t i = 0; i < vids.size(); ++i)
        {
            mapper->Register(vids[i], nodes.Get(i));
        }

        Ptr<V2xMobilityImporter> importer = CreateObject<V2xMobilityImporter>();
        importer->TraceConnectWithoutContext("VehicleActivated",
            MakeCallback(&V2xTestT4TraceSources::OnActivated, this));
        importer->TraceConnectWithoutContext("VehicleDeactivated",
            MakeCallback(&V2xTestT4TraceSources::OnDeactivated, this));
        importer->TraceConnectWithoutContext("VehiclePosition",
            MakeCallback(&V2xTestT4TraceSources::OnPosition, this));

        importer->Install(parser, mapper);

        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_activatedCount, 3, "Expected 3 vehicle activations");
        NS_TEST_ASSERT_MSG_GT(m_positionCount, 0, "Expected positive position sampling events");
        NS_TEST_ASSERT_MSG_EQ(m_activatedVehicles[0], "veh0", "First activated vehicle must be veh0");
    }
};

// ===========================================================================
// T5: Deterministic and reproducibility test with AssignStreams
// ===========================================================================
class V2xTestT5Repeatability : public TestCase
{
public:
    V2xTestT5Repeatability() : TestCase("T5: Deterministic and reproducibility test with AssignStreams") {}
    void DoRun() override
    {
        Ptr<UniformRandomVariable> rng1 = CreateObject<UniformRandomVariable>();
        rng1->SetStream(42);

        Ptr<UniformRandomVariable> rng2 = CreateObject<UniformRandomVariable>();
        rng2->SetStream(42);

        for (int i = 0; i < 5; ++i)
        {
            double v1 = rng1->GetValue(0.0, 1.0);
            double v2 = rng2->GetValue(0.0, 1.0);
            NS_TEST_ASSERT_MSG_EQ_TOL(v1, v2, 1e-12, "Streams with identical seeds must yield identical values");
        }

        Ptr<UniformRandomVariable> rng3 = CreateObject<UniformRandomVariable>();
        rng3->SetStream(99);
        double v3 = rng3->GetValue(0.0, 1.0);
        double v1_next = rng1->GetValue(0.0, 1.0);
        NS_TEST_ASSERT_MSG_NE(v1_next, v3, "Different stream seeds must produce different sequences");
    }
};

// ===========================================================================
// T6: Integration test with ns-3 Wi-Fi and Scenario Helper
// ===========================================================================
class V2xTestT6Integration : public TestCase
{
public:
    V2xTestT6Integration() : TestCase("T6: Integration test with ns-3 Wi-Fi and Scenario Helper") {}
    void DoRun() override
    {
        std::string testFile = CreateTempDirFilename("v2x-t6.xml");
        CreateTestFcdXml(testFile);

        Ptr<V2xScenarioHelper> helper = CreateObject<V2xScenarioHelper>();
        bool ok = helper->LoadFcd(testFile);
        NS_TEST_ASSERT_MSG_EQ(ok, true, "Scenario helper must load valid test FCD XML");

        NodeContainer vehicles = helper->CreateVehicleNodes();
        NS_TEST_ASSERT_MSG_EQ(vehicles.GetN(), 3, "Expected 3 vehicle nodes");

        NodeContainer rsus = helper->CreateRsuNodes(2);
        NS_TEST_ASSERT_MSG_EQ(rsus.GetN(), 2, "Expected 2 RSU nodes");

        helper->InstallMobility();
        helper->InstallExampleNetworking();

        std::string prefix = CreateTempDirFilename("v2x-t6-out");
        helper->EnableTracing(prefix);

        Simulator::Stop(Seconds(1.5));
        Simulator::Run();
        Simulator::Destroy();
        helper->CloseTracing();

        // Verify generated network CSV
        std::ifstream netCsv(prefix + "-network.csv");
        NS_TEST_ASSERT_MSG_EQ(netCsv.is_open(), true, "Network CSV must be created");
        std::string line;
        uint32_t netLines = 0;
        bool hasRxOrTx = false;
        while (std::getline(netCsv, line))
        {
            netLines++;
            if (line.find("RxV2I") != std::string::npos || line.find("TxV2I") != std::string::npos)
            {
                hasRxOrTx = true;
            }
        }
        netCsv.close();
        NS_TEST_ASSERT_MSG_GT(netLines, 1, "Network CSV must contain data rows");
        NS_TEST_ASSERT_MSG_EQ(hasRxOrTx, true, "Network CSV must record real V2I traffic events");

        // Verify generated mobility CSV
        std::ifstream mobCsv(prefix + "-mobility.csv");
        NS_TEST_ASSERT_MSG_EQ(mobCsv.is_open(), true, "Mobility CSV must be created");
        uint32_t mobLines = 0;
        while (std::getline(mobCsv, line))
        {
            mobLines++;
        }
        mobCsv.close();
        NS_TEST_ASSERT_MSG_GT(mobLines, 1, "Mobility CSV must contain data rows");
    }
};

// ===========================================================================
// FR-04: Dynamic vehicle lifecycle (departure/arrival)
// ===========================================================================
class V2xTestFr04Lifecycle : public TestCase
{
public:
    V2xTestFr04Lifecycle() : TestCase("FR-04: Dynamic vehicle lifecycle (departure and arrival)") {}
    void DoRun() override
    {
        std::string testFile = CreateTempDirFilename("v2x-fr04.xml");
        CreateTestFcdXml(testFile);

        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();
        parser->Load(testFile);

        Ptr<V2xMobilityImporter> importer = CreateObject<V2xMobilityImporter>();
        NodeContainer nodes;
        nodes.Create(parser->GetVehicleCount());
        importer->Install(parser, nodes);

        NS_TEST_ASSERT_MSG_EQ(importer->IsVehicleActive("veh0"), false, "veh0 must be inactive before start");
        NS_TEST_ASSERT_MSG_EQ(importer->IsVehicleActive("veh1"), false, "veh1 must be inactive before start");

        Simulator::Stop(Seconds(0.6));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(importer->IsVehicleActive("veh0"), true, "veh0 must be active at t=0.6s");
        NS_TEST_ASSERT_MSG_EQ(importer->IsVehicleActive("veh1"), true, "veh1 must be active at t=0.6s");
        NS_TEST_ASSERT_MSG_EQ(importer->IsVehicleActive("veh2"), false, "veh2 must not be active at t=0.6s");

        Simulator::Destroy();
    }
};

// ===========================================================================
// FR-08: Three-tier telemetry schema conformity
// ===========================================================================
class V2xTestFr08Telemetry : public TestCase
{
public:
    V2xTestFr08Telemetry() : TestCase("FR-08: Three-tier telemetry schema conformity") {}
    void DoRun() override
    {
        std::string testFile = CreateTempDirFilename("v2x-fr08.xml");
        CreateTestFcdXml(testFile);

        Ptr<V2xScenarioHelper> helper = CreateObject<V2xScenarioHelper>();
        helper->LoadFcd(testFile);
        helper->CreateVehicleNodes();
        helper->CreateRsuNodes(2);
        helper->InstallMobility();
        helper->InstallExampleNetworking();

        std::string prefix = CreateTempDirFilename("v2x-fr08-out");
        helper->EnableTracing(prefix);

        Simulator::Stop(Seconds(1.0));
        Simulator::Run();
        Simulator::Destroy();
        helper->CloseTracing();

        std::ifstream mob(prefix + "-mobility.csv");
        std::string header;
        std::getline(mob, header);
        NS_TEST_ASSERT_MSG_EQ(header, "time_s,vehicle_id,node_id,x_m,y_m,z_m,speed_mps,active",
                              "Mobility CSV header must match Common Guidelines schema");

        std::ifstream net(prefix + "-network.csv");
        std::getline(net, header);
        NS_TEST_ASSERT_MSG_EQ(header, "time_s,vehicle_id,peer_id,event,delay_ms,throughput_mbps,loss",
                              "Network CSV header must match Common Guidelines schema");

        std::ifstream com(prefix + "-common-events.csv");
        std::getline(com, header);
        NS_TEST_ASSERT_MSG_EQ(header, "time_s,module,event,node_id,peer_id,flow_id,value1,value2,unit,detail",
                              "Common events CSV header must match Common Guidelines schema");
    }
};

// ===========================================================================
// Late-departing vehicle (InitialPositionIsWaypoint=true)
// ===========================================================================
class V2xTestLateDepartingVehicle : public TestCase
{
public:
    V2xTestLateDepartingVehicle() : TestCase("Late-departing vehicle waypoint positioning") {}
    void DoRun() override
    {
        std::string testFile = CreateTempDirFilename("v2x-late.xml");
        std::ofstream out(testFile);
        out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            << "<fcd-export>\n"
            << "  <timestep time=\"2.00\">\n"
            << "    <vehicle id=\"veh_late\" x=\"200.00\" y=\"300.00\" z=\"0.00\" speed=\"10.00\" angle=\"0.00\" pos=\"0.00\" lane=\"lane0\"/>\n"
            << "  </timestep>\n"
            << "  <timestep time=\"3.00\">\n"
            << "    <vehicle id=\"veh_late\" x=\"210.00\" y=\"300.00\" z=\"0.00\" speed=\"10.00\" angle=\"0.00\" pos=\"10.00\" lane=\"lane0\"/>\n"
            << "  </timestep>\n"
            << "</fcd-export>\n";
        out.close();

        Ptr<SumoFcdParser> parser = CreateObject<SumoFcdParser>();
        parser->Load(testFile);

        NodeContainer nodes;
        nodes.Create(1);

        Ptr<V2xMobilityImporter> importer = CreateObject<V2xMobilityImporter>();
        importer->Install(parser, nodes);

        Ptr<WaypointMobilityModel> mob = nodes.Get(0)->GetObject<WaypointMobilityModel>();
        NS_TEST_ASSERT_MSG_NE(mob, nullptr, "Node must have WaypointMobilityModel");

        // Prior to t=2.0s, initial position should already be the first waypoint (200, 300, 0), NOT (0,0,0)
        Vector pos0 = mob->GetPosition();
        NS_TEST_ASSERT_MSG_EQ_TOL(pos0.x, 200.0, 1e-3, "Initial X before departure must be first waypoint X");
        NS_TEST_ASSERT_MSG_EQ_TOL(pos0.y, 300.0, 1e-3, "Initial Y before departure must be first waypoint Y");

        Simulator::Stop(Seconds(2.5));
        Simulator::Run();

        Vector posMid = mob->GetPosition();
        NS_TEST_ASSERT_MSG_EQ_TOL(posMid.x, 205.0, 1e-2, "Position at t=2.5s must be halfway between waypoints");
        NS_TEST_ASSERT_MSG_EQ_TOL(posMid.y, 300.0, 1e-3, "Position Y at t=2.5s must remain 300.0");

        Simulator::Destroy();
    }
};

// ===========================================================================
// RSU Placement Modes: Explicit Coordinates, Grid Spacing, Junction List
// ===========================================================================
class V2xTestRsuPlacementModes : public TestCase
{
public:
    V2xTestRsuPlacementModes() : TestCase("RSU placement modes and geometry") {}
    void DoRun() override
    {
        RsuPlacementHelper helper;
        helper.SetRsuHeight(6.0);

        // 1. Explicit Coordinates Mode
        NodeContainer rsusExplicit = helper.CreateRsus(3);
        std::vector<Vector> coords = {
            Vector(10.0, 20.0, 6.0),
            Vector(50.0, 60.0, 6.0),
            Vector(90.0, 100.0, 6.0)
        };
        helper.InstallPositions(rsusExplicit, coords);
        for (uint32_t i = 0; i < 3; ++i)
        {
            Ptr<ConstantPositionMobilityModel> mob = rsusExplicit.Get(i)->GetObject<ConstantPositionMobilityModel>();
            NS_TEST_ASSERT_MSG_NE(mob, nullptr, "RSU must have ConstantPositionMobilityModel");
            Vector p = mob->GetPosition();
            NS_TEST_ASSERT_MSG_EQ_TOL(p.x, coords[i].x, 1e-4, "Explicit RSU X mismatch");
            NS_TEST_ASSERT_MSG_EQ_TOL(p.y, coords[i].y, 1e-4, "Explicit RSU Y mismatch");
            NS_TEST_ASSERT_MSG_EQ_TOL(p.z, coords[i].z, 1e-4, "Explicit RSU Z mismatch");
        }

        // 2. Regular Grid Spacing Mode (4 RSUs, start=(100, 200), delta=(50, 60), gridWidth=2)
        NodeContainer rsusGrid = helper.CreateRsus(4);
        helper.InstallGridSpacing(rsusGrid, 100.0, 200.0, 50.0, 60.0, 2);
        std::vector<Vector> expectedGrid = {
            Vector(100.0, 200.0, 6.0),
            Vector(150.0, 200.0, 6.0),
            Vector(100.0, 260.0, 6.0),
            Vector(150.0, 260.0, 6.0)
        };
        for (uint32_t i = 0; i < 4; ++i)
        {
            Ptr<ConstantPositionMobilityModel> mob = rsusGrid.Get(i)->GetObject<ConstantPositionMobilityModel>();
            NS_TEST_ASSERT_MSG_NE(mob, nullptr, "Grid RSU must have ConstantPositionMobilityModel");
            Vector p = mob->GetPosition();
            NS_TEST_ASSERT_MSG_EQ_TOL(p.x, expectedGrid[i].x, 1e-4, "Grid RSU X mismatch");
            NS_TEST_ASSERT_MSG_EQ_TOL(p.y, expectedGrid[i].y, 1e-4, "Grid RSU Y mismatch");
            NS_TEST_ASSERT_MSG_EQ_TOL(p.z, 6.0, 1e-4, "Grid RSU Z mismatch");
        }

        // 3. Junction List Mode
        NodeContainer rsusJunc = helper.CreateRsus(2);
        std::vector<Vector> junctions = {
            Vector(250.0, 350.0, 6.0),
            Vector(450.0, 550.0, 6.0)
        };
        helper.InstallJunctionList(rsusJunc, junctions);
        for (uint32_t i = 0; i < 2; ++i)
        {
            Ptr<ConstantPositionMobilityModel> mob = rsusJunc.Get(i)->GetObject<ConstantPositionMobilityModel>();
            NS_TEST_ASSERT_MSG_NE(mob, nullptr, "Junction RSU must have ConstantPositionMobilityModel");
            Vector p = mob->GetPosition();
            NS_TEST_ASSERT_MSG_EQ_TOL(p.x, junctions[i].x, 1e-4, "Junction RSU X mismatch");
            NS_TEST_ASSERT_MSG_EQ_TOL(p.y, junctions[i].y, 1e-4, "Junction RSU Y mismatch");
        }
    }
};

// ===========================================================================
// TestSuite definition
// ===========================================================================
class FuturenetV2xTestSuite : public TestSuite
{
public:
    FuturenetV2xTestSuite() : TestSuite("futurenet-v2x", Type::UNIT)
    {
        AddTestCase(new V2xTestT01ParserCountIds, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT02WaypointPosition, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT03FirstLastSeen, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT04ScaleAndOffset, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT05MissingMalformedInput, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT06DeterministicMapping, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT1ConstructionDefaults, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT2AttributeBounds, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT3RsuSelectionPolicy, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT4TraceSources, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT5Repeatability, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestT6Integration, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestFr04Lifecycle, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestFr08Telemetry, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestLateDepartingVehicle, TestCase::Duration::QUICK);
        AddTestCase(new V2xTestRsuPlacementModes, TestCase::Duration::QUICK);
    }
};

static FuturenetV2xTestSuite g_futurenetV2xTestSuite;


