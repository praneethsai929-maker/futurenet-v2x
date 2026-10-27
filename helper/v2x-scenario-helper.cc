#include "v2x-scenario-helper.h"
#include "rsu-placement-helper.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <iomanip>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("V2xScenarioHelper");
NS_OBJECT_ENSURE_REGISTERED(V2xScenarioHelper);

TypeId
V2xScenarioHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::V2xScenarioHelper")
            .SetParent<Object>()
            .SetGroupName("FuturenetV2x")
            .AddConstructor<V2xScenarioHelper>()
            .AddAttribute("MobilityStep",
                          "Sampling step interval for mobility position updates",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&V2xScenarioHelper::m_mobilityStep),
                          MakeTimeChecker(MilliSeconds(10), Seconds(2)))
            .AddAttribute("NumRsus",
                          "Default number of Roadside Units to instantiate",
                          UintegerValue(4),
                          MakeUintegerAccessor(&V2xScenarioHelper::m_numRsus),
                          MakeUintegerChecker<uint32_t>(0, 100))
            .AddAttribute("CoordinateScale",
                          "Coordinate scaling multiplier",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&V2xScenarioHelper::m_coordinateScale),
                          MakeDoubleChecker<double>(1e-6, 1e6))
            .AddAttribute("CoordinateOffsetX",
                          "X offset applied to imported coordinates (meters)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&V2xScenarioHelper::m_coordinateOffsetX),
                          MakeDoubleChecker<double>())
            .AddAttribute("CoordinateOffsetY",
                          "Y offset applied to imported coordinates (meters)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&V2xScenarioHelper::m_coordinateOffsetY),
                          MakeDoubleChecker<double>())
            .AddAttribute("V2xMessageBytes",
                          "Size of periodic V2X application messages in bytes",
                          UintegerValue(300),
                          MakeUintegerAccessor(&V2xScenarioHelper::m_v2xMessageBytes),
                          MakeUintegerChecker<uint32_t>(64, 1500))
            .AddAttribute("V2xMessageInterval",
                          "Interval between periodic V2X application messages",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&V2xScenarioHelper::m_v2xMessageInterval),
                          MakeTimeChecker(MilliSeconds(10), Seconds(10)))
            .AddAttribute("ApplicationStartJitter",
                          "Maximum random startup jitter for applications",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&V2xScenarioHelper::m_startJitterMax),
                          MakeTimeChecker(Seconds(0), Seconds(10)));
    return tid;
}

V2xScenarioHelper::V2xScenarioHelper()
{
    NS_LOG_FUNCTION(this);
    m_parser = CreateObject<SumoFcdParser>();
    m_mapper = CreateObject<VehicleNodeMapper>();
    m_importer = CreateObject<V2xMobilityImporter>();
    m_rsuPolicy = CreateObject<NearestRsuSelectionPolicy>();
    m_jitterRng = CreateObject<UniformRandomVariable>();
}

V2xScenarioHelper::~V2xScenarioHelper()
{
    NS_LOG_FUNCTION(this);
    if (m_mobilityCsv.is_open())
    {
        m_mobilityCsv.close();
    }
    if (m_networkCsv.is_open())
    {
        m_networkCsv.close();
    }
    if (m_commonCsv.is_open())
    {
        m_commonCsv.close();
    }
}

int64_t
V2xScenarioHelper::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_jitterRng->SetStream(stream);
    return 1;
}

bool
V2xScenarioHelper::LoadFcd(const std::string& filename)
{
    NS_LOG_FUNCTION(this << filename);
    m_parser->SetCoordinateScale(m_coordinateScale);
    m_parser->SetCoordinateOffsetX(m_coordinateOffsetX);
    m_parser->SetCoordinateOffsetY(m_coordinateOffsetY);

    bool ok = m_parser->Load(filename);
    if (!ok)
    {
        NS_LOG_ERROR("Failed to load FCD: " << m_parser->GetLastError());
    }
    return ok;
}

NodeContainer
V2xScenarioHelper::CreateVehicleNodes()
{
    NS_LOG_FUNCTION(this);
    std::vector<std::string> vids = m_parser->GetVehicleIds();
    m_vehicleNodes.Create(vids.size());

    m_mapper->Clear();
    for (size_t i = 0; i < vids.size(); ++i)
    {
        m_mapper->Register(vids[i], m_vehicleNodes.Get(i));
    }
    return m_vehicleNodes;
}

NodeContainer
V2xScenarioHelper::CreateRsuNodes(uint32_t numRsus)
{
    NS_LOG_FUNCTION(this << numRsus);
    m_numRsus = numRsus;
    RsuPlacementHelper rsuHelper;
    m_rsuNodes = rsuHelper.CreateRsus(numRsus);

    // Place RSUs evenly across a default bounding area
    rsuHelper.InstallGridSpacing(m_rsuNodes, 50.0, 50.0, 200.0, 200.0, 2);
    return m_rsuNodes;
}

void
V2xScenarioHelper::InstallMobility()
{
    NS_LOG_FUNCTION(this);
    m_importer->SetMobilityStep(m_mobilityStep);
    m_importer->Install(m_parser, m_mapper);

    // Connect trace callbacks
    m_importer->TraceConnectWithoutContext("VehiclePosition",
        MakeCallback(&V2xScenarioHelper::LogMobilitySample, this));
    m_importer->TraceConnectWithoutContext("VehicleActivated",
        MakeCallback(&V2xScenarioHelper::LogVehicleActivated, this));
    m_importer->TraceConnectWithoutContext("VehicleDeactivated",
        MakeCallback(&V2xScenarioHelper::LogVehicleDeactivated, this));

    // Schedule RSU association checks
    Simulator::Schedule(m_mobilityStep, &V2xScenarioHelper::PeriodicRsuCheck, this);
}

void
V2xScenarioHelper::InstallExampleNetworking()
{
    NS_LOG_FUNCTION(this);

    // Ad-hoc 802.11a standard wireless configuration
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    if (m_vehicleNodes.GetN() > 0)
    {
        m_vehicleDevices = wifi.Install(wifiPhy, wifiMac, m_vehicleNodes);
    }
    if (m_rsuNodes.GetN() > 0)
    {
        m_rsuDevices = wifi.Install(wifiPhy, wifiMac, m_rsuNodes);
    }

    InternetStackHelper internet;
    if (m_vehicleNodes.GetN() > 0)
    {
        internet.Install(m_vehicleNodes);
    }
    if (m_rsuNodes.GetN() > 0)
    {
        internet.Install(m_rsuNodes);
    }

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.0.0");
    if (m_vehicleDevices.GetN() > 0)
    {
        m_vehicleInterfaces = ipv4.Assign(m_vehicleDevices);
    }
    if (m_rsuDevices.GetN() > 0)
    {
        m_rsuInterfaces = ipv4.Assign(m_rsuDevices);
    }

    // Install PacketSink on RSUs on port 9999
    if (m_rsuNodes.GetN() > 0)
    {
        uint16_t sinkPort = 9999;
        Address sinkAddress(InetSocketAddress(Ipv4Address::GetAny(), sinkPort));
        PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkAddress);
        ApplicationContainer sinkApps = packetSinkHelper.Install(m_rsuNodes);
        sinkApps.Start(Seconds(0.0));
    }
}

void
V2xScenarioHelper::EnableTracing(const std::string& prefix)
{
    NS_LOG_FUNCTION(this << prefix);
    m_tracingEnabled = true;

    m_mobilityCsv.open(prefix + "-mobility.csv");
    if (m_mobilityCsv.is_open())
    {
        m_mobilityCsv << "time_s,vehicle_id,node_id,x_m,y_m,z_m,speed_mps,active\n";
    }

    m_networkCsv.open(prefix + "-network.csv");
    if (m_networkCsv.is_open())
    {
        m_networkCsv << "time_s,vehicle_id,peer_id,event,delay_ms,throughput_mbps,loss\n";
    }

    m_commonCsv.open(prefix + "-common-events.csv");
    if (m_commonCsv.is_open())
    {
        m_commonCsv << "time_s,module,event,node_id,peer_id,flow_id,value1,value2,unit,detail\n";
    }
}

void
V2xScenarioHelper::SetRsuSelectionPolicy(Ptr<RsuSelectionPolicy> policy)
{
    m_rsuPolicy = policy;
}

void
V2xScenarioHelper::PeriodicRsuCheck()
{
    if (m_rsuNodes.GetN() > 0 && m_mapper != nullptr && m_rsuPolicy != nullptr)
    {
        std::vector<std::string> vids = m_mapper->GetAllVehicleIds();
        for (const auto& vid : vids)
        {
            if (m_importer->IsVehicleActive(vid))
            {
                Ptr<Node> vNode = m_mapper->GetNode(vid);
                if (vNode != nullptr)
                {
                    Ptr<MobilityModel> mob = vNode->GetObject<MobilityModel>();
                    if (mob != nullptr)
                    {
                        RsuSelectionState state;
                        state.vehicleNodeId = vNode->GetId();
                        state.vehiclePosition = mob->GetPosition();
                        state.candidateRsus = m_rsuNodes;

                        RsuSelectionDecision decision = m_rsuPolicy->Select(state);
                        if (decision.valid && decision.selectedRsu != nullptr)
                        {
                            uint32_t chosenRsuId = decision.selectedRsu->GetId();
                            uint32_t oldRsuId = UINT32_MAX;
                            if (m_currentRsuAssociation.find(vid) != m_currentRsuAssociation.end())
                            {
                                oldRsuId = m_currentRsuAssociation[vid];
                            }

                            if (oldRsuId != chosenRsuId)
                            {
                                m_currentRsuAssociation[vid] = chosenRsuId;
                                LogRsuAssociation(vid, oldRsuId, chosenRsuId);
                            }
                        }
                    }
                }
            }
        }
    }

    Simulator::Schedule(m_mobilityStep, &V2xScenarioHelper::PeriodicRsuCheck, this);
}

void
V2xScenarioHelper::LogMobilitySample(std::string vehicleId, uint32_t nodeId, Vector position)
{
    if (m_tracingEnabled && m_mobilityCsv.is_open())
    {
        double now = Simulator::Now().GetSeconds();
        m_mobilityCsv << std::fixed << std::setprecision(3)
                      << now << ","
                      << vehicleId << ","
                      << nodeId << ","
                      << position.x << ","
                      << position.y << ","
                      << position.z << ",0.0,1\n";
    }
}

void
V2xScenarioHelper::LogVehicleActivated(std::string vehicleId, uint32_t nodeId)
{
    double now = Simulator::Now().GetSeconds();
    Vector pos(0.0, 0.0, 0.0);
    if (m_mapper)
    {
        Ptr<Node> node = m_mapper->GetNode(vehicleId);
        if (node)
        {
            Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
            if (mob)
            {
                pos = mob->GetPosition();
            }
        }
    }

    if (m_tracingEnabled)
    {
        if (m_commonCsv.is_open())
        {
            m_commonCsv << std::fixed << std::setprecision(3)
                        << now << ",futurenet-v2x,VehicleActivated,"
                        << nodeId << ",0,0,0,0,none," << vehicleId << "\n";
        }
        if (m_mobilityCsv.is_open())
        {
            m_mobilityCsv << std::fixed << std::setprecision(3)
                          << now << "," << vehicleId << "," << nodeId << ","
                          << pos.x << "," << pos.y << "," << pos.z << ",0.0,1\n";
        }
    }
}

void
V2xScenarioHelper::LogVehicleDeactivated(std::string vehicleId, uint32_t nodeId)
{
    double now = Simulator::Now().GetSeconds();
    Vector pos(0.0, 0.0, 0.0);
    if (m_mapper)
    {
        Ptr<Node> node = m_mapper->GetNode(vehicleId);
        if (node)
        {
            Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
            if (mob)
            {
                pos = mob->GetPosition();
            }
        }
    }

    if (m_tracingEnabled)
    {
        if (m_commonCsv.is_open())
        {
            m_commonCsv << std::fixed << std::setprecision(3)
                        << now << ",futurenet-v2x,VehicleDeactivated,"
                        << nodeId << ",0,0,0,0,none," << vehicleId << "\n";
        }
        if (m_mobilityCsv.is_open())
        {
            m_mobilityCsv << std::fixed << std::setprecision(3)
                          << now << "," << vehicleId << "," << nodeId << ","
                          << pos.x << "," << pos.y << "," << pos.z << ",0.0,0\n";
        }
    }
}

void
V2xScenarioHelper::LogRsuAssociation(std::string vehicleId, uint32_t oldRsuNodeId, uint32_t newRsuNodeId)
{
    double now = Simulator::Now().GetSeconds();
    uint32_t nodeId = m_mapper ? m_mapper->GetNodeId(vehicleId) : 0;

    if (m_tracingEnabled)
    {
        if (m_networkCsv.is_open())
        {
            m_networkCsv << std::fixed << std::setprecision(3)
                         << now << "," << vehicleId << "," << newRsuNodeId
                         << ",RsuAssociationChange,0.0,0.0,0\n";
        }
        if (m_commonCsv.is_open())
        {
            m_commonCsv << std::fixed << std::setprecision(3)
                        << now << ",futurenet-v2x,RsuAssociationChange,"
                        << nodeId << "," << newRsuNodeId << ",0,"
                        << oldRsuNodeId << "," << newRsuNodeId
                        << ",node_id," << vehicleId << "\n";
        }
    }
}

NodeContainer
V2xScenarioHelper::GetVehicleNodes() const
{
    return m_vehicleNodes;
}

NodeContainer
V2xScenarioHelper::GetRsuNodes() const
{
    return m_rsuNodes;
}

Ptr<SumoFcdParser>
V2xScenarioHelper::GetParser() const
{
    return m_parser;
}

Ptr<VehicleNodeMapper>
V2xScenarioHelper::GetMapper() const
{
    return m_mapper;
}

Ptr<V2xMobilityImporter>
V2xScenarioHelper::GetImporter() const
{
    return m_importer;
}

} // namespace ns3
