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

#include <cmath>
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
                          MakeTimeChecker(Seconds(0), Seconds(10)))
            .AddAttribute("TxPower",
                          "Wi-Fi Transmission Power in dBm",
                          DoubleValue(21.5),
                          MakeDoubleAccessor(&V2xScenarioHelper::m_txPowerDbm),
                          MakeDoubleChecker<double>(-30.0, 50.0))
            .AddAttribute("RsuHeight",
                          "Height in meters for deployed RSUs",
                          DoubleValue(DEFAULT_RSU_HEIGHT_M),
                          MakeDoubleAccessor(&V2xScenarioHelper::m_rsuHeightM),
                          MakeDoubleChecker<double>(0.0, 100.0))
            .AddAttribute("SubnetBase",
                          "IPv4 subnet base address",
                          StringValue("10.1.0.0"),
                          MakeStringAccessor(&V2xScenarioHelper::m_subnetBase),
                          MakeStringChecker())
            .AddAttribute("SubnetMask",
                          "IPv4 subnet mask",
                          StringValue("255.255.0.0"),
                          MakeStringAccessor(&V2xScenarioHelper::m_subnetMask),
                          MakeStringChecker());
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
    for (auto& kv : m_sendV2iEvents)
    {
        Simulator::Cancel(kv.second);
    }
    m_sendV2iEvents.clear();

    for (auto& kv : m_sendV2vEvents)
    {
        Simulator::Cancel(kv.second);
    }
    m_sendV2vEvents.clear();

    for (auto& kv : m_vehicleV2iSockets)
    {
        if (kv.second)
        {
            kv.second->Close();
        }
    }
    m_vehicleV2iSockets.clear();

    for (auto& kv : m_vehicleV2vSockets)
    {
        if (kv.second)
        {
            kv.second->Close();
        }
    }
    m_vehicleV2vSockets.clear();

    for (auto& s : m_rsuSockets)
    {
        if (s)
        {
            s->Close();
        }
    }
    m_rsuSockets.clear();

    CloseTracing();
}

void
V2xScenarioHelper::CloseTracing()
{
    if (m_mobilityCsv.is_open())
    {
        m_mobilityCsv.flush();
        m_mobilityCsv.close();
    }
    if (m_networkCsv.is_open())
    {
        m_networkCsv.flush();
        m_networkCsv.close();
    }
    if (m_commonCsv.is_open())
    {
        m_commonCsv.flush();
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

void
V2xScenarioHelper::SetRsuPositions(const std::vector<Vector>& positions)
{
    m_rsuPositions = positions;
}

void
V2xScenarioHelper::SetRsuGridPlacement(double startX, double startY, double deltaX, double deltaY, uint32_t gridWidth)
{
    m_rsuGridStartX = startX;
    m_rsuGridStartY = startY;
    m_rsuGridDeltaX = deltaX;
    m_rsuGridDeltaY = deltaY;
    m_rsuGridWidth = gridWidth;
}

void
V2xScenarioHelper::SetRsuJunctionPlacement(const std::vector<Vector>& junctions)
{
    m_rsuJunctions = junctions;
}

NodeContainer
V2xScenarioHelper::CreateRsuNodes(uint32_t numRsus)
{
    NS_LOG_FUNCTION(this << numRsus);
    m_numRsus = numRsus;
    RsuPlacementHelper rsuHelper;
    rsuHelper.SetRsuHeight(m_rsuHeightM);
    m_rsuNodes = rsuHelper.CreateRsus(numRsus);

    if (!m_rsuPositions.empty())
    {
        rsuHelper.InstallPositions(m_rsuNodes, m_rsuPositions);
    }
    else if (!m_rsuJunctions.empty())
    {
        rsuHelper.InstallJunctionList(m_rsuNodes, m_rsuJunctions);
    }
    else
    {
        rsuHelper.InstallGridSpacing(m_rsuNodes,
                                     m_rsuGridStartX,
                                     m_rsuGridStartY,
                                     m_rsuGridDeltaX,
                                     m_rsuGridDeltaY,
                                     m_rsuGridWidth);
    }
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

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);

    YansWifiPhyHelper wifiPhy;
    wifiPhy.Set("TxPowerStart", DoubleValue(m_txPowerDbm));
    wifiPhy.Set("TxPowerEnd", DoubleValue(m_txPowerDbm));
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
    ipv4.SetBase(m_subnetBase.c_str(), m_subnetMask.c_str());
    m_broadcastAddress = Ipv4Address(m_subnetBase.c_str()).GetSubnetDirectedBroadcast(Ipv4Mask(m_subnetMask.c_str()));
    if (m_vehicleDevices.GetN() > 0)
    {
        m_vehicleInterfaces = ipv4.Assign(m_vehicleDevices);
    }
    if (m_rsuDevices.GetN() > 0)
    {
        m_rsuInterfaces = ipv4.Assign(m_rsuDevices);
    }

    // Install UDP receiver sockets on each RSU on V2I_PORT (9999)
    m_rsuSockets.clear();
    for (uint32_t i = 0; i < m_rsuNodes.GetN(); ++i)
    {
        Ptr<Socket> rsuSocket = Socket::CreateSocket(m_rsuNodes.Get(i), UdpSocketFactory::GetTypeId());
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), V2I_PORT);
        rsuSocket->Bind(local);
        rsuSocket->SetRecvCallback(MakeCallback(&V2xScenarioHelper::ReceiveRsuPacket, this));
        m_rsuSockets.push_back(rsuSocket);
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
V2xScenarioHelper::SendV2iMessage(std::string vehicleId)
{
    if (!m_importer || !m_importer->IsVehicleActive(vehicleId))
    {
        return;
    }

    Ptr<Node> vNode = m_mapper ? m_mapper->GetNode(vehicleId) : nullptr;
    if (!vNode || m_vehicleV2iSockets.find(vehicleId) == m_vehicleV2iSockets.end())
    {
        return;
    }

    Ptr<Socket> socket = m_vehicleV2iSockets[vehicleId];
    if (!socket)
    {
        return;
    }

    uint32_t rsuId = UINT32_MAX;
    if (m_currentRsuAssociation.find(vehicleId) != m_currentRsuAssociation.end())
    {
        rsuId = m_currentRsuAssociation[vehicleId];
    }
    else if (m_rsuNodes.GetN() > 0)
    {
        rsuId = m_rsuNodes.Get(0)->GetId();
    }

    if (rsuId != UINT32_MAX && m_rsuNodes.GetN() > 0 && m_rsuInterfaces.GetN() > 0)
    {
        for (uint32_t i = 0; i < m_rsuNodes.GetN(); ++i)
        {
            if (m_rsuNodes.Get(i)->GetId() == rsuId)
            {
                Ipv4Address rsuIp = m_rsuInterfaces.GetAddress(i);
                Address destAddress = InetSocketAddress(rsuIp, V2I_PORT);

                SeqTsHeader seqTs;
                seqTs.SetSeq(m_v2iSeq[vehicleId]++);
                uint32_t headerSize = seqTs.GetSerializedSize();
                uint32_t payloadSize = (m_v2xMessageBytes > headerSize) ? (m_v2xMessageBytes - headerSize) : 0;
                Ptr<Packet> packet = Create<Packet>(payloadSize);
                packet->AddHeader(seqTs);

                socket->SendTo(packet, 0, destAddress);

                double now = Simulator::Now().GetSeconds();
                uint32_t srcNodeId = vNode->GetId();
                double dataRateMbps = (m_v2xMessageBytes * 8.0) / (m_v2xMessageInterval.GetSeconds() * 1e6);

                if (m_tracingEnabled)
                {
                    if (m_networkCsv.is_open())
                    {
                        m_networkCsv << std::fixed << std::setprecision(3)
                                     << now << "," << vehicleId << "," << rsuId << ","
                                     << "TxV2I,0.0," << std::setprecision(4)
                                     << dataRateMbps << ",0\n";
                    }
                    if (m_commonCsv.is_open())
                    {
                        m_commonCsv << std::fixed << std::setprecision(3)
                                    << now << ",futurenet-v2x,TxV2I,"
                                    << srcNodeId << "," << rsuId << ",1,"
                                    << m_v2xMessageBytes << "," << std::setprecision(4)
                                    << dataRateMbps << ",bytes," << vehicleId << "\n";
                    }
                }
                break;
            }
        }
    }

    m_sendV2iEvents[vehicleId] = Simulator::Schedule(m_v2xMessageInterval,
                                                     &V2xScenarioHelper::SendV2iMessage,
                                                     this,
                                                     vehicleId);
}

void
V2xScenarioHelper::SendV2vMessage(std::string vehicleId)
{
    if (!m_importer || !m_importer->IsVehicleActive(vehicleId))
    {
        return;
    }

    Ptr<Node> vNode = m_mapper ? m_mapper->GetNode(vehicleId) : nullptr;
    if (!vNode || m_vehicleV2vSockets.find(vehicleId) == m_vehicleV2vSockets.end())
    {
        return;
    }

    Ptr<Socket> socket = m_vehicleV2vSockets[vehicleId];
    if (!socket)
    {
        return;
    }

    Address destAddress = InetSocketAddress(m_broadcastAddress, V2V_PORT);

    SeqTsHeader seqTs;
    seqTs.SetSeq(m_v2vSeq[vehicleId]++);
    uint32_t headerSize = seqTs.GetSerializedSize();
    uint32_t payloadSize = (m_v2xMessageBytes > headerSize) ? (m_v2xMessageBytes - headerSize) : 0;
    Ptr<Packet> packet = Create<Packet>(payloadSize);
    packet->AddHeader(seqTs);

    socket->SendTo(packet, 0, destAddress);

    double now = Simulator::Now().GetSeconds();
    uint32_t srcNodeId = vNode->GetId();
    double dataRateMbps = (m_v2xMessageBytes * 8.0) / (m_v2xMessageInterval.GetSeconds() * 1e6);

    if (m_tracingEnabled)
    {
        if (m_networkCsv.is_open())
        {
            m_networkCsv << std::fixed << std::setprecision(3)
                         << now << "," << vehicleId << ",0,"
                         << "TxV2V,0.0," << std::setprecision(4)
                         << dataRateMbps << ",0\n";
        }
        if (m_commonCsv.is_open())
        {
            m_commonCsv << std::fixed << std::setprecision(3)
                        << now << ",futurenet-v2x,TxV2V,"
                        << srcNodeId << ",0,2,"
                        << m_v2xMessageBytes << "," << std::setprecision(4)
                        << dataRateMbps << ",bytes," << vehicleId << "\n";
        }
    }

    m_sendV2vEvents[vehicleId] = Simulator::Schedule(m_v2xMessageInterval,
                                                     &V2xScenarioHelper::SendV2vMessage,
                                                     this,
                                                     vehicleId);
}

void
V2xScenarioHelper::ReceiveRsuPacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        SeqTsHeader seqTs;
        if (packet->RemoveHeader(seqTs) > 0)
        {
            Time txTime = seqTs.GetTs();
            Time delay = Simulator::Now() - txTime;
            double delayMs = delay.GetSeconds() * 1000.0;

            uint32_t rsuNodeId = socket->GetNode()->GetId();
            InetSocketAddress inetFrom = InetSocketAddress::ConvertFrom(from);
            Ipv4Address senderIp = inetFrom.GetIpv4();

            std::string vehicleId = "unknown";
            uint32_t senderNodeId = 0;
            for (uint32_t i = 0; i < m_vehicleInterfaces.GetN(); ++i)
            {
                if (m_vehicleInterfaces.GetAddress(i) == senderIp)
                {
                    if (i < m_vehicleNodes.GetN())
                    {
                        senderNodeId = m_vehicleNodes.Get(i)->GetId();
                        if (m_mapper)
                        {
                            vehicleId = m_mapper->GetVehicleId(senderNodeId);
                        }
                    }
                    break;
                }
            }

            double now = Simulator::Now().GetSeconds();
            double throughputMbps = ((packet->GetSize() + seqTs.GetSerializedSize()) * 8.0) /
                                    (m_v2xMessageInterval.GetSeconds() * 1e6);

            uint32_t currentSeq = seqTs.GetSeq();
            auto linkKey = std::make_pair(senderNodeId, rsuNodeId);
            uint32_t loss = 0;
            if (m_hasRxSeq[linkKey])
            {
                if (currentSeq > m_lastRxSeq[linkKey] + 1)
                {
                    loss = currentSeq - (m_lastRxSeq[linkKey] + 1);
                }
            }
            else
            {
                m_hasRxSeq[linkKey] = true;
            }
            m_lastRxSeq[linkKey] = currentSeq;

            if (m_tracingEnabled)
            {
                if (m_networkCsv.is_open())
                {
                    m_networkCsv << std::fixed << std::setprecision(3)
                                 << now << "," << vehicleId << "," << rsuNodeId << ","
                                 << "RxV2I," << std::setprecision(2) << delayMs << ","
                                 << std::setprecision(4) << throughputMbps << "," << loss << "\n";
                }
                if (m_commonCsv.is_open())
                {
                    m_commonCsv << std::fixed << std::setprecision(3)
                                << now << ",futurenet-v2x,RxV2I,"
                                << senderNodeId << "," << rsuNodeId << ",1,"
                                << std::setprecision(2) << delayMs << ","
                                << std::setprecision(4) << throughputMbps << ",ms,"
                                << vehicleId << "\n";
                }
            }
        }
    }
}

void
V2xScenarioHelper::ReceiveVehicleV2vPacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        SeqTsHeader seqTs;
        if (packet->RemoveHeader(seqTs) > 0)
        {
            Time txTime = seqTs.GetTs();
            Time delay = Simulator::Now() - txTime;
            double delayMs = delay.GetSeconds() * 1000.0;

            uint32_t rxNodeId = socket->GetNode()->GetId();
            InetSocketAddress inetFrom = InetSocketAddress::ConvertFrom(from);
            Ipv4Address senderIp = inetFrom.GetIpv4();

            std::string senderVehicleId = "unknown";
            uint32_t senderNodeId = 0;
            for (uint32_t i = 0; i < m_vehicleInterfaces.GetN(); ++i)
            {
                if (m_vehicleInterfaces.GetAddress(i) == senderIp)
                {
                    if (i < m_vehicleNodes.GetN())
                    {
                        senderNodeId = m_vehicleNodes.Get(i)->GetId();
                        if (m_mapper)
                        {
                            senderVehicleId = m_mapper->GetVehicleId(senderNodeId);
                        }
                    }
                    break;
                }
            }

            if (senderNodeId == rxNodeId)
            {
                continue;
            }

            double now = Simulator::Now().GetSeconds();
            double throughputMbps = ((packet->GetSize() + seqTs.GetSerializedSize()) * 8.0) /
                                    (m_v2xMessageInterval.GetSeconds() * 1e6);

            uint32_t currentSeq = seqTs.GetSeq();
            auto linkKey = std::make_pair(senderNodeId, rxNodeId);
            uint32_t loss = 0;
            if (m_hasRxSeq[linkKey])
            {
                if (currentSeq > m_lastRxSeq[linkKey] + 1)
                {
                    loss = currentSeq - (m_lastRxSeq[linkKey] + 1);
                }
            }
            else
            {
                m_hasRxSeq[linkKey] = true;
            }
            m_lastRxSeq[linkKey] = currentSeq;

            if (m_tracingEnabled)
            {
                if (m_networkCsv.is_open())
                {
                    m_networkCsv << std::fixed << std::setprecision(3)
                                 << now << "," << senderVehicleId << "," << rxNodeId << ","
                                 << "RxV2V," << std::setprecision(2) << delayMs << ","
                                 << std::setprecision(4) << throughputMbps << "," << loss << "\n";
                }
                if (m_commonCsv.is_open())
                {
                    m_commonCsv << std::fixed << std::setprecision(3)
                                << now << ",futurenet-v2x,RxV2V,"
                                << senderNodeId << "," << rxNodeId << ",2,"
                                << std::setprecision(2) << delayMs << ","
                                << std::setprecision(4) << throughputMbps << ",ms,"
                                << senderVehicleId << "\n";
                }
            }
        }
    }
}

void
V2xScenarioHelper::LogMobilitySample(std::string vehicleId, uint32_t nodeId, Vector position, double speedMps)
{
    if (m_tracingEnabled && m_mobilityCsv.is_open())
    {
        double now = Simulator::Now().GetSeconds();
        bool active = m_importer ? m_importer->IsVehicleActive(vehicleId) : true;
        m_mobilityCsv << std::fixed << std::setprecision(3)
                      << now << ","
                      << vehicleId << ","
                      << nodeId << ","
                      << position.x << ","
                      << position.y << ","
                      << position.z << ","
                      << std::fixed << std::setprecision(2) << speedMps << ","
                      << (active ? 1 : 0) << "\n";
    }
}

void
V2xScenarioHelper::LogVehicleActivated(std::string vehicleId, uint32_t nodeId)
{
    double now = Simulator::Now().GetSeconds();
    Vector pos(0.0, 0.0, 0.0);
    double speed = 0.0;
    if (m_mapper)
    {
        Ptr<Node> node = m_mapper->GetNode(vehicleId);
        if (node)
        {
            Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
            if (mob)
            {
                pos = mob->GetPosition();
                Vector vel = mob->GetVelocity();
                speed = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
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
                          << pos.x << "," << pos.y << "," << pos.z << ","
                          << std::fixed << std::setprecision(2) << speed << ",1\n";
        }
    }

    // Schedule V2I and V2V traffic generation with jitter drawn from m_jitterRng
    if (m_vehicleInterfaces.GetN() > 0)
    {
        Ptr<Node> vNode = m_mapper ? m_mapper->GetNode(vehicleId) : nullptr;
        if (vNode)
        {
            // V2I Sender Socket
            Ptr<Socket> v2iSocket = Socket::CreateSocket(vNode, UdpSocketFactory::GetTypeId());
            v2iSocket->SetAllowBroadcast(true);
            m_vehicleV2iSockets[vehicleId] = v2iSocket;
            m_v2iSeq[vehicleId] = 0;

            // V2V Sender & Receiver Socket on V2V_PORT (9998)
            Ptr<Socket> v2vSocket = Socket::CreateSocket(vNode, UdpSocketFactory::GetTypeId());
            v2vSocket->SetAllowBroadcast(true);
            InetSocketAddress localV2v = InetSocketAddress(Ipv4Address::GetAny(), V2V_PORT);
            v2vSocket->Bind(localV2v);
            v2vSocket->SetRecvCallback(MakeCallback(&V2xScenarioHelper::ReceiveVehicleV2vPacket, this));
            m_vehicleV2vSockets[vehicleId] = v2vSocket;
            m_v2vSeq[vehicleId] = 0;

            double maxJitterSec = m_startJitterMax.GetSeconds();
            double jitterSecV2I = (maxJitterSec > 0.0) ? m_jitterRng->GetValue(0.0, maxJitterSec) : 0.0;
            double jitterSecV2V = (maxJitterSec > 0.0) ? m_jitterRng->GetValue(0.0, maxJitterSec) : 0.0;

            m_sendV2iEvents[vehicleId] = Simulator::Schedule(Seconds(jitterSecV2I),
                                                             &V2xScenarioHelper::SendV2iMessage,
                                                             this,
                                                             vehicleId);

            m_sendV2vEvents[vehicleId] = Simulator::Schedule(Seconds(jitterSecV2V),
                                                             &V2xScenarioHelper::SendV2vMessage,
                                                             this,
                                                             vehicleId);
        }
    }
}

void
V2xScenarioHelper::LogVehicleDeactivated(std::string vehicleId, uint32_t nodeId)
{
    // Cancel pending send events
    if (m_sendV2iEvents.find(vehicleId) != m_sendV2iEvents.end())
    {
        Simulator::Cancel(m_sendV2iEvents[vehicleId]);
        m_sendV2iEvents.erase(vehicleId);
    }
    if (m_sendV2vEvents.find(vehicleId) != m_sendV2vEvents.end())
    {
        Simulator::Cancel(m_sendV2vEvents[vehicleId]);
        m_sendV2vEvents.erase(vehicleId);
    }

    // Close sockets
    if (m_vehicleV2iSockets.find(vehicleId) != m_vehicleV2iSockets.end())
    {
        m_vehicleV2iSockets[vehicleId]->Close();
        m_vehicleV2iSockets.erase(vehicleId);
    }
    if (m_vehicleV2vSockets.find(vehicleId) != m_vehicleV2vSockets.end())
    {
        m_vehicleV2vSockets[vehicleId]->Close();
        m_vehicleV2vSockets.erase(vehicleId);
    }

    // Capture final arrival position and speed prior to relocating off-channel
    double now = Simulator::Now().GetSeconds();
    Vector pos(0.0, 0.0, 0.0);
    double speed = 0.0;
    if (m_mapper)
    {
        Ptr<Node> node = m_mapper->GetNode(vehicleId);
        if (node)
        {
            Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
            if (mob)
            {
                pos = mob->GetPosition();
                Vector vel = mob->GetVelocity();
                speed = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
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
                          << pos.x << "," << pos.y << "," << pos.z << ","
                          << std::fixed << std::setprecision(2) << speed << ",0\n";
        }
    }

    // Disable applications and move deactivated node outside wireless range to exit channel
    if (m_mapper)
    {
        Ptr<Node> node = m_mapper->GetNode(vehicleId);
        if (node)
        {
            for (uint32_t a = 0; a < node->GetNApplications(); ++a)
            {
                node->GetApplication(a)->SetStopTime(Simulator::Now());
            }
            Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
            if (mob)
            {
                mob->SetPosition(Vector(-100000.0, -100000.0, -1000.0));
            }
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
