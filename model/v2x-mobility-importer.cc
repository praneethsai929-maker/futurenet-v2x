#include "v2x-mobility-importer.h"

#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"
#include "ns3/waypoint.h"
#include "ns3/waypoint-mobility-model.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("V2xMobilityImporter");
NS_OBJECT_ENSURE_REGISTERED(V2xMobilityImporter);

TypeId
V2xMobilityImporter::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::V2xMobilityImporter")
            .SetParent<Object>()
            .SetGroupName("FuturenetV2x")
            .AddConstructor<V2xMobilityImporter>()
            .AddAttribute("MobilityStep",
                          "Sampling step interval for periodic vehicle position tracing",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&V2xMobilityImporter::m_mobilityStep),
                          MakeTimeChecker(MilliSeconds(10), Seconds(2)))
            .AddTraceSource("VehicleActivated",
                            "Fired when a vehicle enters the simulation area",
                            MakeTraceSourceAccessor(&V2xMobilityImporter::m_vehicleActivatedTrace),
                            "ns3::V2xMobilityImporter::VehicleLifecycleTracedCallback")
            .AddTraceSource("VehicleDeactivated",
                            "Fired when a vehicle departs the simulation area",
                            MakeTraceSourceAccessor(&V2xMobilityImporter::m_vehicleDeactivatedTrace),
                            "ns3::V2xMobilityImporter::VehicleLifecycleTracedCallback")
            .AddTraceSource("VehiclePosition",
                            "Fired periodically with sampled vehicle positions",
                            MakeTraceSourceAccessor(&V2xMobilityImporter::m_vehiclePositionTrace),
                            "ns3::V2xMobilityImporter::VehiclePositionTracedCallback");
    return tid;
}

V2xMobilityImporter::V2xMobilityImporter()
{
    NS_LOG_FUNCTION(this);
}

V2xMobilityImporter::~V2xMobilityImporter()
{
    NS_LOG_FUNCTION(this);
    if (m_sampleEvent.IsPending())
    {
        m_sampleEvent.Cancel();
    }
}

void
V2xMobilityImporter::Install(Ptr<SumoFcdParser> parser, Ptr<VehicleNodeMapper> mapper)
{
    NS_LOG_FUNCTION(this << parser << mapper);
    NS_ASSERT_MSG(parser != nullptr, "SumoFcdParser cannot be null");
    NS_ASSERT_MSG(mapper != nullptr, "VehicleNodeMapper cannot be null");

    m_parser = parser;
    m_mapper = mapper;

    std::vector<std::string> vehicleIds = m_parser->GetVehicleIds();
    for (const auto& vid : vehicleIds)
    {
        Ptr<Node> node = m_mapper->GetNode(vid);
        if (node == nullptr)
        {
            NS_LOG_WARN("No node mapped for vehicle '" << vid << "', skipping waypoint installation");
            continue;
        }

        Ptr<WaypointMobilityModel> mobility = node->GetObject<WaypointMobilityModel>();
        if (mobility == nullptr)
        {
            mobility = CreateObject<WaypointMobilityModel>();
            node->AggregateObject(mobility);
        }

        std::vector<VehicleWaypoint> trajectory = m_parser->GetTrajectory(vid);
        for (const auto& wp : trajectory)
        {
            mobility->AddWaypoint(Waypoint(wp.timestamp, wp.position));
        }

        m_firstSeen[vid] = m_parser->GetFirstSeen(vid);
        m_lastSeen[vid] = m_parser->GetLastSeen(vid);
        m_activeStatus[vid] = false;
    }

    ScheduleLifecycleEvents();

    // Start periodic position sampling
    if (!m_sampleEvent.IsPending() && m_mobilityStep.IsStrictlyPositive())
    {
        m_sampleEvent = Simulator::Schedule(m_mobilityStep, &V2xMobilityImporter::SamplePositions, this);
    }
}

void
V2xMobilityImporter::Install(const SumoFcdParser& parser, NodeContainer nodes)
{
    NS_LOG_FUNCTION(this << nodes.GetN());
    Ptr<VehicleNodeMapper> mapper = CreateObject<VehicleNodeMapper>();
    std::vector<std::string> vehicleIds = parser.GetVehicleIds();

    NS_ASSERT_MSG(nodes.GetN() >= vehicleIds.size(),
                  "NodeContainer must have at least as many nodes as vehicles in FCD trace");

    for (size_t i = 0; i < vehicleIds.size(); ++i)
    {
        mapper->Register(vehicleIds[i], nodes.Get(i));
    }

    Ptr<SumoFcdParser> parserPtr = const_cast<SumoFcdParser*>(&parser);
    Install(parserPtr, mapper);
}

void
V2xMobilityImporter::ScheduleLifecycleEvents()
{
    NS_LOG_FUNCTION(this);
    for (const auto& kv : m_firstSeen)
    {
        const std::string& vid = kv.first;
        Time firstTime = kv.second;
        Time lastTime = m_lastSeen[vid];
        uint32_t nodeId = m_mapper->GetNodeId(vid);

        if (firstTime >= Seconds(0))
        {
            Simulator::Schedule(firstTime, &V2xMobilityImporter::OnVehicleActivated, this, vid, nodeId);
        }
        if (lastTime >= firstTime)
        {
            Simulator::Schedule(lastTime, &V2xMobilityImporter::OnVehicleDeactivated, this, vid, nodeId);
        }
    }
}

void
V2xMobilityImporter::OnVehicleActivated(std::string vehicleId, uint32_t nodeId)
{
    NS_LOG_FUNCTION(this << vehicleId << nodeId);
    m_activeStatus[vehicleId] = true;
    m_vehicleActivatedTrace(vehicleId, nodeId);
    NS_LOG_INFO("Vehicle " << vehicleId << " (Node " << nodeId << ") ACTIVATED at " << Simulator::Now().GetSeconds() << "s");
}

void
V2xMobilityImporter::OnVehicleDeactivated(std::string vehicleId, uint32_t nodeId)
{
    NS_LOG_FUNCTION(this << vehicleId << nodeId);
    m_activeStatus[vehicleId] = false;
    m_vehicleDeactivatedTrace(vehicleId, nodeId);
    NS_LOG_INFO("Vehicle " << vehicleId << " (Node " << nodeId << ") DEACTIVATED at " << Simulator::Now().GetSeconds() << "s");
}

void
V2xMobilityImporter::SamplePositions()
{
    NS_LOG_FUNCTION(this);
    if (m_mapper != nullptr)
    {
        std::vector<std::string> vids = m_mapper->GetAllVehicleIds();
        for (const auto& vid : vids)
        {
            if (IsVehicleActive(vid))
            {
                Ptr<Node> node = m_mapper->GetNode(vid);
                if (node != nullptr)
                {
                    Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
                    if (mob != nullptr)
                    {
                        Vector pos = mob->GetPosition();
                        m_vehiclePositionTrace(vid, node->GetId(), pos);
                    }
                }
            }
        }
    }

    m_sampleEvent = Simulator::Schedule(m_mobilityStep, &V2xMobilityImporter::SamplePositions, this);
}

Time
V2xMobilityImporter::GetFirstSeen(const std::string& vehicleId) const
{
    auto it = m_firstSeen.find(vehicleId);
    if (it != m_firstSeen.end())
    {
        return it->second;
    }
    return Seconds(-1);
}

Time
V2xMobilityImporter::GetLastSeen(const std::string& vehicleId) const
{
    auto it = m_lastSeen.find(vehicleId);
    if (it != m_lastSeen.end())
    {
        return it->second;
    }
    return Seconds(-1);
}

bool
V2xMobilityImporter::IsVehicleActive(const std::string& vehicleId) const
{
    auto it = m_activeStatus.find(vehicleId);
    if (it != m_activeStatus.end())
    {
        return it->second;
    }
    return false;
}

void
V2xMobilityImporter::SetMobilityStep(Time step)
{
    m_mobilityStep = step;
}

Time
V2xMobilityImporter::GetMobilityStep() const
{
    return m_mobilityStep;
}

} // namespace ns3
