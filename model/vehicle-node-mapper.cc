#include "vehicle-node-mapper.h"

#include "ns3/assert.h"
#include "ns3/log.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("VehicleNodeMapper");
NS_OBJECT_ENSURE_REGISTERED(VehicleNodeMapper);

TypeId
VehicleNodeMapper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::VehicleNodeMapper")
            .SetParent<Object>()
            .SetGroupName("FuturenetV2x")
            .AddConstructor<VehicleNodeMapper>();
    return tid;
}

VehicleNodeMapper::VehicleNodeMapper()
{
    NS_LOG_FUNCTION(this);
}

VehicleNodeMapper::~VehicleNodeMapper()
{
    NS_LOG_FUNCTION(this);
}

void
VehicleNodeMapper::Register(const std::string& vehicleId, Ptr<Node> node)
{
    NS_LOG_FUNCTION(this << vehicleId << node);
    NS_ASSERT_MSG(!vehicleId.empty(), "Vehicle ID cannot be empty");
    NS_ASSERT_MSG(node != nullptr, "Node pointer cannot be null");

    uint32_t nodeId = node->GetId();
    m_vehicleToNode[vehicleId] = node;
    m_nodeToVehicle[nodeId] = vehicleId;
    NS_LOG_INFO("Mapped SUMO vehicle '" << vehicleId << "' <-> ns-3 Node " << nodeId);
}

Ptr<Node>
VehicleNodeMapper::GetNode(const std::string& vehicleId) const
{
    auto it = m_vehicleToNode.find(vehicleId);
    if (it != m_vehicleToNode.end())
    {
        return it->second;
    }
    return nullptr;
}

std::string
VehicleNodeMapper::GetVehicleId(uint32_t nodeId) const
{
    auto it = m_nodeToVehicle.find(nodeId);
    if (it != m_nodeToVehicle.end())
    {
        return it->second;
    }
    return "";
}

uint32_t
VehicleNodeMapper::GetNodeId(const std::string& vehicleId) const
{
    auto it = m_vehicleToNode.find(vehicleId);
    if (it != m_vehicleToNode.end() && it->second != nullptr)
    {
        return it->second->GetId();
    }
    return UINT32_MAX;
}

bool
VehicleNodeMapper::HasVehicle(const std::string& vehicleId) const
{
    return m_vehicleToNode.find(vehicleId) != m_vehicleToNode.end();
}

bool
VehicleNodeMapper::HasNode(uint32_t nodeId) const
{
    return m_nodeToVehicle.find(nodeId) != m_nodeToVehicle.end();
}

uint32_t
VehicleNodeMapper::GetSize() const
{
    return static_cast<uint32_t>(m_vehicleToNode.size());
}

NodeContainer
VehicleNodeMapper::GetAllNodes() const
{
    NodeContainer container;
    for (const auto& kv : m_vehicleToNode)
    {
        container.Add(kv.second);
    }
    return container;
}

std::vector<std::string>
VehicleNodeMapper::GetAllVehicleIds() const
{
    std::vector<std::string> ids;
    ids.reserve(m_vehicleToNode.size());
    for (const auto& kv : m_vehicleToNode)
    {
        ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

void
VehicleNodeMapper::Clear()
{
    NS_LOG_FUNCTION(this);
    m_vehicleToNode.clear();
    m_nodeToVehicle.clear();
}

} // namespace ns3
