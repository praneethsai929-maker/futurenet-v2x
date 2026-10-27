#ifndef VEHICLE_NODE_MAPPER_H
#define VEHICLE_NODE_MAPPER_H

#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * @ingroup futurenet-v2x
 * @brief Deterministic bidirectional mapping between SUMO vehicle IDs and ns-3 nodes.
 *
 * Implements M09-FR-02 and test T06.
 */
class VehicleNodeMapper : public Object
{
public:
    static TypeId GetTypeId();

    VehicleNodeMapper();
    ~VehicleNodeMapper() override;

    /**
     * @brief Register a mapping between a SUMO vehicle ID and an ns-3 node.
     * @param vehicleId SUMO vehicle identifier.
     * @param node Pointer to ns-3 Node.
     */
    void Register(const std::string& vehicleId, Ptr<Node> node);

    /**
     * @brief Look up the ns-3 node associated with a vehicle ID.
     * @param vehicleId SUMO vehicle identifier.
     * @return Pointer to Node, or nullptr if not found.
     */
    Ptr<Node> GetNode(const std::string& vehicleId) const;

    /**
     * @brief Look up the vehicle ID associated with an ns-3 node ID.
     * @param nodeId ns-3 Node ID.
     * @return Vehicle ID string, or empty string if not found.
     */
    std::string GetVehicleId(uint32_t nodeId) const;

    /**
     * @brief Look up the ns-3 node ID for a given vehicle ID.
     * @param vehicleId SUMO vehicle identifier.
     * @return Node ID, or UINT32_MAX if not found.
     */
    uint32_t GetNodeId(const std::string& vehicleId) const;

    /**
     * @brief Check whether a vehicle ID is registered.
     */
    bool HasVehicle(const std::string& vehicleId) const;

    /**
     * @brief Check whether a node ID is registered.
     */
    bool HasNode(uint32_t nodeId) const;

    /**
     * @brief Get the total number of registered mappings.
     */
    uint32_t GetSize() const;

    /**
     * @brief Get a NodeContainer containing all registered nodes.
     */
    NodeContainer GetAllNodes() const;

    /**
     * @brief Get sorted list of all registered vehicle IDs.
     */
    std::vector<std::string> GetAllVehicleIds() const;

    /**
     * @brief Clear all registrations.
     */
    void Clear();

private:
    std::map<std::string, Ptr<Node>> m_vehicleToNode;
    std::map<uint32_t, std::string> m_nodeToVehicle;
};

} // namespace ns3

#endif // VEHICLE_NODE_MAPPER_H
