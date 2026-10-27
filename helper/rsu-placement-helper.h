#ifndef RSU_PLACEMENT_HELPER_H
#define RSU_PLACEMENT_HELPER_H

#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/vector.h"

#include <vector>

namespace ns3
{

/**
 * @ingroup futurenet-v2x
 * @brief Helper class to deploy and position Roadside Units (RSUs).
 *
 * Implements M09-FR-05, M09-FR-11, and AC-04.
 * Supports explicit coordinates, regular grid spacing, and junction-based placements.
 */
class RsuPlacementHelper : public Object
{
public:
    static TypeId GetTypeId();

    RsuPlacementHelper();
    ~RsuPlacementHelper() override;

    /**
     * @brief Create a container of RSU nodes.
     * @param numRsus Number of RSU nodes to instantiate.
     * @return NodeContainer containing newly created RSU nodes.
     */
    NodeContainer CreateRsus(uint32_t numRsus);

    /**
     * @brief Install fixed positions for RSUs from a vector of explicit coordinates.
     * @param rsus NodeContainer of RSU nodes.
     * @param positions Vector of 3D positions in meters.
     */
    void InstallPositions(NodeContainer rsus, const std::vector<Vector>& positions);

    /**
     * @brief Install RSUs on a regular 2D grid spacing.
     * @param rsus NodeContainer of RSU nodes.
     * @param startX Starting X coordinate in meters.
     * @param startY Starting Y coordinate in meters.
     * @param deltaX Spacing between columns in meters.
     * @param deltaY Spacing between rows in meters.
     * @param gridWidth Number of RSUs per row.
     */
    void InstallGridSpacing(NodeContainer rsus,
                            double startX,
                            double startY,
                            double deltaX,
                            double deltaY,
                            uint32_t gridWidth);

    /**
     * @brief Install RSUs at road intersection junction coordinates.
     * @param rsus NodeContainer of RSU nodes.
     * @param junctions Vector of junction positions.
     */
    void InstallJunctionList(NodeContainer rsus, const std::vector<Vector>& junctions);
};

} // namespace ns3

#endif // RSU_PLACEMENT_HELPER_H
