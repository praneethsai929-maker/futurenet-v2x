#ifndef VEHICLE_WAYPOINT_H
#define VEHICLE_WAYPOINT_H

#include "ns3/nstime.h"
#include "ns3/vector.h"
#include <string>

namespace ns3
{

/**
 * @ingroup futurenet-v2x
 * @brief Represents a single vehicle waypoint from SUMO FCD output.
 */
struct VehicleWaypoint
{
    Time timestamp{Seconds(0)}; //!< Simulation timestamp of the waypoint
    Vector position{0.0, 0.0, 0.0}; //!< 3D Position in meters (x, y, z)
    double speedMps{0.0}; //!< Speed in meters per second
    double angleDeg{0.0}; //!< Heading angle in degrees
    std::string laneId{""}; //!< SUMO lane ID
    double posOnLane{0.0}; //!< Position along the current lane
};

} // namespace ns3

#endif // VEHICLE_WAYPOINT_H
