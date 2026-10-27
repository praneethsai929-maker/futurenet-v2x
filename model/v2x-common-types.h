#ifndef V2X_COMMON_TYPES_H
#define V2X_COMMON_TYPES_H

#include "ns3/nstime.h"
#include "ns3/vector.h"
#include <cmath>
#include <cstdint>

namespace ns3
{

/**
 * @ingroup futurenet-v2x
 * @brief Common Guidelines 4.1 NetworkState representation.
 */
struct NetworkState
{
    Time timestamp;
    uint32_t nodeId{0};
    double oneWayDelayMs{NAN};
    double throughputMbps{0.0};
    double packetLossRatio{0.0};
    double availableBandwidthMbps{0.0};
    uint32_t queuePackets{0};
    double nodeLoad{0.0};
    double energyJ{0.0};
    bool energyValid{false};
};

/**
 * @ingroup futurenet-v2x
 * @brief Common Guidelines 4.2 LinkState representation.
 */
struct LinkState
{
    uint32_t sourceNodeId{0};
    uint32_t destinationNodeId{0};
    bool available{false};
    double dataRateMbps{0.0};
    double propagationDelayMs{0.0};
    double lossRatio{0.0};
};

/**
 * @ingroup futurenet-v2x
 * @brief Common Guidelines 4.3 MobilityState representation.
 */
struct MobilityState
{
    uint32_t nodeId{0};
    Vector positionM{0.0, 0.0, 0.0};
    Vector velocityMps{0.0, 0.0, 0.0};
    Time timestamp{Seconds(0)};
};

/**
 * @ingroup futurenet-v2x
 * @brief Common Guidelines 4.4 Traffic metadata.
 */
struct FutureNetTrafficClass
{
    uint8_t priority{0};
    double importance{1.0};
    Time deadline{Seconds(0)};
    uint32_t flowId{0};
};

} // namespace ns3

#endif // V2X_COMMON_TYPES_H
