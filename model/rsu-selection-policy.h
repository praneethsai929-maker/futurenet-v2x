#ifndef RSU_SELECTION_POLICY_H
#define RSU_SELECTION_POLICY_H

#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/vector.h"

#include <vector>

namespace ns3
{

/**
 * @ingroup futurenet-v2x
 * @brief Input state provided to RsuSelectionPolicy to decide association.
 */
struct RsuSelectionState
{
    uint32_t vehicleNodeId{0};
    Vector vehiclePosition{0.0, 0.0, 0.0};
    NodeContainer candidateRsus;
};

/**
 * @ingroup futurenet-v2x
 * @brief Decision returned by RsuSelectionPolicy.
 */
struct RsuSelectionDecision
{
    Ptr<Node> selectedRsu{nullptr};
    double distanceM{0.0};
    bool valid{false};
};

/**
 * @ingroup futurenet-v2x
 * @brief Base abstract policy interface for RSU selection (Common Guidelines 4.6).
 *
 * Separates association mechanism from decision algorithm, allowing M10 Agent
 * or RL policies to replace it seamlessly.
 */
class RsuSelectionPolicy : public Object
{
public:
    static TypeId GetTypeId();
    ~RsuSelectionPolicy() override = default;

    /**
     * @brief Select the most appropriate RSU for a given vehicle state.
     * @param state Current vehicle position and candidate RSUs.
     * @return RsuSelectionDecision containing the chosen RSU.
     */
    virtual RsuSelectionDecision Select(const RsuSelectionState& state) = 0;
};

/**
 * @ingroup futurenet-v2x
 * @brief Deterministic baseline policy selecting the geometrically closest RSU.
 */
class NearestRsuSelectionPolicy : public RsuSelectionPolicy
{
public:
    static TypeId GetTypeId();

    NearestRsuSelectionPolicy();
    ~NearestRsuSelectionPolicy() override;

    RsuSelectionDecision Select(const RsuSelectionState& state) override;
};

} // namespace ns3

#endif // RSU_SELECTION_POLICY_H
