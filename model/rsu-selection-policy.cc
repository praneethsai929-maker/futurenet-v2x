#include "rsu-selection-policy.h"

#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("RsuSelectionPolicy");
NS_OBJECT_ENSURE_REGISTERED(RsuSelectionPolicy);
NS_OBJECT_ENSURE_REGISTERED(NearestRsuSelectionPolicy);

TypeId
RsuSelectionPolicy::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::RsuSelectionPolicy")
            .SetParent<Object>()
            .SetGroupName("FuturenetV2x");
    return tid;
}

TypeId
NearestRsuSelectionPolicy::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NearestRsuSelectionPolicy")
            .SetParent<RsuSelectionPolicy>()
            .SetGroupName("FuturenetV2x")
            .AddConstructor<NearestRsuSelectionPolicy>();
    return tid;
}

NearestRsuSelectionPolicy::NearestRsuSelectionPolicy()
{
    NS_LOG_FUNCTION(this);
}

NearestRsuSelectionPolicy::~NearestRsuSelectionPolicy()
{
    NS_LOG_FUNCTION(this);
}

RsuSelectionDecision
NearestRsuSelectionPolicy::Select(const RsuSelectionState& state)
{
    NS_LOG_FUNCTION(this << state.vehicleNodeId << state.candidateRsus.GetN());

    RsuSelectionDecision decision;
    decision.valid = false;
    decision.distanceM = std::numeric_limits<double>::max();

    if (state.candidateRsus.GetN() == 0)
    {
        return decision;
    }

    for (uint32_t i = 0; i < state.candidateRsus.GetN(); ++i)
    {
        Ptr<Node> rsuNode = state.candidateRsus.Get(i);
        Ptr<MobilityModel> mob = rsuNode->GetObject<MobilityModel>();
        if (mob == nullptr)
        {
            continue;
        }

        Vector rsuPos = mob->GetPosition();
        double dx = state.vehiclePosition.x - rsuPos.x;
        double dy = state.vehiclePosition.y - rsuPos.y;
        double dz = state.vehiclePosition.z - rsuPos.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < decision.distanceM)
        {
            decision.distanceM = dist;
            decision.selectedRsu = rsuNode;
            decision.valid = true;
        }
    }

    return decision;
}

} // namespace ns3
