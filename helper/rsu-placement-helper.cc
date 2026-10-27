#include "rsu-placement-helper.h"

#include "ns3/constant-position-mobility-model.h"
#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("RsuPlacementHelper");
NS_OBJECT_ENSURE_REGISTERED(RsuPlacementHelper);

TypeId
RsuPlacementHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::RsuPlacementHelper")
            .SetParent<Object>()
            .SetGroupName("FuturenetV2x")
            .AddConstructor<RsuPlacementHelper>();
    return tid;
}

RsuPlacementHelper::RsuPlacementHelper()
{
    NS_LOG_FUNCTION(this);
}

RsuPlacementHelper::~RsuPlacementHelper()
{
    NS_LOG_FUNCTION(this);
}

NodeContainer
RsuPlacementHelper::CreateRsus(uint32_t numRsus)
{
    NS_LOG_FUNCTION(this << numRsus);
    NodeContainer rsus;
    rsus.Create(numRsus);
    return rsus;
}

void
RsuPlacementHelper::InstallPositions(NodeContainer rsus, const std::vector<Vector>& positions)
{
    NS_LOG_FUNCTION(this << rsus.GetN() << positions.size());
    uint32_t n = std::min(rsus.GetN(), static_cast<uint32_t>(positions.size()));
    for (uint32_t i = 0; i < n; ++i)
    {
        Ptr<Node> node = rsus.Get(i);
        Ptr<ConstantPositionMobilityModel> mob = node->GetObject<ConstantPositionMobilityModel>();
        if (mob == nullptr)
        {
            mob = CreateObject<ConstantPositionMobilityModel>();
            node->AggregateObject(mob);
        }
        mob->SetPosition(positions[i]);
        NS_LOG_INFO("RSU node " << node->GetId() << " placed at (" << positions[i].x << ", "
                                << positions[i].y << ", " << positions[i].z << ")");
    }
}

void
RsuPlacementHelper::InstallGridSpacing(NodeContainer rsus,
                                      double startX,
                                      double startY,
                                      double deltaX,
                                      double deltaY,
                                      uint32_t gridWidth)
{
    NS_LOG_FUNCTION(this << rsus.GetN() << startX << startY << deltaX << deltaY << gridWidth);
    if (gridWidth == 0)
    {
        gridWidth = 1;
    }

    std::vector<Vector> positions;
    positions.reserve(rsus.GetN());

    for (uint32_t i = 0; i < rsus.GetN(); ++i)
    {
        uint32_t col = i % gridWidth;
        uint32_t row = i / gridWidth;
        double x = startX + col * deltaX;
        double y = startY + row * deltaY;
        positions.emplace_back(x, y, 5.0); // 5m elevated mast height for RSU
    }

    InstallPositions(rsus, positions);
}

void
RsuPlacementHelper::InstallJunctionList(NodeContainer rsus, const std::vector<Vector>& junctions)
{
    NS_LOG_FUNCTION(this << rsus.GetN() << junctions.size());
    InstallPositions(rsus, junctions);
}

} // namespace ns3
