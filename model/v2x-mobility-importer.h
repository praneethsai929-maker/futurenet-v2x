#ifndef V2X_MOBILITY_IMPORTER_H
#define V2X_MOBILITY_IMPORTER_H

#include "ns3/event-id.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/traced-callback.h"
#include "ns3/vector.h"
#include "sumo-fcd-parser.h"
#include "vehicle-node-mapper.h"

#include <map>
#include <string>

namespace ns3
{

/**
 * @ingroup futurenet-v2x
 * @brief Imports SUMO FCD trajectories into ns-3 WaypointMobilityModel instances.
 *
 * Implements M09-FR-03, M09-FR-04, and M09-FR-08.
 * Handles vehicle lifecycle (departure/arrival) and periodic sampled position tracing.
 */
class V2xMobilityImporter : public Object
{
public:
    static TypeId GetTypeId();

    V2xMobilityImporter();
    ~V2xMobilityImporter() override;

    /**
     * @brief Install waypoints and schedule lifecycle events using parser and mapper.
     * @param parser Pointer to SumoFcdParser containing trajectory data.
     * @param mapper Pointer to VehicleNodeMapper connecting vehicle IDs to nodes.
     */
    void Install(Ptr<SumoFcdParser> parser, Ptr<VehicleNodeMapper> mapper);

    /**
     * @brief Simplified install method matching SRS Section 6 signature.
     * @param parser SumoFcdParser reference.
     * @param nodes NodeContainer of vehicles (must match parser vehicle count).
     */
    void Install(const SumoFcdParser& parser, NodeContainer nodes);

    /**
     * @brief Get earliest departure time for a vehicle.
     */
    Time GetFirstSeen(const std::string& vehicleId) const;

    /**
     * @brief Get latest arrival/departure time for a vehicle.
     */
    Time GetLastSeen(const std::string& vehicleId) const;

    /**
     * @brief Check if a vehicle is currently active in the simulation.
     */
    bool IsVehicleActive(const std::string& vehicleId) const;

    /**
     * @brief Set sampling interval for position tracing.
     */
    void SetMobilityStep(Time step);

    /**
     * @brief Get sampling interval for position tracing.
     */
    Time GetMobilityStep() const;

    /**
     * @brief TraceSources for lifecycle and mobility.
     */
    typedef void (*VehicleLifecycleTracedCallback)(std::string vehicleId, uint32_t nodeId);
    typedef void (*VehiclePositionTracedCallback)(std::string vehicleId, uint32_t nodeId, Vector position);

private:
    Time m_mobilityStep{MilliSeconds(100)}; //!< Position sampling step (10ms to 2s)

    Ptr<SumoFcdParser> m_parser;
    Ptr<VehicleNodeMapper> m_mapper;

    std::map<std::string, Time> m_firstSeen;
    std::map<std::string, Time> m_lastSeen;
    std::map<std::string, bool> m_activeStatus;

    EventId m_sampleEvent;

    TracedCallback<std::string, uint32_t> m_vehicleActivatedTrace;
    TracedCallback<std::string, uint32_t> m_vehicleDeactivatedTrace;
    TracedCallback<std::string, uint32_t, Vector> m_vehiclePositionTrace;

    void ScheduleLifecycleEvents();
    void OnVehicleActivated(std::string vehicleId, uint32_t nodeId);
    void OnVehicleDeactivated(std::string vehicleId, uint32_t nodeId);
    void SamplePositions();
};

} // namespace ns3

#endif // V2X_MOBILITY_IMPORTER_H
