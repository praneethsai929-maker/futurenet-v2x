#ifndef V2X_SCENARIO_HELPER_H
#define V2X_SCENARIO_HELPER_H

#include "ns3/application-container.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"
#include "ns3/traced-callback.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"

#include "../model/rsu-selection-policy.h"
#include "../model/sumo-fcd-parser.h"
#include "../model/v2x-mobility-importer.h"
#include "../model/vehicle-node-mapper.h"

#include <fstream>
#include <string>

namespace ns3
{

/**
 * @ingroup futurenet-v2x
 * @brief High-level scenario orchestrator for V2X simulations with SUMO FCD traces.
 *
 * Implements M09-FR-06, M09-FR-08, and Section 6 of the SRS.
 * Manages vehicle nodes, RSUs, wireless ad hoc channel, V2V/V2I applications,
 * and comprehensive CSV logging matching the FutureNet Common Guidelines.
 */
class V2xScenarioHelper : public Object
{
public:
    static TypeId GetTypeId();

    V2xScenarioHelper();
    ~V2xScenarioHelper() override;

    /**
     * @brief Load SUMO FCD trajectory XML file.
     * @param filename Path to FCD XML file.
     * @return true on success.
     */
    bool LoadFcd(const std::string& filename);

    /**
     * @brief Create ns-3 nodes for all vehicles in the loaded trace.
     * @return NodeContainer of created vehicle nodes.
     */
    NodeContainer CreateVehicleNodes();

    /**
     * @brief Create roadside unit (RSU) nodes.
     * @param numRsus Number of RSUs to create.
     * @return NodeContainer of created RSU nodes.
     */
    NodeContainer CreateRsuNodes(uint32_t numRsus);

    /**
     * @brief Install waypoint mobility onto vehicle nodes and schedule lifecycle.
     */
    void InstallMobility();

    /**
     * @brief Install Wi-Fi (802.11p/adhoc) and IP networking on vehicles and RSUs.
     */
    void InstallExampleNetworking();

    /**
     * @brief Enable CSV tracing for mobility, network events, and common guideline schema.
     * @param prefix Prefix path for generated CSV files.
     */
    void EnableTracing(const std::string& prefix);

    /**
     * @brief Set RSU selection policy.
     */
    void SetRsuSelectionPolicy(Ptr<RsuSelectionPolicy> policy);

    /**
     * @brief Assign deterministic streams to internal random variables.
     */
    int64_t AssignStreams(int64_t stream);

    // Getters for containers
    NodeContainer GetVehicleNodes() const;
    NodeContainer GetRsuNodes() const;
    Ptr<SumoFcdParser> GetParser() const;
    Ptr<VehicleNodeMapper> GetMapper() const;
    Ptr<V2xMobilityImporter> GetImporter() const;

    // Callbacks for traces
    void LogMobilitySample(std::string vehicleId, uint32_t nodeId, Vector position);
    void LogVehicleActivated(std::string vehicleId, uint32_t nodeId);
    void LogVehicleDeactivated(std::string vehicleId, uint32_t nodeId);
    void LogRsuAssociation(std::string vehicleId, uint32_t oldRsuNodeId, uint32_t newRsuNodeId);

private:
    Time m_mobilityStep{MilliSeconds(100)};
    uint32_t m_numRsus{4};
    double m_coordinateScale{1.0};
    double m_coordinateOffsetX{0.0};
    double m_coordinateOffsetY{0.0};

    uint32_t m_v2xMessageBytes{300};
    Time m_v2xMessageInterval{MilliSeconds(100)};
    Time m_startJitterMax{MilliSeconds(100)};

    Ptr<SumoFcdParser> m_parser;
    Ptr<VehicleNodeMapper> m_mapper;
    Ptr<V2xMobilityImporter> m_importer;
    Ptr<RsuSelectionPolicy> m_rsuPolicy;

    NodeContainer m_vehicleNodes;
    NodeContainer m_rsuNodes;
    NetDeviceContainer m_vehicleDevices;
    NetDeviceContainer m_rsuDevices;
    Ipv4InterfaceContainer m_vehicleInterfaces;
    Ipv4InterfaceContainer m_rsuInterfaces;

    Ptr<UniformRandomVariable> m_jitterRng;

    std::map<std::string, uint32_t> m_currentRsuAssociation;

    // CSV File streams
    std::ofstream m_mobilityCsv;
    std::ofstream m_networkCsv;
    std::ofstream m_commonCsv;
    bool m_tracingEnabled{false};

    void PeriodicRsuCheck();
};

} // namespace ns3

#endif // V2X_SCENARIO_HELPER_H
