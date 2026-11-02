#ifndef V2X_SCENARIO_HELPER_H
#define V2X_SCENARIO_HELPER_H

#include "ns3/application-container.h"
#include "ns3/event-id.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"
#include "ns3/seq-ts-header.h"
#include "ns3/socket.h"
#include "ns3/traced-callback.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
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

    static constexpr uint16_t V2I_PORT = 9999;
    static constexpr uint16_t V2V_PORT = 9998;
    static constexpr double DEFAULT_RSU_HEIGHT_M = 5.0;

    /**
     * @brief Configure explicit coordinates for RSU nodes.
     * @param positions Vector of 3D positions in meters.
     */
    void SetRsuPositions(const std::vector<Vector>& positions);

    /**
     * @brief Configure regular 2D grid spacing for RSU deployment.
     */
    void SetRsuGridPlacement(double startX, double startY, double deltaX, double deltaY, uint32_t gridWidth);

    /**
     * @brief Configure junction-based placement for RSU deployment.
     */
    void SetRsuJunctionPlacement(const std::vector<Vector>& junctions);

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
     * @brief Flush and close all active CSV telemetry trace streams.
     */
    void CloseTracing();

    /**
     * @brief Set RSU selection policy.
     * @param policy Pointer to RSU selection policy.
     */
    void SetRsuSelectionPolicy(Ptr<RsuSelectionPolicy> policy);

    /**
     * @brief Assign deterministic streams to internal random variables.
     * @param stream Base stream index.
     * @return Number of assigned streams.
     */
    int64_t AssignStreams(int64_t stream);

    /**
     * @brief Get container of instantiated vehicle nodes.
     * @return Vehicle NodeContainer.
     */
    NodeContainer GetVehicleNodes() const;

    /**
     * @brief Get container of instantiated RSU nodes.
     * @return RSU NodeContainer.
     */
    NodeContainer GetRsuNodes() const;

    /**
     * @brief Get internal SUMO FCD parser instance.
     * @return Pointer to SumoFcdParser.
     */
    Ptr<SumoFcdParser> GetParser() const;

    /**
     * @brief Get internal vehicle-to-node mapper instance.
     * @return Pointer to VehicleNodeMapper.
     */
    Ptr<VehicleNodeMapper> GetMapper() const;

    /**
     * @brief Get internal mobility importer instance.
     * @return Pointer to V2xMobilityImporter.
     */
    Ptr<V2xMobilityImporter> GetImporter() const;

    /**
     * @brief Periodic trace callback logging vehicle mobility samples.
     */
    void LogMobilitySample(std::string vehicleId, uint32_t nodeId, Vector position, double speedMps);

    /**
     * @brief Lifecycle callback logging vehicle departure and starting apps.
     */
    void LogVehicleActivated(std::string vehicleId, uint32_t nodeId);

    /**
     * @brief Lifecycle callback logging vehicle arrival, closing apps and disabling radio.
     */
    void LogVehicleDeactivated(std::string vehicleId, uint32_t nodeId);

    /**
     * @brief Callback logging RSU association and handover events.
     */
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
    double m_txPowerDbm{21.5};
    double m_rsuHeightM{DEFAULT_RSU_HEIGHT_M};
    std::string m_subnetBase{"10.1.0.0"};
    std::string m_subnetMask{"255.255.0.0"};
    Ipv4Address m_broadcastAddress;

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

    // RSU Placement settings
    std::vector<Vector> m_rsuPositions;
    std::vector<Vector> m_rsuJunctions;
    double m_rsuGridStartX{50.0};
    double m_rsuGridStartY{50.0};
    double m_rsuGridDeltaX{200.0};
    double m_rsuGridDeltaY{200.0};
    uint32_t m_rsuGridWidth{2};

    // V2X Application Sockets & Lifecycle tracking (V2I + V2V)
    std::map<std::string, Ptr<Socket>> m_vehicleV2iSockets;
    std::map<std::string, Ptr<Socket>> m_vehicleV2vSockets;
    std::map<std::string, EventId> m_sendV2iEvents;
    std::map<std::string, EventId> m_sendV2vEvents;
    std::map<std::string, uint32_t> m_v2iSeq;
    std::map<std::string, uint32_t> m_v2vSeq;
    std::vector<Ptr<Socket>> m_rsuSockets;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> m_lastRxSeq;
    std::map<std::pair<uint32_t, uint32_t>, bool> m_hasRxSeq;

    // CSV File streams
    std::ofstream m_mobilityCsv;
    std::ofstream m_networkCsv;
    std::ofstream m_commonCsv;
    bool m_tracingEnabled{false};

    void PeriodicRsuCheck();
    void SendV2iMessage(std::string vehicleId);
    void SendV2vMessage(std::string vehicleId);
    void ReceiveRsuPacket(Ptr<Socket> socket);
    void ReceiveVehicleV2vPacket(Ptr<Socket> socket);
};

} // namespace ns3

#endif // V2X_SCENARIO_HELPER_H
