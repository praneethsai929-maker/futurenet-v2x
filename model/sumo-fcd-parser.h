#ifndef SUMO_FCD_PARSER_H
#define SUMO_FCD_PARSER_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "vehicle-waypoint.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * @ingroup futurenet-v2x
 * @brief Stream XML parser for SUMO Floating Car Data (FCD) trajectory output files.
 *
 * Implements M09-FR-01, M09-FR-07, and AC-01.
 * Supports configurable coordinate scaling and offsets via ns-3 Attributes.
 */
class SumoFcdParser : public Object
{
public:
    static TypeId GetTypeId();

    SumoFcdParser();
    ~SumoFcdParser() override;

    /**
     * @brief Load and parse a SUMO FCD XML trajectory file.
     * @param filename Path to the FCD XML file.
     * @return true if successfully parsed, false if missing or malformed.
     */
    bool Load(const std::string& filename);

    /**
     * @brief Get sorted list of unique vehicle IDs discovered in the trace.
     * @return Sorted vector of vehicle ID strings.
     */
    std::vector<std::string> GetVehicleIds() const;

    /**
     * @brief Get the ordered trajectory waypoints for a specific vehicle.
     * @param vehicleId SUMO vehicle ID string.
     * @return Vector of VehicleWaypoint structs in time order.
     */
    std::vector<VehicleWaypoint> GetTrajectory(const std::string& vehicleId) const;

    /**
     * @brief Get the earliest timestamp when the vehicle appeared in the trace.
     * @param vehicleId SUMO vehicle ID string.
     * @return Timestamp (ns3::Time).
     */
    Time GetFirstSeen(const std::string& vehicleId) const;

    /**
     * @brief Get the latest timestamp when the vehicle was present in the trace.
     * @param vehicleId SUMO vehicle ID string.
     * @return Timestamp (ns3::Time).
     */
    Time GetLastSeen(const std::string& vehicleId) const;

    /**
     * @brief Check if a vehicle ID exists in the parsed trace.
     * @param vehicleId SUMO vehicle ID string.
     * @return true if present, false otherwise.
     */
    bool HasVehicle(const std::string& vehicleId) const;

    /**
     * @brief Get total number of distinct vehicles in the trace.
     * @return Vehicle count.
     */
    uint32_t GetVehicleCount() const;

    /**
     * @brief Get coordinate scale factor.
     */
    double GetCoordinateScale() const;

    /**
     * @brief Set coordinate scale factor.
     */
    void SetCoordinateScale(double scale);

    /**
     * @brief Get coordinate offset X in meters.
     */
    double GetCoordinateOffsetX() const;

    /**
     * @brief Set coordinate offset X in meters.
     */
    void SetCoordinateOffsetX(double offsetX);

    /**
     * @brief Get coordinate offset Y in meters.
     */
    double GetCoordinateOffsetY() const;

    /**
     * @brief Set coordinate offset Y in meters.
     */
    void SetCoordinateOffsetY(double offsetY);

    /**
     * @brief Get minimum simulation time in the trace.
     */
    Time GetMinTime() const;

    /**
     * @brief Get maximum simulation time in the trace.
     */
    Time GetMaxTime() const;

    /**
     * @brief Get last error message if Load() failed.
     */
    std::string GetLastError() const;

private:
    double m_coordinateScale{1.0}; //!< Scale factor applied to coordinates
    double m_coordinateOffsetX{0.0}; //!< X offset in meters
    double m_coordinateOffsetY{0.0}; //!< Y offset in meters

    std::map<std::string, std::vector<VehicleWaypoint>> m_trajectories;
    std::map<std::string, Time> m_firstSeen;
    std::map<std::string, Time> m_lastSeen;

    Time m_minTime{Seconds(0)};
    Time m_maxTime{Seconds(0)};
    std::string m_lastError{""};

    void ParseXmlLine(const std::string& line, double& currentTimestepSeconds);
    static std::string ExtractAttribute(const std::string& line, const std::string& attrName);
};

} // namespace ns3

#endif // SUMO_FCD_PARSER_H
