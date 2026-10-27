#include "sumo-fcd-parser.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/fatal-error.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SumoFcdParser");
NS_OBJECT_ENSURE_REGISTERED(SumoFcdParser);

TypeId
SumoFcdParser::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SumoFcdParser")
            .SetParent<Object>()
            .SetGroupName("FuturenetV2x")
            .AddConstructor<SumoFcdParser>()
            .AddAttribute("CoordinateScale",
                          "Scale factor applied to imported coordinates (unitless)",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&SumoFcdParser::m_coordinateScale),
                          MakeDoubleChecker<double>(1e-6, 1e6))
            .AddAttribute("CoordinateOffsetX",
                          "Offset added to imported X coordinates (meters)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&SumoFcdParser::m_coordinateOffsetX),
                          MakeDoubleChecker<double>())
            .AddAttribute("CoordinateOffsetY",
                          "Offset added to imported Y coordinates (meters)",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&SumoFcdParser::m_coordinateOffsetY),
                          MakeDoubleChecker<double>());
    return tid;
}

SumoFcdParser::SumoFcdParser()
{
    NS_LOG_FUNCTION(this);
}

SumoFcdParser::~SumoFcdParser()
{
    NS_LOG_FUNCTION(this);
}

std::string
SumoFcdParser::ExtractAttribute(const std::string& line, const std::string& attrName)
{
    std::string pattern1 = attrName + "=\"";
    size_t start = line.find(pattern1);
    char quoteChar = '"';
    if (start != std::string::npos)
    {
        start += pattern1.length();
    }
    else
    {
        std::string pattern2 = attrName + "='";
        start = line.find(pattern2);
        if (start == std::string::npos)
        {
            return "";
        }
        quoteChar = '\'';
        start += pattern2.length();
    }

    size_t end = line.find(quoteChar, start);
    if (end == std::string::npos)
    {
        return "";
    }
    return line.substr(start, end - start);
}

bool
SumoFcdParser::Load(const std::string& filename)
{
    NS_LOG_FUNCTION(this << filename);
    m_trajectories.clear();
    m_firstSeen.clear();
    m_lastSeen.clear();
    m_lastError = "";

    std::ifstream file(filename);
    if (!file.is_open())
    {
        m_lastError = "Cannot open file: " + filename;
        NS_LOG_WARN(m_lastError);
        return false;
    }

    std::string line;
    double currentTimestepSeconds = -1.0;
    bool hasFcdRoot = false;
    uint64_t waypointCount = 0;

    while (std::getline(file, line))
    {
        // Trim leading whitespace
        size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
        {
            continue;
        }
        std::string trimmed = line.substr(first);

        if (trimmed.find("<fcd-export") != std::string::npos)
        {
            hasFcdRoot = true;
            continue;
        }

        if (trimmed.rfind("<timestep", 0) == 0)
        {
            std::string timeStr = ExtractAttribute(trimmed, "time");
            if (!timeStr.empty())
            {
                try
                {
                    currentTimestepSeconds = std::stod(timeStr);
                }
                catch (...)
                {
                    m_lastError = "Malformed timestep timestamp: " + timeStr;
                    NS_LOG_WARN(m_lastError);
                    return false;
                }
            }
            continue;
        }

        if (trimmed.rfind("<vehicle", 0) == 0)
        {
            if (currentTimestepSeconds < 0.0)
            {
                m_lastError = "Vehicle entry found outside of a valid timestep";
                NS_LOG_WARN(m_lastError);
                return false;
            }

            std::string id = ExtractAttribute(trimmed, "id");
            if (id.empty())
            {
                m_lastError = "Vehicle entry missing mandatory 'id' attribute";
                NS_LOG_WARN(m_lastError);
                return false;
            }

            std::string xStr = ExtractAttribute(trimmed, "x");
            std::string yStr = ExtractAttribute(trimmed, "y");
            std::string zStr = ExtractAttribute(trimmed, "z");
            std::string speedStr = ExtractAttribute(trimmed, "speed");
            std::string angleStr = ExtractAttribute(trimmed, "angle");
            std::string laneStr = ExtractAttribute(trimmed, "lane");
            std::string posStr = ExtractAttribute(trimmed, "pos");

            if (xStr.empty() || yStr.empty())
            {
                m_lastError = "Vehicle '" + id + "' missing x or y coordinates";
                NS_LOG_WARN(m_lastError);
                return false;
            }

            try
            {
                double rawX = std::stod(xStr);
                double rawY = std::stod(yStr);
                double rawZ = zStr.empty() ? 0.0 : std::stod(zStr);
                double speed = speedStr.empty() ? 0.0 : std::stod(speedStr);
                double angle = angleStr.empty() ? 0.0 : std::stod(angleStr);
                double pos = posStr.empty() ? 0.0 : std::stod(posStr);

                // Apply CoordinateScale and Offsets
                double scaledX = (rawX * m_coordinateScale) + m_coordinateOffsetX;
                double scaledY = (rawY * m_coordinateScale) + m_coordinateOffsetY;
                double scaledZ = rawZ * m_coordinateScale;

                Time ts = Seconds(currentTimestepSeconds);

                VehicleWaypoint wp;
                wp.timestamp = ts;
                wp.position = Vector(scaledX, scaledY, scaledZ);
                wp.speedMps = speed;
                wp.angleDeg = angle;
                wp.laneId = laneStr;
                wp.posOnLane = pos;

                m_trajectories[id].push_back(wp);

                if (m_firstSeen.find(id) == m_firstSeen.end())
                {
                    m_firstSeen[id] = ts;
                }
                m_lastSeen[id] = ts;

                if (waypointCount == 0 || ts < m_minTime)
                {
                    m_minTime = ts;
                }
                if (waypointCount == 0 || ts > m_maxTime)
                {
                    m_maxTime = ts;
                }
                waypointCount++;
            }
            catch (const std::exception& e)
            {
                m_lastError = "Failed to parse numeric attributes for vehicle " + id + ": " + e.what();
                NS_LOG_WARN(m_lastError);
                return false;
            }
        }
    }

    if (!hasFcdRoot && waypointCount == 0)
    {
        m_lastError = "Missing <fcd-export> root tag or valid trajectory data";
        NS_LOG_WARN(m_lastError);
        return false;
    }

    NS_LOG_INFO("Successfully loaded FCD trace: " << m_trajectories.size()
                                                  << " vehicles, " << waypointCount << " waypoints.");
    return true;
}

std::vector<std::string>
SumoFcdParser::GetVehicleIds() const
{
    std::vector<std::string> ids;
    ids.reserve(m_trajectories.size());
    for (const auto& kv : m_trajectories)
    {
        ids.push_back(kv.first);
    }
    // Sort deterministically
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<VehicleWaypoint>
SumoFcdParser::GetTrajectory(const std::string& vehicleId) const
{
    auto it = m_trajectories.find(vehicleId);
    if (it != m_trajectories.end())
    {
        return it->second;
    }
    return {};
}

Time
SumoFcdParser::GetFirstSeen(const std::string& vehicleId) const
{
    auto it = m_firstSeen.find(vehicleId);
    if (it != m_firstSeen.end())
    {
        return it->second;
    }
    return Seconds(-1);
}

Time
SumoFcdParser::GetLastSeen(const std::string& vehicleId) const
{
    auto it = m_lastSeen.find(vehicleId);
    if (it != m_lastSeen.end())
    {
        return it->second;
    }
    return Seconds(-1);
}

bool
SumoFcdParser::HasVehicle(const std::string& vehicleId) const
{
    return m_trajectories.find(vehicleId) != m_trajectories.end();
}

uint32_t
SumoFcdParser::GetVehicleCount() const
{
    return static_cast<uint32_t>(m_trajectories.size());
}

double
SumoFcdParser::GetCoordinateScale() const
{
    return m_coordinateScale;
}

void
SumoFcdParser::SetCoordinateScale(double scale)
{
    m_coordinateScale = scale;
}

double
SumoFcdParser::GetCoordinateOffsetX() const
{
    return m_coordinateOffsetX;
}

void
SumoFcdParser::SetCoordinateOffsetX(double offsetX)
{
    m_coordinateOffsetX = offsetX;
}

double
SumoFcdParser::GetCoordinateOffsetY() const
{
    return m_coordinateOffsetY;
}

void
SumoFcdParser::SetCoordinateOffsetY(double offsetY)
{
    m_coordinateOffsetY = offsetY;
}

Time
SumoFcdParser::GetMinTime() const
{
    return m_minTime;
}

Time
SumoFcdParser::GetMaxTime() const
{
    return m_maxTime;
}

std::string
SumoFcdParser::GetLastError() const
{
    return m_lastError;
}

} // namespace ns3
