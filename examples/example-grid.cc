#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/simulator.h"
#include "ns3/v2x-scenario-helper.h"

#include <iostream>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ExampleGrid");

static std::vector<Vector>
ParseRsuPositions(const std::string& str)
{
    std::vector<Vector> res;
    if (str.empty())
    {
        return res;
    }
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, ';'))
    {
        std::stringstream pss(item);
        std::string xStr, yStr, zStr;
        if (std::getline(pss, xStr, ',') && std::getline(pss, yStr, ','))
        {
            double x = std::stod(xStr);
            double y = std::stod(yStr);
            double z = 0.0;
            if (std::getline(pss, zStr, ','))
            {
                z = std::stod(zStr);
            }
            res.push_back(Vector(x, y, z));
        }
    }
    return res;
}

int
main(int argc, char* argv[])
{
    std::string fcdTrace = "contrib/futurenet-v2x/test/data/fcd-small.xml";
    uint32_t numRsus = 4;
    double simTimeSec = 5.0;
    std::string tracePrefix = "v2x-grid";
    double mobilityStepMs = 100.0;
    double v2xIntervalMs = 100.0;
    uint32_t v2xBytes = 300;
    double txPowerDbm = 21.5;
    std::string rsuPositionsStr = "";
    double gridSpacing = 200.0;
    uint32_t gridWidth = 2;

    CommandLine cmd(__FILE__);
    cmd.AddValue("fcdTrace", "Path to SUMO Floating Car Data (FCD) XML trace file", fcdTrace);
    cmd.AddValue("numRsus", "Number of Roadside Units (RSUs) to deploy", numRsus);
    cmd.AddValue("simTime", "Simulation time in seconds", simTimeSec);
    cmd.AddValue("mobilityStep", "Mobility position sampling interval in milliseconds", mobilityStepMs);
    cmd.AddValue("v2xInterval", "V2X message interval in milliseconds", v2xIntervalMs);
    cmd.AddValue("v2xBytes", "Payload size of periodic V2X messages in bytes", v2xBytes);
    cmd.AddValue("txPower", "Wi-Fi TX transmission power in dBm", txPowerDbm);
    cmd.AddValue("rsuPositions", "Semicolon-separated explicit RSU positions 'x,y;x,y...'", rsuPositionsStr);
    cmd.AddValue("gridSpacing", "Spacing between RSUs in regular grid layout (meters)", gridSpacing);
    cmd.AddValue("gridWidth", "Number of RSUs per row in grid layout", gridWidth);
    cmd.AddValue("tracePrefix", "Prefix for generated CSV files", tracePrefix);
    cmd.Parse(argc, argv);

    std::cout << "========================================================\n"
              << " FutureNet M09: V2X Mobility Integration (example-grid) \n"
              << "========================================================\n"
              << " FCD Trace File : " << fcdTrace << "\n"
              << " Num RSUs       : " << numRsus << "\n"
              << " V2X Interval   : " << v2xIntervalMs << " ms (" << v2xBytes << " bytes)\n"
              << " TX Power       : " << txPowerDbm << " dBm\n"
              << " Mobility Step  : " << mobilityStepMs << " ms\n"
              << " Sim Time       : " << simTimeSec << " s\n"
              << " Trace Prefix   : " << tracePrefix << "\n"
              << "========================================================\n";

    Ptr<V2xScenarioHelper> helper = CreateObject<V2xScenarioHelper>();
    helper->SetAttribute("MobilityStep", TimeValue(MilliSeconds(mobilityStepMs)));
    helper->SetAttribute("NumRsus", UintegerValue(numRsus));
    helper->SetAttribute("V2xMessageInterval", TimeValue(MilliSeconds(v2xIntervalMs)));
    helper->SetAttribute("V2xMessageBytes", UintegerValue(v2xBytes));
    helper->SetAttribute("TxPower", DoubleValue(txPowerDbm));

    std::vector<Vector> explicitPositions = ParseRsuPositions(rsuPositionsStr);
    if (!explicitPositions.empty())
    {
        helper->SetRsuPositions(explicitPositions);
        numRsus = explicitPositions.size();
    }
    else
    {
        helper->SetRsuGridPlacement(50.0, 50.0, gridSpacing, gridSpacing, gridWidth);
    }

    bool loaded = helper->LoadFcd(fcdTrace);
    if (!loaded)
    {
        std::cerr << "Error: Failed to load FCD trace from '" << fcdTrace << "'\n";
        return 1;
    }

    NodeContainer vehicles = helper->CreateVehicleNodes();
    NodeContainer rsus = helper->CreateRsuNodes(numRsus);

    std::cout << "[Setup] Instantiated " << vehicles.GetN() << " vehicle nodes and "
              << rsus.GetN() << " roadside unit (RSU) nodes.\n";

    helper->InstallMobility();
    helper->InstallExampleNetworking();
    helper->EnableTracing(tracePrefix);

    // FlowMonitor setup
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();

    std::cout << "[Run] Running simulation for " << simTimeSec << " seconds...\n";
    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();

    flowMonitor->SerializeToXmlFile(tracePrefix + "-flowmon.xml", true, true);
    Simulator::Destroy();

    std::cout << "[Done] Simulation complete.\n"
              << "       - FlowMonitor XML: " << tracePrefix << "-flowmon.xml\n"
              << "       - Mobility CSV   : " << tracePrefix << "-mobility.csv\n"
              << "       - Network CSV    : " << tracePrefix << "-network.csv\n"
              << "       - Common CSV     : " << tracePrefix << "-common-events.csv\n";

    return 0;
}
