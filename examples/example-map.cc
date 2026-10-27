#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/futurenet-v2x-helper.h"
#include "ns3/simulator.h"
#include "ns3/v2x-scenario-helper.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ExampleMap");

int
main(int argc, char* argv[])
{
    std::string fcdTrace = "contrib/futurenet-v2x/test/data/fcd-small.xml";
    uint32_t numRsus = 4;
    double simTimeSec = 10.0;
    std::string tracePrefix = "v2x-map";
    double mobilityStepMs = 100.0;
    double scale = 1.0;
    double offsetX = 0.0;
    double offsetY = 0.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("fcdTrace", "Path to arbitrary user-provided SUMO FCD trace file (AC-01)", fcdTrace);
    cmd.AddValue("numRsus", "Number of RSUs to deploy", numRsus);
    cmd.AddValue("simTime", "Simulation time in seconds", simTimeSec);
    cmd.AddValue("mobilityStep", "Mobility step in ms", mobilityStepMs);
    cmd.AddValue("scale", "Coordinate scaling multiplier", scale);
    cmd.AddValue("offsetX", "Coordinate X offset in meters", offsetX);
    cmd.AddValue("offsetY", "Coordinate Y offset in meters", offsetY);
    cmd.AddValue("tracePrefix", "Prefix for generated CSV files", tracePrefix);
    cmd.Parse(argc, argv);

    std::cout << "========================================================\n"
              << " FutureNet M09: Advanced Map Integration (example-map)  \n"
              << "========================================================\n"
              << " FCD Trace File : " << fcdTrace << "\n"
              << " Num RSUs       : " << numRsus << "\n"
              << " Scale / Offset : " << scale << " / (" << offsetX << ", " << offsetY << ")\n"
              << " Mobility Step  : " << mobilityStepMs << " ms\n"
              << " Sim Time       : " << simTimeSec << " s\n"
              << "========================================================\n";

    Ptr<V2xScenarioHelper> helper = CreateObject<V2xScenarioHelper>();
    helper->SetAttribute("MobilityStep", TimeValue(MilliSeconds(mobilityStepMs)));
    helper->SetAttribute("NumRsus", UintegerValue(numRsus));
    helper->SetAttribute("CoordinateScale", DoubleValue(scale));
    helper->SetAttribute("CoordinateOffsetX", DoubleValue(offsetX));
    helper->SetAttribute("CoordinateOffsetY", DoubleValue(offsetY));

    bool loaded = helper->LoadFcd(fcdTrace);
    if (!loaded)
    {
        std::cerr << "Error: Failed to load FCD trace from '" << fcdTrace << "'\n";
        return 1;
    }

    NodeContainer vehicles = helper->CreateVehicleNodes();
    NodeContainer rsus = helper->CreateRsuNodes(numRsus);

    std::cout << "[Setup] Initialized " << vehicles.GetN() << " vehicles, "
              << rsus.GetN() << " RSUs.\n";

    helper->InstallMobility();
    helper->InstallExampleNetworking();
    helper->EnableTracing(tracePrefix);

    // FlowMonitor setup
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();

    std::cout << "[Run] Executing simulation...\n";
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
