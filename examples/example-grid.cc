#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/futurenet-v2x-helper.h"
#include "ns3/simulator.h"
#include "ns3/v2x-scenario-helper.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ExampleGrid");

int
main(int argc, char* argv[])
{
    std::string fcdTrace = "contrib/futurenet-v2x/test/data/fcd-small.xml";
    uint32_t numRsus = 4;
    double simTimeSec = 5.0;
    std::string tracePrefix = "v2x-grid";
    double mobilityStepMs = 100.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("fcdTrace", "Path to SUMO Floating Car Data (FCD) XML trace file", fcdTrace);
    cmd.AddValue("numRsus", "Number of Roadside Units (RSUs) to deploy", numRsus);
    cmd.AddValue("simTime", "Simulation time in seconds", simTimeSec);
    cmd.AddValue("mobilityStep", "Mobility position sampling interval in milliseconds", mobilityStepMs);
    cmd.AddValue("tracePrefix", "Prefix for generated CSV files", tracePrefix);
    cmd.Parse(argc, argv);

    std::cout << "========================================================\n"
              << " FutureNet M09: V2X Mobility Integration (example-grid) \n"
              << "========================================================\n"
              << " FCD Trace File : " << fcdTrace << "\n"
              << " Num RSUs       : " << numRsus << "\n"
              << " Mobility Step  : " << mobilityStepMs << " ms\n"
              << " Sim Time       : " << simTimeSec << " s\n"
              << " Trace Prefix   : " << tracePrefix << "\n"
              << "========================================================\n";

    Ptr<V2xScenarioHelper> helper = CreateObject<V2xScenarioHelper>();
    helper->SetAttribute("MobilityStep", TimeValue(MilliSeconds(mobilityStepMs)));
    helper->SetAttribute("NumRsus", UintegerValue(numRsus));

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

    std::cout << "[Run] Running simulation for " << simTimeSec << " seconds...\n";
    Simulator::Stop(Seconds(simTimeSec));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "[Done] Simulation complete.\n"
              << "       - Mobility CSV: " << tracePrefix << "-mobility.csv\n"
              << "       - Network CSV : " << tracePrefix << "-network.csv\n"
              << "       - Common CSV  : " << tracePrefix << "-common-events.csv\n";

    return 0;
}
