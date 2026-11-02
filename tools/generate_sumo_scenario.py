#!/usr/bin/env python3
"""
FutureNet M09: Autonomous SUMO Scenario & FCD Generator.
Implements reproducible generation of 100+ vehicle road networks and trajectories
using native Eclipse SUMO tools (netgenerate, randomTrips.py, duarouter, and sumo).
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

def find_sumo_tool(tool_name):
    """Finds a SUMO binary or python tool in system PATH or python environment."""
    # 1. System PATH
    p = shutil.which(tool_name)
    if p:
        return p

    # 2. User Python bin
    user_bin = os.path.expanduser(f"~/Library/Python/3.14/bin/{tool_name}")
    if os.path.exists(user_bin):
        return user_bin

    # 3. SUMO_HOME
    try:
        import sumo
        sumo_home = sumo.SUMO_HOME
        if tool_name.endswith(".py"):
            tool_path = os.path.join(sumo_home, "tools", tool_name)
            if os.path.exists(tool_path):
                return tool_path
        else:
            bin_path = os.path.join(sumo_home, "bin", tool_name)
            if os.path.exists(bin_path):
                return bin_path
    except ImportError:
        pass

    return tool_name

def generate_sumo_scenario(num_vehicles=100, duration_sec=20.0, output_fcd="/tmp/fcd-demo-100.xml"):
    print("==========================================================")
    print(" FutureNet M09: Real SUMO 100+ Vehicle Scenario Generator ")
    print("==========================================================")
    print(f" Vehicles Target  : {num_vehicles}")
    print(f" Duration         : {duration_sec} s")
    print(f" Output FCD File  : {output_fcd}")
    print("==========================================================")

    tmpdir = tempfile.mkdtemp(prefix="sumo_gen_")
    net_file = os.path.join(tmpdir, "grid.net.xml")
    trips_file = os.path.join(tmpdir, "trips.trips.xml")
    routes_file = os.path.join(tmpdir, "routes.rou.xml")

    try:
        # Step 1: Generate Grid Network with netgenerate
        netgen_bin = find_sumo_tool("netgenerate")
        netgen_cmd = [
            sys.executable, netgen_bin,
            "--grid",
            "--grid.number=3",
            "--grid.length=200",
            "-o", net_file
        ]
        print(f"[1/3] Generating 3x3 road network: {' '.join(netgen_cmd)}")
        res1 = subprocess.run(netgen_cmd, capture_output=True, text=True)
        if res1.returncode != 0:
            raise RuntimeError(f"netgenerate failed: {res1.stderr}")

        # Step 2: Generate Trips & Routes with randomTrips.py
        rtrips_tool = find_sumo_tool("randomTrips.py")
        period = max(0.05, duration_sec / (num_vehicles * 1.5))
        rtrips_cmd = [
            sys.executable, rtrips_tool,
            "-n", net_file,
            "-e", str(duration_sec),
            "-p", f"{period:.3f}",
            "-o", trips_file,
            "--route-file", routes_file,
            "--validate"
        ]
        print(f"[2/3] Generating traffic demand with randomTrips.py: {' '.join(rtrips_cmd)}")
        res2 = subprocess.run(rtrips_cmd, capture_output=True, text=True)
        if res2.returncode != 0:
            raise RuntimeError(f"randomTrips.py failed: {res2.stderr}")

        # Step 3: Run SUMO to produce FCD trajectory XML
        sumo_bin = find_sumo_tool("sumo")
        sumo_cmd = [
            sys.executable, sumo_bin,
            "-n", net_file,
            "-r", routes_file,
            "--begin", "0",
            "--end", str(duration_sec),
            "--step-length", "0.2",
            "--fcd-output", output_fcd,
            "--fcd-output.geo", "false"
        ]
        print(f"[3/3] Simulating with SUMO and generating FCD export: {' '.join(sumo_cmd)}")
        res3 = subprocess.run(sumo_cmd, capture_output=True, text=True)
        if res3.returncode != 0:
            raise RuntimeError(f"sumo failed: {res3.stderr}")

        print(f"[Success] Successfully generated native SUMO FCD trace: {output_fcd}")
        return True

    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate 100+ vehicle FCD trace using native SUMO tools")
    parser.add_argument("-v", "--vehicles", type=int, default=100, help="Target number of vehicles")
    parser.add_argument("-d", "--duration", type=float, default=20.0, help="Duration in seconds")
    parser.add_argument("-o", "--output", default="/tmp/fcd-demo-100.xml", help="Output FCD XML trace path")
    args = parser.parse_args()

    generate_sumo_scenario(args.vehicles, args.duration, args.output)
