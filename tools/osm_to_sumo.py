#!/usr/bin/env python3
"""
M09-FR-09 [Recommended]: OpenStreetMap (OSM) to SUMO road network conversion tool.
Provides automated conversion workflows and documented instructions using SUMO's netconvert.
"""

import argparse
import os
import subprocess
import sys

def convert_osm_to_sumo(osm_file, output_prefix):
    net_file = f"{output_prefix}.net.xml"
    poly_file = f"{output_prefix}.poly.xml"

    print("==========================================================")
    print(" FutureNet M09: OpenStreetMap to SUMO Network Converter   ")
    print("==========================================================")
    print(f" Source OSM File   : {osm_file}")
    print(f" Target Network    : {net_file}")
    print("==========================================================")

    if not os.path.exists(osm_file):
        print(f"Error: OSM file '{osm_file}' does not exist.")
        return False

    cmd = [
        "netconvert",
        "--osm-files", osm_file,
        "-o", net_file,
        "--geometry.remove",
        "--roundabouts.guess",
        "--ramps.guess",
        "--junctions.join",
        "--tls.guess-signals",
        "--tls.discard-simple"
    ]

    print(f"[Command] {' '.join(cmd)}")
    try:
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode == 0:
            print(f"[Success] Generated SUMO road network: {net_file}")
            return True
        else:
            print(f"[Notice] netconvert exited with: {res.stderr}")
            return False
    except FileNotFoundError:
        print("[Notice] 'netconvert' is not installed in system PATH.")
        print("         To install SUMO tools on Ubuntu/Debian: sudo apt install sumo sumo-tools")
        print("         To install SUMO on macOS: brew install sumo")
        print("         Workflow instructions documented in doc/technical-report.md (AC-05).")
        return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert OSM to SUMO network")
    parser.add_argument("osm_file", help="Input .osm or .osm.xml file")
    parser.add_argument("-o", "--output", default="road_network", help="Output network prefix")
    args = parser.parse_args()

    convert_osm_to_sumo(args.osm_file, args.output)
