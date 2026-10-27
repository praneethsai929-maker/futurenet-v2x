#!/usr/bin/env python3
"""
Generate synthetic and realistic SUMO Floating Car Data (FCD) XML trajectory files.
Satisfies SRS Section 10: Minimum demonstration scale (>=100 vehicles, 4 RSUs).
"""

import sys
import os
import math
import argparse

def generate_fcd(output_path, num_vehicles=100, duration_s=10.0, timestep_s=0.1):
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    steps = int(duration_s / timestep_s)

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<fcd-export xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/fcd_file.xsd">\n')

        for step_idx in range(steps + 1):
            t = step_idx * timestep_s
            f.write(f'  <timestep time="{t:.2f}">\n')

            for v in range(num_vehicles):
                # Stagger vehicle departure times
                dept_time = (v % 20) * 0.2
                if t < dept_time:
                    continue

                # Grid paths: half horizontal, half vertical
                active_t = t - dept_time
                speed = 10.0 + (v % 5) * 1.5  # speeds 10 to 16 m/s (~36 to ~58 km/h)

                if v % 2 == 0:
                    # Eastbound corridor
                    lane_y = (v // 2) * 20.0
                    pos_x = active_t * speed
                    pos_y = lane_y
                    angle = 90.0
                    lane_id = f"east_corridor_{v // 2}"
                else:
                    # Northbound corridor
                    lane_x = (v // 2) * 20.0
                    pos_x = lane_x
                    pos_y = active_t * speed
                    angle = 0.0
                    lane_id = f"north_corridor_{v // 2}"

                f.write(f'    <vehicle id="veh_{v:03d}" x="{pos_x:.2f}" y="{pos_y:.2f}" z="0.00" '
                        f'speed="{speed:.2f}" angle="{angle:.2f}" pos="{active_t * speed:.2f}" '
                        f'lane="{lane_id}" type="car"/>\n')

            f.write('  </timestep>\n')

        f.write('</fcd-export>\n')

    print(f"[Generated] Successfully wrote {num_vehicles} vehicles over {duration_s}s to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate SUMO FCD trace")
    parser.add_argument("--output", "-o", default="test/data/fcd-demo.xml", help="Output FCD XML path")
    parser.add_argument("--vehicles", "-n", type=int, default=100, help="Number of vehicles (>=100 for SRS Demo)")
    parser.add_argument("--duration", "-d", type=float, default=10.0, help="Simulation duration (seconds)")
    parser.add_argument("--step", "-s", type=float, default=0.1, help="Timestep (seconds)")
    args = parser.parse_args()

    generate_fcd(args.output, args.vehicles, args.duration, args.step)
