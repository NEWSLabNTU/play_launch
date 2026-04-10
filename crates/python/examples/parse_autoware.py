#!/usr/bin/env python3
"""Parse Autoware's planning simulator and list all nodes."""

from play_launch_parser import parse_package

result = parse_package(
    "autoware_launch",
    "planning_simulator.launch.xml",
    args={"vehicle_model": "sample_vehicle", "sensor_model": "sample_sensor_kit"},
)

print(f"Nodes:        {len(result['node'])}")
print(f"Containers:   {len(result['container'])}")
print(f"Composable:   {len(result['load_node'])}")
print(f"Launch files: {len(result['scopes'])}")
print()

print("Standalone nodes:")
for node in result["node"]:
    ns = node["namespace"] or "/"
    name = node["name"] or node["executable"]
    pkg = node["package"] or "-"
    print(f"  {ns}/{name}  ({pkg})")

print()
print("Composable nodes:")
for ln in result["load_node"]:
    print(f"  {ln['namespace']}/{ln['node_name']}  -> {ln['target_container_name']}")
