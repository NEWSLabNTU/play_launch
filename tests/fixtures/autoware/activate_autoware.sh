# Activate Autoware environment for testing.
# Edit this file to source your Autoware install's setup.bash.
#
# Example:
#   source ~/autoware/install/setup.bash
#
# Expected entity counts for the planning_simulator launch graph.
# Update these when switching Autoware versions.
# If unset, tests only check that counts are > 0 and both parsers agree.
#
# Example:
#   export EXPECTED_NODES=34
#   export EXPECTED_CONTAINERS=15
#   export EXPECTED_LOAD_NODES=70
#
# Known counts:
#   Autoware 1.5.0:    nodes=34  containers=15  load_nodes=70
#   Autoware 2025.02:  nodes=46  containers=15  load_nodes=54
