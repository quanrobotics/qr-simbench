# Source me:  source activate.sh   — activates the in-folder ROS 2 Jazzy env.
QR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$("$HOME/miniforge3/bin/conda" shell.bash hook)"
conda activate "$QR_ROOT/env"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_RESOURCE_PATH="$QR_ROOT/simulation/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# Optional ROS packages built from source into ./ws
[ -f "$QR_ROOT/ws/install/setup.bash" ] && source "$QR_ROOT/ws/install/setup.bash"
# Cyclone DDS: with FastDDS defaults, 5+ namespaced Nav2 stacks on one host fail service discovery.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# 10-robot fleets need more DDS participants than Cyclone allows by default (config/cyclonedds.xml).
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$QR_ROOT/config/cyclonedds.xml}"
