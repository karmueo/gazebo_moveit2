#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `panda_description` package
# 通过gz.launch.py调用：ros2 run <description_package> xacro2sdf.bash name:=<name> prefix:=<prefix> collision_arm:=<collision_arm> collision_gripper:=<collision_gripper> ros2_control:=true ros2_control_plugin:=gz ros2_control_command_interface:=<ros2_control_command_interface> gazebo_preserve_fixed_joint:=<gazebo_preserve_fixed_joint>

# 将SCRIPT_DIR变量设置为脚本文件所在的目录路径
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
# 设置XACRO_PATH为包含aubo_i10.urdf.xacro文件的文件夹路径
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/aubo_i10.urdf.xacro"
# 设置SDF_PATH为包含model.sdf文件的文件夹路径
SDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/model.sdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=aubo
    gripper:=true
    collision_arm:=true
    collision_gripper:=true
    ros2_control:=true
    ros2_control_plugin:=gz
    ros2_control_command_interface:=effort
    gazebo_preserve_fixed_joint:=false
)

# Remove old SDF file
rm "${SDF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
# 将xacro处理为URDF，然后将URDF转换为SDF，并编辑SDF以使用网格的相对路径
"${SCRIPT_DIR}/xacro2sdf_direct.bash" "${XACRO_PATH}" "${XACRO_ARGS[@]}" "${@:1}" >"${SDF_PATH}" &&
echo "Created new ${SDF_PATH}"
