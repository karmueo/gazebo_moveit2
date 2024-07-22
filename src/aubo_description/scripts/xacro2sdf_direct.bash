#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `panda_description` package
# 这个脚本将xacro（URDF变体）转换为`panda_description`包的SDF

# TMP_URDF_PATH为临时URDF文件的路径
TMP_URDF_PATH=$(mktemp /tmp/panda_XXXXXX.urdf)

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
# 将xacro处理为URDF，然后将URDF转换为SDF，并编辑SDF以使用网格的相对路径
xacro "${1}" "${@:2}" -o "${TMP_URDF_PATH}" &&
SDF_XML=$(ign sdf -p "${TMP_URDF_PATH}" | sed "s/model:\/\/panda_description\///g")

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null

# Return SDF as XML string
echo "${SDF_XML}"
