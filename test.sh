#!/bin/bash
set -e

# 检查 install/setup.bash 文件是否存在。如果存在，它使用 source 命令执行该脚本
if [ -f install/setup.bash ]; then source install/setup.bash; fi
# 使用 colcon 构建系统运行测试。--merge-install 选项通常用于简化工作空间，使所有安装文件都位于同一个目录中，而不是分散在各个包的独立目录中。
colcon test --merge-install
# 查看前面 colcon test 命令的测试结果，并使用 --verbose 选项提供详细的输出。
colcon test-result --verbose
