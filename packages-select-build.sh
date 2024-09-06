#!/bin/bash

# set -e，脚本开发者可以确保在发生错误时立即停止执行，从而可以更快地定位和修复问题。
set -e

# 检查是否有一个参数传入
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <package_name>"
    exit 1
fi

# 接收包名作为参数
package_name=$1

# 设置构建类型为RelWithDebInfo，这是一种优化级别，它在保持良好性能的同时提供了调试信息。这对于调试和性能分析非常有用。
BUILD_TYPE=RelWithDebInfo

# --merge-install选项指示colcon将所有安装目标合并到一个安装目录中，而不是每个包一个安装目录。这可以简化环境设置，因为只需要将一个目录添加到环境变量中。
# --symlink-install选项指示colcon在安装目标时使用符号链接。这可以减少磁盘空间的使用，因为不同包之间的共享文件只会在磁盘上存储一次。
# --cmake-args选项允许将参数传递给CMake。在这种情况下，我们将使用CMake的-DCMAKE_BUILD_TYPE选项将构建类型设置为RelWithDebInfo。-DCMAKE_EXPORT_COMPILE_COMMANDS=On启用编译命令的导出，这通常用于生成compile_commands.json文件，该文件包含了编译每个文件所用的确切命令。这对于代码分析、自动完成和其他工具非常有用。
# -Wall -Wextra -Wpedantic选项启用了一些额外的警告标志，警告（-Wall）、启用额外警告（-Wextra）和要求严格的ISO C++（-Wpedantic）。以帮助我们发现潜在的问题。
colcon build \
        --merge-install \
        --symlink-install \
        --packages-select $package_name \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic