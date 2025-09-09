#!/bin/bash

# ORB_SLAM2_SSD_Semantic 项目构建脚本 (修复版)
# 自动编译整个项目的所有子模块

set -e

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

print_info "项目根目录: $PROJECT_ROOT"

# 切换到项目根目录
cd "$PROJECT_ROOT"

# 设置环境变量
export LD_LIBRARY_PATH=/usr/local/lib:$PWD/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH

# 创建必要的目录
print_info "创建必要的目录..."
mkdir -p lib bin
mkdir -p Thirdparty/DBoW2/lib Thirdparty/g2o/lib

# 创建库文件软链接
print_info "创建库文件软链接..."
# DBoW2
if [ -f "/usr/local/lib/libDBoW2.so" ]; then
    ln -sf /usr/local/lib/libDBoW2.so Thirdparty/DBoW2/lib/libDBoW2.so
elif [ -f "/usr/local/lib/libDBoW2.a" ]; then
    ln -sf /usr/local/lib/libDBoW2.a Thirdparty/DBoW2/lib/libDBoW2.so
else
    print_warning "DBoW2库未找到，尝试搜索..."
    find /usr/local -name "*DBoW2*" 2>/dev/null || true
fi

# g2o
G2O_LIBS=$(find /usr/local/lib -name "libg2o_*.so" 2>/dev/null | head -5)
if [ -n "$G2O_LIBS" ]; then
    for lib in $G2O_LIBS; do
        ln -sf "$lib" "Thirdparty/g2o/lib/$(basename $lib)"
    done
    # 创建主要的g2o.so链接
    ln -sf /usr/local/lib/libg2o_core.so Thirdparty/g2o/lib/libg2o.so 2>/dev/null || true
else
    print_warning "g2o库未找到，尝试搜索..."
    find /usr/local -name "*g2o*" 2>/dev/null || true
fi

# 函数：编译子项目
build_subproject() {
    local subproject_dir=$1
    local subproject_name=$2
    
    print_info "开始编译 $subproject_name ($subproject_dir)..."
    
    if [ ! -d "$subproject_dir" ]; then
        print_error "子项目目录不存在: $subproject_dir"
        return 1
    fi
    
    # 检查是否有CMakeLists.txt
    if [ ! -f "$subproject_dir/CMakeLists.txt" ]; then
        print_error "CMakeLists.txt未找到: $subproject_dir/CMakeLists.txt"
        return 1
    fi
    
    # 创建构建目录
    local build_dir="$subproject_dir/build"
    rm -rf "$build_dir"
    mkdir -p "$build_dir"
    cd "$build_dir"
    
    # 运行CMake配置
    print_info "运行CMake配置..."
    if ! cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_PREFIX_PATH=/usr/local; then
        print_error "CMake配置失败"
        return 1
    fi
    
    # 编译
    print_info "开始编译..."
    if ! make -j$(nproc); then
        print_error "编译失败"
        return 1
    fi
    
    print_info "$subproject_name 编译完成！"
    cd "$PROJECT_ROOT"
    return 0
}

# 编译各个子项目
print_info "开始编译 ORB_SLAM2_SSD_Semantic 项目..."

# 1. 编译主项目 (src/)
if build_subproject "src" "主项目(src)"; then
    print_info "主项目编译成功"
else
    print_error "主项目编译失败"
    exit 1
fi

# 2. 编译完善版本 (perfect/)
if build_subproject "perfect" "完善版本(perfect)"; then
    print_info "完善版本编译成功"
else
    print_warning "完善版本编译失败，继续..."
fi

# 3. 编译实时检测定位模块 (realtime_dect_loc/)
if build_subproject "realtime_dect_loc" "实时检测定位(realtime_dect_loc)"; then
    print_info "实时检测定位模块编译成功"
else
    print_warning "实时检测定位模块编译失败，继续..."
fi

# 检查生成的文件
print_info "检查编译结果..."

# 检查库文件
if [ -f "lib/libORB_SLAM2_pc.so" ]; then
    print_info "✓ 主库文件生成成功: lib/libORB_SLAM2_pc.so"
else
    print_warning "✗ 主库文件未生成"
fi

# 检查可执行文件
EXECUTABLES=("bin/ty_rgbd" "bin/3d_object_dect")
for exe in "${EXECUTABLES[@]}"; do
    if [ -f "$exe" ]; then
        print_info "✓ 可执行文件: $exe"
    else
        print_warning "✗ 可执行文件未生成: $exe"
    fi
done

# 检查Thirdparty链接
print_info "检查第三方库链接..."
ls -la Thirdparty/DBoW2/lib/ 2>/dev/null || print_warning "DBoW2库链接检查失败"
ls -la Thirdparty/g2o/lib/ 2>/dev/null || print_warning "g2o库链接检查失败"

print_info "构建过程完成！"
print_info "使用以下命令设置环境变量:"
print_info "export LD_LIBRARY_PATH=$PROJECT_ROOT/lib:/usr/local/lib:\$LD_LIBRARY_PATH"
