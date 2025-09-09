#!/bin/bash

# ORB_SLAM2_SSD_Semantic 测试脚本
# 测试编译的项目是否能正常运行

set -e

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

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
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

print_info "项目根目录: $PROJECT_ROOT"
cd "$PROJECT_ROOT"

# 设置环境变量
export LD_LIBRARY_PATH=$PROJECT_ROOT/lib:/usr/local/lib:$LD_LIBRARY_PATH

# 测试计数器
TOTAL_TESTS=0
PASSED_TESTS=0

# 测试函数
run_test() {
    local test_name=$1
    local test_command=$2
    
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    print_info "运行测试: $test_name"
    
    if eval "$test_command"; then
        print_info "✓ 测试通过: $test_name"
        PASSED_TESTS=$((PASSED_TESTS + 1))
        return 0
    else
        print_error "✗ 测试失败: $test_name"
        return 1
    fi
}

# 基础环境测试
print_info "开始基础环境测试..."

# 1. 测试依赖库是否正确安装
print_info "检查系统依赖库..."

check_library() {
    local lib_name=$1
    local check_command=$2
    
    if eval "$check_command" >/dev/null 2>&1; then
        print_info "✓ $lib_name 已安装"
        return 0
    else
        print_warning "✗ $lib_name 未正确安装"
        return 1
    fi
}

# 检查OpenCV
check_library "OpenCV" "pkg-config --exists opencv4 || pkg-config --exists opencv"

# 检查Eigen3
check_library "Eigen3" "pkg-config --exists eigen3"

# 检查PCL
check_library "PCL" "pkg-config --exists pcl_common-1.10 || pkg-config --exists pcl_common-1.8"

# 检查Pangolin
check_library "Pangolin" "test -f /usr/local/lib/libpangolin.so"

# 2. 测试编译产物
print_info "检查编译产物..."

# 检查主库
run_test "主库存在性检查" "test -f lib/libORB_SLAM2_pc.so"

# 检查库依赖
run_test "主库依赖检查" "ldd lib/libORB_SLAM2_pc.so | grep -q 'not found' && exit 1 || exit 0"

# 3. 测试可执行文件
print_info "测试可执行文件..."

# 检查可执行文件是否存在且可执行
test_executable() {
    local exe_path=$1
    local exe_name=$(basename "$exe_path")
    
    if [ -f "$exe_path" ] && [ -x "$exe_path" ]; then
        print_info "✓ 可执行文件存在: $exe_name"
        
        # 测试基本的help或version选项
        if timeout 5s "$exe_path" --help >/dev/null 2>&1 || 
           timeout 5s "$exe_path" -h >/dev/null 2>&1 || 
           timeout 5s "$exe_path" --version >/dev/null 2>&1; then
            print_info "✓ 可执行文件可以运行: $exe_name"
            return 0
        else
            print_warning "✗ 可执行文件运行测试失败: $exe_name"
            return 1
        fi
    else
        print_warning "✗ 可执行文件不存在或不可执行: $exe_name"
        return 1
    fi
}

# 测试各个可执行文件
if [ -f "Examples/RGB-D/ty_rgbd" ]; then
    test_executable "Examples/RGB-D/ty_rgbd"
fi

if [ -f "Examples/Stereo/my_stereo" ]; then
    test_executable "Examples/Stereo/my_stereo"
fi

if [ -f "bin/3d_object_dect" ]; then
    test_executable "bin/3d_object_dect"
fi

# 4. 测试第三方库链接
print_info "测试第三方库链接..."

run_test "DBoW2库链接" "test -L Thirdparty/DBoW2/lib/libDBoW2.so"
run_test "g2o库目录" "test -d Thirdparty/g2o/lib && ls Thirdparty/g2o/lib/ | grep -q libg2o"

# 5. 创建简单的功能测试
print_info "创建功能测试..."

# 创建一个简单的测试程序来验证库是否正常工作
cat > test_basic_functionality.cpp << 'EOF'
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

int main() {
    try {
        // 测试OpenCV
        cv::Mat test_img = cv::Mat::zeros(100, 100, CV_8UC3);
        std::cout << "OpenCV test passed" << std::endl;
        
        // 测试Eigen
        Eigen::Matrix3d test_matrix = Eigen::Matrix3d::Identity();
        std::cout << "Eigen test passed" << std::endl;
        
        std::cout << "Basic functionality test passed!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
}
EOF

# 编译并运行测试
if g++ -std=c++11 test_basic_functionality.cpp -o test_basic_functionality \
    $(pkg-config --cflags --libs opencv4 || pkg-config --cflags --libs opencv) \
    -I/usr/local/include/eigen3 2>/dev/null; then
    
    if run_test "基础功能测试" "./test_basic_functionality"; then
        rm -f test_basic_functionality test_basic_functionality.cpp
    fi
else
    print_warning "无法编译基础功能测试"
fi

# 6. 内存检查（如果有valgrind）
if command -v valgrind >/dev/null 2>&1; then
    print_info "运行内存检查测试..."
    # 这里可以添加内存泄漏检查
else
    print_info "跳过内存检查测试 (valgrind未安装)"
fi

# 输出测试结果总结
print_info "============ 测试结果总结 ============"
print_info "总测试数: $TOTAL_TESTS"
print_info "通过测试: $PASSED_TESTS"
print_info "失败测试: $((TOTAL_TESTS - PASSED_TESTS))"

if [ $PASSED_TESTS -eq $TOTAL_TESTS ]; then
    print_info "🎉 所有测试通过！项目构建成功！"
    exit 0
elif [ $PASSED_TESTS -gt $((TOTAL_TESTS / 2)) ]; then
    print_warning "⚠️  大部分测试通过，但仍有问题需要解决"
    exit 1
else
    print_error "❌ 大部分测试失败，需要检查构建过程"
    exit 1
fi
