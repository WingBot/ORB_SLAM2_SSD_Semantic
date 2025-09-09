#!/bin/bash

# ORB_SLAM2_SSD_Semantic 系统镜像构建脚本
# 构建包含系统依赖的基础镜像

set -e

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}===============================================${NC}"
    echo -e "${BLUE} $1${NC}"
    echo -e "${BLUE}===============================================${NC}"
}

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

# 检查Docker环境
check_docker() {
    print_info "检查Docker环境..."
    
    if ! command -v docker &> /dev/null; then
        print_error "Docker未安装！请先安装Docker。"
        exit 1
    fi
    
    if ! docker info &> /dev/null; then
        print_error "无法连接到Docker守护进程！请检查Docker是否正在运行。"
        exit 1
    fi
    
    print_info "Docker环境检查通过"
}

# 构建系统镜像
build_system_image() {
    print_header "构建系统基础镜像"
    
    cd "$PROJECT_ROOT"
    
    local image_name="orb_slam2_ssd_semantic:system"
    
    print_info "开始构建系统镜像: $image_name"
    print_info "包含内容: 系统依赖、编译工具链、基础库"
    print_info "预计时间: 5-10分钟"
    
    if docker build \
        -t "$image_name" \
        -f docker/Dockerfile.system \
        .; then
        print_info "✓ 系统镜像构建成功: $image_name"
        return 0
    else
        print_error "✗ 系统镜像构建失败"
        return 1
    fi
}

# 验证镜像
verify_system_image() {
    print_header "验证系统镜像"
    
    local image_name="orb_slam2_ssd_semantic:system"
    
    print_info "验证镜像是否存在..."
    if ! docker image inspect "$image_name" &> /dev/null; then
        print_error "系统镜像不存在！"
        return 1
    fi
    
    print_info "测试容器启动..."
    if docker run --rm "$image_name" echo "系统镜像测试成功"; then
        print_info "✓ 系统镜像验证通过"
        return 0
    else
        print_error "✗ 系统镜像验证失败"
        return 1
    fi
}

# 显示构建结果
show_results() {
    print_header "构建结果总结"
    
    echo "Docker镜像："
    docker images | grep "orb_slam2_ssd_semantic" | grep "system" || echo "  无系统镜像"
    
    echo ""
    echo "镜像大小："
    docker images --format "table {{.Repository}}:{{.Tag}}\t{{.Size}}" | grep "orb_slam2_ssd_semantic:system" || echo "  未找到系统镜像"
    
    echo ""
    print_info "下一步："
    print_info "1. 运行 './scripts/build-deps.sh' 构建依赖库镜像"
    print_info "2. 或运行 './scripts/build-all.sh' 一键构建所有镜像"
}

# 主函数
main() {
    print_header "ORB_SLAM2_SSD_Semantic 系统镜像构建"
    
    check_docker
    
    if build_system_image && verify_system_image; then
        show_results
        print_info "🎉 系统镜像构建成功！"
        exit 0
    else
        print_error "❌ 系统镜像构建失败！"
        exit 1
    fi
}

# 运行主函数
main "$@"
