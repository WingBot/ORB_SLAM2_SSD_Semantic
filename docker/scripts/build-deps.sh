#!/bin/bash

# ORB_SLAM2_SSD_Semantic 依赖库镜像构建脚本
# 基于系统镜像构建包含所有第三方库的镜像

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

# 检查依赖
check_prerequisites() {
    print_info "检查构建前提条件..."
    
    # 检查系统镜像是否存在
    if ! docker image inspect "orb_slam2_ssd_semantic:system" &> /dev/null; then
        print_warning "系统镜像不存在，将先构建系统镜像..."
        if ! "$SCRIPT_DIR/build-system.sh"; then
            print_error "系统镜像构建失败"
            exit 1
        fi
    else
        print_info "✓ 系统镜像已存在"
    fi
}

# 构建依赖库镜像
build_deps_image() {
    print_header "构建依赖库镜像"
    
    cd "$PROJECT_ROOT"
    
    local image_name="orb_slam2_ssd_semantic:deps"
    
    print_info "开始构建依赖库镜像: $image_name"
    print_info "包含内容: Eigen3, OpenCV, Pangolin, PCL, OctoMap, NCNN, DBoW2, g2o"
    print_info "预计时间: 20-30分钟（首次构建）"
    
    # 显示构建进度
    print_info "构建过程中可能会花费较长时间编译各个库..."
    print_info "您可以通过以下命令查看详细构建日志:"
    print_info "docker logs -f \$(docker ps -q --filter ancestor=orb_slam2_ssd_semantic:system)"
    
    if docker build \
        -t "$image_name" \
        -f docker/Dockerfile.deps \
        .; then
        print_info "✓ 依赖库镜像构建成功: $image_name"
        return 0
    else
        print_error "✗ 依赖库镜像构建失败"
        return 1
    fi
}

# 验证依赖库
verify_deps_image() {
    print_header "验证依赖库镜像"
    
    local image_name="orb_slam2_ssd_semantic:deps"
    
    print_info "验证镜像是否存在..."
    if ! docker image inspect "$image_name" &> /dev/null; then
        print_error "依赖库镜像不存在！"
        return 1
    fi
    
    print_info "运行依赖验证脚本..."
    if docker run --rm "$image_name" /usr/local/bin/verify-deps.sh; then
        print_info "✓ 依赖库验证通过"
        return 0
    else
        print_error "✗ 依赖库验证失败"
        return 1
    fi
}

# 显示构建结果
show_results() {
    print_header "构建结果总结"
    
    echo "Docker镜像："
    docker images | grep "orb_slam2_ssd_semantic" || echo "  无相关镜像"
    
    echo ""
    echo "镜像大小对比："
    echo "系统镜像:"
    docker images --format "  {{.Repository}}:{{.Tag}}\t{{.Size}}" | grep "orb_slam2_ssd_semantic:system" || echo "  未找到"
    echo "依赖镜像:"
    docker images --format "  {{.Repository}}:{{.Tag}}\t{{.Size}}" | grep "orb_slam2_ssd_semantic:deps" || echo "  未找到"
    
    echo ""
    print_info "下一步："
    print_info "1. 运行 './scripts/build-final.sh' 构建最终开发镜像"
    print_info "2. 或运行 './scripts/run-dev.sh' 启动开发环境"
}

# 清理函数（构建失败时清理）
cleanup_on_failure() {
    print_warning "构建失败，正在清理中间容器..."
    docker container prune -f || true
    docker image prune -f || true
}

# 主函数
main() {
    print_header "ORB_SLAM2_SSD_Semantic 依赖库镜像构建"
    
    # 设置错误处理
    trap cleanup_on_failure ERR
    
    check_prerequisites
    
    if build_deps_image && verify_deps_image; then
        show_results
        print_info "🎉 依赖库镜像构建成功！"
        exit 0
    else
        print_error "❌ 依赖库镜像构建失败！"
        cleanup_on_failure
        exit 1
    fi
}

# 运行主函数
main "$@"
