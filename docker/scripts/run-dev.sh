#!/bin/bash

# ORB_SLAM2_SSD_Semantic 开发环境启动脚本
# 启动交互式开发容器

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
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 配置参数
IMAGE_NAME="orb_slam2_ssd_semantic:deps"
CONTAINER_NAME="orb_slam2_dev_container"

# 检查镜像是否存在
check_image() {
    print_info "检查Docker镜像..."
    
    if ! docker image inspect "$IMAGE_NAME" &> /dev/null; then
        print_error "镜像 $IMAGE_NAME 不存在！"
        print_info "请先运行以下命令构建镜像："
        print_info "  ./docker/scripts/build-all.sh"
        print_info "或者："
        print_info "  ./docker/scripts/build-final.sh"
        exit 1
    fi
    
    print_info "✓ 镜像 $IMAGE_NAME 已存在"
}

# 设置X11转发（GUI支持）
setup_x11() {
    print_info "设置X11转发..."
    
    # 允许X11连接
    if command -v xhost &> /dev/null; then
        xhost +local:docker &> /dev/null || print_warning "无法设置xhost权限"
    fi
    
    # 确保DISPLAY环境变量存在
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:0
        print_warning "DISPLAY环境变量未设置，使用默认值 :0"
    fi
    
    print_info "✓ X11转发配置完成 (DISPLAY=$DISPLAY)"
}

# 清理旧容器
cleanup_old_container() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_info "清理旧容器..."
        docker stop "$CONTAINER_NAME" &> /dev/null || true
        docker rm "$CONTAINER_NAME" &> /dev/null || true
    fi
}

# 启动开发容器
start_dev_container() {
    print_header "启动开发环境"
    
    cleanup_old_container
    
    print_info "启动容器: $CONTAINER_NAME"
    print_info "镜像: $IMAGE_NAME"
    print_info "工作目录: /home/slam/workspace"
    print_info "项目根目录: $PROJECT_ROOT"
    
    # 检查是否有NVIDIA GPU支持
    local gpu_args=""
    if command -v nvidia-docker &> /dev/null || docker info 2>/dev/null | grep -q nvidia; then
        gpu_args="--gpus all"
        print_info "检测到NVIDIA GPU支持"
    fi
    
    # 启动容器
    docker run -it \
        --name "$CONTAINER_NAME" \
        --hostname "orb-slam-dev" \
        --privileged \
        ${gpu_args} \
        -v "$PROJECT_ROOT:/home/slam/workspace" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY="$DISPLAY" \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --network host \
        --workdir /home/slam/workspace \
        "$IMAGE_NAME" \
        /bin/bash
}

# 显示使用提示
show_usage_tips() {
    print_header "开发环境使用提示"
    
    echo "您现在在ORB_SLAM2_SSD_Semantic开发容器中！"
    echo ""
    echo "📁 项目文件位置:"
    echo "   /home/slam/workspace (与主机同步)"
    echo ""
    echo "🔧 常用命令:"
    echo "   ./docker/scripts/build_project.sh    # 构建项目"
    echo "   ./docker/scripts/run_tests.sh        # 运行测试"
    echo "   /usr/local/bin/verify-deps.sh        # 验证依赖"
    echo ""
    echo "📊 项目结构:"
    echo "   src/        # 主要源码"
    echo "   perfect/    # 完善版本"
    echo "   include/    # 头文件"
    echo "   build/      # 构建目录"
    echo "   lib/        # 生成的库"
    echo "   bin/        # 可执行文件"
    echo ""
    echo "🚀 快速开始:"
    echo "   1. 构建项目: ./docker/scripts/build_project.sh"
    echo "   2. 运行测试: ./docker/scripts/run_tests.sh"
    echo "   3. 查看结果: ls -la lib/ bin/"
    echo ""
    echo "📝 注意事项:"
    echo "   - 所有文件修改会自动同步到主机"
    echo "   - 支持GUI应用程序（Pangolin可视化）"
    echo "   - 容器内有完整的开发环境"
    echo ""
    echo "❌ 退出容器: exit 或 Ctrl+D"
}

# 主函数
main() {
    print_header "ORB_SLAM2_SSD_Semantic 开发环境启动"
    
    check_image
    setup_x11
    
    # 显示启动前的信息
    print_info "即将启动开发容器..."
    print_info "容器启动后会显示使用提示"
    
    # 启动容器时先显示提示信息
    show_usage_tips
    
    print_info "启动开发容器..."
    start_dev_container
}

# 运行主函数
main "$@"