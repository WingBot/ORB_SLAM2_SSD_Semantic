#!/bin/bash

# ORB_SLAM2_SSD_Semantic 最终镜像构建脚本
# 基于依赖镜像构建包含项目代码的最终镜像

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
    
    # 检查依赖镜像是否存在
    if ! docker image inspect "orb_slam2_ssd_semantic:deps" &> /dev/null; then
        print_warning "依赖镜像不存在，将先构建依赖镜像..."
        if ! "$SCRIPT_DIR/build-deps.sh"; then
            print_error "依赖镜像构建失败"
            exit 1
        fi
    else
        print_info "✓ 依赖镜像已存在"
    fi
    
    # 检查项目文件
    if [ ! -f "$PROJECT_ROOT/CMakeLists.txt" ]; then
        print_error "项目根目录缺少CMakeLists.txt文件"
        exit 1
    fi
    
    if [ ! -d "$PROJECT_ROOT/src" ]; then
        print_error "项目源码目录不存在"
        exit 1
    fi
    
    print_info "✓ 项目文件检查通过"
}

# 构建最终镜像
build_final_image() {
    print_header "构建最终开发镜像"
    
    cd "$PROJECT_ROOT"
    
    local image_name="orb_slam2_ssd_semantic:latest"
    
    print_info "开始构建最终镜像: $image_name"
    print_info "包含内容: 所有依赖库 + 项目源码 + 构建脚本"
    print_info "预计时间: 3-5分钟"
    
    if docker build \
        -t "$image_name" \
        -f docker/Dockerfile.final \
        .; then
        print_info "✓ 最终镜像构建成功: $image_name"
        return 0
    else
        print_error "✗ 最终镜像构建失败"
        return 1
    fi
}

# 验证最终镜像
verify_final_image() {
    print_header "验证最终镜像"
    
    local image_name="orb_slam2_ssd_semantic:latest"
    
    print_info "验证镜像是否存在..."
    if ! docker image inspect "$image_name" &> /dev/null; then
        print_error "最终镜像不存在！"
        return 1
    fi
    
    print_info "测试项目代码是否正确复制..."
    if docker run --rm "$image_name" test -f CMakeLists.txt; then
        print_info "✓ 项目代码复制正确"
    else
        print_error "✗ 项目代码复制失败"
        return 1
    fi
    
    print_info "测试构建脚本是否可执行..."
    if docker run --rm "$image_name" test -x scripts/build_project.sh; then
        print_info "✓ 构建脚本配置正确"
    else
        print_error "✗ 构建脚本配置失败"
        return 1
    fi
    
    print_info "测试第三方库链接..."
    if docker run --rm "$image_name" test -d Thirdparty/DBoW2/lib; then
        print_info "✓ 第三方库链接正确"
    else
        print_error "✗ 第三方库链接失败"
        return 1
    fi
    
    return 0
}

# 构建并测试项目
build_and_test_project() {
    print_header "构建并测试项目"
    
    local image_name="orb_slam2_ssd_semantic:latest"
    
    print_info "在容器中构建项目..."
    if docker run --rm -v "$PROJECT_ROOT:/home/slam/workspace" "$image_name" ./scripts/build_project.sh; then
        print_info "✓ 项目构建成功"
        return 0
    else
        print_warning "项目构建失败，这可能是正常的（如果缺少某些依赖）"
        print_info "您可以手动进入容器调试构建问题"
        return 1
    fi
}

# 显示使用指南
show_usage_guide() {
    print_header "使用指南"
    
    echo "镜像构建完成！以下是使用方法："
    echo ""
    echo "1. 启动开发环境："
    echo "   ./scripts/run-dev.sh"
    echo ""
    echo "2. 使用Docker Compose启动："
    echo "   docker-compose -f docker/docker-compose.dev.yml up orb-slam-dev"
    echo ""
    echo "3. 手动运行容器："
    echo "   docker run -it --rm \\"
    echo "     -v \$(pwd):/home/slam/workspace \\"
    echo "     -v /tmp/.X11-unix:/tmp/.X11-unix \\"
    echo "     -e DISPLAY=\$DISPLAY \\"
    echo "     orb_slam2_ssd_semantic:latest"
    echo ""
    echo "4. 构建项目："
    echo "   # 在容器内运行"
    echo "   ./scripts/build_project.sh"
    echo ""
    echo "5. 运行测试："
    echo "   # 在容器内运行"
    echo "   ./scripts/run_tests.sh"
}

# 显示构建结果
show_results() {
    print_header "构建结果总结"
    
    echo "所有Docker镜像："
    docker images | grep "orb_slam2_ssd_semantic" || echo "  无相关镜像"
    
    echo ""
    echo "镜像大小："
    docker images --format "table {{.Repository}}:{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" | grep "orb_slam2_ssd_semantic" || echo "  未找到相关镜像"
    
    echo ""
    echo "磁盘使用："
    docker system df
}

# 主函数
main() {
    print_header "ORB_SLAM2_SSD_Semantic 最终镜像构建"
    
    check_prerequisites
    
    if build_final_image && verify_final_image; then
        print_info "✓ 最终镜像验证通过"
        
        # 尝试构建项目（可选）
        print_info "尝试在容器中构建项目..."
        if build_and_test_project; then
            print_info "✓ 项目构建测试通过"
        else
            print_warning "项目构建测试未完全通过，但镜像可用"
        fi
        
        show_results
        show_usage_guide
        print_info "🎉 最终镜像构建成功！"
        exit 0
    else
        print_error "❌ 最终镜像构建失败！"
        exit 1
    fi
}

# 运行主函数
main "$@"
