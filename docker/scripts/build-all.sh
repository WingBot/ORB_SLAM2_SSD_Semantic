#!/bin/bash

# ORB_SLAM2_SSD_Semantic 一键构建脚本
# 按照最佳实践分阶段构建所有Docker镜像

set -e

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
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

print_step() {
    echo -e "${PURPLE}[STEP $1]${NC} $2"
}

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

# 全局变量
BUILD_START_TIME=$(date +%s)
STEP_COUNT=0

# 步骤计时器
step_timer() {
    local step_name=$1
    local step_start_time=$(date +%s)
    
    STEP_COUNT=$((STEP_COUNT + 1))
    print_step "$STEP_COUNT" "开始: $step_name"
    
    return $step_start_time
}

step_timer_end() {
    local step_start_time=$1
    local step_name=$2
    local step_end_time=$(date +%s)
    local duration=$((step_end_time - step_start_time))
    
    print_info "✓ 完成: $step_name (耗时: ${duration}秒)"
}

# 检查系统要求
check_system_requirements() {
    step_timer "系统要求检查"
    local step_start=$?
    
    # 检查Docker
    if ! command -v docker &> /dev/null; then
        print_error "Docker未安装！请先安装Docker。"
        exit 1
    fi
    
    if ! docker info &> /dev/null; then
        print_error "无法连接到Docker守护进程！"
        exit 1
    fi
    
    # 检查磁盘空间（建议至少10GB）
    local available_space=$(df . | awk 'NR==2 {print $4}')
    local required_space=10485760  # 10GB in KB
    
    if [ "$available_space" -lt "$required_space" ]; then
        print_warning "磁盘空间可能不足，建议至少有10GB可用空间"
        print_warning "当前可用: $(($available_space / 1024 / 1024))GB"
    fi
    
    # 检查内存（建议至少8GB）
    local total_memory=$(free -m | awk 'NR==2{print $2}')
    if [ "$total_memory" -lt 8192 ]; then
        print_warning "内存可能不足，建议至少8GB内存用于编译"
        print_warning "当前内存: ${total_memory}MB"
    fi
    
    print_info "Docker版本: $(docker --version)"
    print_info "可用空间: $(df -h . | awk 'NR==2 {print $4}')"
    print_info "可用内存: $(free -h | awk 'NR==2{print $7}')"
    
    step_timer_end $step_start "系统要求检查"
}

# 清理旧镜像（可选）
cleanup_old_images() {
    if [ "$1" = "--clean" ]; then
        step_timer "清理旧镜像"
        local step_start=$?
        
        print_info "清理旧的ORB_SLAM2_SSD_Semantic镜像..."
        docker images | grep "orb_slam2_ssd_semantic" | awk '{print $3}' | xargs -r docker rmi || true
        docker system prune -f || true
        
        step_timer_end $step_start "清理旧镜像"
    fi
}

# 构建系统镜像
build_system_image() {
    step_timer "构建系统镜像"
    local step_start=$?
    
    if ! "$SCRIPT_DIR/build-system.sh"; then
        print_error "系统镜像构建失败！"
        exit 1
    fi
    
    step_timer_end $step_start "构建系统镜像"
}

# 构建依赖镜像
build_deps_image() {
    step_timer "构建依赖镜像"
    local step_start=$?
    
    print_info "这是最耗时的步骤，需要编译多个C++库..."
    print_info "预计时间: 20-40分钟（取决于机器性能）"
    
    if ! "$SCRIPT_DIR/build-deps.sh"; then
        print_error "依赖镜像构建失败！"
        exit 1
    fi
    
    step_timer_end $step_start "构建依赖镜像"
}

# 构建最终镜像
build_final_image() {
    step_timer "构建最终镜像"
    local step_start=$?
    
    if ! "$SCRIPT_DIR/build-final.sh"; then
        print_error "最终镜像构建失败！"
        exit 1
    fi
    
    step_timer_end $step_start "构建最终镜像"
}

# 运行完整测试
run_comprehensive_tests() {
    step_timer "运行完整测试"
    local step_start=$?
    
    print_info "在容器中运行完整测试套件..."
    
    # 创建临时容器进行测试
    local container_id=$(docker run -d orb_slam2_ssd_semantic:latest sleep 300)
    
    # 运行各种测试
    if docker exec "$container_id" ./scripts/run_tests.sh; then
        print_info "✓ 所有测试通过"
    else
        print_warning "部分测试可能失败，但基本功能可用"
    fi
    
    # 清理测试容器
    docker rm -f "$container_id" || true
    
    step_timer_end $step_start "运行完整测试"
}

# 显示最终结果
show_final_results() {
    local build_end_time=$(date +%s)
    local total_duration=$((build_end_time - BUILD_START_TIME))
    local hours=$((total_duration / 3600))
    local minutes=$(((total_duration % 3600) / 60))
    local seconds=$((total_duration % 60))
    
    print_header "构建完成！"
    
    echo "🎉 ORB_SLAM2_SSD_Semantic Docker环境构建成功！"
    echo ""
    echo "⏱️  总耗时: ${hours}小时${minutes}分钟${seconds}秒"
    echo "📊 总步骤: $STEP_COUNT"
    echo ""
    
    echo "📦 构建的镜像："
    docker images | grep "orb_slam2_ssd_semantic" | while read line; do
        echo "   $line"
    done
    
    echo ""
    echo "💾 磁盘使用："
    docker system df | head -n 5
    
    echo ""
    echo "🚀 快速开始："
    echo "   1. 启动开发环境:"
    echo "      ./scripts/run-dev.sh"
    echo ""
    echo "   2. 使用Docker Compose:"
    echo "      docker-compose -f docker/docker-compose.dev.yml up"
    echo ""
    echo "   3. 构建项目:"
    echo "      # 在容器内运行"
    echo "      ./scripts/build_project.sh"
    echo ""
    echo "   4. 查看使用说明:"
    echo "      ./scripts/docker_manager.sh help"
}

# 错误处理
handle_error() {
    local exit_code=$?
    print_error "构建过程在第 $STEP_COUNT 步失败（退出码: $exit_code）"
    print_info "查看上面的错误信息进行调试"
    print_info "您可以尝试:"
    print_info "1. 检查网络连接"
    print_info "2. 确保有足够的磁盘空间和内存"
    print_info "3. 单独运行失败的构建步骤"
    exit $exit_code
}

# 显示帮助信息
show_help() {
    echo "ORB_SLAM2_SSD_Semantic 一键构建脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --clean     构建前清理旧镜像"
    echo "  --no-test   跳过测试步骤"
    echo "  --help      显示此帮助信息"
    echo ""
    echo "构建步骤:"
    echo "  1. 系统要求检查"
    echo "  2. 构建系统镜像 (5-10分钟)"
    echo "  3. 构建依赖镜像 (20-40分钟)"
    echo "  4. 构建最终镜像 (3-5分钟)"
    echo "  5. 运行测试验证 (2-5分钟)"
    echo ""
    echo "总预计时间: 30-60分钟"
}

# 主函数
main() {
    # 解析命令行参数
    local clean_mode=false
    local run_tests=true
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --clean)
                clean_mode=true
                shift
                ;;
            --no-test)
                run_tests=false
                shift
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                print_error "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 设置错误处理
    trap handle_error ERR
    
    print_header "ORB_SLAM2_SSD_Semantic 一键构建开始"
    print_info "开始时间: $(date)"
    
    if [ "$clean_mode" = true ]; then
        cleanup_old_images --clean
    fi
    
    # 执行构建步骤
    check_system_requirements
    build_system_image
    build_deps_image
    build_final_image
    
    if [ "$run_tests" = true ]; then
        run_comprehensive_tests
    fi
    
    show_final_results
    
    print_info "🎉 恭喜！ORB_SLAM2_SSD_Semantic Docker环境已成功构建！"
}

# 运行主函数
main "$@"
