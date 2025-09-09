#!/bin/bash

# ORB_SLAM2_SSD_Semantic 交互式管理脚本
# 提供菜单式操作界面，简化Docker环境使用

set -e

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
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

print_menu_header() {
    clear
    echo -e "${CYAN}"
    cat << 'EOF'
 ____  ____  ____        ____  _        _    __  __ ____  
/ __ \|  _ \| __ )      / ___|| |      / \  |  \/  |___ \ 
| |  | | |_) |  _ \ _____\___ \| |     / _ \ | |\/| | __) |
| |  | |  _ <| |_) |_____|__) | |___ / ___ \| |  | |/ __/ 
| |__| |_| \_\____/      |____/|_____/_/   \_\_|  |_|_____|
                                                           
     ____  ____  ____    ____                             
    / ___||  _ \|  _ \  / ___|  ___ _ __ ___   __ _ _ __   
    \___ \| |_) | | | | \___ \ / _ \ '_ ` _ \ / _` | '_ \  
     ___) |  _ <| |_| |  ___) |  __/ | | | | | (_| | | | |
    |____/|_| \_\____/  |____/ \___|_| |_| |_|\__,_|_| |_|
                                                           
EOF
    echo -e "${NC}"
    echo -e "${PURPLE}    🔬 语义SLAM Docker环境管理系统${NC}"
    echo ""
}

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$SCRIPT_DIR"

# 检查Docker状态
check_docker_status() {
    if ! command -v docker &> /dev/null; then
        return 1
    fi
    
    if ! docker info &> /dev/null; then
        return 2
    fi
    
    return 0
}

# 检查镜像状态
check_image_status() {
    local system_exists=false
    local deps_exists=false
    local final_exists=false
    
    if docker image inspect "orb_slam2_ssd_semantic:system" &> /dev/null; then
        system_exists=true
    fi
    
    if docker image inspect "orb_slam2_ssd_semantic:deps" &> /dev/null; then
        deps_exists=true
    fi
    
    if docker image inspect "orb_slam2_ssd_semantic:latest" &> /dev/null; then
        final_exists=true
    fi
    
    echo "$system_exists,$deps_exists,$final_exists"
}

# 显示系统状态
show_system_status() {
    echo -e "${CYAN}📊 系统状态${NC}"
    echo "─────────────────────────────────────────────────"
    
    # Docker状态
    local docker_status=""
    check_docker_status
    case $? in
        0) docker_status="${GREEN}✓ 运行中${NC}" ;;
        1) docker_status="${RED}✗ 未安装${NC}" ;;
        2) docker_status="${YELLOW}⚠ 服务未启动${NC}" ;;
    esac
    echo -e "Docker服务:     $docker_status"
    
    # 镜像状态
    IFS=',' read -r system_exists deps_exists final_exists <<< "$(check_image_status)"
    
    local system_status="${RED}✗${NC}"
    local deps_status="${RED}✗${NC}"
    local final_status="${RED}✗${NC}"
    
    [ "$system_exists" = "true" ] && system_status="${GREEN}✓${NC}"
    [ "$deps_exists" = "true" ] && deps_status="${GREEN}✓${NC}"
    [ "$final_exists" = "true" ] && final_status="${GREEN}✓${NC}"
    
    echo -e "系统镜像:       $system_status orb_slam2_ssd_semantic:system"
    echo -e "依赖镜像:       $deps_status orb_slam2_ssd_semantic:deps"
    echo -e "最终镜像:       $final_status orb_slam2_ssd_semantic:latest"
    
    # 容器状态
    local container_status=""
    if docker ps --format '{{.Names}}' | grep -q "orb_slam2"; then
        container_status="${GREEN}✓ 运行中${NC}"
    elif docker ps -a --format '{{.Names}}' | grep -q "orb_slam2"; then
        container_status="${YELLOW}⚠ 已停止${NC}"
    else
        container_status="${RED}✗ 无容器${NC}"
    fi
    echo -e "容器状态:       $container_status"
    
    # 磁盘使用
    if command -v docker &> /dev/null && docker info &> /dev/null; then
        local disk_usage=$(docker system df --format "table {{.Type}}\t{{.Size}}" | grep "Images" | awk '{print $2}' || echo "未知")
        echo -e "镜像占用:       $disk_usage"
    fi
    
    echo "─────────────────────────────────────────────────"
    echo ""
}

# 显示主菜单
show_main_menu() {
    print_menu_header
    show_system_status
    
    echo -e "${YELLOW}🏗️  构建选项${NC}"
    echo "  1) 🔧 构建系统基础镜像 (快速，5-10分钟)"
    echo "  2) 📦 构建依赖库镜像 (较慢，20-30分钟)"
    echo "  3) 🚀 构建最终开发镜像 (快速，3-5分钟)"
    echo "  4) ⚡ 一键构建所有镜像 (30-50分钟)"
    echo ""
    echo -e "${GREEN}🎮 运行选项${NC}"
    echo "  5) 🖥️  启动开发环境"
    echo "  6) 🏃 启动项目构建"
    echo "  7) 🧪 运行测试验证"
    echo "  8) 📊 使用Docker Compose"
    echo ""
    echo -e "${BLUE}🛠️  管理选项${NC}"
    echo "  9) 📋 查看容器状态"
    echo " 10) 🧹 清理Docker环境"
    echo " 11) 📖 查看使用文档"
    echo " 12) ⚙️  环境诊断"
    echo ""
    echo -e "${RED}❌ 退出${NC}"
    echo " 13) 退出程序"
    echo ""
    echo -n "请选择操作 [1-13]: "
}

# 构建系统镜像
build_system_image() {
    print_header "构建系统基础镜像"
    print_info "开始构建系统镜像，包含编译工具链和系统依赖..."
    
    if [ -f "$SCRIPT_DIR/docker/scripts/build-system.sh" ]; then
        cd "$SCRIPT_DIR" && docker/scripts/build-system.sh
    else
        print_error "构建脚本不存在！"
    fi
    
    echo ""
    read -p "按Enter键继续..."
}

# 构建依赖镜像
build_deps_image() {
    print_header "构建依赖库镜像"
    print_warning "这是最耗时的步骤，需要编译多个C++库"
    print_info "预计时间: 20-30分钟（取决于机器性能）"
    
    echo "即将编译以下库："
    echo "  • Eigen3 (矩阵运算库)"
    echo "  • OpenCV (计算机视觉库)"
    echo "  • Pangolin (3D可视化库)"
    echo "  • PCL (点云处理库)"
    echo "  • OctoMap (八叉树地图库)"
    echo "  • NCNN (神经网络推理库)"
    echo "  • DBoW2 (词袋模型库)"
    echo "  • g2o (图优化库)"
    echo ""
    
    read -p "确认开始构建？ [y/N]: " confirm
    if [[ $confirm =~ ^[Yy]$ ]]; then
        if [ -f "$SCRIPT_DIR/docker/scripts/build-deps.sh" ]; then
            cd "$SCRIPT_DIR" && docker/scripts/build-deps.sh
        else
            print_error "构建脚本不存在！"
        fi
    else
        print_info "已取消构建"
    fi
    
    echo ""
    read -p "按Enter键继续..."
}

# 构建最终镜像
build_final_image() {
    print_header "构建最终开发镜像"
    print_info "基于依赖镜像构建包含项目代码的开发环境..."
    
    if [ -f "$SCRIPT_DIR/docker/scripts/build-final.sh" ]; then
        cd "$SCRIPT_DIR" && docker/scripts/build-final.sh
    else
        print_error "构建脚本不存在！"
    fi
    
    echo ""
    read -p "按Enter键继续..."
}

# 一键构建所有镜像
build_all_images() {
    print_header "一键构建所有镜像"
    print_warning "这将构建完整的Docker环境，需要较长时间"
    print_info "总预计时间: 30-50分钟"
    
    echo "构建步骤："
    echo "  1. 系统基础镜像 (5-10分钟)"
    echo "  2. 依赖库镜像 (20-30分钟)"
    echo "  3. 最终开发镜像 (3-5分钟)"
    echo "  4. 测试验证 (2-5分钟)"
    echo ""
    
    read -p "确认开始完整构建？ [y/N]: " confirm
    if [[ $confirm =~ ^[Yy]$ ]]; then
        if [ -f "$SCRIPT_DIR/docker/scripts/build-all.sh" ]; then
            cd "$SCRIPT_DIR" && docker/scripts/build-all.sh
        else
            print_error "构建脚本不存在！"
        fi
    else
        print_info "已取消构建"
    fi
    
    echo ""
    read -p "按Enter键继续..."
}

# 启动开发环境
start_dev_environment() {
    print_header "启动开发环境"
    
    # 检查镜像是否存在
    if ! docker image inspect "orb_slam2_ssd_semantic:latest" &> /dev/null; then
        print_error "开发镜像不存在！"
        print_info "请先构建镜像："
        print_info "  选择选项 3 或 4"
        read -p "按Enter键继续..."
        return
    fi
    
    print_info "启动交互式开发容器..."
    print_info "容器启动后您将进入开发环境"
    
    if [ -f "$SCRIPT_DIR/docker/scripts/run-dev.sh" ]; then
        cd "$SCRIPT_DIR" && docker/scripts/run-dev.sh
    else
        print_error "启动脚本不存在！"
        read -p "按Enter键继续..."
    fi
}

# 启动项目构建
start_project_build() {
    print_header "启动项目构建"
    
    if ! docker image inspect "orb_slam2_ssd_semantic:latest" &> /dev/null; then
        print_error "开发镜像不存在！请先构建镜像"
        read -p "按Enter键继续..."
        return
    fi
    
    print_info "在容器中构建ORB_SLAM2_SSD_Semantic项目..."
    
    docker run --rm \
        -v "$SCRIPT_DIR:/home/slam/workspace" \
        orb_slam2_ssd_semantic:latest \
        ./docker/scripts/build_project.sh
    
    echo ""
    read -p "按Enter键继续..."
}

# 运行测试验证
run_tests() {
    print_header "运行测试验证"
    
    if ! docker image inspect "orb_slam2_ssd_semantic:latest" &> /dev/null; then
        print_error "开发镜像不存在！请先构建镜像"
        read -p "按Enter键继续..."
        return
    fi
    
    print_info "在容器中运行测试套件..."
    
    docker run --rm \
        -v "$SCRIPT_DIR:/home/slam/workspace" \
        orb_slam2_ssd_semantic:latest \
        ./docker/scripts/run_tests.sh
    
    echo ""
    read -p "按Enter键继续..."
}

# 使用Docker Compose
use_docker_compose() {
    print_header "Docker Compose 选项"
    
    echo "可用的Docker Compose配置："
    echo "  1) 开发环境 (docker/docker-compose.dev.yml)"
    echo "  2) 生产环境 (docker-compose.yml)"
    echo "  3) 返回主菜单"
    echo ""
    echo -n "请选择 [1-3]: "
    read compose_choice
    
    case $compose_choice in
        1)
            print_info "启动开发环境..."
            cd "$SCRIPT_DIR" && docker-compose -f docker/docker-compose.dev.yml up
            ;;
        2)
            print_info "启动生产环境..."
            cd "$SCRIPT_DIR" && docker-compose up
            ;;
        3)
            return
            ;;
        *)
            print_error "无效选择"
            ;;
    esac
    
    echo ""
    read -p "按Enter键继续..."
}

# 查看容器状态
show_container_status() {
    print_header "容器状态详情"
    
    echo "Docker镜像："
    docker images | grep "orb_slam2_ssd_semantic" || echo "  无相关镜像"
    
    echo ""
    echo "运行中的容器："
    docker ps | grep "orb_slam2" || echo "  无运行中的容器"
    
    echo ""
    echo "所有相关容器："
    docker ps -a | grep "orb_slam2" || echo "  无相关容器"
    
    echo ""
    echo "Docker系统信息："
    docker system df
    
    echo ""
    read -p "按Enter键继续..."
}

# 清理Docker环境
cleanup_docker() {
    print_header "清理Docker环境"
    
    echo "清理选项："
    echo "  1) 清理停止的容器"
    echo "  2) 清理所有ORB_SLAM2相关镜像"
    echo "  3) 清理所有未使用的Docker资源"
    echo "  4) 返回主菜单"
    echo ""
    echo -n "请选择 [1-4]: "
    read cleanup_choice
    
    case $cleanup_choice in
        1)
            print_info "清理停止的容器..."
            docker container prune -f
            ;;
        2)
            print_warning "这将删除所有ORB_SLAM2相关镜像"
            read -p "确认删除？ [y/N]: " confirm
            if [[ $confirm =~ ^[Yy]$ ]]; then
                docker images | grep "orb_slam2_ssd_semantic" | awk '{print $3}' | xargs -r docker rmi -f
            fi
            ;;
        3)
            print_warning "这将清理所有未使用的Docker资源"
            read -p "确认清理？ [y/N]: " confirm
            if [[ $confirm =~ ^[Yy]$ ]]; then
                docker system prune -a -f
            fi
            ;;
        4)
            return
            ;;
        *)
            print_error "无效选择"
            ;;
    esac
    
    echo ""
    read -p "按Enter键继续..."
}

# 查看使用文档
show_documentation() {
    print_header "使用文档"
    
    cat << 'EOF'
📖 ORB_SLAM2_SSD_Semantic Docker环境使用指南

🏗️ 构建流程：
  1. 首次使用建议选择"一键构建所有镜像"
  2. 如需分步构建，请按照 系统镜像 → 依赖镜像 → 最终镜像 的顺序

🎮 开发流程：
  1. 启动开发环境（选项5）
  2. 在容器内运行：./docker/scripts/build_project.sh
  3. 测试：./docker/scripts/run_tests.sh

📊 项目结构：
  • src/        主要源码
  • perfect/    完善版本
  • include/    头文件
  • docker/     Docker配置
  • build/      构建目录
  • lib/        生成的库
  • bin/        可执行文件

🔧 常用命令：
  • 构建项目：./docker/scripts/build_project.sh
  • 运行测试：./docker/scripts/run_tests.sh
  • 验证依赖：/usr/local/bin/verify-deps.sh

❓ 故障排除：
  • 构建失败：检查网络连接和磁盘空间
  • GUI问题：确保X11转发正确设置
  • 权限问题：确保Docker用户权限正确

📞 获取帮助：
  • 查看脚本帮助：./docker/scripts/docker_manager.sh help
  • 查看构建日志：docker logs <container_id>
EOF
    
    echo ""
    read -p "按Enter键继续..."
}

# 环境诊断
diagnose_environment() {
    print_header "环境诊断"
    
    print_info "正在检查环境配置..."
    
    echo "🐳 Docker环境："
    echo "  Docker版本: $(docker --version 2>/dev/null || echo '未安装')"
    echo "  Docker Compose版本: $(docker-compose --version 2>/dev/null || echo '未安装')"
    echo "  Docker服务状态: $(docker info >/dev/null 2>&1 && echo '运行中' || echo '未运行')"
    
    echo ""
    echo "💾 系统资源："
    echo "  可用磁盘空间: $(df -h . | awk 'NR==2 {print $4}')"
    echo "  可用内存: $(free -h | awk 'NR==2{print $7}')"
    echo "  CPU核心数: $(nproc)"
    
    echo ""
    echo "🖥️ 显示支持："
    echo "  DISPLAY变量: ${DISPLAY:-未设置}"
    echo "  X11目录: $(ls -la /tmp/.X11-unix 2>/dev/null | wc -l) 个套接字文件"
    
    echo ""
    echo "🏗️ 项目文件："
    echo "  CMakeLists.txt: $(test -f CMakeLists.txt && echo '存在' || echo '缺失')"
    echo "  src目录: $(test -d src && echo '存在' || echo '缺失')"
    echo "  docker目录: $(test -d docker && echo '存在' || echo '缺失')"
    
    if command -v nvidia-smi &> /dev/null; then
        echo ""
        echo "🎮 GPU支持："
        echo "  NVIDIA GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n1 || echo '未检测到')"
        echo "  CUDA版本: $(nvidia-smi | grep "CUDA Version" | awk '{print $9}' || echo '未知')"
    fi
    
    echo ""
    read -p "按Enter键继续..."
}

# 主循环
main_loop() {
    while true; do
        show_main_menu
        read choice
        
        case $choice in
            1) build_system_image ;;
            2) build_deps_image ;;
            3) build_final_image ;;
            4) build_all_images ;;
            5) start_dev_environment ;;
            6) start_project_build ;;
            7) run_tests ;;
            8) use_docker_compose ;;
            9) show_container_status ;;
            10) cleanup_docker ;;
            11) show_documentation ;;
            12) diagnose_environment ;;
            13) 
                print_info "感谢使用ORB_SLAM2_SSD_Semantic Docker环境！"
                exit 0
                ;;
            *)
                print_error "无效选择，请输入1-13"
                sleep 1
                ;;
        esac
    done
}

# 检查初始环境
check_initial_environment() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker未安装！请先安装Docker。"
        echo ""
        echo "安装指南："
        echo "  Ubuntu: https://docs.docker.com/engine/install/ubuntu/"
        echo "  其他系统: https://docs.docker.com/get-docker/"
        exit 1
    fi
    
    if ! docker info &> /dev/null; then
        print_error "无法连接到Docker守护进程！"
        print_info "请启动Docker服务："
        print_info "  sudo systemctl start docker"
        exit 1
    fi
}

# 主函数
main() {
    # 切换到脚本所在目录
    cd "$SCRIPT_DIR"
    
    # 检查基本环境
    check_initial_environment
    
    # 显示欢迎信息
    print_info "欢迎使用ORB_SLAM2_SSD_Semantic Docker环境管理系统！"
    
    # 进入主循环
    main_loop
}

# 运行主函数
main "$@"
