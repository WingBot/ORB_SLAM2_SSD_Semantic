#!/bin/bash

# Quick Start Script for ORB_SLAM2_SSD_Semantic Development Environment
# This script provides immediate access to a working development environment

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if Docker is available
check_docker() {
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed!"
        echo
        echo "Please install Docker first:"
        echo "  Ubuntu/Debian: sudo apt-get install docker.io"
        echo "  Or run the full installation: ./install.sh"
        return 1
    fi
    
    if ! docker info &> /dev/null; then
        log_error "Docker is not running or you don't have permission!"
        echo
        echo "Try:"
        echo "  sudo systemctl start docker"
        echo "  sudo usermod -aG docker \$USER"
        echo "  Then log out and log back in"
        return 1
    fi
    
    return 0
}

# Function to build minimal image
build_minimal_image() {
    log_info "Building minimal development image..."
    
    if [ -f "Dockerfile.minimal" ]; then
        docker build -f Dockerfile.minimal -t orb-slam2-ssd-semantic:minimal .
    else
        log_error "Dockerfile.minimal not found!"
        return 1
    fi
}

# Function to run development container
run_dev_container() {
    local image_name="orb-slam2-ssd-semantic:minimal"
    local container_name="orb-slam2-quickstart"
    
    log_info "Starting development container..."
    
    # Stop existing container if running
    docker stop $container_name 2>/dev/null || true
    docker rm $container_name 2>/dev/null || true
    
    # Run the container
    docker run -it --rm \
        --name $container_name \
        --privileged \
        --network host \
        -e DISPLAY=${DISPLAY:-:0} \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $(pwd):/workspace/orb_slam2_ssd_semantic \
        -v /dev:/dev \
        -w /workspace/orb_slam2_ssd_semantic \
        $image_name
}

# Function to set up local development
setup_local_dev() {
    log_info "Setting up local development environment..."
    
    # Install essential packages
    if command -v apt-get &> /dev/null; then
        log_info "Installing essential packages..."
        sudo apt-get update
        sudo apt-get install -y \
            build-essential \
            cmake \
            git \
            python3 \
            python3-pip \
            python3-venv \
            libopencv-dev \
            libeigen3-dev
    fi
    
    # Create Python virtual environment
    if [ ! -d "venv" ]; then
        log_info "Creating Python virtual environment..."
        python3 -m venv venv
    fi
    
    # Activate venv and install packages
    log_info "Installing Python packages..."
    source venv/bin/activate
    pip install --upgrade pip
    
    # Install essential packages even if requirements.txt is missing
    pip install numpy opencv-python matplotlib || true
    
    if [ -f "requirements.txt" ]; then
        pip install -r requirements.txt || log_warning "Some Python packages failed to install"
    fi
    
    log_success "Local development environment ready!"
    echo "Run 'source venv/bin/activate' to activate Python environment"
}

# Main function
main() {
    echo "=============================================="
    echo "ORB_SLAM2_SSD_Semantic Quick Start"
    echo "=============================================="
    echo
    
    case "${1:-}" in
        "docker")
            if check_docker; then
                if ! docker image ls | grep -q "orb-slam2-ssd-semantic:minimal"; then
                    build_minimal_image
                fi
                run_dev_container
            fi
            ;;
        "local")
            setup_local_dev
            ;;
        "build")
            if check_docker; then
                build_minimal_image
                log_success "Minimal image built successfully!"
            fi
            ;;
        *)
            echo "Usage: $0 [docker|local|build]"
            echo
            echo "Commands:"
            echo "  docker  - Build and run Docker development environment"
            echo "  local   - Set up local development environment"
            echo "  build   - Build Docker image only"
            echo
            echo "For first-time setup, run: ./install.sh"
            echo
            
            # Auto-detect best option
            if check_docker &> /dev/null; then
                log_info "Docker is available. Recommend: $0 docker"
            else
                log_info "Docker not available. Recommend: $0 local"
            fi
            ;;
    esac
}

main "$@"