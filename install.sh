#!/bin/bash

# ORB_SLAM2_SSD_Semantic Development Environment Setup Script
# This script sets up the complete development environment including Docker and local dependencies

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
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

# Check if running on supported OS
check_os() {
    log_info "Checking operating system..."
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if [ -f /etc/lsb-release ]; then
            . /etc/lsb-release
            if [[ "$DISTRIB_ID" == "Ubuntu" ]]; then
                log_success "Ubuntu detected: $DISTRIB_DESCRIPTION"
                return 0
            fi
        fi
    fi
    log_error "This script is designed for Ubuntu. Other systems may not be fully supported."
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
}

# Install Docker if not present
install_docker() {
    if command -v docker &> /dev/null; then
        log_success "Docker is already installed"
        return 0
    fi
    
    log_info "Installing Docker..."
    
    # Remove old versions
    sudo apt-get remove docker docker-engine docker.io containerd runc || true
    
    # Update package index
    sudo apt-get update
    
    # Install packages to allow apt to use a repository over HTTPS
    sudo apt-get install -y \
        apt-transport-https \
        ca-certificates \
        curl \
        gnupg \
        lsb-release
    
    # Add Docker's official GPG key
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    
    # Set up the stable repository
    echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
        $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    # Install Docker Engine
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io
    
    # Add user to docker group
    sudo usermod -aG docker $USER
    
    log_success "Docker installed successfully"
    log_warning "Please log out and log back in for Docker group changes to take effect"
}

# Install Docker Compose
install_docker_compose() {
    if command -v docker-compose &> /dev/null; then
        log_success "Docker Compose is already installed"
        return 0
    fi
    
    log_info "Installing Docker Compose..."
    
    # Download and install Docker Compose
    sudo curl -L "https://github.com/docker/compose/releases/download/v2.12.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
    
    log_success "Docker Compose installed successfully"
}

# Install local development dependencies
install_local_deps() {
    log_info "Installing local development dependencies..."
    
    # Update package list
    sudo apt-get update
    
    # Install basic development tools
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        curl \
        wget \
        zsh \
        vim \
        nano \
        htop \
        tree \
        unzip \
        software-properties-common \
        python3 \
        python3-pip \
        python3-dev \
        python3-venv
    
    # Install OpenCV development libraries (for local development)
    sudo apt-get install -y \
        libopencv-dev \
        libopencv-contrib-dev \
        libeigen3-dev \
        libpcl-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libatlas-base-dev \
        libsuitesparse-dev
    
    log_success "Local development dependencies installed"
}

# Setup Python virtual environment
setup_python_env() {
    log_info "Setting up Python virtual environment..."
    
    # Create virtual environment if it doesn't exist
    if [ ! -d "venv" ]; then
        python3 -m venv venv
        log_success "Virtual environment created"
    else
        log_info "Virtual environment already exists"
    fi
    
    # Activate virtual environment and install requirements
    source venv/bin/activate
    pip install --upgrade pip
    
    if [ -f "requirements.txt" ]; then
        pip install -r requirements.txt
        log_success "Python requirements installed"
    else
        log_warning "requirements.txt not found, skipping Python package installation"
    fi
}

# Build Docker development image
build_docker_image() {
    log_info "Building Docker development image..."
    
    if [ ! -f "Dockerfile.dev" ]; then
        log_error "Dockerfile.dev not found!"
        return 1
    fi
    
    if [ ! -f "requirements.txt" ]; then
        log_error "requirements.txt not found!"
        return 1
    fi
    
    docker build -f Dockerfile.dev -t orb-slam2-ssd-semantic:dev .
    log_success "Docker development image built successfully"
}

# Create development scripts
create_dev_scripts() {
    log_info "Creating development helper scripts..."
    
    # Create script to run development container
    cat > run-dev-container.sh << 'EOF'
#!/bin/bash

# Script to run the development container with proper volume mounts and settings

CONTAINER_NAME="orb-slam2-dev"
IMAGE_NAME="orb-slam2-ssd-semantic:dev"

# Stop and remove existing container if running
docker stop $CONTAINER_NAME 2>/dev/null || true
docker rm $CONTAINER_NAME 2>/dev/null || true

# Run the development container
docker run -it --rm \
    --name $CONTAINER_NAME \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/workspace/orb_slam2_ssd_semantic \
    -v /dev:/dev \
    $IMAGE_NAME
EOF
    
    chmod +x run-dev-container.sh
    
    # Create script for local development
    cat > setup-local-dev.sh << 'EOF'
#!/bin/bash

# Script to set up local development environment

echo "Setting up local development environment..."

# Activate virtual environment
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
    echo "Virtual environment activated"
else
    echo "Virtual environment not found. Run install.sh first."
    exit 1
fi

# Set environment variables
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/lib

echo "Local development environment ready!"
echo "Run 'source venv/bin/activate' to activate Python environment"
EOF
    
    chmod +x setup-local-dev.sh
    
    log_success "Development helper scripts created"
}

# Create docker-compose for development services
create_docker_compose() {
    log_info "Creating docker-compose configuration..."
    
    cat > docker-compose.dev.yml << 'EOF'
version: '3.8'

services:
  orb-slam2-dev:
    build:
      context: .
      dockerfile: Dockerfile.dev
    image: orb-slam2-ssd-semantic:dev
    container_name: orb-slam2-dev
    privileged: true
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - .:/workspace/orb_slam2_ssd_semantic
      - /dev:/dev
    working_dir: /workspace/orb_slam2_ssd_semantic
    stdin_open: true
    tty: true
    command: /bin/zsh

  # Optional: Add a Jupyter notebook service for development
  jupyter:
    build:
      context: .
      dockerfile: Dockerfile.dev
    image: orb-slam2-ssd-semantic:dev
    container_name: orb-slam2-jupyter
    ports:
      - "8888:8888"
    volumes:
      - .:/workspace/orb_slam2_ssd_semantic
    working_dir: /workspace/orb_slam2_ssd_semantic
    command: jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser --allow-root
EOF
    
    log_success "Docker Compose configuration created"
}

# Main installation function
main() {
    echo "=============================================="
    echo "ORB_SLAM2_SSD_Semantic Development Setup"
    echo "=============================================="
    echo
    
    check_os
    
    echo
    echo "This script will install:"
    echo "1. Docker and Docker Compose"
    echo "2. Local development dependencies"
    echo "3. Python virtual environment with required packages"
    echo "4. Build Docker development image"
    echo "5. Create helper scripts for development"
    echo
    
    read -p "Continue with installation? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "Installation cancelled by user"
        exit 0
    fi
    
    echo
    log_info "Starting installation..."
    
    install_docker
    install_docker_compose
    install_local_deps
    setup_python_env
    create_dev_scripts
    create_docker_compose
    build_docker_image
    
    echo
    echo "=============================================="
    log_success "Installation completed successfully!"
    echo "=============================================="
    echo
    echo "Next steps:"
    echo "1. Log out and log back in for Docker group changes to take effect"
    echo "2. Run './run-dev-container.sh' to start development container"
    echo "3. Or run 'docker-compose -f docker-compose.dev.yml up orb-slam2-dev' for compose version"
    echo "4. For local development, run './setup-local-dev.sh'"
    echo
    echo "For Jupyter notebook access:"
    echo "   docker-compose -f docker-compose.dev.yml up jupyter"
    echo "   Then open http://localhost:8888 in your browser"
    echo
}

# Run main function
main "$@"