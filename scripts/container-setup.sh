#!/bin/bash

# Container Setup Script for ORB_SLAM2_SSD_Semantic Development Environment
# This script sets up the complete environment inside the container

set -e

echo "Setting up ORB_SLAM2_SSD_Semantic development environment..."

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

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Update package repositories
log_info "Updating package repositories..."
apt-get update || {
    log_error "Failed to update repositories. Trying alternative mirrors..."
    sed -i 's|http://archive.ubuntu.com/ubuntu|http://mirrors.aliyun.com/ubuntu|g' /etc/apt/sources.list
    sed -i 's|http://security.ubuntu.com/ubuntu|http://mirrors.aliyun.com/ubuntu|g' /etc/apt/sources.list
    apt-get update
}

# Install basic packages from apt-packages.txt
log_info "Installing system packages..."
if [ -f "/tmp/apt-packages.txt" ]; then
    # Filter out empty lines and comments
    packages=$(grep -v '^#' /tmp/apt-packages.txt | grep -v '^$' | tr '\n' ' ')
    
    # Install packages in chunks to handle failures
    for package in $packages; do
        apt-get install -y $package || log_error "Failed to install $package"
    done
else
    # Install essential packages if list file is missing
    apt-get install -y \
        build-essential \
        cmake \
        git \
        curl \
        wget \
        python3 \
        python3-pip \
        python3-dev \
        vim \
        nano
fi

# Install ROS if available
log_info "Setting up ROS repository..."
if ! command -v curl &> /dev/null; then
    apt-get install -y curl
fi

# Add ROS repository
echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - || true

apt-get update || true

# Install ROS packages
log_info "Installing ROS packages..."
if [ -f "/tmp/ros-packages.txt" ]; then
    ros_packages=$(grep -v '^#' /tmp/ros-packages.txt | grep -v '^$' | tr '\n' ' ')
    for package in $ros_packages; do
        apt-get install -y $package || log_error "Failed to install ROS package $package"
    done
else
    # Install basic ROS if package list is missing
    apt-get install -y ros-noetic-desktop-full || \
    apt-get install -y ros-noetic-desktop || \
    apt-get install -y ros-noetic-ros-base || \
    log_error "Failed to install ROS"
fi

# Initialize rosdep
log_info "Initializing rosdep..."
if command -v rosdep &> /dev/null; then
    rosdep init || true
    rosdep update || true
fi

# Install Python packages
log_info "Installing Python packages..."
if [ -f "/tmp/requirements.txt" ]; then
    pip3 install --upgrade pip
    pip3 install -r /tmp/requirements.txt || log_error "Some Python packages failed to install"
fi

# Set up development environment
log_info "Setting up development environment..."

# Add ROS setup to bashrc
echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc || true

# Install Oh My Zsh (if zsh is available)
if command -v zsh &> /dev/null; then
    log_info "Installing Oh My Zsh..."
    wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true
    echo "source /opt/ros/noetic/setup.zsh" >> /root/.zshrc || true
fi

# Create development directories
mkdir -p /workspace/build
mkdir -p /workspace/data
mkdir -p /workspace/logs

# Set permissions
chown -R developer:developer /workspace 2>/dev/null || true
chown -R developer:developer /home/developer 2>/dev/null || true

# Clean up
apt-get autoremove -y
apt-get autoclean
rm -rf /var/lib/apt/lists/*

# Mark container as configured
touch /.container-configured

log_success "Development environment setup completed!"

# Show installed versions
log_info "Environment summary:"
echo "=================================="
python3 --version || echo "Python3: Not available"
if command -v ros &> /dev/null; then
    echo "ROS: Available"
else
    echo "ROS: Not available"
fi
if command -v cmake &> /dev/null; then
    cmake --version | head -1 || echo "CMake: Not available"
else
    echo "CMake: Not available"
fi
echo "=================================="

log_info "Container setup complete! You can now start development."