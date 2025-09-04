# ORB_SLAM2_SSD_Semantic Development Environment

This document describes the containerized development environment setup for the ORB_SLAM2_SSD_Semantic project.

## Overview

The development environment includes:
- **ROS Noetic** - Robot Operating System for robotics development
- **Python Qt GUI tools** - For graphical user interface development
- **Computer Vision libraries** - OpenCV, PCL, Eigen3
- **Development tools** - Git, Zsh, Curl, build tools
- **Containerized setup** - Docker-based isolated environment

## Quick Start

### 1. Automated Installation

Run the installation script to set up everything automatically:

```bash
./install.sh
```

This will:
- Install Docker and Docker Compose
- Install local development dependencies
- Set up Python virtual environment
- Build the development Docker image
- Create helper scripts

### 2. Using Docker Compose (Recommended)

Start the development environment:

```bash
# Start the main development container
docker-compose -f docker-compose.dev.yml up orb-slam2-dev

# Or run in background
docker-compose -f docker-compose.dev.yml up -d orb-slam2-dev

# Access the container
docker-compose -f docker-compose.dev.yml exec orb-slam2-dev zsh
```

### 3. Using Helper Scripts

After running `install.sh`, use the generated helper scripts:

```bash
# Run development container
./run-dev-container.sh

# Set up local development environment
./setup-local-dev.sh
```

## Development Services

### Main Development Container
- **Service name**: `orb-slam2-dev`
- **Features**: Full development environment with ROS, OpenCV, GUI support
- **GUI**: X11 forwarding enabled for GUI applications
- **Volumes**: Source code mounted, build artifacts persisted

### Jupyter Notebook
- **Service name**: `jupyter`
- **Port**: http://localhost:8888
- **Usage**: Data analysis, visualization, prototyping

```bash
docker-compose -f docker-compose.dev.yml up jupyter
```

### ROS Core
- **Service name**: `roscore`
- **Port**: 11311
- **Usage**: ROS master node for distributed ROS applications

```bash
docker-compose -f docker-compose.dev.yml up roscore
```

### RViz Visualization
- **Service name**: `rviz`
- **Usage**: 3D visualization of SLAM data, point clouds, trajectories

```bash
# Make sure X11 forwarding is set up
xhost +local:docker
docker-compose -f docker-compose.dev.yml up rviz
```

## Development Workflows

### Building the ORB-SLAM2 Project

Inside the development container:

```bash
# Navigate to project directory
cd /workspace/orb_slam2_ssd_semantic

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build the project
make -j$(nproc)

# Install libraries
make install
```

### Running with ROS

```bash
# Start ROS core
docker-compose -f docker-compose.dev.yml up -d roscore

# In the development container
source /opt/ros/noetic/setup.bash

# Run ORB-SLAM2 with ROS
rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml
```

### Python Development

```bash
# Using local virtual environment
source venv/bin/activate
python your_script.py

# Using Jupyter notebook
docker-compose -f docker-compose.dev.yml up jupyter
# Open http://localhost:8888 in browser
```

## File Structure

```
.
├── Dockerfile.dev              # Development environment Docker image
├── docker-compose.dev.yml      # Docker Compose configuration
├── install.sh                  # Automated installation script
├── requirements.txt            # Python dependencies
├── apt-packages.txt            # System package dependencies
├── ros-packages.txt            # ROS package dependencies
├── run-dev-container.sh        # Helper script (generated)
├── setup-local-dev.sh          # Local dev setup (generated)
├── DEV_ENVIRONMENT.md          # This documentation
└── ...                         # ORB-SLAM2 source code
```

## Dependency Management

### Python Dependencies
- Managed via `requirements.txt`
- Install with: `pip install -r requirements.txt`

### System Dependencies
- Listed in `apt-packages.txt`
- Automatically installed in Docker image

### ROS Dependencies
- Listed in `ros-packages.txt`
- Automatically installed in Docker image

## Troubleshooting

### GUI Applications Not Working
```bash
# Enable X11 forwarding
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY
```

### Docker Permission Issues
```bash
# Add user to docker group
sudo usermod -aG docker $USER
# Log out and log back in
```

### Build Failures
```bash
# Clean build directory
rm -rf build/*

# Install missing dependencies
sudo apt-get update
sudo apt-get install -f
```

### ROS Environment Issues
```bash
# Source ROS setup
source /opt/ros/noetic/setup.bash

# Check ROS environment
printenv | grep ROS
```

## Development Tips

### Using Zsh
The development environment includes Oh My Zsh for better terminal experience:
- Tab completion
- Git integration
- Syntax highlighting

### Debugging
```bash
# Use GDB for debugging
gdb ./build/Examples/RGB-D/rgbd_tum

# For Python debugging
python -m pdb your_script.py
```

### Performance
- Use volume mounts for persistent build artifacts
- Mount only necessary directories to improve performance
- Use `.dockerignore` to exclude unnecessary files

### VS Code Integration
You can use VS Code with the Remote-Containers extension:

1. Install "Remote - Containers" extension
2. Open project in VS Code
3. Use "Remote-Containers: Reopen in Container"

## Environment Variables

Key environment variables in the development container:

```bash
DISPLAY=${DISPLAY}                    # X11 display for GUI
ROS_MASTER_URI=http://localhost:11311 # ROS master location
ROS_HOSTNAME=localhost                # ROS hostname
QT_X11_NO_MITSHM=1                   # Qt X11 fix
```

## Updating the Environment

### Updating Python Dependencies
```bash
# Edit requirements.txt
# Rebuild image
docker-compose -f docker-compose.dev.yml build orb-slam2-dev
```

### Updating System Dependencies
```bash
# Edit apt-packages.txt
# Rebuild image
docker-compose -f docker-compose.dev.yml build orb-slam2-dev
```

### Updating Docker Image
```bash
# Pull latest base image
docker pull ubuntu:20.04

# Rebuild development image
docker-compose -f docker-compose.dev.yml build --no-cache orb-slam2-dev
```

## Contributing

When contributing to the project:

1. Use the standardized development environment
2. Test changes in both Docker and local environments
3. Update dependency files if adding new dependencies
4. Document any environment changes

## Support

For issues with the development environment:
1. Check this documentation
2. Look at the troubleshooting section
3. Check Docker and system logs
4. Create an issue with environment details