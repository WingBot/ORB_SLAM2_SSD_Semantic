# Development Environment Setup

This document provides quick setup instructions for the ORB_SLAM2_SSD_Semantic development environment.

## Quick Start

### Option 1: Full Installation (Recommended)
```bash
# Run the comprehensive installation script
./install.sh
```

### Option 2: Quick Start
```bash
# For Docker-based development
./quick-start.sh docker

# For local development
./quick-start.sh local
```

### Option 3: Manual Setup

#### Docker Environment
```bash
# Build the minimal image
docker build -f Dockerfile.minimal -t orb-slam2-ssd-semantic:minimal .

# Run development container
docker run -it --rm \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/workspace/orb_slam2_ssd_semantic \
    orb-slam2-ssd-semantic:minimal
```

#### Local Environment
```bash
# Install essential packages (Ubuntu/Debian)
sudo apt-get update
sudo apt-get install -y build-essential cmake git python3 python3-pip python3-venv

# Create Python environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Environment Components

### Core Tools
- **ROS Noetic** - Robot Operating System
- **OpenCV** - Computer Vision library
- **Python 3** with Qt5 - GUI development
- **CMake & Build tools** - Compilation
- **Git, Zsh, development utilities**

### Python Packages
- NumPy, SciPy, Matplotlib
- OpenCV Python bindings
- PyQt5 for GUI development
- Machine Learning libraries (PyTorch, TensorFlow)
- Point cloud processing (Open3D)

### Docker Services
- `orb-slam2-dev` - Main development container
- `jupyter` - Notebook server (port 8888)
- `roscore` - ROS master node
- `rviz` - 3D visualization

## Usage Examples

### Building ORB-SLAM2
```bash
# Inside container or local environment
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Running with Docker Compose
```bash
# Start development environment
docker-compose -f docker-compose.dev.yml up orb-slam2-dev

# Start Jupyter notebook
docker-compose -f docker-compose.dev.yml up jupyter

# Access at http://localhost:8888
```

### Using Makefile shortcuts
```bash
make help          # Show all available commands
make build         # Build Docker image
make run           # Run development container
make build-orb     # Build ORB-SLAM2 inside container
make test-env      # Test environment setup
```

## File Structure

```
├── Dockerfile.dev              # Full development environment
├── Dockerfile.minimal          # Minimal/quick setup
├── docker-compose.dev.yml      # Multi-service setup
├── install.sh                  # Full installation script
├── quick-start.sh              # Quick setup script
├── scripts/
│   └── container-setup.sh      # Container initialization
├── requirements.txt            # Python dependencies
├── apt-packages.txt            # System packages
├── ros-packages.txt            # ROS packages
├── Makefile                    # Build automation
└── DEV_ENVIRONMENT.md          # Detailed documentation
```

## Troubleshooting

### Docker Issues
```bash
# Permission denied
sudo usermod -aG docker $USER
# Then log out and back in

# X11 forwarding for GUI
xhost +local:docker
```

### Build Failures
```bash
# Clean build
rm -rf build/*
make clean

# Update dependencies
sudo apt-get update -f
```

### Network Issues
```bash
# Use alternative mirrors (inside container)
sed -i 's|archive.ubuntu.com|mirrors.aliyun.com|g' /etc/apt/sources.list
apt-get update
```

## Development Workflow

1. **Setup**: Run `./install.sh` or `./quick-start.sh docker`
2. **Code**: Edit source files in your preferred editor
3. **Build**: Use `make build-orb` or manual cmake
4. **Test**: Run examples and verify functionality
5. **Debug**: Use GDB or integrated debugging tools
6. **Visualize**: Use RViz for 3D data visualization

## Support

- See `DEV_ENVIRONMENT.md` for detailed documentation
- Check the troubleshooting section above
- Review Docker logs: `docker-compose logs`

## Next Steps

After setup, you can:
1. Build the ORB-SLAM2 project
2. Run examples with different datasets
3. Develop new features with semantic mapping
4. Use Jupyter notebooks for data analysis
5. Visualize results with RViz