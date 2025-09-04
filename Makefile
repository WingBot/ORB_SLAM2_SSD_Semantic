# Makefile for ORB_SLAM2_SSD_Semantic Development Environment

.PHONY: help install build run clean test docs

# Default target
help: ## Show this help message
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Targets:'
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

install: ## Install development environment
	@echo "Installing development environment..."
	./install.sh

build: ## Build Docker development image
	@echo "Building development Docker image..."
	docker build -f Dockerfile.dev -t orb-slam2-ssd-semantic:dev .

build-no-cache: ## Build Docker image without cache
	@echo "Building development Docker image (no cache)..."
	docker build --no-cache -f Dockerfile.dev -t orb-slam2-ssd-semantic:dev .

run: ## Run development container
	@echo "Starting development container..."
	./run-dev-container.sh

run-compose: ## Run development environment with docker-compose
	@echo "Starting development environment with docker-compose..."
	docker-compose -f docker-compose.dev.yml up orb-slam2-dev

run-jupyter: ## Start Jupyter notebook server
	@echo "Starting Jupyter notebook server..."
	docker-compose -f docker-compose.dev.yml up jupyter

run-rviz: ## Start RViz visualization
	@echo "Starting RViz..."
	xhost +local:docker
	docker-compose -f docker-compose.dev.yml up rviz

shell: ## Get shell access to running development container
	@echo "Accessing development container shell..."
	docker-compose -f docker-compose.dev.yml exec orb-slam2-dev zsh

build-orb: ## Build ORB-SLAM2 project inside container
	@echo "Building ORB-SLAM2 project..."
	docker-compose -f docker-compose.dev.yml exec orb-slam2-dev bash -c "\
		mkdir -p build && cd build && \
		cmake .. && \
		make -j\$$(nproc)"

test-env: ## Test development environment
	@echo "Testing development environment..."
	docker run --rm orb-slam2-ssd-semantic:dev bash -c "\
		python3 --version && \
		echo 'Python OK' && \
		which ros && echo 'ROS OK' || echo 'ROS not found' && \
		pkg-config --modversion opencv4 && echo 'OpenCV OK' || echo 'OpenCV not found'"

clean: ## Clean up containers and images
	@echo "Cleaning up Docker containers and images..."
	docker-compose -f docker-compose.dev.yml down -v
	docker container prune -f
	docker image prune -f

clean-all: ## Clean up everything including volumes
	@echo "Cleaning up everything..."
	docker-compose -f docker-compose.dev.yml down -v --rmi all
	docker system prune -af

setup-local: ## Setup local development environment
	@echo "Setting up local development environment..."
	./setup-local-dev.sh

docs: ## Generate documentation
	@echo "Opening development documentation..."
	@if command -v xdg-open > /dev/null; then \
		xdg-open DEV_ENVIRONMENT.md; \
	elif command -v open > /dev/null; then \
		open DEV_ENVIRONMENT.md; \
	else \
		echo "Please open DEV_ENVIRONMENT.md manually"; \
	fi

update-deps: ## Update Python dependencies
	@echo "Updating Python dependencies..."
	docker-compose -f docker-compose.dev.yml exec orb-slam2-dev pip install --upgrade -r requirements.txt

logs: ## Show container logs
	@echo "Showing container logs..."
	docker-compose -f docker-compose.dev.yml logs -f

status: ## Show status of containers
	@echo "Container status:"
	docker-compose -f docker-compose.dev.yml ps

# Development workflow targets
dev-start: build run-compose ## Start full development environment
	@echo "Development environment started!"

dev-stop: ## Stop development environment
	@echo "Stopping development environment..."
	docker-compose -f docker-compose.dev.yml down

dev-restart: dev-stop dev-start ## Restart development environment
	@echo "Development environment restarted!"

# Quick access targets
qt-test: ## Test Qt GUI in container
	@echo "Testing Qt GUI..."
	xhost +local:docker
	docker run --rm -e DISPLAY=$$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
		orb-slam2-ssd-semantic:dev python3 -c "from PyQt5.QtWidgets import QApplication, QLabel; app = QApplication([]); label = QLabel('Qt Test OK'); label.show(); print('Qt GUI test completed')"

ros-test: ## Test ROS functionality
	@echo "Testing ROS..."
	docker-compose -f docker-compose.dev.yml exec orb-slam2-dev bash -c "\
		source /opt/ros/noetic/setup.bash && \
		rosversion -d && \
		echo 'ROS test completed'"