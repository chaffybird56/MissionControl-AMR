# ROS2 AMR Operations Makefile
# Provides convenient commands for managing the AMR system

.PHONY: help up down logs clean build status restart shell bag monitor

# Default target
help: ## Show this help message
	@echo "ROS2 AMR Operations"
	@echo "=================="
	@echo ""
	@echo "Available commands:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "Examples:"
	@echo "  make up          # Start all services"
	@echo "  make logs        # View logs"
	@echo "  make shell       # Open ROS2 shell"
	@echo "  make clean       # Clean up everything"

# Core operations
up: ## Start all services
	@echo "Starting ROS2 AMR system..."
	docker-compose up -d
	@echo "Services started. Dashboard: http://localhost:3000"
	@echo "API Server: http://localhost:8000"

down: ## Stop all services
	@echo "Stopping ROS2 AMR system..."
	docker-compose down

restart: ## Restart all services
	@echo "Restarting ROS2 AMR system..."
	docker-compose restart

build: ## Build all Docker images
	@echo "Building Docker images..."
	docker-compose build --no-cache

# Logging and monitoring
logs: ## View logs from all services
	docker-compose logs -f

logs-ros2: ## View ROS2 core logs
	docker-compose logs -f ros2-core

logs-server: ## View server logs
	docker-compose logs -f server

logs-dashboard: ## View dashboard logs
	docker-compose logs -f dashboard

logs-webots: ## View Webots logs
	docker-compose logs -f webots

# Status and debugging
status: ## Show status of all services
	@echo "Service Status:"
	@echo "==============="
	docker-compose ps
	@echo ""
	@echo "Container Resource Usage:"
	@echo "========================="
	docker stats --no-stream --format "table {{.Container}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}\t{{.BlockIO}}"

shell: ## Open shell in ROS2 container
	docker-compose exec ros2-core bash

shell-server: ## Open shell in server container
	docker-compose exec server bash

shell-dashboard: ## Open shell in dashboard container
	docker-compose exec dashboard bash

# ROS2 specific commands
ros2-topics: ## List ROS2 topics
	docker-compose exec ros2-core ros2 topic list

ros2-nodes: ## List ROS2 nodes
	docker-compose exec ros2-core ros2 node list

ros2-services: ## List ROS2 services
	docker-compose exec ros2-core ros2 service list

ros2-actions: ## List ROS2 actions
	docker-compose exec ros2-core ros2 action list

ros2-info: ## Show ROS2 system information
	@echo "ROS2 System Information:"
	@echo "========================"
	@echo "Topics:"
	docker-compose exec ros2-core ros2 topic list
	@echo ""
	@echo "Nodes:"
	docker-compose exec ros2-core ros2 node list
	@echo ""
	@echo "Services:"
	docker-compose exec ros2-core ros2 service list

# Data management
bag: ## Start rosbag recording
	@echo "Starting rosbag recording..."
	docker-compose exec ros2-core ros2 bag record -o /bags/amr_mission_$(shell date +%Y%m%d_%H%M%S) /odom /scan /camera/image_raw /cmd_vel /tf /tf_static /mission/state /mission/progress

bag-list: ## List available rosbags
	@echo "Available rosbags:"
	docker-compose exec ros2-core ls -la /bags/

bag-play: ## Play latest rosbag (usage: make bag-play BAG=bag_name)
	@if [ -z "$(BAG)" ]; then \
		echo "Usage: make bag-play BAG=bag_name"; \
		echo "Available bags:"; \
		docker-compose exec ros2-core ls /bags/; \
	else \
		echo "Playing bag: $(BAG)"; \
		docker-compose exec ros2-core ros2 bag play /bags/$(BAG); \
	fi

# Monitoring
monitor: ## Start monitoring services (Prometheus + Grafana)
	@echo "Starting monitoring services..."
	docker-compose --profile monitoring up -d prometheus grafana
	@echo "Prometheus: http://localhost:9090"
	@echo "Grafana: http://localhost:3001 (admin/admin)"

monitor-stop: ## Stop monitoring services
	docker-compose --profile monitoring down

# Development
dev: ## Start development environment
	@echo "Starting development environment..."
	docker-compose -f compose.yml -f compose.dev.yml up -d

dev-stop: ## Stop development environment
	docker-compose -f compose.yml -f compose.dev.yml down

# Testing
test: ## Run system tests
	@echo "Running system tests..."
	@echo "Checking service health..."
	@curl -f http://localhost:8000/health || echo "API server not responding"
	@curl -f http://localhost:3000 || echo "Dashboard not responding"
	@echo "Testing ROS2 topics..."
	docker-compose exec ros2-core timeout 5 ros2 topic echo /odom --once || echo "No odometry data"

# Cleanup
clean: ## Clean up everything (containers, images, volumes)
	@echo "Cleaning up ROS2 AMR system..."
	docker-compose down -v --remove-orphans
	docker system prune -f
	@echo "Cleanup complete"

clean-bags: ## Clean up rosbag files
	@echo "Cleaning up rosbag files..."
	docker-compose exec ros2-core rm -rf /bags/*
	@echo "Rosbag cleanup complete"

clean-logs: ## Clean up log files
	@echo "Cleaning up log files..."
	docker-compose exec ros2-core rm -rf /logs/*
	@echo "Log cleanup complete"

# Utility commands
install: ## Install system dependencies
	@echo "Installing system dependencies..."
	@command -v docker >/dev/null 2>&1 || { echo "Docker is required but not installed. Aborting." >&2; exit 1; }
	@command -v docker-compose >/dev/null 2>&1 || { echo "Docker Compose is required but not installed. Aborting." >&2; exit 1; }
	@command -v node >/dev/null 2>&1 || { echo "Node.js is required but not installed. Aborting." >&2; exit 1; }
	@echo "All dependencies are installed"

check: ## Check system requirements
	@echo "Checking system requirements..."
	@echo "Docker: $$(docker --version 2>/dev/null || echo 'Not installed')"
	@echo "Docker Compose: $$(docker-compose --version 2>/dev/null || echo 'Not installed')"
	@echo "Node.js: $$(node --version 2>/dev/null || echo 'Not installed')"
	@echo "Python: $$(python3 --version 2>/dev/null || echo 'Not installed')"

# Webots specific
webots-start: ## Start Webots simulation
	@echo "Starting Webots simulation..."
	docker-compose up -d webots

webots-stop: ## Stop Webots simulation
	docker-compose stop webots

webots-shell: ## Open shell in Webots container
	docker-compose exec webots bash

# Quick start
demo: ## Start demo mission
	@echo "Starting demo mission..."
	@curl -X POST http://localhost:8000/api/mission/demo_mission_001/start
	@echo "Demo mission started"

demo-stop: ## Stop demo mission
	@echo "Stopping demo mission..."
	@curl -X POST http://localhost:8000/api/mission/stop
	@echo "Demo mission stopped"

# System information
info: ## Show system information
	@echo "ROS2 AMR System Information"
	@echo "==========================="
	@echo "Project: ROS2 AMR Showfloor"
	@echo "Version: 1.0.0"
	@echo "ROS2 Version: Humble"
	@echo "Simulator: Webots 2023b"
	@echo "Dashboard: Next.js"
	@echo "API: FastAPI"
	@echo ""
	@echo "Endpoints:"
	@echo "  Dashboard: http://localhost:3000"
	@echo "  API Server: http://localhost:8000"
	@echo "  Metrics: http://localhost:8080/metrics"
	@echo "  Prometheus: http://localhost:9090"
	@echo "  Grafana: http://localhost:3001"

