#!/bin/bash

# ROS2 AMR Topic Verification Script
# Verifies that all required ROS2 topics and services are available

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}ROS2 AMR Topic Verification Script${NC}"
echo "======================================"

# Required topics
REQUIRED_TOPICS=(
    "/odom"
    "/scan"
    "/camera/image_raw"
    "/cmd_vel"
    "/tf"
    "/tf_static"
    "/mission/state"
    "/mission/progress"
    "/mission/metrics"
    "/landmarks"
    "/detected_goal"
)

# Required services
REQUIRED_SERVICES=(
    "/mission/start"
    "/mission/stop"
    "/mission/pause"
    "/mission/resume"
)

# Required actions
REQUIRED_ACTIONS=(
    "/navigate_to_pose"
)

# Check if ROS2 is running
echo -e "${YELLOW}Checking ROS2 system...${NC}"
if ! docker ps | grep -q "ros2-amr-core"; then
    echo -e "${RED}Error: ROS2 AMR core container is not running${NC}"
    echo "Please start the system first:"
    echo "  cd ops && make up"
    exit 1
fi

echo -e "${GREEN}ROS2 AMR core container is running${NC}"

# Function to check if a topic exists and is publishing
check_topic() {
    local topic=$1
    echo -n "Checking topic $topic... "
    
    # Check if topic exists
    if docker exec ros2-amr-core ros2 topic list | grep -q "^$topic$"; then
        # Check if topic has publishers
        PUBLISHERS=$(docker exec ros2-amr-core ros2 topic info $topic --verbose 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
        if [ "$PUBLISHERS" -gt 0 ]; then
            echo -e "${GREEN}✓ (${PUBLISHERS} publisher(s))${NC}"
            return 0
        else
            echo -e "${YELLOW}⚠ (no publishers)${NC}"
            return 1
        fi
    else
        echo -e "${RED}✗ (not found)${NC}"
        return 1
    fi
}

# Function to check if a service exists
check_service() {
    local service=$1
    echo -n "Checking service $service... "
    
    if docker exec ros2-amr-core ros2 service list | grep -q "^$service$"; then
        echo -e "${GREEN}✓${NC}"
        return 0
    else
        echo -e "${RED}✗${NC}"
        return 1
    fi
}

# Function to check if an action exists
check_action() {
    local action=$1
    echo -n "Checking action $action... "
    
    if docker exec ros2-amr-core ros2 action list | grep -q "^$action$"; then
        echo -e "${GREEN}✓${NC}"
        return 0
    else
        echo -e "${RED}✗${NC}"
        return 1
    fi
}

# Check topics
echo ""
echo -e "${BLUE}Checking Required Topics:${NC}"
echo "=========================="
TOPIC_FAILURES=0
for topic in "${REQUIRED_TOPICS[@]}"; do
    if ! check_topic "$topic"; then
        TOPIC_FAILURES=$((TOPIC_FAILURES + 1))
    fi
done

# Check services
echo ""
echo -e "${BLUE}Checking Required Services:${NC}"
echo "============================="
SERVICE_FAILURES=0
for service in "${REQUIRED_SERVICES[@]}"; do
    if ! check_service "$service"; then
        SERVICE_FAILURES=$((SERVICE_FAILURES + 1))
    fi
done

# Check actions
echo ""
echo -e "${BLUE}Checking Required Actions:${NC}"
echo "============================="
ACTION_FAILURES=0
for action in "${REQUIRED_ACTIONS[@]}"; do
    if ! check_action "$action"; then
        ACTION_FAILURES=$((ACTION_FAILURES + 1))
    fi
done

# Summary
echo ""
echo -e "${BLUE}Verification Summary:${NC}"
echo "===================="

TOTAL_FAILURES=$((TOPIC_FAILURES + SERVICE_FAILURES + ACTION_FAILURES))

if [ $TOTAL_FAILURES -eq 0 ]; then
    echo -e "${GREEN}✓ All required topics, services, and actions are available!${NC}"
    echo -e "${GREEN}The ROS2 AMR system is properly configured and running.${NC}"
    exit 0
else
    echo -e "${RED}✗ $TOTAL_FAILURES issues found:${NC}"
    echo "  - $TOPIC_FAILURES topic(s) missing or not publishing"
    echo "  - $SERVICE_FAILURES service(s) missing"
    echo "  - $ACTION_FAILURES action(s) missing"
    echo ""
    echo -e "${YELLOW}Troubleshooting tips:${NC}"
    echo "1. Make sure all ROS2 nodes are running: make ros2-nodes"
    echo "2. Check ROS2 logs: make logs-ros2"
    echo "3. Restart the system: make down && make up"
    echo "4. Check Webots simulation is running: make logs-webots"
    exit 1
fi

# Additional diagnostic information
echo ""
echo -e "${BLUE}Additional System Information:${NC}"
echo "==============================="

echo "Active ROS2 nodes:"
docker exec ros2-amr-core ros2 node list | sed 's/^/  /'

echo ""
echo "ROS2 parameter server:"
docker exec ros2-amr-core ros2 param list 2>/dev/null | head -10 | sed 's/^/  /' || echo "  No parameters found"

echo ""
echo "System uptime:"
docker exec ros2-amr-core uptime | sed 's/^/  /'

