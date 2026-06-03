#!/bin/bash

# ROS2 AMR Demo Recording Script
# Records a demo of the AMR system in action

set -e

# Configuration
OUTPUT_DIR="../media"
RECORDING_DURATION=60  # seconds
FPS=30
RESOLUTION="1920x1080"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}ROS2 AMR Demo Recording Script${NC}"
echo "=================================="

# Check if ffmpeg is installed
if ! command -v ffmpeg &> /dev/null; then
    echo -e "${RED}Error: ffmpeg is not installed${NC}"
    echo "Please install ffmpeg to record demos"
    echo "  macOS: brew install ffmpeg"
    echo "  Ubuntu: sudo apt install ffmpeg"
    exit 1
fi

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Generate filename with timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT_FILE="$OUTPUT_DIR/ros2_amr_demo_$TIMESTAMP.mp4"

echo -e "${YELLOW}Recording configuration:${NC}"
echo "  Duration: ${RECORDING_DURATION}s"
echo "  Resolution: $RESOLUTION"
echo "  FPS: $FPS"
echo "  Output: $OUTPUT_FILE"
echo ""

# Check if system is running
echo -e "${YELLOW}Checking if ROS2 AMR system is running...${NC}"
if ! curl -f http://localhost:8000/health &> /dev/null; then
    echo -e "${RED}Error: ROS2 AMR API server is not running${NC}"
    echo "Please start the system first:"
    echo "  cd ops && make up"
    exit 1
fi

if ! curl -f http://localhost:3000 &> /dev/null; then
    echo -e "${RED}Error: Dashboard is not running${NC}"
    echo "Please start the system first:"
    echo "  cd ops && make up"
    exit 1
fi

echo -e "${GREEN}System is running!${NC}"

# Get screen recording settings
echo -e "${YELLOW}Preparing screen recording...${NC}"
echo "This will record your entire screen for $RECORDING_DURATION seconds"
echo "Please position your browser window with the dashboard visible"
echo ""
echo -e "${YELLOW}Starting recording in 5 seconds...${NC}"
echo "Press Ctrl+C to cancel"

# Countdown
for i in 5 4 3 2 1; do
    echo -n "$i... "
    sleep 1
done
echo ""

# Start recording
echo -e "${GREEN}Recording started!${NC}"
echo "Navigate to http://localhost:3000 and interact with the dashboard"
echo ""

# Record the screen
ffmpeg -f avfoundation -i "1:0" -t $RECORDING_DURATION -r $FPS -s $RESOLUTION -c:v libx264 -preset fast -crf 23 -c:a aac -b:a 128k "$OUTPUT_FILE" 2>/dev/null

# Check if recording was successful
if [ $? -eq 0 ]; then
    echo -e "${GREEN}Recording completed successfully!${NC}"
    echo "Output file: $OUTPUT_FILE"
    
    # Get file size
    FILE_SIZE=$(ls -lh "$OUTPUT_FILE" | awk '{print $5}')
    echo "File size: $FILE_SIZE"
    
    # Option to open the file
    echo ""
    read -p "Would you like to open the recorded video? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        if command -v open &> /dev/null; then
            open "$OUTPUT_FILE"
        elif command -v xdg-open &> /dev/null; then
            xdg-open "$OUTPUT_FILE"
        else
            echo "Please open $OUTPUT_FILE manually"
        fi
    fi
else
    echo -e "${RED}Recording failed!${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Demo recording complete!${NC}"
echo "You can find the video at: $OUTPUT_FILE"

