#!/bin/bash
# ============================================
# FTC Robot Wireless Deployment Script
# ============================================
# This script connects to the robot wirelessly
# and optionally deploys the latest build.
#
# Usage:
#   ./push-to-robot.sh          - Connect only
#   ./push-to-robot.sh deploy   - Connect and install APK
# ============================================

# Robot IP address (change if needed)
ROBOT_IP="192.168.43.1:5555"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}FTC Robot Wireless Connection${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if ADB is installed
if ! command -v adb &> /dev/null; then
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}✗ ERROR: ADB not found${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo -e "${YELLOW}ADB (Android Debug Bridge) is not installed or not in PATH.${NC}"
    echo ""
    echo -e "Please install Android Studio and add ADB to your PATH:"
    echo -e "  Mac/Linux: export PATH=\"\$PATH:\$HOME/Library/Android/sdk/platform-tools\""
    echo -e "  Or add to ~/.zshrc or ~/.bashrc"
    echo ""
    exit 1
fi

# Step 1: Kill existing ADB server
echo -e "${YELLOW}[1/4] Stopping ADB server...${NC}"
adb kill-server 2>/dev/null &
KILL_PID=$!
# Wait max 5 seconds for kill to complete
for i in {1..5}; do
    if ! ps -p $KILL_PID > /dev/null 2>&1; then
        break
    fi
    sleep 1
done
# Force kill if still running
kill -9 $KILL_PID 2>/dev/null || true
sleep 1

# Step 2: Start fresh ADB server
echo -e "${YELLOW}[2/4] Starting ADB server...${NC}"
adb start-server > /dev/null 2>&1 &
START_PID=$!
# Wait max 10 seconds for server to start
for i in {1..10}; do
    if ! ps -p $START_PID > /dev/null 2>&1; then
        break
    fi
    sleep 1
done
sleep 1

# Step 3: Connect to robot
echo -e "${YELLOW}[3/4] Connecting to robot at ${ROBOT_IP}...${NC}"

# Run adb connect with timeout (max 10 seconds)
CONNECT_OUTPUT=""
(adb connect $ROBOT_IP 2>&1) > /tmp/adb_connect_output.txt &
CONNECT_PID=$!

# Wait max 10 seconds for connection attempt
CONNECT_SUCCESS=0
for i in {1..10}; do
    if ! ps -p $CONNECT_PID > /dev/null 2>&1; then
        CONNECT_SUCCESS=1
        break
    fi
    sleep 1
done

# Force kill if still running
if [ $CONNECT_SUCCESS -eq 0 ]; then
    kill -9 $CONNECT_PID 2>/dev/null || true
    echo -e "${RED}Connection attempt timed out after 10 seconds${NC}"
fi

# Read the output
if [ -f /tmp/adb_connect_output.txt ]; then
    CONNECT_OUTPUT=$(cat /tmp/adb_connect_output.txt)
    echo "$CONNECT_OUTPUT"
    rm /tmp/adb_connect_output.txt
fi

# Give it a moment to settle
sleep 1

# Step 4: Verify connection
echo -e "${YELLOW}[4/4] Verifying connection...${NC}"
DEVICES_OUTPUT=$(adb devices)
echo "$DEVICES_OUTPUT"
echo ""

# Check if robot is actually connected
if echo "$CONNECT_OUTPUT" | grep -q "cannot connect\|failed to connect\|Connection refused"; then
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}✗ CONNECTION FAILED${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo -e "${YELLOW}Possible reasons:${NC}"
    echo -e "  1. ${RED}Not connected to Robot WiFi${NC}"
    echo -e "     → Connect to the Robot Control Hub WiFi network"
    echo -e "     → WiFi name usually starts with: FTC-XXXX or similar"
    echo ""
    echo -e "  2. ${RED}Wrong IP address${NC}"
    echo -e "     → Check robot IP in Robot Controller app"
    echo -e "     → Current IP in script: ${ROBOT_IP}"
    echo ""
    echo -e "  3. ${RED}Robot is not powered on${NC}"
    echo -e "     → Make sure Robot Control Hub is on"
    echo ""
    echo -e "  4. ${RED}Wireless ADB not enabled${NC}"
    echo -e "     → Connect via USB first, then enable wireless debugging"
    echo ""
    exit 1
elif ! echo "$DEVICES_OUTPUT" | grep -q "192.168.43.1:5555.*device$"; then
    echo -e "${YELLOW}========================================${NC}"
    echo -e "${YELLOW}⚠ WARNING: Connection may be unstable${NC}"
    echo -e "${YELLOW}========================================${NC}"
    echo ""
    echo -e "${YELLOW}The robot appears in the list but may not be fully connected.${NC}"
    echo -e "Try running the script again, or connect via USB if issues persist."
    echo ""
fi

# Check if we should deploy
if [ "$1" == "deploy" ] || [ "$1" == "install" ]; then
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Building and Installing APK${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Build the project
    echo -e "${YELLOW}Building APK...${NC}"
    ./gradlew assembleDebug
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Build successful!${NC}"
        echo ""
        
        # Install the APK
        echo -e "${YELLOW}Installing APK to robot...${NC}"
        ./gradlew installDebug
        
        if [ $? -eq 0 ]; then
            echo ""
            echo -e "${GREEN}========================================${NC}"
            echo -e "${GREEN}✓ SUCCESS!${NC}"
            echo -e "${GREEN}APK installed on robot successfully!${NC}"
            echo -e "${GREEN}========================================${NC}"
        else
            echo ""
            echo -e "${RED}========================================${NC}"
            echo -e "${RED}✗ FAILED${NC}"
            echo -e "${RED}Installation failed!${NC}"
            echo -e "${RED}========================================${NC}"
            exit 1
        fi
    else
        echo ""
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}✗ FAILED${NC}"
        echo -e "${RED}Build failed!${NC}"
        echo -e "${RED}========================================${NC}"
        exit 1
    fi
else
    # Just connection
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ Connection Complete!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "To also install APK, run:"
    echo -e "${BLUE}./push-to-robot.sh deploy${NC}"
    echo ""
fi

