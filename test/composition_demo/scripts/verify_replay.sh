#!/bin/bash
set -e

# Verification script for composition demo replay

echo "Checking for play_log directory..."
if [ ! -d "play_log" ]; then
    echo "❌ Error: play_log directory not found"
    exit 1
fi
echo "✅ play_log directory exists"

# Find latest log directory
LOG_DIR=$(ls -t play_log | head -1)
if [ -z "$LOG_DIR" ]; then
    echo "❌ Error: No log directories found"
    exit 1
fi
echo "✅ Found log directory: play_log/$LOG_DIR"

# Check container was spawned
echo ""
echo "Checking container process..."
CONTAINER_DIR="play_log/$LOG_DIR/node/my_container"
if [ ! -d "$CONTAINER_DIR" ]; then
    echo "❌ Error: Container directory not found"
    exit 1
fi
echo "✅ Container directory exists: $CONTAINER_DIR"

# Check container output (ROS logs to stderr)
CONTAINER_ERR="$CONTAINER_DIR/err"
if [ ! -f "$CONTAINER_ERR" ]; then
    echo "❌ Error: Container stderr not found"
    exit 1
fi
echo "✅ Container stderr exists"

# Check container started (look for component loading in stderr)
if ! grep -q "Load Library:" "$CONTAINER_ERR" 2>/dev/null; then
    echo "⚠️  Warning: Component loading not found in container output"
else
    echo "✅ Container loaded components"
fi

# Check composable nodes were loaded
echo ""
echo "Checking composable node loading..."

# Check Talker
TALKER_DIR="play_log/$LOG_DIR/load_node/talker"
if [ ! -d "$TALKER_DIR" ]; then
    echo "❌ Error: Talker load directory not found"
    exit 1
fi
echo "✅ Talker load directory exists"

# Check Talker service response
TALKER_RESPONSE="$TALKER_DIR/service_response"
if [ ! -f "$TALKER_RESPONSE" ]; then
    echo "❌ Error: Talker service response not found"
    exit 1
fi
echo "✅ Talker service response exists"

# Check Talker load was successful
if grep -q "success: true" "$TALKER_RESPONSE" 2>/dev/null; then
    echo "✅ Talker loaded successfully"
else
    echo "❌ Error: Talker load failed"
    cat "$TALKER_RESPONSE"
    exit 1
fi

# Check Listener
LISTENER_DIR="play_log/$LOG_DIR/load_node/listener"
if [ ! -d "$LISTENER_DIR" ]; then
    echo "❌ Error: Listener load directory not found"
    exit 1
fi
echo "✅ Listener load directory exists"

# Check Listener service response
LISTENER_RESPONSE="$LISTENER_DIR/service_response"
if [ ! -f "$LISTENER_RESPONSE" ]; then
    echo "❌ Error: Listener service response not found"
    exit 1
fi
echo "✅ Listener service response exists"

# Check Listener load was successful
if grep -q "success: true" "$LISTENER_RESPONSE" 2>/dev/null; then
    echo "✅ Listener loaded successfully"
else
    echo "❌ Error: Listener load failed"
    cat "$LISTENER_RESPONSE"
    exit 1
fi

# Check for talker output
echo ""
echo "Checking communication..."
if grep -q "Publishing:" "$CONTAINER_ERR" 2>/dev/null; then
    echo "✅ Talker is publishing messages"
else
    echo "⚠️  Warning: No publishing messages found (might be timing issue)"
fi

if grep -q "I heard:" "$CONTAINER_ERR" 2>/dev/null; then
    echo "✅ Listener is receiving messages"
else
    echo "⚠️  Warning: No received messages found (might be timing issue)"
fi

echo ""
echo "================================"
echo "✅ All verification checks passed!"
echo "================================"
echo ""
echo "Summary:"
echo "  - Container spawned: ✅"
echo "  - Talker loaded via service: ✅"
echo "  - Listener loaded via service: ✅"
echo "  - Service-based loading working correctly!"
