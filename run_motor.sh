#!/bin/bash
echo "=== SIMPLE 10-SECOND MOTOR CONTROL ==="
# 1. Stop fb-robot service (releases CAN bus)
systemctl stop fb-robot
# 2. Reset CAN
ip link set can1 down
sleep 0.5
ip link set can1 up type can bitrate 1000000
sleep 1
# 3. Increase queue size to prevent overflow
echo 1000 > /proc/sys/fs/mqueue/msg_max 2>/dev/null
# 4. Clean existing queues
pkill -9 fb-axis-move 2>/dev/null
ipcrm -a 2>/dev/null || true
# 5. Run motor for 10 seconds
echo "Running left motor at 0.2 rad/s for 10 seconds..."
echo -e "2\n" | fb-axis-move left 0.2 10
echo "=== DONE ==="
