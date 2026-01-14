import can
import time

bus = can.Bus(channel='can0', interface='socketcan')

# Try pinging nodes 2, 3, 4
for node_id in [1, 2, 3, 4]:
    print(f"\nTrying Node {node_id}...")
    
    # Send SDO read request for statusword (0x6041)
    req_id = 0x600 + node_id
    resp_id = 0x580 + node_id
    
    msg = can.Message(
        arbitration_id=req_id,
        data=[0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0],
        is_extended_id=False
    )
    bus.send(msg)
    time.sleep(0.1)
    
    # Check for response
    response = bus.recv(timeout=0.5)
    if response and response.arbitration_id == resp_id:
        print(f"  ✓ Node {node_id} responded!")
        print(f"    Response: {response}")
    else:
        print(f"  ✗ No response from Node {node_id}")

bus.shutdown()