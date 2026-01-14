import can
from datetime import datetime
from pathlib import Path

def dump_can_bus(channel="can0"):
    script_dir = Path(__file__).resolve().parent
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = script_dir / f"{channel}_{ts}.log"

    print(f"--- Starting CAN Dump on socketcan/{channel} ---")
    print("Press CTRL+C to stop...")
    print(f"Logging to: {log_path}")

    bus = can.interface.Bus(channel=channel, interface="socketcan")

    try:
        with open(log_path, "w", buffering=1) as f:
            while True:
                msg = bus.recv(timeout=1.0)
                if msg is None:
                    continue

                can_id = f"{msg.arbitration_id:08X}" if msg.is_extended_id else f"{msg.arbitration_id:03X}"
                data = msg.data.hex(" ").upper()
                line = f"{msg.timestamp:.6f} {channel} {can_id} [{msg.dlc}] {data}"
                print(line)
                f.write(line + "\n")

    except KeyboardInterrupt:
        print("\n--- Stopped CAN Dump ---")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    dump_can_bus("can0")
