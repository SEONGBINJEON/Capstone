"""Reboot one follower motor to clear a latched Hardware Error (e.g. 0x20 overload).
    python3 sim_teleop/reboot_motor.py 13
The motor comes back with torque OFF; support the arm if that joint carries load."""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_config
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
mid = int(sys.argv[1]); cfg = load_config()
port = PortHandler(cfg['buses']['followers']); assert port.openPort(); port.setBaudRate(cfg['baud']); time.sleep(.1)
pk = PacketHandler(2.0)
def rd(a, n):
    d, r, e = pk.readTxRx(port, mid, a, n); assert r == COMM_SUCCESS, pk.getTxRxResult(r); return int.from_bytes(bytes(d), 'little')
print(f'before: torque={rd(64,1)} hw_error=0x{rd(70,1):02x} pos={rd(132,4)} temp={rd(146,1)}')
r, e = pk.reboot(port, mid); print('reboot:', pk.getTxRxResult(r)); time.sleep(1.0)
for _ in range(20):
    try:
        print(f'after:  torque={rd(64,1)} hw_error=0x{rd(70,1):02x} pos={rd(132,4)} temp={rd(146,1)}'); break
    except AssertionError:
        time.sleep(.2)
port.closePort()
