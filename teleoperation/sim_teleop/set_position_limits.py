"""One-off EEPROM maintenance: set Min/Max Position Limit of one follower motor.
Requires torque OFF. Records the previous values in work/eeprom_limit_backup_<id>_<time>.json.
    python3 sim_teleop/set_position_limits.py 13 0 4095
"""
import json, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_config
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

mid, lo, hi = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
assert 0 <= lo < hi <= 4095
cfg = load_config()
port = PortHandler(cfg['buses']['followers']); assert port.openPort(); port.setBaudRate(cfg['baud']); time.sleep(.1)
pk = PacketHandler(2.0)
def rd(addr, n):
    d, r, e = pk.readTxRx(port, mid, addr, n); assert r == COMM_SUCCESS and not (e & 0x7f), (r, e); return int.from_bytes(bytes(d), 'little')
def wr(addr, val):
    r, e = pk.write4ByteTxRx(port, mid, addr, val); assert r == COMM_SUCCESS and not (e & 0x7f), (pk.getTxRxResult(r), pk.getRxPacketError(e))
model = pk.ping(port, mid)[0]
before = dict(id=mid, model=model, torque=rd(64, 1), min=rd(52, 4), max=rd(48, 4), position=rd(132, 4), unix_time=time.time())
print('before:', before)
if before['torque']: sys.exit('토크가 켜져 있어 EEPROM을 쓸 수 없습니다.')
os.makedirs('work', exist_ok=True)
path = f"work/eeprom_limit_backup_{mid}_{int(time.time())}.json"
json.dump(before, open(path, 'w'), indent=1); print('backup:', path)
wr(52, lo); time.sleep(.05); wr(48, hi); time.sleep(.05)
after = dict(min=rd(52, 4), max=rd(48, 4)); print('after:', after)
assert after == dict(min=lo, max=hi), '읽기 검증 실패'
port.closePort(); print('완료')
