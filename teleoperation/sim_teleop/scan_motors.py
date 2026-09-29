"""Read-only scan: ping IDs, read model/mode/torque/position. Never writes."""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vendor'))
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORTS = {
 'OpenCR (follower)': '/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00',
 'OpenRB-150 (leader)': '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_4CF730625157375037202020FF111C0D-if00',
}
MODELS={1030:"XM430-W210",1060:"XL430-W250",1070:"XC430-W150",1200:"XL330-M288",1190:"XL330-M077",1020:"XM430-W350"}
def s32(v): return v-(1<<32) if v&(1<<31) else v
pk=PacketHandler(2.0)
for name,path in PORTS.items():
    for baud in (1000000,57600):
        port=PortHandler(path); port.setBaudRate(baud) if port.openPort() else None
        if not port.is_open: print(name,'open fail'); break
        port.setBaudRate(baud); time.sleep(.1)
        found=[]
        for i in range(0,41):
            model,res,err=pk.ping(port,i)
            if res==COMM_SUCCESS: found.append((i,model))
        print(f"== {name} @ {baud}: found {[(i,m) for i,m in found]}")
        for i,model in found:
            data,res,err=pk.readTxRx(port,i,0,64)
            def num(o,n): return int.from_bytes(bytes(data[o:o+n]),'little')
            ram,res2,err2=pk.readTxRx(port,i,64,84)
            def r(o,n): return int.from_bytes(bytes(ram[o-64:o-64+n]),'little')
            print(f"  ID {i:2d} {MODELS.get(model,model):12s} fw={num(6,1)} drive={num(10,1)} opmode={num(11,1)} homing={s32(num(20,4))} "
                  f"posmin={num(52,4)} posmax={num(48,4)} torque={r(64,1)} hwerr={r(70,1)} wd={r(98,1)} pos={s32(r(132,4))} volt={r(144,2)/10} temp={r(146,1)}")
        port.closePort()
        if found: break
