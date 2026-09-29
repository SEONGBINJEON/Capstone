"""Only this module accesses physical motors. SDK packets use Protocol 2.0."""
import fcntl
import os
import math
import termios
import time
from collections import defaultdict
from dataclasses import asdict, dataclass

import serial
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
from .config import MODELS, SafetyError, motors, TICKS_PER_DEG


def profile_values(drive_mode, target, present, speed_deg_s, period_s):
    if not all(math.isfinite(v) for v in (target, present, speed_deg_s, period_s)) or speed_deg_s <= 0 or period_s <= 0:
        raise SafetyError("프로파일 입력값이 잘못됐습니다.")
    if drive_mode & 8:
        raise SafetyError("목표 갱신 시 자동 토크 ON 모드는 지원하지 않습니다.")
    if drive_mode & 4:
        # Time-based profiles use milliseconds, not the velocity-mode RPM units.
        distance_deg = abs(target-present)/TICKS_PER_DEG
        duration = max(2, math.ceil(period_s*2000), math.ceil(2000*distance_deg/speed_deg_s))
        if duration > 32767:
            raise SafetyError("필요한 시간 기반 프로파일이 범위를 초과합니다.")
        return max(1, duration//4), duration
    return 5, max(1, math.floor(speed_deg_s/6/.229))


def signed32(value):
    return value - (1 << 32) if value & (1 << 31) else value


def position_mode_coordinate(raw, metadata):
    """Coordinate the mode-3 follower will use when torque is enabled.

    Torque-off feedback is multi-turn, but enabling mode 3 resets it to a
    single-turn absolute position plus Homing Offset. Preserve raw separately.
    This does not allow a goal trajectory to cross the 0/4095 boundary.
    """
    if metadata['operating_mode'] != 3:
        return raw
    offset = metadata['homing_offset']
    if not -1024 <= offset <= 1024:
        raise SafetyError('위치 제어 모드의 Homing Offset이 유효 범위 밖입니다.')
    return (raw-offset) % 4096 + offset


class BoundedPort(PortHandler):
    def getCurrentTime(self):
        return time.monotonic() * 1000

    def setupPort(self, cflag_baud):
        if self.is_open:
            self.closePort()
        self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0,
                                 write_timeout=.05, exclusive=True)
        try:
            fcntl.ioctl(self.ser.fileno(), termios.TIOCEXCL)
        except BaseException:
            self.ser.close()
            raise
        self.is_open = True
        self.ser.reset_input_buffer()
        self.tx_time_per_byte = 10000 / self.baudrate
        return True

    def closePort(self):
        if self.is_open:
            try:
                fcntl.ioctl(self.ser.fileno(), termios.TIOCNXCL)
            finally:
                self.ser.close()
                self.is_open = False


@dataclass
class State:
    id: int
    position: int
    voltage: float
    temperature: int
    torque: int
    hardware_error: int
    watchdog: int
    sampled_at: float
    raw_position: int | None = None
    goal_position: int | None = None
    position_trajectory: int | None = None
    present_velocity: int | None = None
    present_pwm: int | None = None
    moving_status: int | None = None
    profile_acceleration: int | None = None
    profile_velocity: int | None = None
    position_p_gain: int | None = None
    position_i_gain: int | None = None
    position_d_gain: int | None = None
    goal_pwm: int | None = None
    present_load_or_current_raw: int | None = None


class Rig:
    def __init__(self, config, pair=None, allow_motion=False):
        pair = pair or config.get('active_pair', 'A')
        self.config = config
        self.specs = {m["id"]: m for m in motors(config, pair)}
        self.follower_ids = {j['follower']['id'] for name,js in config['pairs'].items()
                             if pair in ('both',name) for j in js if j.get('enabled',True)}
        self.write_ids = set()
        if allow_motion:
            for name, js in config["pairs"].items():
                if pair in ("both", name):
                    self.write_ids.update(j["follower"]["id"] for j in js if j.get('enabled',True))
        self.ports = {}
        self.groups = {}
        self.metadata = {}
        self.extended_turn_offsets = {}
        self.packet = PacketHandler(2.0)

    def __enter__(self):
        try:
            assigned = defaultdict(list)
            for m in self.specs.values():
                assigned[m["bus"]].append(m["id"])
            resolved = set()
            for name, ids in assigned.items():
                path = self.config["buses"][name]
                actual = os.path.realpath(path)
                if actual in resolved:
                    raise SafetyError("두 보드 설정이 같은 USB 포트를 가리킵니다.")
                resolved.add(actual)
                port = BoundedPort(path)
                port.baudrate = self.config["baud"]
                self.ports[name] = port
                if not port.openPort():
                    raise SafetyError(f"포트를 열지 못했습니다: {path}")
                time.sleep(.15)
                group = GroupSyncRead(port, self.packet, 64, 83)
                for i in sorted(ids):
                    group.addParam(i)
                self.groups[name] = group
            for i, spec in self.specs.items():
                port = self.ports[spec["bus"]]
                model, result, error = self.packet.ping(port, i)
                self.check(result, error, f"ID {i} Ping")
                if model != spec["model"]:
                    raise SafetyError(f"ID {i}: 모델 불일치. 예상 {spec['model']}, 실제 {model}. 포트/배선을 확인하세요.")
                data, result, error = self.packet.readTxRx(port, i, 0, 64)
                self.check(result, error, f"ID {i} 설정 읽기")
                def num(offset, size):
                    return int.from_bytes(bytes(data[offset:offset+size]), 'little')
                self.metadata[i] = dict(model=model, model_name=MODELS[model], firmware=num(6, 1),
                                        drive_mode=num(10, 1), operating_mode=num(11, 1),
                                        homing_offset=signed32(num(20, 4)),
                                        pwm_limit=num(36, 2),
                                        position_max=num(48, 4), position_min=num(52, 4),
                                        voltage_min=num(34, 2)/10, voltage_max=num(32, 2)/10)
            return self
        except BaseException:
            self.close()
            raise

    def check(self, result, error, operation):
        if result != COMM_SUCCESS:
            raise SafetyError(f"{operation}: {self.packet.getTxRxResult(result)}")
        if error:
            raise SafetyError(f"{operation}: {self.packet.getRxPacketError(error)}")

    def sample(self):
        states = {}
        for name, group in self.groups.items():
            self.check(group.txRxPacket(), 0, f"{name} 위치/상태 읽기")
            stamp = time.monotonic()
            for i in group.data_dict:
                if not group.isAvailable(i, 64, 83):
                    raise SafetyError(f"ID {i}: 불완전한 상태 응답")
                get = lambda address, size: group.getData(i, address, size)
                raw = signed32(get(132, 4))
                torque = get(64, 1)
                position = position_mode_coordinate(raw, self.metadata[i]) if i in self.follower_ids and not torque else raw
                goal = signed32(get(116,4))
                if i in self.follower_ids and self.metadata[i]['operating_mode']==4:
                    window=self.extended_window(i)
                    if i not in self.extended_turn_offsets:
                        raw_reference=window.get('raw_reference',window['reference'])
                        logical=window['reference']+(raw-raw_reference+2048)%4096-2048
                        if not window['min'] <= logical <= window['max']:
                            raise SafetyError(f'ID {i}: 시작 위치가 확장 위치 제어의 소프트웨어 범위 밖입니다.')
                        self.extended_turn_offsets[i]=raw-logical
                    position=raw-self.extended_turn_offsets[i]
                    goal-=self.extended_turn_offsets[i]
                pwm = get(124,2)
                load_or_current = get(126,2)
                states[i] = State(i, position, get(144, 2)/10, get(146, 1),
                                  torque, get(70, 1), get(98, 1), stamp, raw, goal,
                                  position_trajectory=signed32(get(140,4)),present_velocity=signed32(get(128,4)),
                                  present_pwm=pwm-65536 if pwm & 32768 else pwm,moving_status=get(123,1),
                                  profile_acceleration=get(108,4),profile_velocity=get(112,4),
                                  position_p_gain=get(84,2),position_i_gain=get(82,2),position_d_gain=get(80,2),
                                  goal_pwm=get(100,2),
                                  present_load_or_current_raw=load_or_current-65536 if load_or_current & 32768 else load_or_current)
        if set(states) != set(self.specs):
            raise SafetyError("모터 응답이 누락됐습니다.")
        return states

    def signature(self, i):
        return {key: self.metadata[i][key] for key in
                ("model", "drive_mode", "operating_mode", "homing_offset")}

    def read_register(self, i, address, size):
        if i not in self.specs or size not in (1,2,4):
            raise SafetyError('유효하지 않은 모터 설정 읽기')
        data,result,error = self.packet.readTxRx(self.ports[self.specs[i]['bus']],i,address,size)
        self.check(result,error,f'ID {i} 주소 {address} 읽기')
        return int.from_bytes(bytes(data),'little')

    def extended_window(self,i):
        w=self.specs[i].get('extended_position')
        if not isinstance(w,dict) or not all(isinstance(w.get(k),int) and not isinstance(w[k],bool) for k in ('min','max','reference')):
            raise SafetyError(f'ID {i}: 확장 위치 제어의 명시적 소프트웨어 범위가 없습니다.')
        if not 0 <= w['min'] < w['reference'] < w['max'] <= 4095 or max(w['reference']-w['min'],w['max']-w['reference'])>=2048:
            raise SafetyError(f'ID {i}: 확장 위치 제어 범위가 유효하지 않습니다.')
        return w

    def position_limits(self,i):
        if self.metadata[i]['operating_mode']==4:
            w=self.extended_window(i)
            return w['min'],w['max']
        return max(0,self.metadata[i]['position_min']),min(4095,self.metadata[i]['position_max'])

    def raw_goal(self,i,logical):
        lo,hi=self.position_limits(i)
        if not lo <= logical <= hi:
            raise SafetyError(f'ID {i}: 목표가 허용된 위치 범위 밖입니다.')
        if self.metadata[i]['operating_mode']==4:
            if i not in self.extended_turn_offsets:
                raise SafetyError('확장 위치 기준을 읽기 전에 목표를 보낼 수 없습니다.')
            raw=logical+self.extended_turn_offsets[i]
            if not -1048575 <= raw <= 1048575:
                raise SafetyError('확장 위치 목표의 하드웨어 범위 초과')
            return raw
        return logical

    def set_extended_mode(self,i):
        if i not in self.write_ids or self.read_register(i,64,1)!=0 or self.read_register(i,11,1)!=3:
            raise SafetyError('확장 위치 모드 전환은 선택된 팔로워의 토크 OFF / 모드3 상태에서만 가능합니다.')
        self.extended_window(i)
        result,error=self.packet.writeTxRx(self.ports[self.specs[i]['bus']],i,11,1,[4])
        self.check(result,error,f'ID {i} 확장 위치 모드 저장')
        actual=self.read_register(i,11,1);self.metadata[i]['operating_mode']=actual
        self.extended_turn_offsets.pop(i,None)
        if actual!=4:raise SafetyError('확장 위치 모드 저장 확인 실패')

    def set_homing_offset(self, i, offset):
        """Explicit maintenance only. Ordinary write() still rejects every EEPROM address."""
        if i not in self.write_ids or not isinstance(offset,int) or isinstance(offset,bool) or not -1024 <= offset <= 1024:
            raise SafetyError('원점 변경 대상 또는 값이 허용 범위 밖입니다.')
        if self.read_register(i,64,1) != 0 or self.read_register(i,11,1) != 3:
            raise SafetyError('원점 변경은 토크 OFF / 위치 모드에서만 가능합니다.')
        result,error = self.packet.writeTxRx(self.ports[self.specs[i]['bus']],i,20,4,
                                             list(offset.to_bytes(4,'little',signed=True)))
        self.check(result,error,f'ID {i} Homing Offset 저장')
        actual = signed32(self.read_register(i,20,4))
        self.metadata[i]['homing_offset'] = actual
        if actual != offset:
            raise SafetyError(f'ID {i}: 저장된 원점이 요청값과 다릅니다. 토크를 켜지 않습니다.')

    def write(self, i, address, value, size):
        if i not in self.write_ids:
            raise SafetyError(f"ID {i}: 현재 모드에서 쓰기가 금지되어 있습니다.")
        # Restrict all writes to reviewed RAM operations. No EEPROM writes.
        if (address, size) not in ((64, 1), (98, 1), (108, 4), (112, 4), (116, 4)):
            raise SafetyError("허용되지 않은 모터 쓰기 주소")
        port = self.ports[self.specs[i]["bus"]]
        if address==116:value=self.raw_goal(i,value)
        data = list(int(value).to_bytes(size, 'little', signed=address==116))
        result, error = self.packet.writeTxRx(port, i, address, size, data)
        self.check(result, error, f"ID {i} 주소 {address} 쓰기")

    def tune_a_shoulder(self, address, value):
        """One supervised XM430 shoulder experiment; ordinary writes stay closed."""
        if (12 not in self.write_ids or self.signature(12) !=
                dict(model=1030, drive_mode=0, operating_mode=3, homing_offset=0)):
            raise SafetyError('A 어깨 위치 제어 모터만 조정할 수 있습니다.')
        if (address, value) not in ((100,250), (84,1200), (84,800)):
            raise SafetyError('검토한 어깨 출력 제한/게인 이외의 조정입니다.')
        if self.read_register(12,64,1)!=1 or self.read_register(12,98,1)!=15:
            raise SafetyError('어깨 토크와 통신 감시 상태를 확인하세요.')
        if address==84 and self.read_register(12,100,2)!=250:
            raise SafetyError('게인 변경 전에 출력 제한250 확인이 필요합니다.')
        data=list(value.to_bytes(2,'little'))
        result,error=self.packet.writeTxRx(self.ports[self.specs[12]['bus']],12,address,2,data)
        self.check(result,error,f'ID12 제한된 위치 제어 조정 주소{address}')
        if self.read_register(12,address,2)!=value:
            raise SafetyError('어깨 제어 조정 읽기 검증 실패')

    def restore_a_shoulder_gain(self):
        """Restore the already verified RAM gain after a power cycle, before arming."""
        if (12 not in self.write_ids or self.signature(12)!=
                dict(model=1030,drive_mode=0,operating_mode=3,homing_offset=0)):
            raise SafetyError('저장된 A 어깨 설정과 모터가 다릅니다.')
        gain=self.read_register(12,84,2)
        if gain==1200:return
        expected={64:0,98:0,84:800,82:0,80:0,100:885}
        if any(self.read_register(12,a,1 if a in (64,98) else 2)!=v for a,v in expected.items()):
            raise SafetyError('A 어깨 RAM 복원은 기본값의 토크 OFF 상태에서만 가능합니다.')
        result,error=self.packet.writeTxRx(self.ports[self.specs[12]['bus']],12,84,2,list((1200).to_bytes(2,'little')))
        self.check(result,error,'A 어깨 저장된 P Gain 복원')
        if self.read_register(12,84,2)!=1200:raise SafetyError('A 어깨 P Gain 복원 확인 실패')

    def restore_a_shoulder_output(self, previous_goal_pwm):
        """Restore the recorded pre-experiment RAM cap, within the EEPROM limit."""
        if (12 not in self.write_ids or self.signature(12) !=
                dict(model=1030, drive_mode=0, operating_mode=3, homing_offset=0)):
            raise SafetyError('A 어깨 이외의 출력 복원은 허용하지 않습니다.')
        if previous_goal_pwm != 885 or previous_goal_pwm > self.metadata[12]['pwm_limit']:
            raise SafetyError('기록된 원래 출력 상한과 다릅니다.')
        expected={64:1,98:15,100:250,84:1200,82:0,80:0}
        if any(self.read_register(12,a,1 if a in (64,98) else 2)!=v for a,v in expected.items()):
            raise SafetyError('출력 복원 전 어깨 상태가 예상과 다릅니다.')
        result,error=self.packet.writeTxRx(self.ports[self.specs[12]['bus']],12,100,2,list(previous_goal_pwm.to_bytes(2,'little')))
        self.check(result,error,'ID12 원래 출력 상한 복원')
        if self.read_register(12,100,2)!=previous_goal_pwm:
            raise SafetyError('출력 상한 복원 읽기 검증 실패')

    def goals(self, targets, *, current_positions, speed_deg_s, period_s):
        if not set(targets) <= self.write_ids:
            raise SafetyError("구동이 허용되지 않은 모터에 목표값이 지정됐습니다.")
        grouped = defaultdict(dict)
        for i, target in targets.items():
            raw_target=self.raw_goal(i,target)
            acceleration, velocity = profile_values(self.metadata[i]['drive_mode'], target,
                                                      current_positions[i], speed_deg_s, period_s)
            grouped[self.specs[i]["bus"]][i] = (acceleration, velocity, raw_target)
        for name, values in grouped.items():
            writer = GroupSyncWrite(self.ports[name], self.packet, 108, 12)
            for i, profile in values.items():
                data = b''.join(int(value).to_bytes(4, 'little',signed=index==2) for index,value in enumerate(profile))
                if not writer.addParam(i, list(data)):
                    raise SafetyError("Sync Write 목표값 등록 실패")
            self.check(writer.txPacket(), 0, f"{name} 목표 위치 전송")

    def stream_goals(self, targets, states, *, profile_speed_deg_s, period_s):
        """Plan from the internal trajectory, avoiding static-load profile inflation."""
        if not set(targets)<=self.write_ids or not 0<profile_speed_deg_s<=180 or not 0<period_s<=.2:
            raise SafetyError('연속 목표 전송 설정 오류')
        grouped=defaultdict(dict)
        for i,target in targets.items():
            raw=self.raw_goal(i,target)
            drive=self.metadata[i]['drive_mode']
            if drive&8:raise SafetyError('자동 토크 활성화 모드 금지')
            if drive&4:
                trajectory=states[i].position_trajectory
                if trajectory is None:raise SafetyError('내부 궤적 응답 누락')
                accel,velocity=profile_values(drive,raw,trajectory,profile_speed_deg_s,period_s)
                if profile_speed_deg_s>30:
                    # With acceleration T/4, peak speed is 4*distance/(3*T).
                    # One frame of interpolation; do not stack a second slow ramp.
                    velocity=max(2,math.ceil(period_s*1000),math.ceil(4000*abs(raw-trajectory)/(3*TICKS_PER_DEG*profile_speed_deg_s)))
                    accel=max(1,velocity//4)
            else:
                accel,velocity=profile_values(drive,raw,raw,profile_speed_deg_s,period_s)
                if profile_speed_deg_s>30:accel=50
                velocity_limit=self.metadata[i].get('velocity_limit',1023)
                acceleration_limit=self.metadata[i].get('acceleration_limit',32767)
                if velocity_limit and velocity>velocity_limit:
                    raise SafetyError(f'ID {i}: 요청 속도가 저장된 모터 속도 한계를 넘습니다.')
                if acceleration_limit:accel=min(accel,acceleration_limit)
            grouped[self.specs[i]['bus']][i]=(accel,velocity,raw)
        for name,values in grouped.items():
            writer=GroupSyncWrite(self.ports[name],self.packet,108,12)
            for i,profile in values.items():
                data=b''.join(int(v).to_bytes(4,'little',signed=k==2) for k,v in enumerate(profile))
                if not writer.addParam(i,list(data)):raise SafetyError('연속 목표 등록 실패')
            self.check(writer.txPacket(),0,f'{name} 연속 목표 전송')

    def close(self):
        for port in self.ports.values():
            if port.is_open:
                try:
                    port.closePort()
                except OSError:
                    pass  # A disconnected USB device must not prevent other ports closing.
        self.ports.clear()

    def __exit__(self, *exc):
        self.close()


def extend_rig(rig, config, pair):
    """Add read access on already-owned buses without reconnecting loaded arms."""
    pending=[]
    for spec in motors(config,pair):
        i=spec['id']
        if i in rig.specs:continue
        if spec['bus'] not in rig.ports:
            raise SafetyError('새 버스는 정지 상태에서 연결해야 합니다.')
        rig.sample()  # Keep the existing followers' watchdogs alive.
        port=rig.ports[spec['bus']]
        model,result,error=rig.packet.ping(port,i)
        rig.check(result,error,f'ID {i} Ping')
        if model!=spec['model']:raise SafetyError(f'ID {i}: 모델 불일치')
        data,result,error=rig.packet.readTxRx(port,i,0,64)
        rig.check(result,error,f'ID {i} 설정 읽기')
        def num(offset,size):return int.from_bytes(bytes(data[offset:offset+size]),'little')
        meta=dict(model=model,model_name=MODELS[model],firmware=num(6,1),
                  drive_mode=num(10,1),operating_mode=num(11,1),homing_offset=signed32(num(20,4)),
                  pwm_limit=num(36,2),position_max=num(48,4),position_min=num(52,4),
                  voltage_min=num(34,2)/10,voltage_max=num(32,2)/10)
        pending.append((spec,meta))
    for spec,meta in pending:
        i=spec['id'];rig.specs[i]=spec;rig.metadata[i]=meta
        rig.groups[spec['bus']].addParam(i)
    rig.follower_ids.update(j['follower']['id'] for name,js in config['pairs'].items()
                            if pair in ('both',name) for j in js)
    return rig.sample()


def report(rig, states):
    return [dict(**asdict(states[i]), **rig.metadata[i], bus=rig.specs[i]["bus"],
                 port=rig.config["buses"][rig.specs[i]["bus"]]) for i in sorted(states)]
