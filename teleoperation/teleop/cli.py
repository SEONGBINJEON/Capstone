import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

from .config import SafetyError, joints, load, require_calibration, require_gripper, save
from .control import health
from .hardware import Rig, report
from .runtime import Events, run_session, hold_session


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('양의 유한한 수가 필요합니다.')
    return number


def build_parser():
    p = argparse.ArgumentParser(description='리더 A/B → 팔로워 A/B 텔레오퍼레이션')
    p.add_argument('--config', default='config/robot.json')
    sub = p.add_subparsers(dest='command', required=True)
    inspect = sub.add_parser('inspect', help='모터 모델·상태·설정 읽기 (쓰지 않음)')
    inspect.add_argument('--save')
    inspect.add_argument('--pair', choices=['A','B','both'])
    monitor = sub.add_parser('monitor', help='모터를 구동하지 않고 연속 위치 읽기')
    monitor.add_argument('--duration', type=positive, default=5)
    monitor.add_argument('--jsonl')
    monitor.add_argument('--pair', choices=['A','B','both'])
    capture = sub.add_parser('capture-neutral', help='맞춘 기준 자세를 설정 파일에 저장')
    capture.add_argument('--pair', choices=['A','B','both'])
    capture.add_argument('--poses-aligned', action='store_true', help='양쪽 팔의 기준 자세를 직접 맞췄음을 확인')
    grip = sub.add_parser('capture-gripper', help='그리퍼의 열린/닫힌 위치를 설정 파일에 저장')
    grip.add_argument('--pair', choices=['A','B'])
    grip.add_argument('--state', choices=['open','closed'], required=True)
    grip.add_argument('--poses-set', action='store_true')
    grip.add_argument('--reviewed', action='store_true', help='양쪽 끝 위치를 기구에서 확인했음을 기록')
    joint = sub.add_parser('set-joint', help='확인한 관절 방향/범위를 설정 파일에 저장')
    joint.add_argument('--pair', choices=['A','B'], required=True)
    joint.add_argument('--joint', type=int, choices=range(1,6), required=True)
    joint.add_argument('--direction', type=int, choices=[-1,1], required=True)
    joint.add_argument('--scale', type=positive, default=1)
    joint.add_argument('--leader-range', type=float, nargs=2, metavar=('MIN_DEG','MAX_DEG'), required=True)
    joint.add_argument('--follower-range', type=float, nargs=2, metavar=('MIN_DEG','MAX_DEG'), required=True)
    joint.add_argument('--reviewed', action='store_true', help='기구에서 이 범위와 방향을 확인했음을 기록')
    run = sub.add_parser('run', help='기본은 쓰기 없는 추종 계산; --enable-motion으로만 실물 구동')
    run.add_argument('--pair', choices=['A','B','both'])
    run.add_argument('--duration', type=positive)
    run.add_argument('--enable-motion', action='store_true')
    run.add_argument('--startup', choices=['matched','align'],
                     help='align: 리더의 시작 자세까지 저속 정렬 후 추종; matched: 자세 차이가 크면 시작 거부')
    run.add_argument('--jsonl')
    run.add_argument('--udp-port', type=int, help='같은 PC의 향후 시뮬레이터에 목표값 송신')
    hold = sub.add_parser('hold', help='현재 자세에서 팔로워 토크 ON; 리더 추종은 하지 않음')
    hold.add_argument('--pair', choices=['A','B'])
    hold.add_argument('--enable-hold', action='store_true', help='팔로워의 토크를 켜서 현재 자세 유지')
    hold.add_argument('--resume-hold', action='store_true', help='이미 토크 ON이며 목표에 도착한 팔로워의 자세 유지 재개')
    hold.add_argument('--recover-watchdog', action='store_true', help='유지 재개 시 목표/현재 위치 일치를 검사한 뒤 통신 감시 오류 복구')
    hold.add_argument('--jog-joint', type=int, choices=[1,2,3,4], help='관찰하며 방향을 확인할 팔로워 관절 하나')
    hold.add_argument('--jog-degrees', type=float, help='방향 확인 이동량: -3~3도, 0 제외; 이동 공간을 직접 확인한 뒤 사용')
    hold.add_argument('--home-joint', type=int, choices=[1,2,3,4], help='현재 자세를 받치고 원점 좌표를 변경할 팔 관절')
    hold.add_argument('--home-offset', type=int, help='저장할 Homing Offset: -1024~1024 tick')
    hold.add_argument('--supported', action='store_true', help='변경 대상 관절의 토크가 잠깐 꺼져도 자세가 유지되도록 받쳤음')
    hold.add_argument('--extended-base', action='store_true', help='A 베이스 11번만 확장 위치 모드로 전환; 프로그램에서 기준 좌우45도로 제한')
    hold.add_argument('--duration', type=positive)
    hold.add_argument('--jsonl')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    events = None
    try:
        config = load(args.config)
        if args.command == 'set-joint':
            if not args.reviewed:
                raise SafetyError('실제 기구의 방향·이동 범위를 확인한 뒤 --reviewed를 지정하세요.')
            joint = config['pairs'][args.pair][args.joint-1]
            if joint.get('kind') == 'gripper':
                raise SafetyError('그리퍼는 capture-gripper로 열린/닫힌 위치를 각각 기록하세요.')
            c = joint['calibration']
            c.update(direction=args.direction, scale=args.scale, leader_min_deg=args.leader_range[0],
                     leader_max_deg=args.leader_range[1], follower_min_deg=args.follower_range[0],
                     follower_max_deg=args.follower_range[1], reviewed=True)
            require_calibration({'pairs':{args.pair:[joint]}}, args.pair)
            save(args.config, config)
            print(f'{args.pair}/{joint["name"]} 보정 저장 완료 (모터 쓰기 없음)')
            return 0
        pair = getattr(args, 'pair', None) or config.get('active_pair', 'A')
        motion = getattr(args, 'enable_motion', False) or (args.command == 'hold' and args.enable_hold)
        if args.command == 'hold':
            if not args.enable_hold or pair == 'both':
                raise SafetyError('한 쌍을 선택하고 --enable-hold를 지정해야 현재 자세에서 토크를 켭니다.')
            events = Events(args.jsonl)
        if args.command in ('run','hold'):
            if args.command == 'run':
                require_calibration(config, pair)  # Fail before opening ports or writing hardware.
                events = Events(args.jsonl, args.udp_port)
            def interrupt(*_):
                raise KeyboardInterrupt
            signal.signal(signal.SIGTERM, interrupt)
        if args.command == 'capture-neutral' and not args.poses_aligned:
            raise SafetyError('팔을 받치고 리더/팔로워의 기준 자세를 맞춘 뒤 --poses-aligned를 지정하세요.')
        if args.command == 'capture-gripper':
            if pair == 'both' or not args.poses_set:
                raise SafetyError('A/B 한 쌍을 선택하고 두 그리퍼를 맞춘 뒤 --poses-set을 지정하세요.')
        with Rig(config, pair, allow_motion=motion) as rig:
            if args.command == 'inspect':
                data = report(rig, rig.sample())
                if args.save:
                    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
                    Path(args.save).write_text(json.dumps(data, indent=2, ensure_ascii=False)+'\n')
                print('ID  모델          위치(tick)  전압    온도  토크  모드  HW오류  보드')
                for r in data:
                    print(f"{r['id']:2}  {r['model_name']:12} {r['position']:7}    {r['voltage']:4.1f}V  {r['temperature']:2}°C   {r['torque']}    {r['operating_mode']}    {r['hardware_error']:02x}    {r['bus']}")
                print(f'{len(data)}개 확인. 모터 쓰기 없음.')
            elif args.command == 'monitor':
                events = Events(args.jsonl)
                start = time.monotonic(); count = 0; longest = 0
                while time.monotonic()-start < args.duration:
                    begin = time.monotonic(); states = rig.sample()
                    elapsed = time.monotonic()-begin
                    longest = max(longest, elapsed)
                    health(config, states, rig.metadata)
                    events.emit({'schema':'teleop.observation.v1','sequence':count,'unix_time':time.time(),
                                 'motors':report(rig, states)})
                    count += 1
                    time.sleep(max(0, 1/config['safety']['rate_hz']-elapsed))
                total = time.monotonic()-start
                print(json.dumps({'motors':len(rig.specs),'samples':count,'effective_hz':round(count/total,2),
                                  'max_read_ms':round(longest*1000,2),'motor_writes':0}, ensure_ascii=False))
            elif args.command == 'capture-neutral':
                states = rig.sample(); health(config, states, rig.metadata)
                if any(s.torque for s in states.values()):
                    raise SafetyError('기준 자세 기록 전 선택한 모터의 토크가 모두 꺼져 있어야 합니다.')
                for _, joint in joints(config, pair):
                    if joint.get('kind') == 'gripper':
                        continue
                    lid, fid = joint['leader']['id'], joint['follower']['id']
                    joint['calibration'].update(leader_zero=states[lid].position,
                                                follower_zero=states[fid].position, reviewed=False,
                                                signature={'leader':rig.signature(lid),'follower':rig.signature(fid)})
                save(args.config, config)
                print('기준 자세 저장 완료. 방향/이동 범위 확인은 별도이며 reviewed=false로 표시했습니다.')
            elif args.command == 'capture-gripper':
                states = rig.sample(); health(config, states, rig.metadata)
                joint = next(j for _,j in joints(config,pair) if j.get('kind') == 'gripper')
                lid, fid = joint['leader']['id'], joint['follower']['id']
                if states[lid].torque or states[fid].torque:
                    raise SafetyError('그리퍼 기록 전 두 그리퍼의 토크가 꺼져 있어야 합니다.')
                c = joint['calibration']
                signature = {'leader':rig.signature(lid),'follower':rig.signature(fid)}
                if c.get('signature') not in (None, signature):
                    raise SafetyError('닫힘/열림 기록 사이에 모터 설정이 달라졌습니다. 두 끝점 재보정 필요')
                c.update({f'leader_{args.state}':states[lid].position,
                          f'follower_{args.state}':states[fid].position,
                          'signature':signature,'reviewed':args.reviewed})
                if args.reviewed:
                    require_gripper(c, f'{pair}/gripper')
                    lo,hi = sorted((c['follower_closed'],c['follower_open']))
                    if not rig.metadata[fid]['position_min'] <= lo < hi <= rig.metadata[fid]['position_max']:
                        raise SafetyError('그리퍼 끝점이 모터에 저장된 위치 제한 밖입니다.')
                save(args.config,config)
                print(f'{pair} 그리퍼 {args.state} 기록: 리더 {states[lid].position}, 팔로워 {states[fid].position}; 모터 쓰기 없음')
            elif args.command == 'run':
                print('실물 추종 시작' if motion else '추종 계산 시작 (모터 쓰기 없음)', flush=True)
                result = run_session(rig, config, pair, args.duration, motion, events,
                                     startup_mode=args.startup or config.get('startup_mode', 'matched'))
                print(json.dumps(result, ensure_ascii=False))
            elif args.command == 'hold':
                print('팔로워 현재 자세 유지 시작 (리더 추종 없음)',flush=True)
                print(json.dumps(hold_session(rig,config,pair,args.duration,events,
                                              resume=args.resume_hold,recover_watchdog=args.recover_watchdog,
                                              jog_joint=args.jog_joint,jog_degrees=args.jog_degrees,
                                              home_joint=args.home_joint,home_offset=args.home_offset,
                                              supported=args.supported,config_path=args.config,
                                              extended_base=args.extended_base),ensure_ascii=False))
        return 0
    except KeyboardInterrupt:
        print('중지했습니다. 실물 구동 중이었다면 토크를 자동 해제하지 않고 위치 유지 명령을 시도했습니다.', file=sys.stderr)
        return 130
    except (SafetyError, OSError, KeyError, ValueError) as exc:
        print(f'중지: {exc}', file=sys.stderr)
        return 1
    finally:
        if events:
            events.close()
