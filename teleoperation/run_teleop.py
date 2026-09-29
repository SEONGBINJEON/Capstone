"""Start, monitor or pause the persistent A/B teleoperation service."""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parent
os.chdir(ROOT)
CONTROL=Path('work/live_A_control.json')
STATUS=Path('work/teleop_status.json')
SERVICE=Path('work/teleop_service.json')


def service_alive():
    try:
        pid=int(json.loads(SERVICE.read_text())['pid'])
        os.kill(pid,0)
        cmd=Path(f'/proc/{pid}/cmdline').read_bytes().decode().replace('\0',' ')
        return 'live_A_service.py' in cmd or 'teleop.service' in cmd or 'teleop/service.py' in cmd
    except (OSError,ValueError,KeyError):return False


def request(action):
    old=json.loads(CONTROL.read_text()) if CONTROL.exists() else {'nonce':0}
    value={'nonce':int(old['nonce'])+1,'action':action,'motor_limits':True}
    if action.startswith('follow_') and service_alive():
        selected={'A','B'} if action=='follow_both' else {action.removeprefix('follow_')}
        value['start_paused_pairs']=sorted(set(read_status().get('paused_pairs',{})) & selected)
    temp=CONTROL.with_suffix('.tmp');temp.write_text(json.dumps(value));temp.replace(CONTROL)
    return time.time()


def resume_pair(pair):
    state=read_status()
    requested={'A','B'} if pair=='both' else {pair}
    selected=sorted(requested & set(state.get('paused_pairs',{})))
    if not service_alive() or not selected:
        return False
    value=json.loads(CONTROL.read_text())
    value.update(resume_token=time.time_ns(),resume_pairs=selected)
    temp=CONTROL.with_suffix('.tmp');temp.write_text(json.dumps(value));temp.replace(CONTROL)
    return True


def read_status():
    try:return json.loads(STATUS.read_text())
    except (OSError,ValueError):return {}


def describe(s):
    phase=s.get('phase','preparing')
    name={'preparing':'시작 준비','aligning':'시작 자세 정렬 중','following':'실시간 추종 중',
          'partial':'일부 팔 대기 / 나머지 추종','paused':'자세 유지 대기 / 연결 유지',
          'hold':'자세 유지 / 추종 정지','torque_off':'토크 OFF / 추종 정지','fault':'정지','stopped':'서비스 종료'}.get(phase,phase)
    if phase in ('following','aligning','partial','paused') and time.time()-s.get('unix_time',0)>2:
        return '통신 상태 갱신이 끊겼습니다. 추종 상태를 확인할 수 없습니다.'
    motors=s.get('motors',{});errors=[]
    for i,v in motors.items():
        if i in ('11','12','13','14','15','21','22','23','24','25') and v.get('goal_position') is not None:
            errors.append(abs(v['position']-v['goal_position'])*360/4096)
    suffix=f' | 목표와 실제 위치 최대 차이 {max(errors):.1f}°' if errors else ''
    remaining=[abs(float(goal)-motors[str(i)]['position'])*360/4096
               for i,goal in s.get('desired',{}).items() if str(i) in motors]
    if remaining:suffix+=f' | 리더 목표까지 최대 {max(remaining):.1f}° 남음'
    if s.get('limited_ids'):suffix+=' | 범위 끝: '+','.join(map(str,s['limited_ids']))
    if s.get('reason'):suffix+=' | '+str(s['reason'])
    for pair,pause in s.get('paused_pairs',{}).items():
        suffix+=f' | {pair} 대기: {pause["reason"]} (리더 자세 차이 {pause["rejoin_gap_deg"]:.1f}°)'
    return name+suffix


def main():
    parser=argparse.ArgumentParser(description='A/B 자동 정렬 후 텔레오퍼레이션')
    parser.add_argument('--pair',choices=['A','B','both'],default='both')
    parser.add_argument('--hold',action='store_true',help='추종을 멈추고 현재 자세 유지')
    parser.add_argument('--status',action='store_true',help='현재 상태만 출력')
    parser.add_argument('--resume',choices=['A','B','both'],help='장애물을 제거한 뒤 선택한 팔을 저속으로 재개')
    args=parser.parse_args();Path('work').mkdir(exist_ok=True)
    if args.status:
        print(describe(read_status()) if service_alive() else '추종 서비스가 꺼져 있습니다.');return
    if args.resume:
        print('선택한 팔의 저속 재개를 요청했습니다.' if resume_pair(args.resume) else '재개할 대기 상태의 팔이 없습니다.')
        return
    if args.hold:
        if service_alive():request('hold');print('추종 정지를 요청했습니다. 토크는 유지합니다.')
        else:print('추종 서비스가 꺼져 있습니다.')
        return
    started=request('follow_'+args.pair)
    if not service_alive():
        with open('work/teleop_console.log','a') as log:
            subprocess.Popen([sys.executable,'-m','teleop.service'],stdout=log,stderr=log,start_new_session=True)
    print('리더를 편한 시작 자세에 두세요. 정렬 뒤 빠른 추종으로 전환됩니다. Ctrl+C: 자세 유지.')
    previous=None
    try:
        while True:
            state=read_status()
            if state.get('unix_time',0)>=started:
                text=describe(state)
                if text!=previous:print(text,flush=True);previous=text
                if state.get('phase') in ('fault','stopped'):
                    print('원인을 해결한 뒤 다시 실행하세요.');return
            if time.time()-started>5 and not service_alive():
                print('추종 서비스가 종료됐습니다. work/teleop_console.log를 확인하세요.');return
            time.sleep(.3)
    except KeyboardInterrupt:
        request('hold');print('\n추종 정지 요청. 토크와 통신 연결은 유지합니다.')


if __name__=='__main__':main()
