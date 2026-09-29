"""Persistent serial owner for both arms; frontend exit pauses without dropping torque."""
import json
import os
import signal
import time
from pathlib import Path
from dataclasses import asdict
from teleop.awake import inhibit_sleep
from teleop.config import load
from teleop.hardware import Rig
from teleop.runtime import Events,hold
from teleop.live_actions import execute_action
from teleop.direct import write_status


def main():
    inhibit_sleep()
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    control=Path('work/live_A_control.json');events=Events('work/teleop_events.jsonl')
    Path('work/teleop_service.json').write_text(json.dumps({'pid':os.getpid(),'control_path':str(control)}))
    seen=None;last_status=0.;last_fault=None
    try:
        with Rig(load('config/robot.json'),'both',allow_motion=True) as rig:
            try:
                while True:
                    request=json.loads(control.read_text())
                    if request.get('nonce')!=seen:
                        seen=request.get('nonce')
                        if request.get('action')=='shutdown':break
                        last_fault=None
                        if request.get('action')!='hold':
                            try:execute_action(rig,load('config/robot.json'),events,request,str(control))
                            except Exception as exc:last_fault=str(exc)
                    s=rig.sample()
                    fault=[i for i,v in s.items() if v.hardware_error or v.temperature>=rig.config['safety']['max_temperature_c']]
                    if fault:last_fault='모터 오류/과열: '+str(fault)
                    if time.monotonic()-last_status>.2:
                        write_status({'unix_time':time.time(),'pair':'both','phase':'fault' if last_fault else 'hold',
                                      'reason':last_fault,'motors':{i:asdict(v) for i,v in s.items()}})
                        last_status=time.monotonic()
                    time.sleep(.03)
            finally:
                hold(rig,rig.follower_ids,preserve_stationary=True)
    except KeyboardInterrupt:
        write_status({'unix_time':time.time(),'phase':'stopped','reason':'서비스 종료; 토크 해제 안 함'})
    except Exception as exc:
        write_status({'unix_time':time.time(),'phase':'fault','reason':str(exc)})
        raise
    finally:events.close()


if __name__=='__main__':main()
