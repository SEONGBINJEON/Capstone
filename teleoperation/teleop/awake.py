"""Keep Linux awake only while the hardware process is alive."""
import os
import shutil
import sys


def inhibit_sleep():
    if os.environ.get('TELEOP_SLEEP_INHIBITED') == '1':
        return
    executable = shutil.which('systemd-inhibit')
    if executable is None:
        raise RuntimeError('systemd-inhibit가 없어 USB 구동 중 절전을 방지할 수 없습니다.')
    env = dict(os.environ, TELEOP_SLEEP_INHIBITED='1')
    os.execvpe(executable, [executable, '--what=idle:sleep', '--mode=block',
                           '--who=Teleoperation', '--why=Active USB motor control',
                           sys.executable, *sys.argv], env)
