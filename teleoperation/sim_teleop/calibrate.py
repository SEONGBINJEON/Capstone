"""Read-only calibration helpers (never writes to motors; edits config.json only).

    python3 sim_teleop/calibrate.py watch        # live leader/follower positions
    python3 sim_teleop/calibrate.py zero         # both arms in the SAME reference pose -> capture zeros
    python3 sim_teleop/calibrate.py direction    # per joint: move leader & follower the same way -> sign
    python3 sim_teleop/calibrate.py gripper      # capture leader/follower closed & open positions
"""
import sys
import time

from common import TICKS_PER_DEG, delta_ticks, load_config, open_rig, save_config


def read(rig):
    return rig.sample()


def watch(cfg, rig):
    print('Ctrl+C로 종료')
    try:
        while True:
            s = read(rig)
            row = []
            for j in cfg['joints']:
                lid, fid = j['leader']['id'], j['follower']['id']
                row.append(f"{j['name']}: L{lid}={s[lid].position:5d} F{fid}={s[fid].position:5d}")
            print('\r' + ' | '.join(row), end='', flush=True)
            time.sleep(.1)
    except KeyboardInterrupt:
        print()


def stable(rig, ids, seconds=1.0):
    """Average position over a short window, requiring the motors to be still."""
    samples = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        samples.append(read(rig)); time.sleep(.05)
    out = {}
    for i in ids:
        vals = [s[i].position for s in samples]
        if max(vals) - min(vals) > 6:
            raise RuntimeError(f'ID {i}가 움직이는 중입니다. 잠시 멈춘 뒤 다시 시도하세요.')
        out[i] = round(sum(vals) / len(vals))
    return out


def zero(cfg, rig):
    print('리더와 팔로워를 물리적으로 같은 기준 자세(예: 일자로 세운 자세)로 맞추세요.')
    input('준비되면 Enter...')
    ids = [j['leader']['id'] for j in cfg['joints']] + [j['follower']['id'] for j in cfg['joints']]
    p = stable(rig, ids)
    for j in cfg['joints']:
        if j.get('kind') == 'gripper':
            continue
        j['leader_zero'] = p[j['leader']['id']]
        j['follower_zero'] = p[j['follower']['id']]
        print(f"  {j['name']}: leader_zero={j['leader_zero']} follower_zero={j['follower_zero']}")
    save_config(cfg); print('config.json 저장 완료')


def direction(cfg, rig):
    for j in cfg['joints']:
        if j.get('kind') == 'gripper':
            continue
        lid, fid = j['leader']['id'], j['follower']['id']
        print(f"\n[{j['name']}] 리더 ID {lid}와 팔로워 ID {fid}를 손으로 '같은 물리적 방향'으로 20° 이상 돌리세요.")
        input('  시작 위치에서 Enter...')
        a = stable(rig, [lid, fid], .5)
        input('  같은 방향으로 움직인 뒤 Enter...')
        b = stable(rig, [lid, fid], .5)
        dl = delta_ticks(b[lid], a[lid]); df = delta_ticks(b[fid], a[fid])
        if abs(dl) < 10 * TICKS_PER_DEG or abs(df) < 10 * TICKS_PER_DEG:
            print(f'  이동량 부족 (리더 {dl/TICKS_PER_DEG:.1f}°, 팔로워 {df/TICKS_PER_DEG:.1f}°). 건너뜀.'); continue
        j['direction'] = 1 if dl * df > 0 else -1
        print(f"  리더 {dl/TICKS_PER_DEG:+.1f}°, 팔로워 {df/TICKS_PER_DEG:+.1f}° -> direction={j['direction']}")
    save_config(cfg); print('config.json 저장 완료')


def gripper(cfg, rig):
    g = next(j for j in cfg['joints'] if j.get('kind') == 'gripper')
    lid, fid = g['leader']['id'], g['follower']['id']
    for state in ('closed', 'open'):
        label = '완전히 닫힌' if state == 'closed' else '완전히 열린'
        input(f'리더 그리퍼(ID {lid})와 팔로워 그리퍼(ID {fid})를 {label} 상태로 두고 Enter...')
        p = stable(rig, [lid, fid], .5)
        g[f'leader_{state}'] = p[lid]; g[f'follower_{state}'] = p[fid]
        print(f"  leader_{state}={p[lid]} follower_{state}={p[fid]}")
    if abs(delta_ticks(g['leader_open'], g['leader_closed'])) < 8 or abs(g['follower_open'] - g['follower_closed']) < 8:
        print('개폐 범위가 너무 작습니다. 저장하지 않습니다.'); return
    save_config(cfg); print('config.json 저장 완료')


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'watch'
    cfg = load_config()
    with open_rig(cfg, allow_motion=False) as rig:
        {'watch': watch, 'zero': zero, 'direction': direction, 'gripper': gripper}[mode](cfg, rig)


if __name__ == '__main__':
    main()
