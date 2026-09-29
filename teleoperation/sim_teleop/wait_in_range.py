"""Poll (read-only) until every follower is inside its motor limits, then exit 0."""
import sys, time
from common import load_config, open_rig, TICKS_PER_DEG
cfg = load_config(); deadline = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 240)
with open_rig(cfg, allow_motion=False) as rig:
    while time.time() < deadline:
        s = rig.sample(); bad = []
        for j in cfg['joints']:
            fid = j['follower']['id']; lo, hi = rig.position_limits(fid); p = s[fid].position
            if not lo <= p <= hi: bad.append(f"ID {fid} {p} (범위 {lo}~{hi}, {(lo-p)/TICKS_PER_DEG if p<lo else (hi-p)/TICKS_PER_DEG:+.0f}° 필요)")
        if not bad: print('모든 팔로워가 범위 안입니다.'); sys.exit(0)
        print('\r대기: ' + '; '.join(bad) + '      ', end='', flush=True); time.sleep(0.3)
print('\n시간 초과'); sys.exit(1)
