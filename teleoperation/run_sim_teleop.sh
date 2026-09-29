#!/usr/bin/env bash
# Isaac Sim 뷰어(백그라운드) + 텔레오퍼레이션(전경) 동시 실행.
# 사용: ./run_sim_teleop.sh [teleop.py 옵션...]   예) ./run_sim_teleop.sh --match
set -e
cd "$(dirname "$0")"
ISAAC=${ISAAC:-$HOME/isaacsim-6.0.1}
mkdir -p work
if ! pgrep -f "sim_teleop/isaac_view.py" >/dev/null; then
  echo "Isaac Sim 뷰어 시작 (로그: work/isaac_view.log) ..."
  nohup "$ISAAC/python.sh" sim_teleop/isaac_view.py > work/isaac_view.log 2>&1 &
  echo "  PID $!  (창이 뜨기까지 30~60초)"
else
  echo "Isaac Sim 뷰어가 이미 실행 중입니다."
fi
echo "텔레오퍼레이션 시작: python3 sim_teleop/teleop.py $*"
exec python3 sim_teleop/teleop.py "$@"
