#!/usr/bin/env bash
# 추종 정지(팔로워는 자세 유지, 토크 ON). Ctrl+C와 동일.
pkill -INT -f "sim_teleop/teleop[.]py" && echo "정지 요청 보냄" || echo "실행 중인 추종 없음"
