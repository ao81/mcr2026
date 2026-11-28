#!/bin/bash
# release/release.ino をコースの模擬で走らせる（tools/course_sim/README.md の 4-7 を参照。release.ino 自体は変えない）
#   使い方: tools/course_sim/run_release.sh [コース名 ...] [名前=値 ...]
#     コース名   crank_l crank_r lane_l lane_r curve_l curve_r s_l s_r crank2（省略すると crank2 以外のすべて）
#     名前=値    車とコースの本当の値（course_sim.cpp の先頭を参照）。初期値は release_sim/config.h
#                クランクのコースは line_w=0.02、レーンのコースは line_w=0.04 を付けないと、線の幅は 0.032 になる
#                （ブラウザ版はコースの種類ごとにこの初期値を入れる）
#                avg=1 を付けると、スタートから特別な線までの距離を 1.400〜1.500m の7通りに変えて走らせ、平均を出す
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT="$HERE/../.."
COURSES=(); OPTS=(); AVG=0
for a in "$@"; do
  case "$a" in
    avg=1) AVG=1 ;;
    avg=*) ;;
    *=*) OPTS+=("$a") ;;
    *) COURSES+=("$a") ;;
  esac
done
[ ${#COURSES[@]} -eq 0 ] && COURSES=(crank_l crank_r lane_l lane_r curve_l curve_r s_l s_r)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
INO="$ROOT/release/release.ino"
DEFS=()
for n in PWM_CYCLE_M SERVO_CENTER HANDLE_STEP; do
  v=$(sed -nE "s/^#define +$n +([0-9]+).*/\1/p" "$INO")
  [ -n "$v" ] || { echo "release.ino に #define $n がありません"; exit 2; }
  DEFS+=("-D$n=$v")
done
# AGTCallback は release_sim/stub.cpp が同じ名前で持つので、release.ino の方の名前を変える（sketch.cpp の中だけ）
{ echo '#include "mock_arduino.h"'; echo '#define AGTCallback release_AGTCallback'; cat "$INO"; } > "$TMP/sketch.cpp"
g++ -std=gnu++17 -O1 -w -I"$ROOT/tools/host_test/mock" -I"$HERE/release_sim" "${DEFS[@]}" -o "$TMP/sim" \
  "$TMP/sketch.cpp" "$HERE/course_sim.cpp" "$HERE/release_sim/stub.cpp"
rc=0
for c in "${COURSES[@]}"; do
  if [ $AVG -eq 0 ]; then
    "$TMP/sim" "$c" "${OPTS[@]}" || rc=1
    continue
  fi
  tot=0; n=0; fail=0
  for ap in 1.400 1.417 1.433 1.450 1.467 1.483 1.500; do
    line=$("$TMP/sim" "$c" "${OPTS[@]}" approach=$ap 2>&1 | tail -1) || true
    if echo "$line" | grep -q "ゴール "; then
      t=$(echo "$line" | sed -E 's/.*ゴール +([0-9]+) ms.*/\1/'); tot=$((tot + t)); n=$((n + 1))
    else
      fail=$((fail + 1))
    fi
  done
  if [ $n -gt 0 ]; then printf "%-8s 平均 %5d ms  失敗 %d/7\n" "$c" $((tot / n)) $fail; else printf "%-8s 7回とも失敗\n" "$c"; fi
  [ $fail -eq 0 ] || rc=1
done
exit $rc
