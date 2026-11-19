#!/bin/bash
# コースを模擬して走らせる（tools/course_sim/README.md を参照）
#   使い方: tools/course_sim/run.sh <スイッチ> [コース名 ...] [名前=値 ...]
#     <スイッチ>  config.h の書き換えをカンマで区切って並べる（書き換えるのはコピーで、元のファイルは変えない）
#                 off : USE_NEW_* をすべて 0（元のプログラムと同じ動き）
#                 all : USE_NEW_* をすべて 1
#                 名前=値 : その #define を書き換える。例: all,CROSS_TO_CRANK_M=0.70f
#                 例: off,USE_NEW_CRANK_TURN=1,USE_NEW_CRANK_BRAKE=1
#     コース名   crank_l crank_r lane_l lane_r curve_l curve_r s_l s_r crank2（省略すると crank2 以外のすべて）
#     名前=値    車とコースの本当の値（course_sim.cpp の先頭を参照）。例: grip_lat=6 v_max=2.7
#                avg=1 を付けると、スタートから特別な線までの距離を 1.400〜1.500m の7通りに変えて走らせ、
#                平均の時間・失敗の回数・横Gの最大を1行で出す（ラインの通過時間は1ms単位なので、
#                車がラインに来る時刻のずれで速さの測定値が変わる。その影響をならして比べるため）
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT="$HERE/../.."
MODE=${1:-off}; shift || true
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
cp -r "$ROOT/basic" "$TMP/basic"
CFG="$TMP/basic/config.h"
# 模擬のコースはクランク・レーンチェンジが1つずつなので、config.h の表（COURSE_LIST）は使わず、
# CROSS_TO_CRANK_M などの1つの値を使う（表を試すときは COURSE_LIST_USE=1 を付ける）
sed -i -E 's/^(#define COURSE_LIST_USE +)[0-9]+/\10/' "$CFG"
IFS=',' read -ra SW <<< "$MODE"
for kv in "${SW[@]}"; do
  case "$kv" in
    off) sed -i -E 's/^(#define USE_NEW_[A-Z_]+ +)[0-9]+/\10/' "$CFG" ;;
    all) sed -i -E 's/^(#define USE_NEW_[A-Z_]+ +)[0-9]+/\11/' "$CFG" ;;
    *=*)
      k=${kv%%=*}; v=${kv#*=}
      grep -qE "^#define $k " "$CFG" || { echo "config.h に $k がありません"; exit 2; }
      sed -i -E "s/^(#define $k +)[-0-9.f()]+/\1$v/" "$CFG" ;;
    *) echo "スイッチの指定が違います: $kv"; exit 2 ;;
  esac
done
{
  echo '#include "mock_arduino.h"'
  cat "$TMP/basic/basic.ino"
  for f in $(ls "$TMP"/basic/*.ino | sort); do
    [ "$(basename "$f")" = basic.ino ] || cat "$f"
  done
} > "$TMP/sketch.cpp"
g++ -std=gnu++17 -O1 -w -I"$ROOT/tools/host_test/mock" -I"$TMP/basic" -o "$TMP/sim" "$TMP/sketch.cpp" "$HERE/course_sim.cpp"
rc=0
for c in "${COURSES[@]}"; do
  if [ $AVG -eq 0 ]; then
    "$TMP/sim" "$c" "${OPTS[@]}" || rc=1
    continue
  fi
  tot=0; n=0; fail=0; lat=0
  for ap in 1.400 1.417 1.433 1.450 1.467 1.483 1.500; do
    line=$("$TMP/sim" "$c" "${OPTS[@]}" approach=$ap 2>&1 | tail -1) || true
    if echo "$line" | grep -q "ゴール "; then
      t=$(echo "$line" | sed -E 's/.*ゴール +([0-9]+) ms.*/\1/'); tot=$((tot + t)); n=$((n + 1))
    else
      fail=$((fail + 1))
    fi
    l=$(echo "$line" | sed -nE 's/.*横G +([0-9]+)%.*/\1/p'); [ -n "$l" ] && [ "$l" -gt "$lat" ] && lat=$l
  done
  if [ $n -gt 0 ]; then
    printf "%-8s 平均 %5d ms  失敗 %d/7  横G 最大 %3d%%\n" "$c" $((tot / n)) $fail $lat
  else
    printf "%-8s 7回とも失敗      横G 最大 %3d%%\n" "$c" $lat
  fi
  [ $fail -eq 0 ] || rc=1
done
exit $rc
