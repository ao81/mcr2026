#!/bin/bash
# debug/ のプログラムを模擬のコースで走らせ、実機と同じ形の詳しいログ（dbg 形式の CSV）を作る
#   使い方: debug/tools/sim_dbg.sh <出力ファイル> <スイッチ> <コース名> [名前=値 ...]
#     <スイッチ> <コース名> [名前=値] は tools/course_sim/run.sh と同じ（tools/course_sim/README.md）
#     例: debug/tools/sim_dbg.sh sim.csv keep crank_l v_max=1.75 line_w=0.020
#         keep : debug/config.h をそのまま使う（実機に書き込んだ設定と同じ）
#   実機のログと比べるときは:  python3 debug/tools/analyze.py DBG0001.CSV --sim sim.csv
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
DBG="$HERE/.."
ROOT="$DBG/.."
OUT=${1:?出力ファイルを指定してください}; shift
MODE=${1:-keep}; shift || true
COURSE=${1:?コース名を指定してください}; shift
case "$OUT" in /*) ;; *) OUT="$PWD/$OUT" ;; esac
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
mkdir "$TMP/src"
cp "$DBG"/*.ino "$DBG"/*.h "$TMP/src/"
CFG="$TMP/src/config.h"
# 模擬のコースはクランク・レーンチェンジが1つずつなので COURSE_LIST は使わない（run.sh と同じ）
sed -i -E 's/^(#define COURSE_LIST_USE +)[0-9]+/\10/' "$CFG"
IFS=',' read -ra SW <<< "$MODE"
for kv in "${SW[@]}"; do
  case "$kv" in
    keep) ;;
    off) sed -i -E 's/^(#define USE_NEW_[A-Z_]+ +)[0-9]+/\10/' "$CFG" ;;
    all) sed -i -E 's/^(#define USE_NEW_[A-Z_]+ +)[0-9]+/\11/' "$CFG" ;;
    *=*)
      k=${kv%%=*}; v=${kv#*=}
      f="$CFG"; grep -qE "^#define $k " "$f" || f="$TMP/src/dbg_config.h"
      grep -qE "^#define $k " "$f" || { echo "config.h にも dbg_config.h にも $k がありません"; exit 2; }
      sed -i -E "s/^(#define $k +)[-0-9.f()]+/\1$v/" "$f" ;;
    *) echo "スイッチの指定が違います: $kv"; exit 2 ;;
  esac
done
{
  echo '#include "mock_arduino.h"'
  cat "$TMP/src/debug.ino"
  for f in $(ls "$TMP"/src/*.ino | sort); do
    [ "$(basename "$f")" = debug.ino ] || cat "$f"
  done
} > "$TMP/sketch.cpp"
g++ -std=gnu++17 -O1 -w -DDBG_HOST -I"$ROOT/tools/host_test/mock" -I"$TMP/src" -o "$TMP/sim" "$TMP/sketch.cpp" "$ROOT/tools/course_sim/course_sim.cpp"
DBG_HOST_OUT="$OUT" "$TMP/sim" "$COURSE" "$@" | tail -1
echo "→ $OUT"
