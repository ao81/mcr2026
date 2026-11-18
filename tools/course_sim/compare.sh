#!/bin/bash
# 元のプログラムと新しいアルゴリズムを、同じ模擬コースで走らせて比べる（tools/course_sim/README.md を参照）
#   使い方: tools/course_sim/compare.sh [名前=値 ...]
#     名前=値 は車とコースの本当の値（run.sh と同じ）。例: tools/course_sim/compare.sh grip_lat=7.2
#   クロスライン→クランク、ハーフライン→中央線の切れ目の距離を 0.5m と 1.0m にした場合を試す
#   （config.h の CROSS_TO_CRANK_M / HALF_TO_LANE_M もコースに合わせて書き換える）
#   どれもスタートの位置を7通りに変えた平均（run.sh の avg=1）
HERE=$(cd "$(dirname "$0")" && pwd)
CL="USE_NEW_CRANK_BRAKE=1,USE_NEW_CRANK_TURN=1,USE_NEW_LANE_BRAKE=1,USE_NEW_LANE_STEER=1"
for d in 0.5 1.0; do
  echo "=== クロスライン/ハーフラインから ${d} m ==="
  for mode in off "off,$CL" all; do
    echo "--- $mode"
    "$HERE/run.sh" "$mode,CROSS_TO_CRANK_M=${d}f,HALF_TO_LANE_M=${d}f" crank_l crank_r lane_l lane_r cross_to=$d half_to=$d avg=1 "$@"
  done
done
for r in 0.8 1.0 1.5; do
  echo "=== 半径 ${r} m のカーブ（直線 1.5m の後）==="
  for mode in off all; do
    echo "--- $mode"
    "$HERE/run.sh" "$mode" curve_l curve_r curve_r=$r avg=1 "$@"
  done
done
for r in 0.8 1.0; do
  echo "=== 半径 ${r} m の S字（90° ずつ、直線 1.5m の後）==="
  for mode in off all; do
    echo "--- $mode"
    "$HERE/run.sh" "$mode" s_l s_r s_rad=$r avg=1 "$@"
  done
done
exit 0
