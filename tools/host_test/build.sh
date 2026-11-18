#!/bin/sh
# スケッチのフォルダ（または .ino ファイル1つ）を PC 用にビルドする
#   使い方: build.sh <スケッチのフォルダ or .inoファイル> <出力ファイル名>
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$1; OUT=$2
TMP=$(mktemp -d)
{
  echo '#include "mock_arduino.h"'
  if [ -d "$SRC" ]; then
    MAIN="$SRC/$(basename "$SRC").ino"
    cat "$MAIN"
    for f in $(ls "$SRC"/*.ino | sort); do [ "$f" = "$MAIN" ] || cat "$f"; done
  else
    cat "$SRC"
  fi
} > "$TMP/sketch.cpp"
INC=$SRC; [ -d "$SRC" ] || INC=$(dirname "$SRC")
g++ -std=gnu++17 -O1 -w -I"$HERE/mock" -I"$INC" -o "$OUT" "$TMP/sketch.cpp" "$HERE/sim.cpp"
rm -rf "$TMP"
