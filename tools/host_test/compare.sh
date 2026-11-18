#!/bin/bash
# 元のプログラム（reference/basic_original.ino）と basic/ の新しいプログラムを、
# 同じ入力で動かして出力が完全に一致するかを調べる。
#   使い方: tools/host_test/compare.sh [試す入力パターンの数(既定 3000)]
# 一致しない入力があると、その seed と最初の違いを表示する。
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
N=${1:-3000}
OUT=$(mktemp -d)
"$HERE/build.sh" "$HERE/reference/basic_original.ino" "$OUT/orig"
"$HERE/build.sh" "$HERE/../../basic" "$OUT/new"
fail=0
for s in $(seq 1 "$N"); do
  "$OUT/orig" "$s" 9000 > "$OUT/o.txt"
  "$OUT/new"  "$s" 9000 > "$OUT/n.txt"
  if ! cmp -s "$OUT/o.txt" "$OUT/n.txt"; then
    echo "不一致 seed=$s"; diff "$OUT/o.txt" "$OUT/n.txt" | head -4
    fail=$((fail + 1))
  fi
  grep -oE ' p=[0-9]+' "$OUT/o.txt" | sort -u >> "$OUT/cov.txt"
done
echo "試した数: $N  不一致: $fail"
echo "通った pattern: $(sort -u "$OUT/cov.txt" | sed 's/ p=//' | sort -n | tr '\n' ' ')"
rm -rf "$OUT"
[ "$fail" -eq 0 ]
