#!/bin/bash
# basic/ の走行プログラムを debug/ にコピーして、デバッグモード（詳しいログ）を足す
#   使い方: debug/sync.sh        （basic/ を直したら毎回実行する）
#
#   debug/ は Arduino のスケッチ（debug.ino が本体）。basic/ のファイルをすべてコピーし、
#   basic.ino を debug.ino という名前にして、次の3か所だけ書き足す。
#     ① #include "config.h" の後に #include "dbg_config.h" と関数の宣言
#     ② 1ms 割り込みの speed_model_update(); の後に dbg_sample();
#     ③ loop の log_task(); の後に dbg_task();
#   dbglog.ino、dbg_config.h、sync.sh、README.md、tools/ は上書きしない。
#   debug/ のコピーしたファイル（config.h など）を直接直しても、次に実行した時に basic/ の内容に戻る。
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../basic"
OWN="dbglog.ino dbg_config.h"

# basic/ にもうないファイルを消す（debug/ だけのファイルは残す）
for f in "$HERE"/*.ino "$HERE"/*.h; do
  [ -e "$f" ] || continue
  b=$(basename "$f")
  case " $OWN debug.ino " in *" $b "*) continue ;; esac
  [ -e "$SRC/$b" ] || rm -f "$f"
done

for f in "$SRC"/*.ino "$SRC"/*.h; do
  b=$(basename "$f")
  case " $OWN " in *" $b "*) echo "basic/ に $b があります。debug/ の同じ名前のファイルと重なるので止めます"; exit 2 ;; esac
  [ "$b" = basic.ino ] && continue
  cp "$f" "$HERE/$b"
done

# 書き足す場所がちょうど1か所ずつあるか確かめる
check() {
  n=$(grep -cE "$1" "$SRC/basic.ino" || true)
  if [ "$n" != 1 ]; then
    echo "basic.ino に「$2」が $n か所あります（1か所のはず）。debug/sync.sh の書き足す場所を直してください"
    exit 2
  fi
}
check '^#include "config.h"' '#include "config.h"'
check '^[[:space:]]*speed_model_update\(\);' 'speed_model_update();'
check '^[[:space:]]*log_task\(\);' 'log_task();'

sed -E \
  -e 's|^#include "config.h".*|&\n#include "dbg_config.h"       // デバッグモードの設定（debug/ だけ）\nvoid dbg_sample(void);           // dbglog.ino\nvoid dbg_task(void);             // dbglog.ino|' \
  -e 's|^([[:space:]]*)speed_model_update\(\);.*|&\n\1dbg_sample();                        // デバッグモード：詳しいログを RAM に貯める（dbglog.ino）|' \
  -e 's|^([[:space:]]*)log_task\(\);.*|&\n\1dbg_task();             // デバッグモード：走り終わった後に dbgNNNN.csv へ書き出す（dbglog.ino）|' \
  "$SRC/basic.ino" > "$HERE/debug.ino"

src_rev=$(cd "$SRC" && git log -1 --format=%h -- . 2>/dev/null || echo "?")
echo "basic/（$src_rev）を debug/ にコピーしました。Arduino IDE で debug/debug.ino を開いて書き込んでください"
