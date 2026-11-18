#!/bin/bash
# 模擬の走りを、ブラウザで動かして見られる HTML にする（tools/course_sim/README.md を参照）
#   使い方: tools/course_sim/view.sh <コース名> [<スイッチ> ...] [名前=値 ...]
#     <コース名>  crank_l crank_r lane_l lane_r curve_l curve_r のどれか1つ
#     <スイッチ>  run.sh と同じ書き方（off / all で始める）。並べた数だけ走らせて、重ねて表示する
#                 省略すると off と all の2つ
#     名前=値    車とコースの本当の値（run.sh と同じ）。すべての走りに同じ値を使う
#                dump=N  N ms ごとに記録する（初期値 2）
#                out=ファイル名  書き出す先（初期値 tools/course_sim/out/<コース名>.html）
#   例: tools/course_sim/view.sh crank_l off all
#       tools/course_sim/view.sh lane_r all all,USE_NEW_LANE_STEER=0,USE_NEW_LANE_BRAKE=0 grip_lat=6
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
exec python3 - "$HERE" "$@" <<'PY'
import json, os, subprocess, sys

here, args = sys.argv[1], sys.argv[2:]
if not args or "=" in args[0] or args[0].startswith(("off", "all")):
    sys.exit("1つ目にコース名を書いてください。例: view.sh crank_l off all")
course, switches, opts, dump, out = args[0], [], [], 2, None
for a in args[1:]:
    if a.startswith(("off", "all")):
        switches.append(a)
    elif a.startswith("dump="):
        dump = int(a[5:])
    elif a.startswith("out="):
        out = a[4:]
    elif a.startswith("avg="):
        print("avg は view.sh では使えないので無視します", file=sys.stderr)
    elif "=" in a:
        opts.append(a)
    else:
        sys.exit("分からない指定です: " + a)
if not switches:
    switches = ["off", "all"]
if dump < 1:
    sys.exit("dump は 1 以上にしてください")
if out is None:
    out = os.path.join(here, "out", course + ".html")

data = {"dump": dump, "opts": " ".join(opts), "course": None, "car": None, "runs": []}
for sw in switches:
    print("模擬中: " + sw, file=sys.stderr)
    p = subprocess.run([os.path.join(here, "run.sh"), sw, course] + opts + ["dump=%d" % dump],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    samples, result = [], ""
    for line in p.stdout.splitlines():
        if line.startswith("#COURSE "):
            data["course"] = json.loads(line[8:])
        elif line.startswith("#CAR "):
            data["car"] = json.loads(line[5:])
        elif line.startswith("#S "):
            samples.append([float(v) if "." in v else int(v) for v in line[3:].split(",")])
        elif line.strip():
            result = line.strip()
    if not samples:
        sys.exit("模擬が動きませんでした（" + sw + "）:\n" + p.stdout)
    print("  " + result, file=sys.stderr)
    data["runs"].append({"label": sw, "result": result, "out": "コースアウト" in result, "s": samples})

with open(os.path.join(here, "viewer.html"), encoding="utf-8") as f:
    page = f.read()
if "// __DATA__" not in page:
    sys.exit("viewer.html に // __DATA__ の行がありません")
js = "const DATA = " + json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/") + ";"
page = page.replace("// __DATA__", js, 1)
os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
with open(out, "w", encoding="utf-8") as f:
    f.write(page)
print("書き出しました: " + os.path.abspath(out), file=sys.stderr)
PY
