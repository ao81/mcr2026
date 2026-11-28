#!/usr/bin/env python3
# release/release.ino をブラウザで動かすシミュレータ（tools/course_sim/release_simulator.html。
# GitHub Actions は public ブランチの release-sim/index.html）を作る（tools/course_sim/README.md の 4-7 を参照）
#   release.ino と course_sim.cpp を WebAssembly にして、release_template.html に埋め込む。
#   release.ino 自体は変えない。コピーを次のように書き換えてからビルドする
#     ・先頭の const int / const unsigned int の値を、画面から変えられる変数にする
#     ・AGTCallback の名前を release_AGTCallback に変える（release_sim/stub.cpp が呼んで、表示用の値を取り出す）
#   車の値（コースと車の初期値）は release_sim/config.h から読む。
#   必要なもの: build.py と同じ（python3、wasm32-wasi に対応した clang、wasi-libc、wasm32 用の libc++）
import base64, datetime, json, os, re, shutil, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROOT = os.path.abspath(os.path.join(SIM, "..", ".."))
INO = os.path.join(ROOT, "release", "release.ino")
REL = os.path.join(SIM, "release_sim")
OUT = os.environ.get("SIM_OUT") or os.path.join(SIM, "release_simulator.html")
CLANG = os.environ.get("CLANG", "clang++")

# 画面の見出し（名前の先頭で分ける）
SECTIONS = [("CRANK_", "クランク"), ("RIGHT_LANE_", "右レーンチェンジ"), ("HALF_LINE_", "右レーンチェンジ")]
# course_sim.cpp も使う、release.ino の #define（同じ値を -D で渡す）
HW_DEFINES = ("PWM_CYCLE_M", "SERVO_CENTER", "HANDLE_STEP")
# release_sim/config.h から画面の初期値として読む名前
CAR_NAMES = ("V_MAX_MPS", "MOTOR_TAU_S", "MOTOR_FRICTION_PERCENT", "GRIP_LAT_MPS2", "GRIP_LONG_MPS2", "SERVO_SPEED_DPS",
             "SERVO_DEAD_S", "SENSOR_TO_REAR_AXLE_M", "CAR_FRONT_M", "CAR_HALF_WIDTH_M", "SIM_DRIVE_DEFAULT",
             "CROSS_TO_CRANK_M", "HALF_TO_LANE_M", "WHEELBASE_M")


def convert_ino(text):
    """先頭の const int / const unsigned int NAME = 値; を web_cfg[] の参照にする。画面に出す項目の一覧も返す"""
    lines = text.split("\n")
    items, out = [], []
    for i, line in enumerate(lines):
        m = re.match(r"^const\s+(unsigned\s+)?int\s+([A-Z][A-Z0-9_]*)\s*=\s*(-?[0-9]+)\s*;\s*(?://\s*(.*))?$", line)
        if not m:
            out.append(line)
            continue
        unsigned, name, val, comment = m.group(1), m.group(2), m.group(3), (m.group(4) or "").strip()
        if not comment:
            # すぐ上の // の行（続けて書いた説明）
            j, above = i - 1, []
            while j >= 0 and re.match(r"^//", lines[j]) and not re.match(r"^//\s*=", lines[j]):
                above.insert(0, lines[j][2:].strip())
                j -= 1
            comment = " ".join(above)
        section = next((s for p, s in SECTIONS if name.startswith(p)), "そのほか")
        idx = len(items)
        items.append({"name": name, "kind": "int", "def": float(val), "section": section, "comment": comment,
                      "unsigned": bool(unsigned)})
        out.append("#define %s ((%s)web_cfg[%d])" % (name, "unsigned int" if unsigned else "int", idx))
    return "\n".join(out), items


def read_defines(text, names):
    vals = {}
    for n in names:
        m = re.search(r"^#define\s+%s\s+\(?\s*(-?[0-9]+(?:\.[0-9]*)?)f?\s*\)?" % n, text, re.M)
        if m:
            vals[n] = float(m.group(1))
    return vals


def main():
    with open(INO, encoding="utf-8") as f:
        ino = f.read()
    with open(os.path.join(REL, "config.h"), encoding="utf-8") as f:
        car = read_defines(f.read(), CAR_NAMES)
    hw = read_defines(ino, HW_DEFINES)
    missing = [n for n in HW_DEFINES if n not in hw]
    if missing:
        raise SystemExit("release.ino に #define %s がありません" % ", ".join(missing))
    src, items = convert_ino(ino)
    tmp = tempfile.mkdtemp()
    try:
        with open(os.path.join(tmp, "sketch.cpp"), "w", encoding="utf-8") as f:
            # AGTCallback は release_sim/stub.cpp が同じ名前で持つので、release.ino の方の名前を変える（このファイルの中だけ）
            f.write('#include "mock_arduino.h"\nextern float web_cfg[];\n#define AGTCallback release_AGTCallback\n')
            f.write(src)
        with open(os.path.join(tmp, "web_cfg.cpp"), "w", encoding="utf-8") as f:
            f.write("float web_cfg[%d] = {%s};\n" % (max(1, len(items)), ", ".join(repr(it["def"]) + "f" for it in items) or "0"))
            f.write('extern "C" __attribute__((export_name("cfg_ptr"))) float* cfg_ptr() { return web_cfg; }\n')
            f.write('#include <stdlib.h>\nextern "C" void* __cxa_allocate_exception(unsigned long) { abort(); }\n')
            f.write('extern "C" void __cxa_throw(void*, void*, void (*)(void*)) { abort(); }\n')
        wasm = os.path.join(tmp, "sim.wasm")
        cmd = [CLANG, "--target=wasm32-wasi", "-std=gnu++17", "-O2", "-w", "-fno-exceptions",
               "-I" + os.path.join(ROOT, "tools", "host_test", "mock"), "-I" + REL]
        cmd += ["-D%s=%d" % (n, int(v)) for n, v in hw.items()]
        cmd += ["-o", wasm, os.path.join(tmp, "sketch.cpp"), os.path.join(SIM, "course_sim.cpp"),
                os.path.join(REL, "stub.cpp"), os.path.join(tmp, "web_cfg.cpp"), "-Wl,--strip-all"]
        r = subprocess.run(cmd)
        if r.returncode:
            raise SystemExit("ビルドに失敗しました")
        with open(wasm, "rb") as f:
            wasm_b64 = base64.b64encode(f.read()).decode()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    try:
        rev = subprocess.run(["git", "-C", ROOT, "log", "-1", "--format=%h %cd", "--date=format:%Y-%m-%d", "--", "release/release.ino"],
                             capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", ROOT, "status", "--porcelain", "release/release.ino"],
                               capture_output=True, text=True).stdout.strip()
        if dirty:
            rev += "（コミットしていない変更あり）"
    except OSError:
        rev = "不明"
    info = {"built": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S" if os.environ.get("SIM_LOCAL") else "%Y-%m-%d"), "rev": rev or "不明"}
    with open(os.path.join(HERE, "release_template.html"), encoding="utf-8") as f:
        page = f.read()
    data = "const WASM_B64 = \"%s\";\nconst CFG_ITEMS = %s;\nconst CAR_DEFS = %s;\nconst BUILD = %s;" % (
        wasm_b64, json.dumps(items, ensure_ascii=False), json.dumps(car, ensure_ascii=False), json.dumps(info, ensure_ascii=False))
    if "// __BUILD_DATA__" not in page:
        raise SystemExit("release_template.html に // __BUILD_DATA__ の行がありません")
    page = page.replace("// __BUILD_DATA__", data.replace("</", "<\\/"), 1)
    os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(page)
    print("書き出しました: %s（%d KB、設定項目 %d）" % (OUT, len(page) // 1024, len(items)))


if __name__ == "__main__":
    main()
