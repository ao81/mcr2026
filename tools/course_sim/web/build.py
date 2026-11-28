#!/usr/bin/env python3
# ブラウザで動くシミュレータ（tools/course_sim/simulator.html。GitHub Actions は public ブランチの simulator/index.html）を作る（tools/course_sim/README.md を参照）
#   basic/ と course_sim.cpp を WebAssembly にして、simulator_template.html に埋め込む。
#   basic/ のコピーを次のように書き換えてからビルドする（basic/ 自体は変えない）
#     ・config.h の「新しい走行アルゴリズム」より下の数値の #define を、画面から変えられる変数にする
#     ・#if USE_NEW_... 〜 #endif を if (...) { 〜 } にする（スイッチを画面から切り替えるため）
#     ・static_assert を消す（同じ確認を画面で行う）
#   必要なもの: python3、clang（wasm32-wasi に対応したもの）、wasi-libc、wasm32 用の libc++
#     Ubuntu: apt install clang lld wasi-libc libc++-18-dev-wasm32 libclang-rt-18-dev-wasm32
import base64, datetime, json, os, re, shutil, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROOT = os.path.abspath(os.path.join(SIM, "..", ".."))
# 書き出す場所。GitHub Actions は環境変数 SIM_OUT で public ブランチの simulator/index.html に書く
OUT = os.environ.get("SIM_OUT") or os.path.join(SIM, "simulator.html")
CLANG = os.environ.get("CLANG", "clang++")

NUM = r"\(?\s*-?[0-9]+(?:\.[0-9]*)?f?\s*\)?"
# USE_NEW_TRACTION より上にあるが、画面から変えられるようにする値
EXTRA = {"AUTO_STOP_MS"}


def convert_config(text):
    """config.h の数値の #define を web_cfg[] の参照にする。画面に出す項目の一覧も返す"""
    lines = text.split("\n")
    start = next(i for i, l in enumerate(lines) if "#define USE_NEW_" in l)
    items, section, out = [], "", []
    for i, line in enumerate(lines):
        m = re.match(r"^// (.+)$", line)
        if m and i > 0 and lines[i - 1].startswith("//====="):
            section = m.group(1).strip()
        m = re.match(r"^#define\s+([A-Z0-9_]+)\s+(" + NUM + r")(\s*//\s*(.*))?$", line)
        if m and (i >= start or m.group(1) in EXTRA):
            if not m.group(1).startswith("LOG_"):
                name, val = m.group(1), m.group(2)
                v = val.replace("(", "").replace(")", "").replace("f", "").strip()
                kind = "switch" if name.startswith("USE_NEW_") else ("float" if "." in v else "int")
                comment = (m.group(4) or "").strip()
                # 次の行から続く説明（行頭が空白と // だけの行）
                j = i + 1
                while j < len(lines) and re.match(r"^\s+//", lines[j]):
                    comment += " " + lines[j].strip()[2:].strip()
                    j += 1
                if name == "COURSE_LIST_USE":
                    # 模擬のコースはクランク・レーンチェンジが1つずつなので、初めは表を使わない（画面の値を使う）
                    v = "0"
                idx = len(items)
                items.append({"name": name, "kind": kind, "def": float(v), "section": "スイッチ" if kind == "switch" else section, "comment": comment})
                cast = "(float)" if kind == "float" else "(int)"
                out.append("#define %s (%sweb_cfg[%d])" % (name, cast, idx))
                continue
        out.append(line)
    text = "\n".join(out)
    text = text.replace("#define MCR_CONFIG_H", "#define MCR_CONFIG_H\nextern float web_cfg[];", 1)
    return text, items


def convert_switch_ifs(text, names=()):
    """#if USE_NEW_... と、画面から変える値（names）を使う #if を if 文にする（ほかの #if はそのまま）"""
    runtime_re = re.compile(r"\b(USE_NEW_[A-Z0-9_]*|%s)\b" % "|".join(map(re.escape, names))) if names else re.compile(r"\bUSE_NEW_")
    out, stack = [], []
    for line in text.split("\n"):
        s = line.strip()
        m = re.match(r"^#\s*if\s+(.*)$", s)
        if m and runtime_re.search(m.group(1)):
            out.append("if (%s) {" % m.group(1)); stack.append(True); continue
        if re.match(r"^#\s*if(n?def)?\b", s):
            out.append(line); stack.append(False); continue
        m = re.match(r"^#\s*elif\s+(.*)$", s)
        if m and stack and stack[-1]:
            out.append("} else if (%s) {" % m.group(1)); continue
        if m and runtime_re.search(m.group(1)):
            raise SystemExit("ふつうの #if のあとの、画面から変える値を使う #elif... には対応していません: " + s)
        if re.match(r"^#\s*else\b", s):
            out.append("} else {" if stack and stack[-1] else line); continue
        if re.match(r"^#\s*endif\b", s):
            out.append("}" if stack.pop() else line); continue
        out.append(line)
    # static_assert(...); を消す（複数行にまたがる）
    text = "\n".join(out)
    text = re.sub(r"static_assert\s*\((?:[^;]|\n)*?\);", "/* static_assert は画面で確認する */", text)
    return text


def main():
    tmp = tempfile.mkdtemp()
    try:
        b = os.path.join(tmp, "basic")
        shutil.copytree(os.path.join(ROOT, "basic"), b)
        with open(os.path.join(b, "config.h"), encoding="utf-8") as f:
            cfg, items = convert_config(f.read())
        with open(os.path.join(b, "config.h"), "w", encoding="utf-8") as f:
            f.write(cfg)
        inos = sorted(n for n in os.listdir(b) if n.endswith(".ino") and n != "basic.ino")
        src = ['#include "mock_arduino.h"']
        for n in ["basic.ino"] + inos:
            with open(os.path.join(b, n), encoding="utf-8") as f:
                src.append(convert_switch_ifs(f.read(), [it["name"] for it in items]))
        with open(os.path.join(tmp, "sketch.cpp"), "w", encoding="utf-8") as f:
            f.write("\n".join(src))
        with open(os.path.join(tmp, "web_cfg.cpp"), "w", encoding="utf-8") as f:
            f.write("float web_cfg[%d] = {%s};\n" % (len(items), ", ".join(repr(it["def"]) + "f" for it in items)))
            f.write('extern "C" __attribute__((export_name("cfg_ptr"))) float* cfg_ptr() { return web_cfg; }\n')
            # Ubuntu の wasm32 用 libc++ は例外の関数を参照するが、例外は使わないので止めるだけにする
            f.write('#include <stdlib.h>\nextern "C" void* __cxa_allocate_exception(unsigned long) { abort(); }\n')
            f.write('extern "C" void __cxa_throw(void*, void*, void (*)(void*)) { abort(); }\n')
        wasm = os.path.join(tmp, "sim.wasm")
        cmd = [CLANG, "--target=wasm32-wasi", "-std=gnu++17", "-O2", "-w", "-fno-exceptions",
               "-I" + os.path.join(ROOT, "tools", "host_test", "mock"), "-I" + b,
               "-o", wasm, os.path.join(tmp, "sketch.cpp"), os.path.join(SIM, "course_sim.cpp"), os.path.join(tmp, "web_cfg.cpp"),
               "-Wl,--strip-all"]
        r = subprocess.run(cmd)
        if r.returncode:
            raise SystemExit("ビルドに失敗しました")
        with open(wasm, "rb") as f:
            wasm_b64 = base64.b64encode(f.read()).decode()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    try:
        # basic/ を最後に変えたコミット（画面の上に出す）
        rev = subprocess.run(["git", "-C", ROOT, "log", "-1", "--format=%h %cd", "--date=format:%Y-%m-%d", "--", "basic"],
                             capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", ROOT, "status", "--porcelain", "basic"],
                               capture_output=True, text=True).stdout.strip()
        if dirty:
            rev += "（コミットしていない変更あり）"
    except OSError:
        rev = "不明"
    # tools/local_site.py（ローカル版）は SIM_LOCAL=1 で、作った時刻も出す
    info = {"built": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S" if os.environ.get("SIM_LOCAL") else "%Y-%m-%d"), "rev": rev}
    with open(os.path.join(HERE, "simulator_template.html"), encoding="utf-8") as f:
        page = f.read()
    data = "const WASM_B64 = \"%s\";\nconst CFG_ITEMS = %s;\nconst BUILD = %s;" % (
        wasm_b64, json.dumps(items, ensure_ascii=False), json.dumps(info, ensure_ascii=False))
    if "// __BUILD_DATA__" not in page:
        raise SystemExit("simulator_template.html に // __BUILD_DATA__ の行がありません")
    page = page.replace("// __BUILD_DATA__", data.replace("</", "<\\/"), 1)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(page)
    if os.environ.get("SIM_JSON"):
        # tools/local_site.py が、GitHub Pages のシミュレータへ手元の版を渡すために使う
        with open(os.environ["SIM_JSON"], "w", encoding="utf-8") as f:
            json.dump({"wasm": wasm_b64, "items": items, "build": info}, f, ensure_ascii=False)
    print("書き出しました: %s（%d KB、設定項目 %d）" % (OUT, len(page) // 1024, len(items)))


if __name__ == "__main__":
    main()
