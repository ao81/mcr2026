"""道具のページ（GitHub Pages と同じ 選択ページ・config・log・simulator）を、手元のファイルで動かすローカル版。
Python 3 標準ライブラリのみ。WSL の ~/mcr2026 で

    python3 tools/local_site.py

を実行し、表示された http://127.0.0.1:8770/ をブラウザで開く（Ctrl+C で止める）。

  ・パラメータ編集（/config/）：手元の basic/config.h・basic/tables.h・debug/dbg_config.h を読み書きする
    （tools/config_editor.py と同じ。保存すると debug/ も basic/ に合わせる）
  ・シミュレータ（/simulator/）：手元の basic/ の走行プログラムと config.h の値で作る。
    basic/ などが変わっていれば、ページを開き直した（再読み込みした）時に作り直す（10〜30 秒）。
    作るには WebAssembly 用の clang が要る（Ubuntu 24.04:
      sudo apt install clang lld wasi-libc libc++-18-dev-wasm32 libclang-rt-18-dev-wasm32）。
    clang が無い・コンパイルに失敗した時は、GitHub の走行プログラムに手元の config.h の値を初期値として入れて開く
    （画面の上に理由を出す）。
  ・詳細ログ（/log/）と選択ページ：ページは public ブランチのものをそのまま使う（CSV はページで選ぶ）。
ページの HTML は、起動時に git fetch した origin/public から読む（読めなければ GitHub から取ってくる）。
"""
import argparse
import hashlib
import html
import json
import os
import re
import subprocess
import sys
import tempfile
import threading
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import config_editor as ce  # noqa: E402

ROOT = ce.ROOT
SIM_DIR = ROOT / "tools" / "course_sim"
BUILD_PY = SIM_DIR / "web" / "build.py"
# これらが変わったらシミュレータを作り直す
SIM_SOURCES = (ROOT / "basic", SIM_DIR / "course_sim.cpp", SIM_DIR / "web", ROOT / "tools" / "host_test" / "mock")
RAW_URL = "https://raw.githubusercontent.com/ao81/mcr2026/public/"
PAGE_FILES = ("index.html", "config/index.html", "config/trial.js", "log/index.html", "simulator/index.html")
PAGES = {}
ROUTES = {
    "/": ("index.html", "text/html"),
    "/index.html": ("index.html", "text/html"),
    "/config/": ("config/index.html", "text/html"),
    "/config/index.html": ("config/index.html", "text/html"),
    "/config/trial.js": ("config/trial.js", "text/javascript"),
    "/log/": ("log/index.html", "text/html"),
    "/log/index.html": ("log/index.html", "text/html"),
}


def load_pages():
    try:
        subprocess.run(["git", "-C", str(ROOT), "fetch", "-q", "origin", "public"], timeout=30,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for n in PAGE_FILES:
            PAGES[n] = subprocess.run(["git", "-C", str(ROOT), "show", "origin/public:" + n],
                                      capture_output=True, check=True, timeout=20).stdout
        return "git の origin/public"
    except (OSError, subprocess.SubprocessError):
        pass
    for n in PAGE_FILES:
        with urllib.request.urlopen(RAW_URL + n, timeout=30) as r:
            PAGES[n] = r.read()
    return RAW_URL


def banner(page, text, error=""):
    """ページの一番上に「ローカル版」の帯を入れる"""
    color = "#b3261e" if error else "#2f6fd6"
    bar = ('<div style="background:%s;color:#fff;font:13px/1.5 system-ui,\'Yu Gothic UI\',Meiryo,sans-serif;'
           'padding:6px 12px;flex:none">%s' % (color, html.escape(text)))
    if error:
        bar += ('<details style="margin-top:4px"><summary style="cursor:pointer">理由を見る</summary>'
                '<pre style="white-space:pre-wrap;max-height:40vh;overflow:auto;margin:4px 0 0;'
                'background:#0006;padding:6px">%s</pre></details>' % html.escape(error[-6000:]))
    bar += "</div>"
    text = page.decode("utf-8")
    m = re.search(r"<body[^>]*>", text)
    if not m:
        return page
    return (text[:m.end()] + bar + text[m.end():]).encode("utf-8")


def sources_hash():
    digest = hashlib.sha256()
    for base in SIM_SOURCES:
        files = [base] if base.is_file() else sorted(p for p in base.rglob("*") if p.is_file())
        for p in files:
            digest.update(p.relative_to(ROOT).as_posix().encode() + b"\0" + hashlib.sha256(p.read_bytes()).digest())
    return digest.hexdigest()


SIM = {"hash": None, "page": None}
SIM_LOCK = threading.Lock()


def local_defaults_page(reason):
    """clang が使えない時：GitHub のシミュレータに、手元の config.h の値を初期値として入れる"""
    sys.path.insert(0, str(BUILD_PY.parent))
    import build  # tools/course_sim/web/build.py
    text = PAGES["simulator/index.html"].decode("utf-8")
    m = re.search(r"const CFG_ITEMS = (.*);\n", text)
    if not m:
        raise ValueError("GitHub のシミュレータに CFG_ITEMS が見つかりません")
    items = json.loads(m.group(1))
    _, local = build.convert_config((ROOT / "basic" / "config.h").read_text(encoding="utf-8"))
    local = {it["name"]: it for it in local}
    changed, missing = [], []
    for it in items:
        lo = local.pop(it["name"], None)
        if lo is None:
            missing.append(it["name"])
        elif lo["def"] != it["def"]:
            changed.append("%s %g → %g" % (it["name"], it["def"], lo["def"]))
            it["def"] = lo["def"]
    data = json.dumps(items, ensure_ascii=False).replace("</", "<\\/")
    page = (text[:m.start(1)] + data + text[m.end(1):]).encode("utf-8")
    notes = [reason, "", "手元の config.h の値に変えた初期値（%d 個）:" % len(changed)] + changed
    if local:
        notes += ["", "手元の config.h にあるが GitHub の走行プログラムに無い値（使われない）: " + ", ".join(sorted(local))]
    if missing:
        notes += ["", "GitHub の走行プログラムにあるが手元の config.h に無い値（GitHub の値のまま）: " + ", ".join(missing)]
    return banner(page, "ローカル版：走行プログラムを手元の basic/ から作れなかったので、GitHub の走行プログラムに"
                  "手元の config.h の値を初期値として入れています。プログラムの変更は入っていません。", "\n".join(notes))


def simulator_page():
    with SIM_LOCK:
        h = sources_hash()
        if SIM["hash"] == h:
            return SIM["page"]
        print("シミュレータを手元の basic/ から作っています…", flush=True)
        fd, out = tempfile.mkstemp(suffix=".html")
        os.close(fd)
        try:
            env = dict(os.environ, SIM_OUT=out, SIM_LOCAL="1")
            r = subprocess.run([sys.executable, str(BUILD_PY)], env=env, capture_output=True, text=True, timeout=600)
            if r.returncode == 0:
                page = banner(Path(out).read_bytes(), "ローカル版：手元の basic/ の走行プログラムと config.h の値で作ったシミュレータです。"
                              "basic/ を変えたら、このページを再読み込みすると作り直します。")
                print("作りました", flush=True)
            else:
                print(r.stdout + r.stderr, flush=True)
                page = local_defaults_page("走行プログラムのビルドに失敗しました:\n" + (r.stdout + r.stderr).strip())
        except (OSError, subprocess.SubprocessError) as exc:
            page = local_defaults_page("走行プログラムのビルドを実行できませんでした: %s" % exc)
        finally:
            Path(out).unlink(missing_ok=True)
        SIM["hash"], SIM["page"] = h, page
        return page


class Site(ce.Editor):
    def do_GET(self):
        host = self.headers.get("Host", "")
        if host == "localhost:%d" % self.server.server_port:
            # パラメータ編集は 127.0.0.1 で開いた時だけローカル保存になるので、こちらに移す
            self.send_response(302)
            self.send_header("Location", "http://127.0.0.1:%d%s" % (self.server.server_port, self.path))
            self.end_headers()
            return
        if not self.allowed():
            return self.reply(403, {})
        path = self.path.split("?")[0]
        if path in ("/config", "/log", "/simulator"):
            self.send_response(301)
            self.send_header("Location", path + "/")
            self.end_headers()
            return
        if path == "/api/config":
            return super().do_GET()
        if path in ("/simulator/", "/simulator/index.html"):
            try:
                return self.reply(200, simulator_page(), "text/html")
            except Exception as exc:  # 画面に理由を出す
                return self.reply(500, ("シミュレータを用意できませんでした: %s" % exc).encode(), "text/plain")
        if path in ROUTES:
            name, kind = ROUTES[path]
            page = PAGES[name]
            if kind == "text/html":
                page = banner(page, "ローカル版：手元の %s のファイルを使っています（GitHub のファイルではありません）。" % ROOT)
            return self.reply(200, page, kind)
        self.reply(404, {})

    def log_message(self, fmt, *args):
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="道具のページを手元のファイルで動かす")
    parser.add_argument("--port", type=int, default=8770)
    args = parser.parse_args()
    print("ページを読みました: " + load_pages(), flush=True)
    print("使うフォルダ: %s" % ROOT, flush=True)
    with ThreadingHTTPServer(("127.0.0.1", args.port), Site) as server:
        print(f"ブラウザで開く: http://127.0.0.1:{server.server_port}/  （止めるときは Ctrl+C）", flush=True)
        threading.Thread(target=simulator_page, daemon=True).start()  # 先に作っておく
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
