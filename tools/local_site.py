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
  ・GitHub Pages のシミュレータ（https://ao81.github.io/mcr2026/simulator/）の「手元のファイル」→「手元のサーバから読む」も、
    このサーバの /api/sim から手元の版を読む（このサーバを動かしている間だけ）。
ページの HTML は、起動時に git fetch した origin/public から読む（読めなければ GitHub から取ってくる）。
"""
import argparse
import hashlib
import html
import json
import os
import re
import shutil
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


SIM = {"hash": None}
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


def build_sim():
    """手元の basic/ からシミュレータを作る（変わっていなければ前に作ったものを使う）。SIM を返す"""
    with SIM_LOCK:
        h = sources_hash()
        if SIM["hash"] == h:
            return SIM
        print("シミュレータを手元の basic/ から作っています…", flush=True)
        tmp = tempfile.mkdtemp()
        out, js = os.path.join(tmp, "sim.html"), os.path.join(tmp, "sim.json")
        result = {"hash": h, "ok": False, "html": None, "json": None, "error": ""}
        try:
            env = dict(os.environ, SIM_OUT=out, SIM_JSON=js, SIM_LOCAL="1")
            r = subprocess.run([sys.executable, str(BUILD_PY)], env=env, capture_output=True, text=True, timeout=600)
            if r.returncode == 0:
                result.update(ok=True, html=Path(out).read_bytes(), json=json.loads(Path(js).read_text(encoding="utf-8")))
                print("作りました", flush=True)
            else:
                result["error"] = "走行プログラムのビルドに失敗しました:\n" + (r.stdout + r.stderr).strip()
                print(result["error"], flush=True)
        except (OSError, subprocess.SubprocessError) as exc:
            result["error"] = "走行プログラムのビルドを実行できませんでした: %s" % exc
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        SIM.clear()
        SIM.update(result)
        return SIM


def simulator_page():
    sim = build_sim()
    if sim["ok"]:
        return banner(sim["html"], "ローカル版：手元の basic/ の走行プログラムと config.h の値で作ったシミュレータです。"
                      "basic/ を変えたら、このページを再読み込みすると作り直します。")
    return local_defaults_page(sim["error"])


# GitHub Pages のシミュレータ（「手元のファイル」→「手元のサーバから読む」）から読めるようにする
PAGES_ORIGIN = "https://ao81.github.io"


class Site(ce.Editor):
    def cors(self):
        """GitHub Pages のページからの読み込みだけ許す"""
        if self.headers.get("Origin") != PAGES_ORIGIN:
            return False
        self.send_header("Access-Control-Allow-Origin", PAGES_ORIGIN)
        self.send_header("Vary", "Origin")
        return True

    def do_OPTIONS(self):
        if self.path.split("?")[0] != "/api/sim" or not self.allowed() or self.headers.get("Origin") != PAGES_ORIGIN:
            return self.reply(403, {})
        self.send_response(204)
        self.cors()
        self.send_header("Access-Control-Allow-Methods", "GET")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Access-Control-Max-Age", "600")
        self.end_headers()

    def api_sim(self):
        """手元の版の走行プログラム（WebAssembly）と設定項目を JSON で返す。have が今の版と同じなら same だけ"""
        have = ""
        m = re.search(r"[?&]have=([0-9a-f]*)", self.path)
        if m:
            have = m.group(1)
        if have and have == sources_hash():
            body = {"same": True}
            status = 200
        else:
            sim = build_sim()
            if sim["ok"]:
                body, status = dict(sim["json"], hash=sim["hash"]), 200
            else:
                body, status = {"error": "手元の basic/ から走行プログラムを作れませんでした", "detail": sim["error"][-6000:]}, 422
        data = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(status)
        self.cors()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

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
        if path == "/api/sim":
            return self.api_sim()
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
        threading.Thread(target=build_sim, daemon=True).start()  # 先に作っておく
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
