"""mcr2026 のローカル編集・スケッチ同期。Python 3 標準ライブラリのみ。"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EDITABLE = ("basic/config.h", "basic/tables.h", "debug/dbg_config.h")
OWN = {"dbglog.ino", "dbg_config.h"}
HOOKS = (
    ('#include "config.h"', '#include "dbg_config.h"       // デバッグモードの設定（debug/ だけ）\nvoid dbg_sample(void);           // dbglog.ino\nvoid dbg_task(void);             // dbglog.ino'),
    ('speed_model_update();', '    dbg_sample();                        // デバッグモード：詳しいログを RAM に貯める（dbglog.ino）'),
    ('log_task();', '    dbg_task();             // デバッグモード：走り終わった後に dbgNNNN.csv へ書き出す（dbglog.ino）'),
)


def sketch(source):
    eol = b"\r\n" if b"\r\n" in source else b"\n"
    lines = source.decode().splitlines()
    for anchor, extra in HOOKS:
        hits = [i for i, line in enumerate(lines) if line.split("//")[0].strip() == anchor]
        if len(hits) != 1:
            raise ValueError("同期位置を特定できません: " + anchor)
        lines[hits[0] + 1:hits[0] + 1] = extra.splitlines()
    return eol.join(line.encode() for line in lines) + eol


def snapshot():
    return {p.relative_to(ROOT).as_posix(): p.read_bytes()
            for folder in ("basic", "debug") for p in (ROOT / folder).iterdir()
            if p.suffix in (".h", ".ino")}


def revision(files):
    digest = hashlib.sha256()
    for name, data in sorted(files.items()):
        digest.update(name.encode() + b"\0" + hashlib.sha256(data).digest())
    return digest.hexdigest()


def desired(files, edits):
    result = files.copy()
    for name, value in edits.items():
        if name not in EDITABLE or not isinstance(value, str):
            raise ValueError("不正な編集対象")
        result[name] = value.encode()
    # 生成対象以外（ユーザー独自の debug タブを含む）はそのまま残す。
    for name, data in list(result.items()):
        if not name.startswith("basic/"):
            continue
        base = name.split("/")[1]
        if base in OWN or base == "debug.ino":
            raise ValueError("debug 専用ファイルと重複: " + name)
        dst = "debug/" + ("debug.ino" if base == "basic.ino" else base)
        result[dst] = sketch(data) if base == "basic.ino" else data
    return result


def replace(name, data):
    target = ROOT / name
    if data is None:
        target.unlink(missing_ok=True)
        return
    with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as tmp:
        temp = Path(tmp.name)
        try:
            tmp.write(data)
        except BaseException:
            tmp.close()
            temp.unlink(missing_ok=True)
            raise
    try:
        os.replace(temp, target)
    finally:
        temp.unlink(missing_ok=True)


def sync(old, edits):
    new = desired(old, edits)
    changed = sorted(k for k in new.keys() | old.keys() if new.get(k) != old.get(k))
    done = []
    try:
        for name in changed:
            target = ROOT / name
            if (target.read_bytes() if target.exists() else None) != old.get(name):
                raise ValueError("同期中に変更されました: " + name)
            replace(name, new.get(name))
            done.append(name)
    except BaseException:
        for name in reversed(done):
            replace(name, old.get(name))
        raise
    return new


PAGE_URL = "https://raw.githubusercontent.com/ao81/mcr2026/public/config/"
PAGES = {}


def load_pages():
    """編集ページ（public ブランチの config/index.html と trial.js）を読む。
    手元の git に public ブランチがあればそれを使い、なければ GitHub から取ってくる"""
    names = ("index.html", "trial.js")
    try:
        subprocess.run(["git", "-C", str(ROOT), "fetch", "-q", "origin", "public"], timeout=20,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for n in names:
            PAGES[n] = subprocess.run(["git", "-C", str(ROOT), "show", "origin/public:config/" + n],
                                      capture_output=True, check=True, timeout=20).stdout
        return "git の origin/public"
    except (OSError, subprocess.SubprocessError):
        pass
    for n in names:
        with urllib.request.urlopen(PAGE_URL + n, timeout=20) as r:
            PAGES[n] = r.read()
    return PAGE_URL


def state(files):
    return dict(revision=revision(files), needsSync=desired(files, {}) != files,
                files=[dict(path=k, text=files[k].decode()) for k in EDITABLE])


class Editor(BaseHTTPRequestHandler):
    def reply(self, status, body, kind="application/json"):
        self.send_response(status)
        self.send_header("Content-Type", kind + "; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        self.wfile.write(body if isinstance(body, bytes) else json.dumps(body).encode())

    def allowed(self):
        return self.headers.get("Host") == f"127.0.0.1:{self.server.server_port}"

    def do_GET(self):
        if not self.allowed():
            return self.reply(403, {})
        if self.path == "/":
            return self.reply(200, PAGES["index.html"], "text/html")
        if self.path == "/trial.js":
            return self.reply(200, PAGES["trial.js"], "text/javascript")
        if self.path == "/api/config":
            try:
                return self.reply(200, state(snapshot()))
            except (OSError, ValueError, KeyError) as exc:
                return self.reply(422, dict(error=str(exc)))
        self.reply(404, {})

    def do_POST(self):
        if (not self.allowed() or self.headers.get("Origin") != "http://" + self.headers["Host"]
                or self.headers.get_content_type() != "application/json"):
            return self.reply(403, {})
        if self.path != "/api/config":
            return self.reply(404, {})
        try:
            size = int(self.headers.get("Content-Length", 0))
            if not 0 < size <= 512000:
                raise ValueError("リクエストサイズが不正です")
            request = json.loads(self.rfile.read(size))
            old = snapshot()
            if request["revision"] != revision(old):
                return self.reply(409, dict(error="外部変更があります。再読み込みしてから編集してください。"))
            if not isinstance(request["edits"], dict):
                raise ValueError("編集内容が不正です")
            self.reply(200, state(sync(old, request["edits"])))
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.reply(422, dict(error=str(exc)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sync", action="store_true", help="basic から debug を生成して終了")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if args.sync:
        sync(snapshot(), {})
    else:
        print("編集ページを読みました: " + load_pages(), flush=True)
        with HTTPServer(("127.0.0.1", args.port), Editor) as server:
            print(f"Brave / Chrome / Edge: http://127.0.0.1:{server.server_port}/", flush=True)
            server.serve_forever()
