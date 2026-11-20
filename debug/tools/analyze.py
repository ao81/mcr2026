#!/usr/bin/env python3
# デバッグモードの詳しいログ（dbgNNNN.csv）を読んで、区間の流れと次に変える値の候補を出す
#   使い方:
#     python3 debug/tools/analyze.py DBG0001.CSV
#     python3 debug/tools/analyze.py DBG0001.CSV --sim sim.csv      模擬（debug/tools/sim_dbg.sh）と並べる
#     python3 debug/tools/analyze.py DBG0001.CSV --html dbg0001.html グラフを HTML で書き出す（ブラウザで開く）
#   Python 3 だけで動く（追加のライブラリは要らない）
import argparse
import html
import sys

PT_NAMES = {
    20: "通常走行", 100: "クロスライン検出", 101: "クロスライン通過中", 102: "減速(判定前)", 103: "減速・クランク探し",
    110: "左クランク突入", 111: "左クランク低速旋回", 112: "左クランク円弧", 120: "左クランク復帰(線を見失った)",
    121: "左クランク外へはみ出し", 150: "右クランク突入", 151: "右クランク低速旋回", 152: "右クランク円弧",
    160: "右クランク復帰(線を見失った)", 161: "右クランク外へはみ出し",
    200: "右ハーフライン検出", 201: "右ハーフライン通過中", 202: "右レーンチェンジ段階決め",
    500: "左ハーフライン検出", 501: "左ハーフライン通過中", 502: "左レーンチェンジ段階決め",
    800: "自動停止", 801: "完全停止(コースアウト等)", 1001: "プッシュスイッチで終了",
}
REASONS = {1: "区間が終わった", 2: "停止した（コースアウト・自動停止・プッシュスイッチ）", 3: "時間指定の終わり", 4: "模擬の終わり"}


def pt_name(p):
    if p in PT_NAMES:
        return PT_NAMES[p]
    if 20 <= p < 30:
        return "通常走行 段階%d" % (p - 20)
    if 30 <= p < 40:
        return "右大曲がり"
    if 40 <= p < 50:
        return "左大曲がり"
    for base, side in ((300, "右"), (600, "左")):
        if base <= p < base + 160:
            kind = ["直進(切れ目待ち)", "ハンドルを切る", "切り終わって直進", "S字円弧", "S字後に線探し", "新しい線に合わせる", "終了処理", "外へふくらんだ"][(p - base) // 20]
            return "%sレーンチェンジ %s" % (side, kind)
    return str(p)


class Log:
    def __init__(self, path):
        self.path = path
        self.head = {}
        self.param = {}
        self.switch = {}
        self.rows = []
        cols = None
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip().strip('"')
                if not line:
                    continue
                if line.startswith("#"):
                    a = line[1:].split(",")
                    if a[0] == "param":
                        self.param[a[1]] = float(a[2])
                    elif a[0] == "switch":
                        for i in range(1, len(a) - 1, 2):
                            self.switch[a[i]] = int(a[i + 1])
                    else:
                        self.head[a[0]] = a[1:]
                    continue
                if cols is None:
                    cols = line.split(",")
                    continue
                v = line.split(",")
                if len(v) < len(cols):
                    continue
                r = {}
                for k, x in zip(cols, v):
                    x = x.strip().lstrip("'")
                    if k == "sensor":
                        r[k] = x.zfill(8)
                    else:
                        r[k] = int(float(x))
                r["steer"] = r["steer_deg10"] / 10.0
                r["servo"] = r["servo_deg10"] / 10.0
                r["v"] = r["speed_mm_s"] / 1000.0
                r["yaw"] = r["yaw_deg10"] / 10.0
                self.rows.append(r)
        if not self.rows:
            sys.exit("%s にデータの行がありません" % path)
        self.target = int(self.head.get("target", ["1"])[0])
        rs = self.head.get("reason", ["0", "events", "0", "trigger_ms", "0", "end_ms", "0"])
        self.reason = int(rs[0])
        self.trigger = int(rs[4]) if len(rs) > 4 else 0
        self.end = int(rs[6]) if len(rs) > 6 else 0
        if self.trigger == 0:
            self.trigger = self.rows[0]["ms"]


def first(rows, cond):
    for r in rows:
        if cond(r):
            return r
    return None


def segments(rows):
    out = []
    for r in rows:
        if not out or out[-1][0] != r["pattern"]:
            out.append([r["pattern"], r, r])
        else:
            out[-1][2] = r
    return out


def crank_summary(L):
    """クランクの区間の数値をまとめる。dict を返す"""
    P = L.param
    rows = [r for r in L.rows if r["ms"] >= L.trigger]
    s = {}
    if not rows or not (100 <= rows[0]["pattern"] <= 161):
        return s
    t0 = L.trigger
    s["t0"] = t0
    pre = [r for r in L.rows if r["ms"] < t0]
    s["v_before"] = pre[-1]["v"] if pre else rows[0]["v"]
    # クロスライン通過時間と、測った速さで推定がどれだけ変わったか
    p102 = first(rows, lambda r: r["pattern"] >= 102)
    if p102:
        s["line_pass_ms"] = p102["line_pass_ms"]
        s["t_cross_done"] = p102["ms"] - t0
        before = [r for r in rows if r["ms"] < p102["ms"]]
        s["v_model_before_meas"] = before[-1]["v"] if before else None
        s["v_after_meas"] = p102["v"]
        s["vmax_before"] = before[-1]["vmax_mm_s"] / 1000.0 if before else None
        s["vmax_after"] = p102["vmax_mm_s"] / 1000.0
    b = first(rows, lambda r: r["brake"] == 1 or r["motor_l"] < 0 or r["motor_r"] < 0)
    if b:
        s["t_brake"] = b["ms"] - t0
        s["v_brake"] = b["v"]
        s["dist_brake"] = b["dist_mm"] / 1000.0
    turn = first(rows, lambda r: r["pattern"] in (110, 150, 112, 152))
    if turn:
        s["turn_ptn"] = turn["pattern"]
        s["t_turn_found"] = turn["ms"] - t0
        s["v_turn_found"] = turn["v"]
        before = [r for r in rows if r["ms"] <= turn["ms"]]
        s["v_min_before_turn"] = min(r["v"] for r in before)
        # ハンドルを切り始めた所
        st = first([r for r in rows if r["ms"] >= turn["ms"]], lambda r: abs(r["steer"]) >= 5.0)
        if st:
            s["t_steer"] = st["ms"] - t0
            s["v_steer"] = st["v"]
            s["dist_steer"] = st["dist_mm"] / 1000.0
        uw = first(rows, lambda r: r["unwind"] == 1)
        if uw:
            s["t_unwind"] = uw["ms"] - t0
            s["yaw_unwind"] = uw["yaw"]
            s["v_unwind"] = uw["v"]
    out = first(rows, lambda r: r["pattern"] in (120, 121, 160, 161))
    if out:
        s["trouble_ptn"] = out["pattern"]
        s["t_trouble"] = out["ms"] - t0
    back = first(rows, lambda r: 20 <= r["pattern"] < 30 and r["ms"] > t0 + 5)
    if back:
        s["t_exit"] = back["ms"] - t0
        s["v_exit"] = back["v"]
    stop = first(rows, lambda r: r["pattern"] >= 800)
    if stop:
        s["t_stop"] = stop["ms"] - t0
        s["stop_ptn"] = stop["pattern"]
    s["model_off"] = any(r["model_ok"] == 0 for r in rows)
    return s


def fmt(x, unit="", nd=3):
    if x is None:
        return "-"
    if isinstance(x, float):
        return ("%." + str(nd) + "f%s") % (x, unit)
    return "%s%s" % (x, unit)


def print_timeline(L):
    print("■ pattern の移り変わり（時間はクロスライン等を見つけた時を 0 とした ms）")
    print("   開始    終了   pattern                         速さ[m/s]始→終   距離[m]始→終  モータ左/右(始)  ハンドル指令[度](始)")
    for p, a, b in segments(L.rows):
        print("  %6d %6d  %4d %-26s  %5.2f → %5.2f    %6.3f→%6.3f   %4d/%4d        %6.1f" % (
            a["ms"] - L.trigger, b["ms"] - L.trigger, p, pt_name(p), a["v"], b["v"],
            a["dist_mm"] / 1000.0, b["dist_mm"] / 1000.0, a["motor_l"], a["motor_r"], a["steer"]))


def advice_crank(L, s):
    P = L.param
    tips = []
    if not s:
        return ["クランクの区間が記録に入っていません（DBG_TARGET・DBG_TARGET_NO と、区間が DBG_SAMPLES に収まっているかを確認）"]
    if L.rows[0]["ms"] > L.trigger:
        tips.append("区間の始まりが記録から押し出されています。DBG_SAMPLES を増やすか DBG_INTERVAL_MS を 2 以上にしてください")
    vb, va = s.get("vmax_before"), s.get("vmax_after")
    if vb and va and abs(va / vb - 1) > 0.08:
        tips.append("クロスラインで測った結果、出力100%%の速さが %.2f → %.2f m/s に直されました（%.0f%%）。"
                    "config.h の V_MAX_MPS を %.2f にすると、最初から計算が合います（電池の状態で変わるので、何回か見て決める）"
                    % (vb, va, (va / vb - 1) * 100, va))
    if s.get("model_off"):
        tips.append("途中で速さの推定が当てにならないと判断され（model_ok=0）、元の表の動きに戻りました。"
                    "CROSS_LINE_WIDTH_M（見える幅）と V_MAX_MPS が実機と合っているか確認してください")
    lp = s.get("line_pass_ms")
    if lp and lp < 10:
        tips.append("クロスラインの通過時間が %d ms と短いので、1ms の数え方だけで速さが ±%.0f%% ずれます。数回の走行で比べてください" % (lp, 100.0 / lp))
    tp = s.get("trouble_ptn")
    vp = P.get("plan_speed_mps", 0)
    vs = s.get("v_steer")
    if s.get("turn_ptn") in (110, 150) and P.get("plan_speed_mps", 0) > 0 and L.switch.get("CRANK_TURN"):
        tips.append("円弧で曲がらず元の方法（pattern %d）で曲がりました。曲がり角を見つけた時の速さ %.2f m/s が計画 %.2f m/s の %.2f 倍（PLAN_OVERSPEED）を超えたためと考えられます。"
                    "ブレーキが遅いので CRANK_BRAKE_MARGIN_M を +0.03 してください"
                    % (s["turn_ptn"], s.get("v_turn_found", 0), vp, P.get("PLAN_OVERSPEED", 1.05)))
    if tp in (121, 161):
        if vs and vp and vs > vp * 1.03:
            tips.append("外へはみ出しました（pattern %d）。ハンドルを切り始めた時の速さ %.2f m/s が計画 %.2f m/s より %.0f%% 速いので、"
                        "減速が足りません。CRANK_BRAKE_MARGIN_M を +0.03、または PLAN_GRIP_RATIO_LONG を -0.05 してください"
                        % (tp, vs, vp, (vs / vp - 1) * 100))
        else:
            tips.append("外へはみ出しました（pattern %d）が、速さは計画どおりでした。曲がる速さ自体が速すぎるので、PLAN_GRIP_RATIO_LAT を -0.05（0.90→0.85）するか、"
                        "GRIP_LAT_MPS2 を測り直してください。曲がり始めが遅い場合は SENSOR_TO_REAR_AXLE_M の実測も確認してください" % tp)
    elif tp in (120, 160):
        tips.append("曲がっている途中で線を見失いました（pattern %d）。ハンドルを戻し始めた時の向きの変化 %s 度を見て、90 度より大きければ戻すのが遅く、"
                    "小さければ早すぎます" % (tp, fmt(s.get("yaw_unwind"), "", 1)))
    if s.get("stop_ptn") == 801:
        tips.append("コースアウトで止まりました（全センサが白）。止まる直前の pattern を表で見てください")
    if not tp and vs and vp and vs < vp * 0.9:
        tips.append("曲がり始めの速さ %.2f m/s が計画 %.2f m/s より %.0f%% 遅く、手前で減速しすぎています。CRANK_BRAKE_MARGIN_M を -0.02（%.2f → %.2f）してください"
                    % (vs, vp, (1 - vs / vp) * 100, P.get("CRANK_BRAKE_MARGIN_M", 0.05), max(0.0, P.get("CRANK_BRAKE_MARGIN_M", 0.05) - 0.02)))
    if not tp and s.get("t_exit") and vs and vp and abs(vs / vp - 1) <= 0.1:
        tips.append("計画どおりの速さで曲がり、通常走行に戻りました。もっと速くするなら PLAN_GRIP_RATIO_LAT を +0.03 ずつ上げて、はみ出さないか確かめてください")
    return tips


def print_crank(L, s, title):
    print("■ クランクのまとめ（%s）" % title)
    if not s:
        print("  クランクの区間が見つかりません")
        return
    rows = [
        ("クロスラインの手前の速さ（推定）", fmt(s.get("v_before"), " m/s", 2)),
        ("クロスラインの通過時間", fmt(s.get("line_pass_ms"), " ms")),
        ("通過時間で直す前 → 後の速さ", "%s → %s" % (fmt(s.get("v_model_before_meas"), "", 2), fmt(s.get("v_after_meas"), " m/s", 2))),
        ("出力100%の速さ 直す前 → 後", "%s → %s" % (fmt(s.get("vmax_before"), "", 2), fmt(s.get("vmax_after"), " m/s", 2))),
        ("ブレーキを始めた時間・速さ・距離", "%s / %s / %s" % (fmt(s.get("t_brake"), " ms"), fmt(s.get("v_brake"), " m/s", 2), fmt(s.get("dist_brake"), " m"))),
        ("曲がり角を見つけた時間・速さ", "%s / %s（pattern %s）" % (fmt(s.get("t_turn_found"), " ms"), fmt(s.get("v_turn_found"), " m/s", 2), fmt(s.get("turn_ptn")))),
        ("曲がり角までの一番遅い速さ", fmt(s.get("v_min_before_turn"), " m/s", 2)),
        ("ハンドルを切り始めた時間・速さ・距離", "%s / %s / %s" % (fmt(s.get("t_steer"), " ms"), fmt(s.get("v_steer"), " m/s", 2), fmt(s.get("dist_steer"), " m"))),
        ("計画の速さ・直進する距離・ハンドル角", "%s / %s / %s" % (fmt(L.param.get("plan_speed_mps"), " m/s", 2), fmt(L.param.get("plan_wait_m"), " m"), fmt(L.param.get("plan_angle"), " 度", 1))),
        ("ハンドルを戻し始めた時間・向きの変化", "%s / %s" % (fmt(s.get("t_unwind"), " ms"), fmt(s.get("yaw_unwind"), " 度", 1))),
        ("はみ出し・見失い", "%s（%s ms）" % (pt_name(s["trouble_ptn"]), s["t_trouble"]) if s.get("trouble_ptn") else "なし"),
        ("通常走行に戻った時間・速さ", "%s / %s" % (fmt(s.get("t_exit"), " ms"), fmt(s.get("v_exit"), " m/s", 2))),
        ("止まった", "%s（%s ms）" % (pt_name(s["stop_ptn"]), s["t_stop"]) if s.get("stop_ptn") else "いいえ"),
    ]
    for k, v in rows:
        print("  %-24s %s" % (k, v))


def compare(L, S):
    print("■ 実機と模擬の比較（クロスラインを見つけた時を 0 ms として 20ms ごと）")
    print("    時間   pattern 実機/模擬    速さ[m/s] 実機/模擬   モータ左 実機/模擬  ハンドル指令[度] 実機/模擬")

    def at(log, t):
        best = None
        for r in log.rows:
            if r["ms"] - log.trigger <= t:
                best = r
            else:
                break
        return best
    t_end = max(L.rows[-1]["ms"] - L.trigger, S.rows[-1]["ms"] - S.trigger)
    t = 0
    while t <= t_end:
        a, b = at(L, t), at(S, t)
        if a or b:
            fa = lambda r, k, f: (f % r[k]) if r else "  -  "
            print("  %6d    %4s / %-4s        %5s / %-5s          %4s / %-4s        %6s / %-6s" % (
                t, fa(a, "pattern", "%d"), fa(b, "pattern", "%d"), fa(a, "v", "%.2f"), fa(b, "v", "%.2f"),
                fa(a, "motor_l", "%d"), fa(b, "motor_l", "%d"), fa(a, "steer", "%.1f"), fa(b, "steer", "%.1f")))
        t += 20
    print("  模擬の方が速さ・時間が実機とずれている所は、模擬の車の値（v_max・grip_lat・tau・servo_dps など）が実機と違います。")
    print("  模擬の値を変えて実機に近づけてから、模擬で config.h の値を試すと、実機で試す回数を減らせます。")


def write_html(path, logs):
    # 時間を横軸に、速さ・モータ出力・ハンドル角を縦に並べた SVG
    W, H, PAD = 900, 170, 40
    charts = [
        ("速さ [m/s]", lambda r: r["v"], None),
        ("モータ出力 左(実線)・右(点線) [%]", lambda r: r["motor_l"], lambda r: r["motor_r"]),
        ("ハンドル 指令(実線)・推定(点線) [度]", lambda r: r["steer"], lambda r: r["servo"]),
        ("pattern", lambda r: r["pattern"], None),
    ]
    colors = ["#2563eb", "#dc2626"]
    t_lo = min(min(r["ms"] for r in L.rows) - L.trigger for L in logs)
    t_hi = max(max(r["ms"] for r in L.rows) - L.trigger for L in logs)
    if t_hi <= t_lo:
        t_hi = t_lo + 1
    parts = ["<!doctype html><meta charset='utf-8'><title>詳しいログ</title>",
             "<style>body{font-family:sans-serif;background:#fff;color:#111;margin:16px}svg{background:#fafafa;border:1px solid #ddd;margin:4px 0;max-width:100%}</style>",
             "<h2>詳しいログ</h2><p>横軸：区間に入った時を 0 とした時間 [ms]。"]
    for i, L in enumerate(logs):
        parts.append("<span style='color:%s'>■ %s</span> " % (colors[i % 2], html.escape(L.path)))
    parts.append("</p>")
    for title, f1, f2 in charts:
        vals = [f(r) for L in logs for r in L.rows for f in (f1, f2) if f]
        lo, hi = min(vals), max(vals)
        if hi <= lo:
            hi = lo + 1
        sx = lambda t: PAD + (t - t_lo) / (t_hi - t_lo) * (W - 2 * PAD)
        sy = lambda v: H - 20 - (v - lo) / (hi - lo) * (H - 40)
        g = ["<div>%s</div><svg viewBox='0 0 %d %d' width='%d'>" % (html.escape(title), W, H, W)]
        g.append("<line x1='%d' y1='%d' x2='%d' y2='%d' stroke='#999'/>" % (sx(0), 10, sx(0), H - 20))
        for v in (lo, hi):
            g.append("<text x='2' y='%.0f' font-size='10'>%.2f</text>" % (sy(v) + 3, v))
        for tt in range(int(t_lo // 100 * 100), int(t_hi) + 1, 100):
            g.append("<text x='%.0f' y='%d' font-size='10' text-anchor='middle'>%d</text>" % (sx(tt), H - 5, tt))
        for i, L in enumerate(logs):
            for f, dash in ((f1, ""), (f2, "4,3")):
                if not f:
                    continue
                pts = " ".join("%.1f,%.1f" % (sx(r["ms"] - L.trigger), sy(f(r))) for r in L.rows)
                g.append("<polyline fill='none' stroke='%s' stroke-width='1.3' stroke-dasharray='%s' points='%s'/>" % (colors[i % 2], dash, pts))
        g.append("</svg>")
        parts.append("".join(g))
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    print("グラフを %s に書きました（ブラウザで開く）" % path)


def main():
    ap = argparse.ArgumentParser(description="デバッグモードの詳しいログを読む")
    ap.add_argument("log", help="dbgNNNN.csv")
    ap.add_argument("--sim", help="模擬のログ（debug/tools/sim_dbg.sh で作る）。並べて比べる")
    ap.add_argument("--html", help="グラフを書き出す HTML ファイル")
    ap.add_argument("--no-table", action="store_true", help="pattern の移り変わりの表を出さない")
    a = ap.parse_args()
    L = Log(a.log)
    print("ファイル: %s" % a.log)
    print("記録を止めた理由: %s / 記録した時間 %d〜%d ms（%d 行）" % (REASONS.get(L.reason, L.reason), L.rows[0]["ms"], L.rows[-1]["ms"], len(L.rows)))
    print("スイッチ: " + " ".join("%s=%d" % kv for kv in L.switch.items()))
    print()
    if not a.no_table:
        print_timeline(L)
        print()
    tips = []
    if L.target == 1:
        s = crank_summary(L)
        print_crank(L, s, "実機" if a.sim else a.log)
        tips = advice_crank(L, s)
        if a.sim:
            S = Log(a.sim)
            print()
            print_crank(S, crank_summary(S), "模擬")
    else:
        print("（レーンチェンジ・時間指定の自動のまとめはまだありません。上の表とグラフで見てください）")
    print()
    print("■ 次に試す値の候補")
    if not tips:
        print("  特になし")
    for i, t in enumerate(tips, 1):
        print("  %d. %s" % (i, t))
    print("  ※ 1回に変える値は1つにして、変えた値とログの番号を記録しておくと、効果が分かります")
    if a.sim:
        print()
        compare(L, Log(a.sim))
    if a.html:
        write_html(a.html, [L] + ([Log(a.sim)] if a.sim else []))


if __name__ == "__main__":
    main()
