// コースを模擬して、スケッチを PC 上で走らせるシミュレータ（テスト専用。実機では使わない）
//
//   使い方: course_sim <コース> [名前=値 ...]
//     コース: crank_l crank_r lane_l lane_r curve_l curve_r
//     名前=値: 車とコースの「本当の値」を変える（プログラム側の config.h の値はそのまま）
//       v_max tau grip_lat grip_long servo_dps servo_dead line_w cross_to half_to lane_sec lane_new curve_r
//       sensor_to_axle half_w front ms trace spot dip approach dump
//     dump=N : N ms ごとの状態を、視覚的なシミュレータ（view.sh / viewer.html）が読む形で出力する
//
//   車の動き（後輪の軸の中心で計算）
//     ・速さ : dv/dt = (出力の平均/100 × v_max − v) ÷ tau
//     ・向き : 曲率 = tan(ハンドル角度) ÷ ホイールベース
//     ・摩擦円 : 横の加速度と前後の加速度が (横/grip_lat)² + (前後/grip_long)² ≦ 1 を超えるときは、両方を同じ割合で縮める
//               （曲がりきれずに外へふくらみ、ブレーキも弱くなる）
//     ・ハンドル : 指令の角度へ servo_dps [度/s] の速さで動く
//   センサ : 後輪の軸から sensor_to_axle 前、SENSOR_X_MM の位置。白線の上なら 1
//   コースアウト : 後輪・前輪の外側の点（横 half_w、前 0 と front）がコースの外に出たら失敗
#include "mock_arduino.h"
#include "mcr_gpt_lib.h"
#include "SD.h"
#include "config.h"
#include <vector>
#include <string>
#include <set>

MockPort mock_port[5];
MockGpt mock_gpt1;
MockSerial Serial;
SDClass SD;
std::map<std::string, std::string> mock_files;
bool mock_sd_ok = true;
volatile uint32_t GTIOC1B, GTIOC6B, GTIOC3A;

extern volatile int pattern;
extern volatile float speed_mps;
extern volatile float dist_m, plan_wait_m, plan_len_m, plan_speed_mps;
extern volatile float plan_angle;
void setup();
void loop();
void AGTCallback(timer_callback_args_t* p_args);

// ---------------- コースの形 ----------------
struct Rect { double x0, y0, x1, y1; };
struct Ring { double cx, cy, r0, r1, a0, a1; };   // 中心、内径、外径、角度の範囲[rad]
struct Shapes {
    std::vector<Rect> rects;
    std::vector<Ring> rings;
    bool inside(double x, double y, double grow) const {
        for (const Rect& r : rects)
            if (x >= r.x0 - grow && x <= r.x1 + grow && y >= r.y0 - grow && y <= r.y1 + grow) return true;
        for (const Ring& g : rings) {
            double dx = x - g.cx, dy = y - g.cy;
            double d = sqrt(dx * dx + dy * dy);
            double a = atan2(dy, dx);
            if (a < g.a0 - 1e-9) a += 2 * M_PI;
            if (d >= g.r0 - grow && d <= g.r1 + grow && a >= g.a0 && a <= g.a1) return true;
        }
        return false;
    }
};
struct Course {
    Shapes white;     // 白線
    Shapes surface;   // コースの面
    int kind = 0;     // 0:crank 1:lane 2:curve
    int dir = 1;      // +1 右, -1 左
    double finish = 0;
};

static std::map<std::string, double> P = {
    {"v_max", V_MAX_MPS}, {"tau", MOTOR_TAU_S}, {"grip_lat", GRIP_LAT_MPS2}, {"grip_long", GRIP_LONG_MPS2},
    {"servo_dps", SERVO_SPEED_DPS}, {"servo_dead", SERVO_DEAD_S}, {"line_w", 0.032}, {"cross_to", CROSS_TO_CRANK_M}, {"half_to", HALF_TO_LANE_M},
    {"lane_sec", LANE_SECTION_M}, {"lane_new", 0.0}, {"curve_r", 0.45}, {"sensor_to_axle", SENSOR_TO_REAR_AXLE_M},
    {"half_w", CAR_HALF_WIDTH_M}, {"front", CAR_FRONT_M}, {"ms", 6000}, {"trace", 0}, {"spot", 0.004},
    {"dip", 3}, {"approach", 1.5}, {"dump", 0},
};

static Course make_course(const std::string& name) {
    Course c;
    const double W = 0.15, LW = 0.01;            // コース幅の半分、中心線の幅の半分
    double y1 = P["approach"];                    // スタートから特別な線まで
    double lw = P["line_w"];
    c.dir = (name.back() == 'r') ? 1 : -1;
    if (name.rfind("crank", 0) == 0) {
        c.kind = 0;
        double y0 = y1 + lw + P["cross_to"] + LW;   // 曲がり角の線の中心
        c.white.rects.push_back({-W, y1, W, y1 + lw});                           // クロスライン
        c.white.rects.push_back({-LW, -1, LW, y0 + LW});                         // 中心線
        if (c.dir > 0) c.white.rects.push_back({-LW, y0 - LW, 4, y0 + LW});
        else           c.white.rects.push_back({-4, y0 - LW, LW, y0 + LW});
        c.surface.rects.push_back({-W, -1, W, y0 + W});
        if (c.dir > 0) c.surface.rects.push_back({-W, y0 - W, 4, y0 + W});
        else           c.surface.rects.push_back({-4, y0 - W, W, y0 + W});
        c.finish = 1.0;
    } else if (name.rfind("lane", 0) == 0) {
        c.kind = 1;
        double d = 0.30 * c.dir;
        double ye = y1 + lw + P["half_to"];         // 中央線が切れる所
        double ls = P["lane_sec"];
        if (c.dir > 0) c.white.rects.push_back({-LW, y1, W, y1 + lw});           // ハーフライン
        else           c.white.rects.push_back({-W, y1, LW, y1 + lw});
        c.white.rects.push_back({-LW, -1, LW, ye});
        c.white.rects.push_back({d - LW, ye + P["lane_new"], d + LW, ye + 4});    // 新しい車線の線
        c.surface.rects.push_back({-W, -1, W, ye});
        c.surface.rects.push_back({std::min(-W, d - W), ye, std::max(W, d + W), ye + ls});
        c.surface.rects.push_back({d - W, ye + ls, d + W, ye + 4});
        c.finish = ye + 2.0;
    } else {
        c.kind = 2;
        double rc = P["curve_r"];
        double cx = rc * c.dir;                       // 円の中心 (cx, y1)
        c.white.rects.push_back({-LW, -1, LW, y1});
        c.white.rings.push_back({cx, y1, rc - LW, rc + LW, 0, M_PI});
        c.white.rects.push_back({2 * cx - LW, -1, 2 * cx + LW, y1});
        c.surface.rects.push_back({-W, -1, W, y1});
        c.surface.rings.push_back({cx, y1, rc - W, rc + W, 0, M_PI});
        c.surface.rects.push_back({2 * cx - W, -1, 2 * cx + W, y1});
        c.finish = 0.3;
    }
    return c;
}

// 視覚的なシミュレータ用：形を JSON で出力する
static void dump_shapes(const char* key, const Shapes& sh) {
    printf("\"%s\":{\"rects\":[", key);
    for (size_t i = 0; i < sh.rects.size(); i++)
        printf("%s[%.4f,%.4f,%.4f,%.4f]", i ? "," : "", sh.rects[i].x0, sh.rects[i].y0, sh.rects[i].x1, sh.rects[i].y1);
    printf("],\"rings\":[");
    for (size_t i = 0; i < sh.rings.size(); i++)
        printf("%s[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]", i ? "," : "", sh.rings[i].cx, sh.rings[i].cy, sh.rings[i].r0, sh.rings[i].r1, sh.rings[i].a0, sh.rings[i].a1);
    printf("]}");
}

static void set_inputs(uint8_t s, bool push) {    // s: 1 = 白
    const int bitpos[8] = {4, 5, 6, 7, 8, 10, 11, 12};
    uint16_t v = 0xffff;
    for (int b = 0; b < 8; b++) if (s & (1 << b)) v &= ~(1u << bitpos[b]);
    MockPort& p = mock_port[0];
    p.PIDR_b.PIDR4 = (v >> 4) & 1; p.PIDR_b.PIDR5 = (v >> 5) & 1; p.PIDR_b.PIDR6 = (v >> 6) & 1;
    p.PIDR_b.PIDR7 = (v >> 7) & 1; p.PIDR_b.PIDR8 = (v >> 8) & 1; p.PIDR_b.PIDR10 = (v >> 10) & 1;
    p.PIDR_b.PIDR11 = (v >> 11) & 1; p.PIDR_b.PIDR12 = (v >> 12) & 1;
    p.PIDR_b.PIDR3 = !push;
}

static double motor_power(uint32_t duty, int dir_pin) {
    double p = duty * 100.0 / (PWM_CYCLE_M + 1);
    return dir_pin ? -p : p;
}

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: course_sim <course> [name=value ...]\n"); return 2; }
    std::string name = argv[1];
    for (int i = 2; i < argc; i++) {
        std::string a = argv[i];
        size_t e = a.find('=');
        if (e == std::string::npos || !P.count(a.substr(0, e))) { fprintf(stderr, "unknown option %s\n", argv[i]); return 2; }
        P[a.substr(0, e)] = atof(a.substr(e + 1).c_str());
    }
    Course c = make_course(name);
    const double wb = WHEELBASE_M, dt = 0.001;
    const double vmax = P["v_max"], tau = P["tau"], gy = P["grip_lat"], gx = P["grip_long"];
    const double sf = P["sensor_to_axle"], hw = P["half_w"], front = P["front"], spot = P["spot"];
    int dip = (int)P["dip"];
    mock_port[3].PIDR_b.PIDR6 = !(dip & 2);
    mock_port[3].PIDR_b.PIDR7 = !(dip & 1);

    double x = 0, y = 0, psi = M_PI / 2, v = 0, steer = 0;    // psi: 進行方向（+x から反時計回り）
    set_inputs(0x19, false);
    setup();

    const int dump = (int)P["dump"];
    if (dump > 0) {
        printf("#COURSE {\"name\":\"%s\",", name.c_str());
        dump_shapes("white", c.white);
        printf(",");
        dump_shapes("surface", c.surface);
        printf("}\n");
        printf("#CAR {\"wheelbase\":%.4f,\"tread\":%.4f,\"sensor\":%.4f,\"half_w\":%.4f,\"front\":%.4f,\"sx\":[", wb, TREAD_M, sf, hw, front);
        for (int b = 0; b < 8; b++) printf("%s%.4f", b ? "," : "", SENSOR_X_MM[b] * 0.001);
        printf("],\"grip_lat\":%.3f}\n", gy);
    }

    long t_start = -1, t_finish = -1, slip_ms = 0, t_out = -1;
    double v_peak = 0, v_event = -1, min_room = 1, lat_peak = 0;
    int event_ptn = -1;
    std::vector<int> seq;
    std::vector<double> cmd_hist;
    bool passed_mid = false;
    long total = (long)P["ms"];
    for (long t = 0; t < total; t++) {
        // センサ
        uint8_t s = 0;
        double hx = cos(psi), hy = sin(psi), rx = sin(psi), ry = -cos(psi);   // 前向き・右向き
        for (int b = 0; b < 8; b++) {
            double lat = SENSOR_X_MM[b] * 0.001;
            double px = x + sf * hx + lat * rx, py = y + sf * hy + lat * ry;
            if (c.white.inside(px, py, spot)) s |= (uint8_t)(0x80 >> b);
        }
        bool bar = (t < 400);
        if (bar) s |= 0x01;
        set_inputs(s, t >= 50 && t < 80);

        AGTCallback(0);
        loop();

        // 出力を読む
        double ang_cmd = ((double)GTIOC1B - SERVO_CENTER) / (SERVO_DIR * HANDLE_STEP);
        double pl = motor_power(GTIOC6B, mock_port[4].PODR_b.PODR0);
        double pr = motor_power(GTIOC3A, mock_port[4].PODR_b.PODR4);
        if (seq.empty() || seq.back() != pattern) seq.push_back((int)pattern);
        if (pattern >= 20 && t_start < 0) t_start = t;
        if (t_start < 0) continue;

        // ハンドル（servo_dead だけ遅れて指令が届く）
        cmd_hist.push_back(ang_cmd);
        size_t lag = (size_t)(P["servo_dead"] / dt + 0.5);
        if (cmd_hist.size() > lag + 1) cmd_hist.erase(cmd_hist.begin());
        ang_cmd = cmd_hist.front();
        double step = P["servo_dps"] * dt;
        if (ang_cmd > steer + step) steer += step; else if (ang_cmd < steer - step) steer -= step; else steer = ang_cmd;
        double k = tan(steer * M_PI / 180) / wb;       // + が右
        // 速さ
        double u = (pl + pr) * 0.5 / 100 * vmax;
        double a = (u - v) / tau;
        if (a > gx) a = gx;                              // タイヤが出せる前後の力は grip_long まで（それ以上は空回り・すべり）
        if (a < -gx) a = -gx;
        // 摩擦円：求められた横の加速度 v²k と前後の加速度 a が楕円 (横/grip_lat)² + (前後/grip_long)² ≦ 1 を
        // 超えるときは、両方を同じ割合で縮める（曲がりきれずに外へふくらみ、減速も弱くなる）
        double qlat = v * v * fabs(k) / gy, qlong = a / gx;
        double q = sqrt(qlat * qlat + qlong * qlong);
        if (v > 0.05 && qlat > lat_peak) lat_peak = qlat;
        if (q > 1.02) slip_ms++;                         // 2% を超えてグリップが足りない時間
        if (q > 1.0) { k /= q; a /= q; }
        v += a * dt;
        x += v * hx * dt; y += v * hy * dt;
        psi -= v * k * dt;                               // 右に曲がると psi は減る
        if (v > v_peak) v_peak = v;
        if (event_ptn < 0 && ((pattern >= 110 && pattern <= 161) || (pattern >= 320 && pattern <= 390) || (pattern >= 620 && pattern <= 690))) {
            event_ptn = pattern; v_event = v;
        }
        if (dump > 0 && t % dump == 0) {
            printf("#S %ld,%d,%u,%.4f,%.4f,%.2f,%.3f,%.1f,%.1f,%.0f,%.0f,%.3f,%.3f,%.3f\n",
                   t - t_start, (int)pattern, (unsigned)s, x, y, psi * 180 / M_PI, v, steer, ang_cmd, pl, pr,
                   (double)speed_mps, (double)dist_m, qlat);
        }
        // コースアウト判定（タイヤの外側の4点）
        double pts[4][2] = {{0, -hw}, {0, hw}, {front, -hw}, {front, hw}};
        bool out = false;
        for (auto& q : pts) {
            double px = x + q[0] * hx + q[1] * rx, py = y + q[0] * hy + q[1] * ry;
            if (!c.surface.inside(px, py, 0)) out = true;
        }
        if (out) {
            t_out = t - t_start;
            if ((int)P["trace"]) {
                for (auto& q : pts) {
                    double px = x + q[0] * hx + q[1] * rx, py = y + q[0] * hy + q[1] * ry;
                    printf("  角(前%.2f,横%.2f) x=%.3f y=%.3f %s\n", q[0], q[1], px, py, c.surface.inside(px, py, 0) ? "" : "← 外");
                }
            }
            break;
        }
        if ((int)P["trace"] && (t % 10 == 0))
            printf("t=%ld p=%d s=%02x x=%.3f y=%.3f psi=%.1f v=%.2f steer=%.1f cmd=%.0f pl=%.0f pr=%.0f est=%.2f dist=%.3f plan(wait=%.3f len=%.3f v=%.2f ang=%.1f)\n",
                   t - t_start, pattern, s, x, y, psi * 180 / M_PI, v, steer, ang_cmd, pl, pr, speed_mps, dist_m,
                   plan_wait_m, plan_len_m, plan_speed_mps, plan_angle);
        // ゴール
        bool fin = false;
        if (c.kind == 0) fin = (c.dir > 0) ? (x >= c.finish) : (x <= -c.finish);
        if (c.kind == 1) fin = (y >= c.finish);
        if (c.kind == 2) { if (y > P["approach"] + 0.2) passed_mid = true; fin = passed_mid && y <= c.finish; }
        if (fin) { t_finish = t - t_start; break; }
        if (pattern >= 800) break;
    }
    (void)min_room;
    printf("%-8s ", name.c_str());
    if (t_finish >= 0) printf("ゴール %5ld ms", t_finish);
    else if (t_out >= 0) printf("コースアウト %ld ms", t_out);
    else printf("ゴールできず");
    printf("  最高 %.2f m/s", v_peak);
    if (event_ptn >= 0) printf("  曲がり始め p=%d %.2f m/s", event_ptn, v_event);
    printf("  横G %3.0f%%  横すべり %ld ms  pattern:", lat_peak * 100, slip_ms);
    std::set<int> shown;
    for (int p : seq) if (p >= 100 && p < 800 && !shown.count(p)) { shown.insert(p); printf(" %d", p); }
    printf("\n");
    return (t_finish >= 0) ? 0 : 1;
}
