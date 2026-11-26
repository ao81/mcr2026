// コースを模擬して、スケッチを PC 上で走らせるシミュレータ（テスト専用。実機では使わない）
//
//   使い方: course_sim <コース> [名前=値 ...]
//     コース: crank_l crank_r lane_l lane_r curve_l curve_r s_l s_r crank2
//       crank2 : クランク（cross_to、ラインの幅 line_w、向き turn1）→ 直線 gap → クランク（cross_to2、line_w2、turn2）
//                （クランクごとに距離が違うコース。config.h の COURSE_LIST を試すときに使う。turn は +1 右、-1 左）
//       s_r / s_l : 直線 → 右（左）へ s_deg 度曲がる半径 s_rad の円弧 → 反対へ s_deg 度の円弧 → 直線（S字）
//     名前=値: 車とコースの「本当の値」を変える（プログラム側の config.h の値はそのまま）
//       v_max tau grip_lat grip_long servo_dps servo_dead line_w cross_to（クロスラインの終わりから曲がった後の中心線まで） half_to
//       lane_pre（合流部分の始まりから中心線の切れ目まで） lane_sec（切れ目から合流部分の終わりまで） lane_new curve_r s_rad（中心線の半径） s_deg
//       shoulder（1: 路肩の白 30mm あり） gray_white（1: 中心線の両側の灰色をセンサが白と見る）
//       cross_to2 line_w2 gap turn1 turn2（crank2）  fric（摩擦で走らない出力 [%]）
//       sensor_to_axle sensor_half_w（センサの基板の幅の半分。レーンチェンジでは腕と先も判定） half_w front ms trace spot dip approach dump
//     dump=N : N ms ごとの状態を、ブラウザのシミュレータ（simulator.html）が読む形で出力する
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
#include <array>
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
extern volatile float dist_m, plan_wait_m, plan_len_m, plan_speed_mps, plan_switch_m, lane_end_m;
extern volatile float plan_angle;
extern volatile float obs_z1, obs_z2, obs_z3, servo_est_deg;
void setup();
void loop();
void AGTCallback(timer_callback_args_t* p_args);

// ---------------- コースの形 ----------------
// 公式のコースの寸法（ジャパンマイコンカーラリー Basic Class 競技規則 第3条、キットの解説、JMCR2027 地区大会のコース図）
//   ・コースの幅 300mm。横から見た並び：路肩の白 30 ｜ 黒 ｜ 灰 10・中心線の白 20・灰 10 ｜ 黒 ｜ 路肩の白 30
//   ・クロスライン：白1本、幅 20〜40mm、コースの幅いっぱい。クランクの曲がり角は直角
//     模擬の cross_to は「クロスラインの終わりから、曲がった後の車線の中心線（の真ん中）まで」（ao 2026-11-26：
//     幅 20/30/40mm → 800/700/600mm。2027 のコース図では、その先コースの端まで 140）
//   ・ハーフライン：白1本、幅 20〜40mm、曲がる側の半分だけ。合流部分の 300〜1000mm 手前（コース図では 400）
//   ・レーンチェンジの合流部分（チェンジ区間）：長さ 600mm、幅 600mm（2つの車線が並んだ形）。中の中心線は 200 と 400 の2本で、
//     古い車線の線が切れた所から新しい車線の線が始まる。外側の路肩には白 30mm。角の2か所にポール
//   ・カーブ・S字：内側の端の半径 450mm 以上（中心線の半径 600mm 以上）
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
    Shapes white;     // 白（中心線・クロスライン・ハーフライン・路肩）
    Shapes gray;      // 中心線の両側の灰色（gray_white=1 のときだけセンサに白く見える）
    Shapes surface;   // コースの面
    int kind = 0;     // 0:crank 1:lane 2:curve 3:S字
    int dir = 1;      // +1 右, -1 左
    double finish = 0;
    double white_x = 0;   // crank2 : 2つ目の曲がり角の x
    double fin_sy = 1;    // crank2 : 2つ目の後に進む y の向き
};

static std::map<std::string, double> P = {
    {"v_max", V_MAX_MPS}, {"tau", MOTOR_TAU_S}, {"grip_lat", GRIP_LAT_MPS2}, {"grip_long", GRIP_LONG_MPS2},
    {"servo_dps", SERVO_SPEED_DPS}, {"servo_dead", SERVO_DEAD_S}, {"line_w", 0.032}, {"cross_to", CROSS_TO_CRANK_M}, {"half_to", HALF_TO_LANE_M},
    {"lane_sec", 0.4}, {"lane_new", 0.0}, {"lane_pre", 0.2}, {"curve_r", 0.60}, {"s_rad", 0.60}, {"s_deg", 90}, {"sensor_to_axle", SENSOR_TO_REAR_AXLE_M},
    {"half_w", CAR_HALF_WIDTH_M}, {"front", CAR_FRONT_M}, {"ms", 6000}, {"trace", 0}, {"spot", 0.004},
    {"dip", 3}, {"approach", 1.5}, {"dump", 0}, {"surf_w", 0.15},
    {"fric", MOTOR_FRICTION_PERCENT}, {"steer_gain", 1.0}, {"sensor_half_w", SENSOR_HALF_WIDTH_M}, {"cross_to2", 0.6}, {"line_w2", 0.032}, {"gap", 1.5}, {"turn1", 1}, {"turn2", -1},
    {"shoulder", 1}, {"gray_white", 0},
};

static const double CW = 0.150;    // コースの幅の半分
static const double CLW = 0.010;   // 中心線（白 20mm）の幅の半分
static const double CGW = 0.020;   // 中心線と両側の灰色（10mm ずつ）を合わせた幅の半分
static const double CSH = 0.030;   // 路肩の白の幅

static Rect box(double xa, double ya, double xb, double yb) {
    return Rect{std::min(xa, xb), std::min(ya, yb), std::max(xa, xb), std::max(ya, yb)};
}

// 直角に曲がるだけの道（x・y の向きの直線をつないだもの）。pts は中心線の折れ点、ext0・ext1 は始まりと終わりの先に延ばす長さ
// 曲がり角の外側は、路肩の白がコースの端まで回り込む（突き当たりの壁の側も白）。内側は、曲がった先の車線の路肩につながる
static void add_path(Course& c, const std::vector<std::array<double, 2>>& pts, double ext0, double ext1) {
    const int n = (int)pts.size() - 1;
    for (int i = 0; i < n; i++) {
        const double ax = pts[i][0], ay = pts[i][1], bx = pts[i + 1][0], by = pts[i + 1][1];
        const double L = hypot(bx - ax, by - ay), ux = (bx - ax) / L, uy = (by - ay) / L;
        const double nx = -uy, ny = ux;   // 左
        auto R = [&](double s0, double s1, double l0, double l1) {
            return box(ax + ux * s0 + nx * l0, ay + uy * s0 + ny * l0, ax + ux * s1 + nx * l1, ay + uy * s1 + ny * l1);
        };
        const bool first = (i == 0), last = (i == n - 1);
        const double s0 = first ? -ext0 : 0, s1 = last ? L + ext1 : L;
        c.surface.rects.push_back(R(first ? s0 : -CW, last ? s1 : L + CW, -CW, CW));
        c.gray.rects.push_back(R(first ? s0 : -CGW, last ? s1 : L + CGW, -CGW, CGW));
        c.white.rects.push_back(R(first ? s0 : -CLW, last ? s1 : L + CLW, -CLW, CLW));
        if (!P["shoulder"]) continue;
        for (int side = -1; side <= 1; side += 2) {   // -1 右、+1 左
            const double sx = nx * side, sy = ny * side;
            double t0 = s0, t1 = s1;
            if (!first) {   // 始まりの曲がり角：前の道の向きの側（突き当たりの側）は端から、反対側は内側の角から
                double px = pts[i][0] - pts[i - 1][0], py = pts[i][1] - pts[i - 1][1];
                t0 = (sx * px + sy * py > 0) ? -CW : CW - CSH;
            }
            if (!last) {    // 終わりの曲がり角：曲がる側は曲がった先の車線の手前で終わる。反対側は端まで
                double qx = pts[i + 2][0] - bx, qy = pts[i + 2][1] - by;
                t1 = (sx * qx + sy * qy > 0) ? L - CW : L + CW;
            }
            c.white.rects.push_back(side > 0 ? R(t0, t1, CW - CSH, CW) : R(t0, t1, -CW, -CW + CSH));
        }
    }
}

// 円弧の道（中心 (cx, cy)、中心線の半径 r、角度 a0〜a1）。sw はコースアウトを判定する面の幅の半分
static void add_arc(Course& c, double cx, double cy, double r, double a0, double a1, double sw) {
    c.surface.rings.push_back({cx, cy, r - sw, r + sw, a0, a1});
    c.gray.rings.push_back({cx, cy, r - CGW, r + CGW, a0, a1});
    c.white.rings.push_back({cx, cy, r - CLW, r + CLW, a0, a1});
    if (P["shoulder"]) {
        c.white.rings.push_back({cx, cy, r - CW, r - CW + CSH, a0, a1});
        c.white.rings.push_back({cx, cy, r + CW - CSH, r + CW, a0, a1});
    }
}
// y の向きの直線の道（x = x0、y = y0〜y1）。sw はコースアウトを判定する面の幅の半分
static void add_straight_y(Course& c, double x0, double y0, double y1, double sw) {
    c.surface.rects.push_back({x0 - sw, y0, x0 + sw, y1});
    c.gray.rects.push_back({x0 - CGW, y0, x0 + CGW, y1});
    c.white.rects.push_back({x0 - CLW, y0, x0 + CLW, y1});
    if (P["shoulder"]) {
        c.white.rects.push_back({x0 - CW, y0, x0 - CW + CSH, y1});
        c.white.rects.push_back({x0 + CW - CSH, y0, x0 + CW, y1});
    }
}

static Course make_course(const std::string& name) {
    Course c;
    const double W = CW;
    const double SW = P["surf_w"];               // カーブ・S字のコースアウトを判定する面の幅の半分（初期値 0.15。ハンドルの揺れだけを見るときに広げる）
    double y1 = P["approach"];                    // スタートから特別な線まで
    double lw = P["line_w"];
    c.dir = (name.back() == 'r') ? 1 : -1;
    if (name == "crank2") {
        // 1つ目：+y へ進み、turn1 の向きへ曲がって ±x へ。2つ目：turn2 の向きへ曲がって ±y へ
        c.kind = 4;
        c.dir = 1;
        const double sx = P["turn1"] >= 0 ? 1 : -1;                              // 1つ目の後に進む x の向き
        const double sy = ((P["turn1"] >= 0) == (P["turn2"] >= 0)) ? -1 : 1;     // 2つ目の後に進む y の向き
        double y0 = y1 + lw + P["cross_to"];                                     // 1つ目の曲がり角（曲がった後の中心線）
        double xc = sx * P["gap"];                                               // 2つ目のクロスラインの始まり
        double lw2 = P["line_w2"];
        double x0 = sx * (P["gap"] + lw2 + P["cross_to2"]);                      // 2つ目の曲がり角
        add_path(c, {{{0, -1}}, {{0, y0}}, {{x0, y0}}, {{x0, y0 + sy * 4}}}, 0, 0);
        c.white.rects.push_back({-W, y1, W, y1 + lw});                           // 1つ目のクロスライン
        c.white.rects.push_back(box(xc, y0 - W, xc + sx * lw2, y0 + W));         // 2つ目のクロスライン
        c.finish = y0 + sy * 1.0;
        c.white_x = x0;
        c.fin_sy = sy;
        return c;
    }
    if (name.rfind("crank", 0) == 0) {
        c.kind = 0;
        double y0 = y1 + lw + P["cross_to"];   // 曲がった後の車線の中心線
        add_path(c, {{{0, -1}}, {{0, y0}}, {{c.dir * 4.0, y0}}}, 0, 0);
        c.white.rects.push_back({-W, y1, W, y1 + lw});                           // クロスライン
        c.finish = 1.0;
    } else if (name.rfind("lane", 0) == 0) {
        // 合流部分（長さ lane_pre + lane_sec、幅 600mm）。古い車線の中心線は合流部分に lane_pre 入った所で切れ、
        // 新しい車線（中心が 300mm 横）の中心線はそこから lane_new 先で始まる（規則では lane_pre + lane_sec = 0.6、2本は 200 と 400）
        c.kind = 1;
        const double s = c.dir, d = 0.30 * s;
        double yb = y1 + lw + P["half_to"];         // 中央線が切れる所
        double ys = yb - P["lane_pre"];             // 合流部分の始まり
        double ye = yb + P["lane_sec"];             // 合流部分の終わり
        double yn = yb + P["lane_new"];             // 新しい車線の線の始まり
        c.white.rects.push_back(box(-s * CLW, y1, s * W, y1 + lw));               // ハーフライン（曲がる側の半分）
        c.surface.rects.push_back({-W, -1, W, ys});                               // 古い車線
        c.surface.rects.push_back(box(-s * W, ys, d + s * W, ye));                // 合流部分
        c.surface.rects.push_back({d - W, ye, d + W, ye + 4});                    // 新しい車線
        c.gray.rects.push_back({-CGW, -1, CGW, yb});
        c.gray.rects.push_back({d - CGW, yn, d + CGW, ye + 4});
        c.white.rects.push_back({-CLW, -1, CLW, yb});                             // 古い車線の中心線
        c.white.rects.push_back({d - CLW, yn, d + CLW, ye + 4});                  // 新しい車線の中心線
        if (P["shoulder"]) {
            c.white.rects.push_back(box(-s * W, -1, -s * (W - CSH), ye));         // 曲がらない側の路肩（合流部分の外側まで）
            c.white.rects.push_back(box(s * (W - CSH), -1, s * W, ys));           // 古い車線の曲がる側の路肩（合流部分の手前まで）
            c.white.rects.push_back(box(d + s * (W - CSH), ys, d + s * W, ye + 4));   // 合流部分の外側と新しい車線の外側の路肩
            c.white.rects.push_back(box(d - s * W, ye, d - s * (W - CSH), ye + 4));   // 新しい車線の内側の路肩
        }
        c.finish = yb + 2.0;
    } else if (name.rfind("s_", 0) == 0) {
        // 右の S字: 中心 (R, y1) の円弧で右へ θ 曲がり、中心 (R − 2R cosθ, y1 + 2R sinθ) の円弧で左へ θ 曲がる。
        // 左の S字はこれを左右に反転する（x → −x、角度 a → π − a）
        c.kind = 3;
        double rs = P["s_rad"], th = P["s_deg"] * M_PI / 180;
        double c1x = rs, c1y = y1;
        double c2x = rs - 2 * rs * cos(th), c2y = y1 + 2 * rs * sin(th);
        double ex = 2 * rs - 2 * rs * cos(th), ey = y1 + 2 * rs * sin(th);   // 出口
        double a1 = M_PI - th, b1 = M_PI, a2 = -th, b2 = 0;
        if (c.dir < 0) {
            c1x = -c1x; c2x = -c2x; ex = -ex;
            a1 = 0; b1 = th; a2 = M_PI; b2 = M_PI + th;
        }
        add_straight_y(c, 0, -1, y1, SW);
        add_arc(c, c1x, c1y, rs, a1, b1, SW);
        add_arc(c, c2x, c2y, rs, a2, b2, SW);
        add_straight_y(c, ex, ey, ey + 4, SW);
        c.finish = ey + 0.5;
    } else {
        c.kind = 2;
        double rc = P["curve_r"];                     // 中心線の半径（規則：内側の端の半径 450mm 以上 → 0.60 以上）
        double cx = rc * c.dir;                       // 円の中心 (cx, y1)
        add_straight_y(c, 0, -1, y1, SW);
        add_arc(c, cx, y1, rc, 0, M_PI, SW);
        add_straight_y(c, 2 * cx, -1, y1, SW);
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
    const bool gray_white = P["gray_white"] != 0;
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
        dump_shapes("gray", c.gray);
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
    // ハンドルの揺れ：本当のハンドル角度が向きを変えた回数（2度以上戻ったら1回）と、ハンドルが動いた角度の合計
    int wob_n = 0; double wob_sum = 0, wob_ext = 0, wob_prev = 0; int wob_dir = 0;
    long total = (long)P["ms"];
    for (long t = 0; t < total; t++) {
        // センサ
        uint8_t s = 0;
        double hx = cos(psi), hy = sin(psi), rx = sin(psi), ry = -cos(psi);   // 前向き・右向き
        for (int b = 0; b < 8; b++) {
            double lat = SENSOR_X_MM[b] * 0.001;
            double px = x + sf * hx + lat * rx, py = y + sf * hy + lat * ry;
            if (c.white.inside(px, py, spot) || (gray_white && c.gray.inside(px, py, spot))) s |= (uint8_t)(0x80 >> b);
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
        if (v > 0.3) {                                   // 2度以上戻ったら1回と数える。振れ幅はハンドルが動いた角度の合計
            wob_sum += fabs(steer - wob_prev);
            if (wob_dir >= 0 && steer >= wob_ext) { wob_ext = steer; if (!wob_dir && steer - wob_prev > 0) wob_dir = 1; }
            else if (wob_dir <= 0 && steer <= wob_ext) { wob_ext = steer; if (!wob_dir && steer - wob_prev < 0) wob_dir = -1; }
            else if (fabs(steer - wob_ext) >= 2.0) { wob_n++; wob_ext = steer; wob_dir = -wob_dir; }
        }
        wob_prev = steer;
        double k = tan(steer * P["steer_gain"] * M_PI / 180) / wb;       // + が右（steer_gain：実際のタイヤの角度 ÷ 指令の角度）
        // 速さ
        const double pa = (pl + pr) * 0.5, fr = P["fric"];
        double u = (pa - fr) / (100 - fr) * vmax;      // 出力が fric [%] 以下では摩擦に負けて走らない（既定は config.h の MOTOR_FRICTION_PERCENT）
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
        if (v < 0 && pa >= 0) v = 0;                    // 摩擦で止まる（前へ出力していて後ろへは進まない）
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
        // レーンチェンジは、センサの腕と先（左右の端）も区間の外に出てはいけない（角の2か所に棒が立つ）
        const double sf = P["sensor_to_axle"], sw = P["sensor_half_w"];
        std::vector<std::array<double, 2>> pts = {{0, -hw}, {0, hw}, {front, -hw}, {front, hw}};
        if (c.kind == 1) {
            for (int i = 1; i <= 4; i++) pts.push_back({front + (sf - front) * i / 5.0, 0});
            pts.push_back({sf, -sw});
            pts.push_back({sf, sw});
        }
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
            printf("t=%ld p=%d s=%02x x=%.3f y=%.3f psi=%.1f v=%.2f steer=%.1f(est %.1f) cmd=%.1f pl=%.0f pr=%.0f est=%.2f dist=%.3f obs(%.1fmm %.3f %.2f) plan(wait=%.3f len=%.3f v=%.2f ang=%.1f sw=%.3f end=%.3f)\n",
                   t - t_start, pattern, s, x, y, psi * 180 / M_PI, v, steer, (double)servo_est_deg, ang_cmd, pl, pr, speed_mps, dist_m,
                   obs_z1 * 1000, (double)obs_z2, (double)obs_z3, plan_wait_m, plan_len_m, plan_speed_mps, plan_angle, plan_switch_m, lane_end_m);
        // ゴール
        bool fin = false;
        if (c.kind == 0) fin = (c.dir > 0) ? (x >= c.finish) : (x <= -c.finish);
        if (c.kind == 1 || c.kind == 3) fin = (y >= c.finish);
        if (c.kind == 4) fin = (fabs(x - c.white_x) < 0.2 && c.fin_sy * (y - c.finish) >= 0);
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
    printf("  横G %3.0f%%  横すべり %ld ms  揺れ %d回 %.0f度  pattern:", lat_peak * 100, slip_ms, wob_n, wob_sum);
    std::set<int> shown;
    for (int p : seq) if (p >= 100 && p < 800 && !shown.count(p)) { shown.insert(p); printf(" %d", p); }
    printf("\n");
    return (t_finish >= 0) ? 0 : 1;
}
