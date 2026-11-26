//**************************************************************************
// planner.ino  クランク・レーンチェンジの経路の計算と、ピュアパシュートのハンドル角度（新しい走行アルゴリズム）
//
//  横すべりせずに曲がれる速さは v = √(GRIP_LAT_USE × 半径)。半径が大きいほど速く曲がれる。
//  そこで、コースからはみ出さない範囲で「一番大きい半径」の経路を計算し、その速さで走る。
//   ・クランク      : 90°を1つの円弧で曲がる。半径はコースの外側の端と内側の角で決まる
//   ・レーンチェンジ : 同じ半径の円弧2つ（右→左、または左→右）で横にずれる。
//                     決まった距離だけ横にずれる経路のうち、一番きつい所の半径が最大になる形
//
//  座標は「後輪の軸の中心」で考える。ハンドル角度 θ のときの半径 R = WHEELBASE_M ÷ tan θ
//  計算した角度は小数のまま ha_deg() で出す（1度単位に丸めると半径が数％変わるため）
//**************************************************************************

// ハンドル角度[度] → 半径 [m]（0度は直進なので大きな値を返す）
float radius_of_angle(float angle) {
    float a = fabsf(angle);
    if (a < 0.01f) return 1.0e6f;
    return WHEELBASE_M / tanf(a * DEG_TO_RAD_F);
}

// 半径 r [m] で曲がるハンドル角度[度]（SERVO_MAX_ANGLE まで）
float angle_for_radius(float r) {
    float deg = atanf(WHEELBASE_M / r) / DEG_TO_RAD_F;
    return deg > SERVO_MAX_ANGLE ? (float)SERVO_MAX_ANGLE : deg;
}

//**********************************************************************
// ハンドルを 0度から angle 度まで切ったとき、その角度で曲がり始めたとみなせるまでの遅れ [s]
//   サーボは SERVO_SPEED_DPS の速さで角度を変えるので、変わっている途中の真ん中で一度に切り替わったと考える
//**********************************************************************
float servo_delay_s(float angle) {
    return SERVO_DEAD_S + fabsf(angle) / (2.0f * SERVO_SPEED_DPS);
}

//**********************************************************************
// クランクの円弧の計算
//   曲がり角の線をセンサが見つけた時、後輪の軸は線の SENSOR_TO_REAR_AXLE_M 手前にいる。
//   新しい道の手前側の端は線の w/2 手前なので、後輪の軸からの前向きの距離は D0 = s_f − w/2。
//   ハンドルが効くまでの遅れ δ（servo_delay_s）の間は直進するので、曲がり始める所では
//     D = D0 − v·δ − （わざと待つ距離）
//   半径 R の円弧で 90°曲がるとき（hw = CAR_HALF_WIDTH_M, Lf = CAR_FRONT_M, m = COURSE_MARGIN_M）
//   ① 外側：外側の前の角が、新しい道の向こう側の端（前方 D + w）を越えない
//        √((R + hw)² + Lf²) ≦ D + w − m        → R ≦ R_far(D) = √((D + w − m)² − Lf²) − hw
//   ② 内側：内側の後輪が、内側の角（横に A = w/2 − m、前に D + m）を切り込まない
//        (R − A)² + (D + m)² ≦ (R − hw)²        → R ≧ R_in(D) = (A² + (D + m)² − hw²) ÷ 2(A − hw)
//   R_far が R_in 以上なら曲がれる。半径は R_far（一番大きい値）にする。
//   今すぐ曲がると内側の角に当たる場合は、①と②がちょうど等しくなる所まで直進してから曲がる。
//   戻り値 1:曲がれる 0:曲がれない（元の方法で曲がる）
//**********************************************************************
float crank_r_far(float d) {
    float b = d + COURSE_WIDTH_M - COURSE_MARGIN_M;
    float q = b * b - CAR_FRONT_M * CAR_FRONT_M;
    if (b <= 0.0f || q <= 0.0f) return -1.0f;
    return sqrtf(q) - CAR_HALF_WIDTH_M;
}

float crank_r_in(float d) {
    const float a = COURSE_WIDTH_M * 0.5f - COURSE_MARGIN_M;
    const float hw = CAR_HALF_WIDTH_M;
    float f = d + COURSE_MARGIN_M;
    if (f <= 0.0f) return 0.0f;                    // 内側の角はもう後ろにある
    return (a * a + f * f - hw * hw) / (2.0f * (a - hw));
}

float crank_room(float d) {                       // プラスなら、その位置から曲がり始めてよい
    float rf = crank_r_far(d);
    if (rf < 0.0f) return -1.0f;
    return rf - crank_r_in(d);
}

//**********************************************************************
// USE_NEW_CRANK_QUICK：小さい半径で素早く曲がるときの、半径と曲がり始める位置（setup で1回だけ計算する）
//   半径 R で曲がるとき、曲がり始める所 D（後輪の軸から新しい道の手前の端まで）は次の範囲でないといけない
//     D ≦ D_hi(R)：② 内側の角を切らない      (R − A)² + (D + m)² ≦ (R − hw)²  → D_hi = √(2R(A − hw) − A² + hw²) − m
//     D ≧ D_lo(R)：① 外側の前の角が出ない    √((R + hw)² + Lf²) ≦ D + w − m    → D_lo = √((R + hw)² + Lf²) − w + m
//   この幅 D_hi − D_lo は、ハンドルを最大に切った半径の近くで一番広く、半径が大きくなると狭くなる。
//   幅が CRANK_QUICK_WINDOW_M 以上ある一番大きい半径を選び、幅の真ん中から曲がり始める
//   （どの半径でも足りないときは、幅が一番広い半径にする。幅がまったくなければ crank_q_r = 0 で、元の方法で曲がる）
//   半径と位置は速さによらないので、ここで1回だけ計算しておく。速さで変わるのは、線を見つけてから直進する距離だけ
//**********************************************************************
void crank_quick_setup(void) {
    const float a = COURSE_WIDTH_M * 0.5f - COURSE_MARGIN_M;
    const float hw = CAR_HALF_WIDTH_M;
    const float m = COURSE_MARGIN_M;
    const float r_min = radius_of_angle(SERVO_MAX_ANGLE);
    float best_r = 0.0f, best_w = 0.0f, best_lo = 0.0f, best_hi = 0.0f;   // 幅が一番広い半径
    float pick_r = 0.0f, pick_lo = 0.0f, pick_hi = 0.0f;                  // 幅が足りる一番大きい半径
    crank_q_r = 0.0f;
    if (a <= hw) return;
    for (int i = 0; i < 100; i++) {
        float r = r_min + 0.002f * i;
        float q = 2.0f * r * (a - hw) - a * a + hw * hw;
        if (q <= 0.0f) continue;
        float hi = sqrtf(q) - m;
        float lo = sqrtf((r + hw) * (r + hw) + CAR_FRONT_M * CAR_FRONT_M) - COURSE_WIDTH_M + m;
        float w = hi - lo;
        if (w > best_w) { best_w = w; best_r = r; best_lo = lo; best_hi = hi; }
        if (w >= CRANK_QUICK_WINDOW_M) { pick_r = r; pick_lo = lo; pick_hi = hi; }
    }
    if (pick_r <= 0.0f) { pick_r = best_r; pick_lo = best_lo; pick_hi = best_hi; }
    if (pick_r <= 0.0f) return;                   // 曲がれる半径がない
    crank_q_r = pick_r;
    crank_q_d = (pick_lo + pick_hi) * 0.5f + CRANK_TURN_EARLY_M;   // 真ん中より少し早く（D が大きいほど手前で曲がり始める）
    if (crank_q_d > pick_hi) crank_q_d = pick_hi;
    crank_q_dlo = pick_lo;
}

// ハンドルを angle 度から 0度へ戻す間に、車の向きがさらに変わる角度 [rad]（速さ v）
//   指令してから SERVO_DEAD_S は angle のまま、その後 SERVO_SPEED_DPS で 0 へ戻るとして
//     v × ( SERVO_DEAD_S × tan(angle) ÷ L  ＋  ∫ tan θ dθ ÷ (L × サーボの速さ[rad/s]) )、∫ tan θ dθ = −ln(cos angle)
float crank_unwind_yaw(float v, float angle) {
    float th = fabsf(angle) * DEG_TO_RAD_F;
    float c = cosf(th);
    if (c < 0.05f) c = 0.05f;
    return v * (SERVO_DEAD_S * tanf(th) + (-logf(c)) / (SERVO_SPEED_DPS * DEG_TO_RAD_F)) / WHEELBASE_M;
}

int crank_plan_delay(float v, float delay, float* r_out, float* wait_out) {
    if (COURSE_WIDTH_M * 0.5f - COURSE_MARGIN_M <= CAR_HALF_WIDTH_M) return 0;
    const float d_avail = SENSOR_TO_REAR_AXLE_M - COURSE_WIDTH_M * 0.5f - v * delay;
#if USE_NEW_CRANK_QUICK
    if (crank_q_r <= 0.0f) return 0;
    // センサは曲がり角の線の中心ではなく、線の手前の端で線を見つける。その分だけ後輪の軸は手前にいる
    const float d_seen = d_avail + CENTER_LINE_WIDTH_M * 0.5f + LINE_SEEN_EXTRA_M * 0.5f;
    float dq = crank_q_d;
    if (d_seen < dq) {                             // 速くて、真ん中の位置はもう過ぎた
        if (d_seen < crank_q_dlo) return 0;        // 範囲の外 → 曲がれないので元の方法で曲がる
        dq = d_seen;                               // 範囲の中なら、すぐ曲がる
    }
    *r_out = crank_q_r;
    *wait_out = d_seen - dq;
    return 1;
#endif
    float d = d_avail;
    if (crank_room(d) < 0.0f) {
        // 少しずつ手前（D を小さく = 待つ距離を長く）にして、曲がれる所を探す
        float hi = d;
        float lo = d;
        int found = 0;
        for (int i = 0; i < 40; i++) {
            lo = lo - 0.010f;
            if (crank_room(lo) >= 0.0f) { found = 1; break; }
            hi = lo;
        }
        if (!found) return 0;
        for (int i = 0; i < 12; i++) {            // 曲がれる一番奥の所を二分法で求める
            float mid = (lo + hi) * 0.5f;
            if (crank_room(mid) >= 0.0f) lo = mid; else hi = mid;
        }
        d = lo;
    }
    float r = crank_r_far(d);
    if (r < radius_of_angle(SERVO_MAX_ANGLE)) return 0;   // ハンドルを最大に切っても足りない
    *r_out = r;
    *wait_out = d_avail - d;
    return 1;
}

// 遅れ δ はハンドル角度で変わり、ハンドル角度は半径で決まるので、数回くり返して合わせる
int crank_plan(float v, float* r_out, float* wait_out) {
    float delay = servo_delay_s(SERVO_MAX_ANGLE * 0.5f);
    float r = 0.0f, wait = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (!crank_plan_delay(v, delay, &r, &wait)) return 0;
        delay = servo_delay_s(angle_for_radius(r));
    }
    *r_out = r;
    *wait_out = wait;
    return 1;
}

//**********************************************************************
// レーンチェンジの2つの円弧の計算
//   同じ半径 R の円弧2つ（それぞれ φ だけ回る）で、横に d、前に L 進むとき
//     2R(1 − cos φ) = d、2R·sin φ = L   →   R = (L² + d²) ÷ 4d
//   L が長いほど R が大きく、速く走れる。L は次の2つの条件を満たす一番長い値にする。
//   ① 古い車線：中央線が切れる所（センサが見つけた所 = 後輪の軸の s_f 前）より手前にいる間、
//      車線を変える側の角（後輪の外側・前の外側）が古い車線の端（w/2 − m）を越えない
//   ② 新しい車線：区間の終わり（中央線が切れてから LANE_SECTION_M 先）を越える時、
//      反対側の角が新しい車線の端（LANE_SHIFT_M − w/2 + m）より内側に入っている
//   ①が満たせないときは、少し直進してから曲がり始める。
//   車体の角の位置は、車の向きのずれ角 α を使って
//     前向き = f + 前の距離·cos α − 横·sin α、横向き = x(f) + 前の距離·sin α + 横·cos α
//**********************************************************************
float scurve_x(float r, float len, float f, float d) {   // 前に f 進んだ所の横のずれ（d は横に動く距離）
    if (f >= len) return d;
    float g = (f <= len * 0.5f) ? f : len - f;
    if (g < 0.0f) g = 0.0f;
    float x = r - sqrtf(r * r - g * g);
    return (f <= len * 0.5f) ? x : d - x;
}

float scurve_sin(float r, float len, float f) {  // 前に f 進んだ所での、車の向きのずれ角 α の sin
    if (f >= len) return 0.0f;
    float g = (f <= len * 0.5f) ? f : len - f;
    if (g < 0.0f) g = 0.0f;
    float sa = g / r;
    return sa > 1.0f ? 1.0f : sa;
}

float lane_r_for(float len, float d) {           // 長さ len で横に d ずれる円弧の半径 = (L² + d²) ÷ 4d
    return (len * len + d * d) / (4.0f * d);     //  （φ = 2·atan(d/L), R = L ÷ 2sin φ を三角関数なしで書いたもの）
}

// 車体の角（前に fwd、車線を変える側へ横に lat）の前向きの位置が f_line になる所での、その角の横の位置
//   戻り値 1: 求まった  0: 曲がり始める前に、もう f_line を越えている
int scurve_corner_x(float r, float len, float d, float f_line, float fwd, float lat, float* x_out) {
    float f = f_line - fwd;
    for (int i = 0; i < 4; i++) {                // f = f_line − fwd·cos α + lat·sin α を数回くり返して解く
        if (f < 0.0f) f = 0.0f;
        float sa = scurve_sin(r, len, f);
        float ca = sqrtf(1.0f - sa * sa);
        f = f_line - fwd * ca + lat * sa;
    }
    if (f < 0.0f) return 0;
    float sa = scurve_sin(r, len, f);
    float ca = sqrtf(1.0f - sa * sa);
    *x_out = scurve_x(r, len, f, d) + fwd * sa + lat * ca;
    return 1;
}

// ① 古い車線からはみ出さないか（s0 = 曲がり始めから合流部分の始まりまでの前向きの距離。マイナスならもう合流部分にいる）
// レーンチェンジでは、センサの腕と先も区間の外に出てはいけない（角の2か所に棒が立つ）。
// 腕は車体の前（CAR_FRONT_M）からセンサまでの真ん中の線、先はセンサの基板の左右の端（SENSOR_HALF_WIDTH_M）
#define LANE_ARM_N 4
static float lane_arm_fwd(int i) { return CAR_FRONT_M + (SENSOR_TO_REAR_AXLE_M - CAR_FRONT_M) * (i + 1) / (LANE_ARM_N + 1); }

int lane_old_ok(float r, float len, float d, float x_start, float s0) {
    const float limit = COURSE_WIDTH_M * 0.5f - COURSE_MARGIN_M;
    float x;
    if (scurve_corner_x(r, len, d, s0, 0.0f, CAR_HALF_WIDTH_M, &x) && x_start + x > limit) return 0;
    if (scurve_corner_x(r, len, d, s0, CAR_FRONT_M, CAR_HALF_WIDTH_M, &x) && x_start + x > limit) return 0;
    for (int i = 0; i < LANE_ARM_N; i++) {
        if (scurve_corner_x(r, len, d, s0, lane_arm_fwd(i), 0.0f, &x) && x_start + x > limit) return 0;
    }
    return 1;
}

// センサの先（車線を変える側の端）が、新しい車線の向こう側の端（LANE_SHIFT_M + w/2 − m）を越えないか
//   一番外へ出るのは、向きのずれが大きい1つ目の円弧の終わりのあたり。円弧の上の何か所かで調べる
int lane_outer_ok(float r, float len, float d, float x_start) {
    const float limit = LANE_SHIFT_M + COURSE_WIDTH_M * 0.5f - COURSE_MARGIN_M;
    for (int i = 2; i <= 8; i++) {
        float f = len * 0.1f * i;
        float sa = scurve_sin(r, len, f);
        float ca = sqrtf(1.0f - sa * sa);
        float x = scurve_x(r, len, f, d) + SENSOR_TO_REAR_AXLE_M * sa + SENSOR_HALF_WIDTH_M * ca;
        if (x_start + x > limit) return 0;
    }
    return 1;
}

// ② 新しい車線に入る時に内側に入っているか（f_end = 曲がり始めから区間の終わりまでの前向きの距離）
int lane_new_ok(float r, float len, float d, float x_start, float f_end) {
    const float limit = LANE_SHIFT_M - COURSE_WIDTH_M * 0.5f + COURSE_MARGIN_M;
    float x;
    if (!scurve_corner_x(r, len, d, f_end, 0.0f, -CAR_HALF_WIDTH_M, &x) || x_start + x < limit) return 0;
    if (!scurve_corner_x(r, len, d, f_end, CAR_FRONT_M, -CAR_HALF_WIDTH_M, &x) || x_start + x < limit) return 0;
    for (int i = 0; i < LANE_ARM_N; i++) {
        if (!scurve_corner_x(r, len, d, f_end, lane_arm_fwd(i), 0.0f, &x) || x_start + x < limit) return 0;
    }
    if (!scurve_corner_x(r, len, d, f_end, SENSOR_TO_REAR_AXLE_M, -SENSOR_HALF_WIDTH_M, &x) || x_start + x < limit) return 0;
    return 1;
}

// ② を、走った距離を (1 − LANE_DIST_ERR) 倍に短く走ってしまった場合にも満たすか
//   距離は推定した速さから求めるので、速さを実際より速く見ていると、どちらの円弧も短く走ってしまう。
//   半径 r は同じまま、円弧1つで回る角度が φ(1 − e) になる。その時 前向き L' = 2r·sin φ'、横 d' = 2r(1 − cos φ')。
//   曲がり始めるまでの距離 u も (1 − e) 倍になるので、区間の終わりは u·e だけ遠くなる
int lane_new_ok_robust(float len, float d, float x_start, float f_end, float u) {
    float r = lane_r_for(len, d);
    if (!lane_new_ok(r, len, d, x_start, f_end)) return 0;
    if (LANE_DIST_ERR <= 0.0f) return 1;
    float phi = 2.0f * atanf(d / len) * (1.0f - LANE_DIST_ERR);
    float len2 = 2.0f * r * sinf(phi);
    float d2 = 2.0f * r * (1.0f - cosf(phi));
    return lane_new_ok(r, len2, d2, x_start, f_end + u * LANE_DIST_ERR);
}

//   x_start : 曲がり始める時の、元の線に対する車の横の位置 [m]（車線を変える側がプラス）
//   計算は 1ms の割り込みの中で行うので、くり返しの回数を少なくしている
//   計算時間の目安（48MHz、FPU あり）：車体の角の位置1回が約 200 クロック。lane_plan 全体で
//   待つ距離 0 で決まる普通の場合は約 13000 クロック（0.3ms）。待つ距離を 8cm まで全部試す一番遅い場合は約 2.4ms で、
//   その時だけ次の割り込みが 1〜2回遅れる（3m/s で 6mm ほど遅れて曲がり始める。COURSE_MARGIN_M の内に収まる）
int lane_plan_delay(float v, float delay, float x_start, float* r_out, float* len_out, float* wait_out) {
    const float d = LANE_SHIFT_M - x_start;      // 横に動く距離
    if (d <= 0.0f) return 0;
    for (int k = 0; k <= 8; k++) {               // 曲がり始める前に直進する距離 wait を 0, 1cm, 2cm … と試す
        float wait = k * 0.010f;
        float u = v * delay + wait;              // センサが中央線の終わりを見つけてから、実際に曲がり始めるまでの距離
        float s0 = SENSOR_TO_REAR_AXLE_M - LANE_PRE_M - u;   // 合流部分は中央線が切れる所の LANE_PRE_M 手前から始まる
        float f_end = SENSOR_TO_REAR_AXLE_M + LANE_SECTION_M - u;
        if (f_end <= 0.0f) return 0;
        // ②を満たす一番長い L を二分法で求める（短いほど早く横に動き終わるので②は満たしやすい）
        float lo = 0.05f, hi = f_end + 1.0f;
        if (!lane_new_ok_robust(lo, d, x_start, f_end, u)) continue;
        for (int i = 0; i < 14; i++) {
            float mid = (lo + hi) * 0.5f;
            if (lane_new_ok_robust(mid, d, x_start, f_end, u)) lo = mid; else hi = mid;
        }
        float r = lane_r_for(lo, d);
        if (r < radius_of_angle(SERVO_MAX_ANGLE)) return 0;
        if (!lane_outer_ok(r, lo, d, x_start)) return 0;   // 一番長い（緩い）経路でもセンサの先が外へ出るなら、短い経路はもっと出る
        if (lane_old_ok(r, lo, d, x_start, s0)) {
            *r_out = r;
            *len_out = lo;
            *wait_out = wait;
            return 1;
        }
    }
    return 0;
}

//**********************************************************************
// クイッと曲がる経路（LANE_QUICK_LAT_MPS2）
//   速さ v で、横の加速度がちょうど LANE_QUICK_LAT_MPS2 になる半径 r = v² ÷ a の S字にする（一番緩い経路 len_lo より短い）。
//   長さは 2R(1 − cos φ) = d、2R·sin φ = L から L = √(4dR − d²)。曲がり始める所は一番緩い経路と同じ（wait）。
//   短いほど早く新しい車線に入り終える（区間の終わり・2本目の棒には余裕が増える）が、向きのずれが大きくなるので、
//   古い車線の端（1本目の棒）とセンサの先（新しい車線の向こう側の端）を調べ、越えるなら越えない長さまで延ばす
//**********************************************************************
static int lane_quick_ok(float len, float d, float x_start, float s0, float f_end) {
    float r = lane_r_for(len, d);
    return lane_old_ok(r, len, d, x_start, s0) && lane_outer_ok(r, len, d, x_start) && lane_new_ok(r, len, d, x_start, f_end);
}

float lane_quick_len(float v, float d, float x_start, float r_lo, float len_lo, float wait) {
    float rq = v * v / LANE_QUICK_LAT_MPS2;
    float rmin = radius_of_angle(LANE_QUICK_MAX_ANGLE);
    if (rq < rmin) rq = rmin;
    if (rq >= r_lo || 4.0f * d * rq <= d * d) return len_lo;
    float lq = sqrtf(4.0f * d * rq - d * d);
    float u = wait + v * servo_delay_s(angle_for_radius(rq));
    float s0 = SENSOR_TO_REAR_AXLE_M - LANE_PRE_M - u;
    float f_end = SENSOR_TO_REAR_AXLE_M + LANE_SECTION_M - u;
    if (lane_quick_ok(lq, d, x_start, s0, f_end)) return lq;
    float lo = lq, hi = len_lo;                    // 越えない一番短い長さを二分法で探す
    for (int i = 0; i < 10; i++) {
        float mid = (lo + hi) * 0.5f;
        if (lane_quick_ok(mid, d, x_start, s0, f_end)) hi = mid; else lo = mid;
    }
    return hi;
}

int lane_plan(float v, float x_start, float* r_out, float* len_out, float* wait_out) {
    float delay = servo_delay_s(SERVO_MAX_ANGLE * 0.25f);
    float r = 0.0f, len = 0.0f, wait = 0.0f;
    for (int i = 0; i < 2; i++) {
        if (!lane_plan_delay(v, delay, x_start, &r, &len, &wait)) return 0;
        delay = servo_delay_s(angle_for_radius(r));
    }
    *r_out = r;
    *len_out = len;
    *wait_out = wait;
    return 1;
}

//**********************************************************************
// 曲がる速さを自動で決める（CRANK_ENTRY_MPS / LANE_ENTRY_MPS が 0 のとき）
//   速さ v で経路を計算 → その半径で曲がれる速さ → もう一度計算… を繰り返す
//   （速いほど v·δ の分だけ曲がり始めが遅れて、半径が小さくなるため）
//   計算した経路を使わない、または曲がれない場合は、元のプログラムのハンドル角度の半径で決める
//**********************************************************************
float crank_turn_speed_auto(void) {
#if USE_NEW_CRANK_TURN
    float v = 1.0f;
    int ok = 1;
    for (int i = 0; i < 8 && ok; i++) {
        float r, wait;
        ok = crank_plan(v, &r, &wait);
        if (ok) v = corner_speed(1.0f / r);
    }
    if (ok) return v;
#endif
    return corner_speed(1.0f / radius_of_angle(CRANK_ANGLE));
}

float lane_turn_speed_auto(void) {
#if USE_NEW_LANE_STEER
    float v = 1.0f;
    int ok = 1;
    for (int i = 0; i < 8 && ok; i++) {
        float r, len, wait;
        ok = lane_plan(v, 0.0f, &r, &len, &wait);
        if (ok) v = corner_speed(1.0f / r);
    }
    if (ok) return v;
#endif
    return corner_speed(1.0f / radius_of_angle(LANE_STEER_ANGLE));
}

// 速さ v で半径 r の円弧に入るとき、曲がり角の線（クランク）・中央線の切れ目（レーンチェンジ）を見つけてから、
// 本当に曲がり始めるまでに進む距離 [m]（わざと待つ距離 wait ＋ サーボが効くまでの遅れの間に進む距離）
float pre_turn_m(float v, float r, float wait) {
    return wait + v * servo_delay_s(angle_for_radius(r));
}

// setup から呼ぶ：走る前に済ませておける計算をしておく（1msの割り込みの中で時間のかかる計算をしないため）
void plan_setup(void) {
    model_ok = model_check_config();
    model_bad_cnt = 0;
    course_list_setup();
#if USE_NEW_CRANK_QUICK
    crank_quick_setup();
#endif
    crank_entry_mps = (CRANK_ENTRY_MPS > 0.0f) ? CRANK_ENTRY_MPS : crank_turn_speed_auto();
    lane_entry_mps  = (LANE_ENTRY_MPS  > 0.0f) ? LANE_ENTRY_MPS  : lane_turn_speed_auto();
    pursuit_ld = pursuit_lookahead_m();
    obs_k_lim = 2.0f / radius_of_angle(SERVO_MAX_ANGLE);

    // USE_NEW_BRAKE_RANGE：線を見つけてから円弧・S字に入るまでの距離（入る速さで計算）
    crank_extra_m = 0.0f;
    lane_extra_m = 0.0f;
    float r, len, wait;
#if USE_NEW_CRANK_TURN
    if (crank_plan(crank_entry_mps, &r, &wait)) crank_extra_m = pre_turn_m(crank_entry_mps, r, wait);
#endif
#if USE_NEW_LANE_STEER
    if (lane_plan(lane_entry_mps, 0.0f, &r, &len, &wait)) lane_extra_m = pre_turn_m(lane_entry_mps, r, wait);
    lane_table_setup();
#endif
    (void)r; (void)len; (void)wait;

    // USE_NEW_CRANK_ANY_COURSE：曲がり角の線を見つけた時に、この速さ以下なら、そこからのブレーキで円弧に入る速さまで落とせる。
    //   crank_start_arc と同じ判断（線を見つけてから円弧に入るまでに進む距離 pre_turn_m の間、GRIP_LONG_USE で減速）を、
    //   CRANK_BRAKE_MARGIN_M の余裕を残して満たす一番速い速さを、二分法で探す
    crank_app_mps = 0.0f;
#if USE_NEW_CRANK_ANY_COURSE && USE_NEW_CRANK_QUICK
    if (crank_q_r > 0.0f) {
        float rq, wq;
        if (crank_plan(corner_speed(1.0f / crank_q_r), &rq, &wq)) {
            float vt = corner_speed(1.0f / rq);
            float lo = vt, hi = V_MAX_MPS * 3.0f;
            for (int i = 0; i < 30; i++) {
                float mid = (lo + hi) * 0.5f;
                float d = pre_turn_m(mid, rq, wq) - CRANK_BRAKE_MARGIN_M;
                if (d > 0.0f && speed_after_brake(mid, d) <= vt) lo = mid; else hi = mid;
            }
            crank_app_mps = lo;
        }
    }
#endif

    // TRACE_SAFE_RADIUS_M：この半径のカーブが急に来ても間に合う直線の速さ
    //   センサは後輪の SENSOR_TO_REAR_AXLE_M 前にある。線がセンサのきざみの半分（e）だけ動いたら曲がりに気づけるとすると、
    //   気づくのはセンサがカーブに √(2·R·e) 入った時。そこから後輪がカーブに着くまでの距離 d で、曲がれる速さまで落とせる速さ
    //     v = √(v_c² + 2·GRIP_LONG_USE·d)、v_c = √(GRIP_LAT_USE·R)
    trace_v_cap = 1.0e3f;
    if (TRACE_SAFE_RADIUS_M > 0.0f) {
        float pitch = (SENSOR_X_MM[7] - SENSOR_X_MM[0]) / 7.0f * 0.001f;
        float d = SENSOR_TO_REAR_AXLE_M - sqrtf(2.0f * TRACE_SAFE_RADIUS_M * pitch * 0.5f);
        if (d < 0.0f) d = 0.0f;
        trace_v_cap = sqrtf(GRIP_LAT_USE * TRACE_SAFE_RADIUS_M + 2.0f * GRIP_LONG_USE * d);
    }
}

//**********************************************************************
// レーンチェンジの経路の表（USE_NEW_LANE_STEER）
//   lane_plan は一番遅い場合に約 2.4ms かかり、1msの割り込みに収まらない。そこで、中央線が切れる前の
//   車の横の位置 x_start を 5mm おきに変えた経路を setup で計算しておき、走行中は表を引くだけにする
//   （センサは 10mm おきで、線の位置は 5mm きざみで求まるので、ほとんどの場合は表の値そのものになる）
//   経路は「一番速く受け付ける速さ」（曲がれる速さ × PLAN_OVERSPEED）で計算し、線を見つけてから曲がり始めるまでの
//   距離 u を覚えておく。走行中の速さ v がそれより遅いときは、待つ距離を u − v·遅れ にすると、同じ位置から同じ経路を走れる
//**********************************************************************
#define LANE_TAB_N    17                         // x_start = −40mm 〜 +40mm（5mm おき）
#define LANE_TAB_MID  8
#define LANE_TAB_STEP 0.005f
float lane_tab_r[LANE_TAB_N];
float lane_tab_len[LANE_TAB_N];
float lane_tab_u[LANE_TAB_N];
signed char lane_tab_ok[LANE_TAB_N];

void lane_table_setup(void) {
    for (int i = 0; i < LANE_TAB_N; i++) {
        float x = (i - LANE_TAB_MID) * LANE_TAB_STEP;
        float v = 1.0f, r = 0.0f, len = 0.0f, wait = 0.0f;
        int ok = 1;
        for (int k = 0; k < 6 && ok; k++) {      // 曲がれる速さと経路が合うまでくり返す
            ok = lane_plan(v, x, &r, &len, &wait);
            if (ok) v = corner_speed(1.0f / r);
        }
        float vp = v * PLAN_OVERSPEED;
        if (ok) ok = lane_plan(vp, x, &r, &len, &wait);
        lane_tab_ok[i] = (signed char)ok;
        lane_tab_r[i] = r;
        lane_tab_len[i] = len;
        lane_tab_u[i] = ok ? pre_turn_m(vp, r, wait) : 0.0f;
    }
}

// 表から経路を引く。戻り値 1: 引けた  0: 表の範囲の外か、速すぎて同じ位置から曲がれない（その場で計算する）
int lane_table_plan(float v, float x_start, float* r_out, float* len_out, float* wait_out, float* x_used) {
    float f = x_start / LANE_TAB_STEP;
    int i = (int)(f >= 0.0f ? f + 0.5f : f - 0.5f) + LANE_TAB_MID;
    if (i < 0 || i >= LANE_TAB_N || !lane_tab_ok[i]) return 0;
    float r = lane_tab_r[i];
    float wait = lane_tab_u[i] - v * servo_delay_s(angle_for_radius(r));
    if (wait < 0.0f) return 0;
    *r_out = r;
    *len_out = lane_tab_len[i];
    *wait_out = wait;
    *x_used = (i - LANE_TAB_MID) * LANE_TAB_STEP;
    return 1;
}

//**********************************************************************
// 白線の位置 [m]：白線を見つけたセンサの位置の平均（右がプラス）。線が見えなければ 0
//**********************************************************************
float line_position_m(unsigned char c) {
    long sum = 0;
    int n = 0;
    for (int b = 0; b < 8; b++) {
        if (c & (0x80 >> b)) {
            sum += SENSOR_X_MM[b];
            n++;
        }
    }
    if (n == 0) return 0.0f;
    return (float)sum / n * 0.001f;
}

//**********************************************************************
// ピュアパシュート（USE_NEW_STEER）
//   線の上の点（前に Ld、横に e）を通り、今の向きに接する円の曲率 κ = 2e ÷ (Ld² + e²) で曲がる。
//   Ld はセンサまでの距離 SENSOR_TO_REAR_AXLE_M だが、それだと急なカーブで線がセンサの外へ出てしまう場合は短くする：
//   一番外側の2つのセンサで線を見ている位置 e_edge のとき、ちょうどコースの最小半径 TRACE_MIN_RADIUS_M で
//   曲がるように  Ld = √(2·e_edge·R_min − e_edge²)  とする（線がセンサの中にあるうちに、一番急なカーブを曲がれる）
//   ハンドル角度 θ = atan(WHEELBASE_M × κ)
//   （Ld を速さに合わせて長くする方法も模擬で試したが、急なカーブで減速が遅れてコースアウトしたので使わない）
//**********************************************************************
float pursuit_lookahead_m(void) {
    float e_edge = (SENSOR_X_MM[7] + SENSOR_X_MM[6]) * 0.5f * 0.001f;
    float q = 2.0f * e_edge * TRACE_MIN_RADIUS_M - e_edge * e_edge;
    float ld = SENSOR_TO_REAR_AXLE_M;
    if (q > 0.0f && sqrtf(q) < ld) ld = sqrtf(q);
    return ld;
}

int pursuit_angle(unsigned char c) {
    if (c == 0) return 0;
    float e = line_position_m(c);
    float ld = pursuit_ld;                        // plan_setup で計算しておいた値（毎ms √ を計算しない）
    float k = 2.0f * e / (ld * ld + e * e);
    float deg = atanf(WHEELBASE_M * k) / DEG_TO_RAD_F;
    int a = (int)(deg >= 0.0f ? deg + 0.5f : deg - 0.5f);
    return clamp_int(a, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
}


//**********************************************************************
// 線の形の推定（オブザーバ。USE_NEW_LINE_SHAPE / USE_NEW_CURVE_PREVIEW）
//   センサの所（後輪の軸の Ls = SENSOR_TO_REAR_AXLE_M 前）での線を、横の位置 z1・向き z2・曲がり具合 z3 で表す。
//   車が ds 進み、その間に曲率 κc（サーボの本当の角度の推定から）で曲がると、車から見た線は
//     z1 ← z1 + (z2 − Ls·κc)·ds     （線の向きの分ずれ、車が向きを変えた分だけ前の点が反対へ動く）
//     z2 ← z2 + (z3 − κc)·ds        （線の曲がりの分と、車が曲がった分の差だけ向きが変わる）
//     z3 ← z3                       （曲がり具合はゆっくり変わるとみる）
//   と動く（予測）。センサで見た位置 e との差 r = e − z1 で、3つを直す
//     z1 += 3p·ds·r、 z2 += 3p²·ds·r、 z3 += p³·ds·r       （p = LINE_OBS_BAND）
//   この係数で、推定の誤差は (s + p)³ の形で、進んだ距離 1/p ごとにほぼ 1/e ずつ小さくなる（極を -p に3つ置く）。
//   時間ではなく進んだ距離で考えるので、速さが変わっても同じ距離で追いつく
//   κc には、摩擦円を超えた分を縮めた「車が本当に曲がっている曲率」car_k_eff を使う（横すべりしているときに、
//   線が大きく曲がったと思い込まないように）
//   通常走行・大曲がりの外にいた時間が 20ms を超えたら、測った位置から始め直す（それより短いときは、その分進める）
//   線が端のセンサ1つだけ（0x80 / 0x01）のときは、それより外にあるかもしれないので、予測だけ進める
//**********************************************************************
void line_obs_update(unsigned char c) {
    const float ls = SENSOR_TO_REAR_AXLE_M;
    const float p = LINE_OBS_BAND;
    unsigned long now = cnt0;
    unsigned long gap = now - line_obs_ms;
    int fresh = (gap == 0 || gap > 20);
    line_obs_ms = now;
    int seen = (c != 0x00 && c != 0x80 && c != 0x01);
    float kc = car_k_eff;
    if (fresh) {
        obs_z1 = seen ? line_position_m(c) : 0.0f;
        obs_z2 = 0.0f;
        obs_z3 = 0.0f;
        return;
    }
    float ds = speed_mps * 0.001f * gap;
    if (ds < 0.0f) ds = 0.0f;
    float z1 = obs_z1 + (obs_z2 - ls * kc) * ds;
    float z2 = obs_z2 + (obs_z3 - kc) * ds;
    float z3 = obs_z3;
    if (seen) {
        float r = line_position_m(c) - z1;
#if USE_NEW_LINE_SHAPE && USE_NEW_CURVE_PREVIEW
        // センサは 10mm おきなので、同じ形に見える線の位置には幅がある。その幅の中なら直さない（LINE_OBS_DEADZONE_M）
        if (r > LINE_OBS_DEADZONE_M) r -= LINE_OBS_DEADZONE_M;
        else if (r < -LINE_OBS_DEADZONE_M) r += LINE_OBS_DEADZONE_M;
        else r = 0.0f;
#endif
        z1 = z1 + 3.0f * p * ds * r;
        z2 = z2 + 3.0f * p * p * ds * r;
        z3 = z3 + p * p * p * ds * r;
    }
    // 起こりえない値にならないように（線の向きは ±0.6rad、曲がり具合はハンドルを最大に切った時の2倍まで。
    // その曲がり具合 obs_k_lim は plan_setup で1回だけ計算しておく）
    const float k_lim = obs_k_lim;
    if (!finite_f(z1) || !finite_f(z2) || !finite_f(z3)) { z1 = 0.0f; z2 = 0.0f; z3 = 0.0f; }
    if (z2 > 0.6f) z2 = 0.6f;
    if (z2 < -0.6f) z2 = -0.6f;
    if (z3 > k_lim) z3 = k_lim;
    if (z3 < -k_lim) z3 = -k_lim;
    obs_z1 = z1;
    obs_z2 = z2;
    obs_z3 = z3;
}

//**********************************************************************
// 推定した線の形からハンドル角度を決める（USE_NEW_LINE_SHAPE）
//   ① 今出す指令が効くのは STEER_LEAD_S 後。その間に進む距離 d だけ先の、センサの所の線の横の位置と向きを予測する
//        eL = z1 + (z2 − Ls·κc)·d + (z3 − κc)·d²/2、  z2L = z2 + (z3 − κc)·d
//      （その間の車の曲がり具合 κc は、サーボの本当の角度と、前の1msに出した指令の平均とする。
//        この1msの指令は、表を引いた時に一度書き換わっているので使わない）
//   ② 車が曲率 κ で曲がると、センサの所の線の横の位置は  dz1/ds = z2 − Ls·κ  で変わる（線の推定と同じ式）。
//      これが −a·z1 になる曲率  κ = (z2 + a·z1) ÷ Ls（a = LINE_STEER_GAIN）で曲がると、線とのずれは
//      1/a [m] 進むごとに 1/e に減る（進んだ距離で決まるので、速さが変わっても同じ形で戻る）。
//      ずれが 0 の時は κ = z2 ÷ Ls。円のカーブでは z2 = Ls·(線の曲率) なので、ちょうど線の曲率で曲がる。
//      車の向きは  dz2/ds = z3 − κ  で変わり、ずれが 0 なら Ls 進むごとに 1/e ずつ線の向きにそろう。
//      ピュアパシュート（線の位置だけを見る）は線の向き z2 を見ないので、ずれが 0 を行き過ぎてハンドルが揺れた
//      （模擬で、半径1.5mのカーブで ±3°、0.3秒周期。揺れの山に合わせて速さを落とすので遅くなる）
//   ②は USE_NEW_CURVE_PREVIEW も 1 のときだけ使う。②はずれを急いで直さない分、カーブの入口で線が外へずれるので、
//   線の曲がり具合 z3 で先に減速しておく必要がある。USE_NEW_CURVE_PREVIEW が 0 のときは、
//   USE_NEW_STEER のピュアパシュートと同じ式 κ = 2eL ÷ (Ld² + eL²)（Ld = pursuit_ld）で曲がる
//**********************************************************************
float line_obs_angle(void) {
    const float ls = SENSOR_TO_REAR_AXLE_M;
    float kc = 0.5f * (curvature_of_angle(servo_est_deg) + curvature_of_angle(servo_last_cmd()));
    float d = speed_mps * STEER_LEAD_S;
    if (d < 0.0f) d = 0.0f;
    float el = obs_z1 + (obs_z2 - ls * kc) * d + (obs_z3 - kc) * d * d * 0.5f;
    float k = 2.0f * el / (pursuit_ld * pursuit_ld + el * el);
#if USE_NEW_CURVE_PREVIEW
    float z2l = obs_z2 + (obs_z3 - kc) * d;
    k = (z2l + LINE_STEER_GAIN * el) / ls;
#endif
    float deg = atanf(WHEELBASE_M * k) / DEG_TO_RAD_F;
    if (deg > SERVO_MAX_ANGLE) deg = SERVO_MAX_ANGLE;
    if (deg < -SERVO_MAX_ANGLE) deg = -SERVO_MAX_ANGLE;
    return deg;
}
