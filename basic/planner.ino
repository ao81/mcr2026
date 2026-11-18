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

int crank_plan_delay(float v, float delay, float* r_out, float* wait_out) {
    if (COURSE_WIDTH_M * 0.5f - COURSE_MARGIN_M <= CAR_HALF_WIDTH_M) return 0;
    const float d_avail = SENSOR_TO_REAR_AXLE_M - COURSE_WIDTH_M * 0.5f - v * delay;
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

// ① 古い車線からはみ出さないか（s0 = 曲がり始めから中央線の終わりまでの前向きの距離）
int lane_old_ok(float r, float len, float d, float x_start, float s0) {
    const float limit = COURSE_WIDTH_M * 0.5f - COURSE_MARGIN_M;
    float x;
    if (scurve_corner_x(r, len, d, s0, 0.0f, CAR_HALF_WIDTH_M, &x) && x_start + x > limit) return 0;
    if (scurve_corner_x(r, len, d, s0, CAR_FRONT_M, CAR_HALF_WIDTH_M, &x) && x_start + x > limit) return 0;
    return 1;
}

// ② 新しい車線に入る時に内側に入っているか（f_end = 曲がり始めから区間の終わりまでの前向きの距離）
int lane_new_ok(float r, float len, float d, float x_start, float f_end) {
    const float limit = LANE_SHIFT_M - COURSE_WIDTH_M * 0.5f + COURSE_MARGIN_M;
    float x;
    if (!scurve_corner_x(r, len, d, f_end, 0.0f, -CAR_HALF_WIDTH_M, &x) || x_start + x < limit) return 0;
    if (!scurve_corner_x(r, len, d, f_end, CAR_FRONT_M, -CAR_HALF_WIDTH_M, &x) || x_start + x < limit) return 0;
    return 1;
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
        float s0 = SENSOR_TO_REAR_AXLE_M - u;
        float f_end = SENSOR_TO_REAR_AXLE_M + LANE_SECTION_M - u;
        if (f_end <= 0.0f) return 0;
        // ②を満たす一番長い L を二分法で求める（短いほど早く横に動き終わるので②は満たしやすい）
        float lo = 0.05f, hi = f_end + 1.0f;
        if (!lane_new_ok(lane_r_for(lo, d), lo, d, x_start, f_end)) continue;
        for (int i = 0; i < 14; i++) {
            float mid = (lo + hi) * 0.5f;
            if (lane_new_ok(lane_r_for(mid, d), mid, d, x_start, f_end)) lo = mid; else hi = mid;
        }
        float r = lane_r_for(lo, d);
        if (r < radius_of_angle(SERVO_MAX_ANGLE)) return 0;
        if (lane_old_ok(r, lo, d, x_start, s0)) {
            *r_out = r;
            *len_out = lo;
            *wait_out = wait;
            return 1;
        }
    }
    return 0;
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

// setup から呼ぶ：クランク・レーンチェンジに入る速さを決めておく
void plan_setup(void) {
    crank_entry_mps = (CRANK_ENTRY_MPS > 0.0f) ? CRANK_ENTRY_MPS : crank_turn_speed_auto();
    lane_entry_mps  = (LANE_ENTRY_MPS  > 0.0f) ? LANE_ENTRY_MPS  : lane_turn_speed_auto();
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
    float ld = pursuit_lookahead_m();
    float k = 2.0f * e / (ld * ld + e * e);
    float deg = atanf(WHEELBASE_M * k) / DEG_TO_RAD_F;
    int a = (int)(deg >= 0.0f ? deg + 0.5f : deg - 0.5f);
    return clamp_int(a, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
}
