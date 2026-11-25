//**************************************************************************
// speed_model.ino  速度の推定と、計算によるモータ出力の決め方（新しい走行アルゴリズム）
//
//  Basicマイコンカーには速度を測るセンサ（エンコーダ）が無い。そこで次の2つで速さを求める。
//   1. モータに出した出力から、車の速さを毎ms計算する（一次遅れのモデル）
//        v ← v + (出力/100 × v_max_mps − v) × 0.001 ÷ MOTOR_TAU_S
//      ただし、タイヤがすべらずに出せる加速度（GRIP_LONG_USE）より速くは変化しないとする
//      v_max_mps は出力100%のときの速さ。初めは config.h の V_MAX_MPS で、USE_NEW_VMAX_LEARN が 1 なら
//      ラインを通過するたびに直す（電池の電圧が下がると遅くなるため）
//   2. 幅の分かっているライン（クロスライン・ハーフライン）を通過した時間で、v を測り直す
//        v = ラインの幅 ÷ 通過時間
//  グリップの値は次のように使い分ける（どちらも、推定した速さが実際より速めになる側）
//   ・加速   : 測った値そのまま（GRIP_LAT_MPS2 / GRIP_LONG_MPS2）
//   ・ブレーキと曲がる速さ : 測った値に余裕の割合をかけた値（GRIP_LAT_USE / GRIP_LONG_USE。basic.ino）
//
//  クランク・レーンチェンジ手前の減速は「最短時間で目標の速さまで落とす」問題になる。
//  加速と減速の強さに上限がある場合、最短時間になるのは
//    全開で進む → 止まりきれる限界の地点から最大ブレーキ → 目標の速さを保つ
//  という切り替え方（バン・バン制御）である。
//  ここでは「今ブレーキをかけたら何m で目標の速さまで落ちるか」を毎ms計算し、
//  それが残りの距離に達した瞬間にブレーキへ切り替える。
//
//  目標の速さに近づけるときは、モデルの上で「次の1msでちょうど目標の速さになる出力」を出す
//  （デッドビート制御）。出力の上限・下限とグリップの上限で頭打ちになるので、
//  目標から遠いときは全開か最大ブレーキになり、近づくと自動で保つ出力に変わる。
//**************************************************************************

// 使う変数 speed_mps（速さ）と dist_m（距離）は basic.ino で宣言している

//**********************************************************************
// ハンドル角度[度]から、後輪の軸の中心が描く円の曲がり具合（曲率 = 1 ÷ 半径）を求める
//   戻り値 [1/m]。プラスが右に曲がる、0 は直進
//**********************************************************************
float curvature_of_angle(float angle) {
    if (angle == 0.0f) return 0.0f;
    return tanf(angle * DEG_TO_RAD_F) / WHEELBASE_M;
}

//**********************************************************************
// 摩擦円から、今の速さと曲がり具合で使ってよい加速の上限 [m/s²] を求める
//   横向きの加速度 = v² × 曲率。その分だけ前後に使えるグリップが減る
//   ブレーキは減らさない（速さを落とすほど横向きの加速度も小さくなるため、常に GRIP_LONG_USE まで使う）
//   ここは測った値そのままを使う。加速を少なめに見ると推定した速さが遅めになり、ブレーキが遅れるため
//**********************************************************************
float accel_up_limit(float v, float k_abs) {
    float r = v * v * k_abs / GRIP_LAT_MPS2;      // 横のグリップをどれだけ使っているか（1 で限界）
    if (r >= 1.0f) return 0.0f;
    return GRIP_LONG_MPS2 * sqrtf(1.0f - r * r);
}

//**********************************************************************
// 1msごとに呼ぶ：最後に出したモータ出力から速さと距離を更新
//**********************************************************************
void speed_model_update(void) {
    const float dt = 0.001f;
    servo_model_update();
    float power = (log_raw.ml + log_raw.mr) * 0.5f;          // DIPスイッチで縮小した後の出力
    float target = power / 100.0f * v_max_mps;
    float v = speed_mps;
    float a = (target - v) / MOTOR_TAU_S;
    float k = curvature_of_angle(log_raw.ang);
    car_k_eff = k;
#if USE_NEW_CURVE_PREVIEW
    // 指令ではなく、サーボの本当の角度の推定で曲がり具合を求める。
    // 求められた横の加速度と前後の加速度が、測ったグリップの摩擦円を超えるときは、両方を同じ割合で縮める
    // （曲がりきれずに外へふくらみ、ブレーキも弱くなる）。縮めた曲率は、線の形の推定で「車が本当に曲がった分」に使う
    k = curvature_of_angle(servo_est_deg);
    if (a > GRIP_LONG_MPS2) a = GRIP_LONG_MPS2;
    if (a < -GRIP_LONG_MPS2) a = -GRIP_LONG_MPS2;
    float q_lat = v * v * k / GRIP_LAT_MPS2;
    float q_long = a / GRIP_LONG_MPS2;
    float q = sqrtf(q_lat * q_lat + q_long * q_long);
    car_k_eff = k;
    if (q > 1.0f) {
        a = a / q;
        car_k_eff = k / q;
    }
#endif
    const float a_free = a;
    float a_up = accel_up_limit(v, fabsf(k));
    if (a > a_up) a = a_up;                                   // タイヤがすべるほどの加速はできない
    if (a < -GRIP_LONG_USE) a = -GRIP_LONG_USE;               // ブレーキは余裕をみた値（遅めに減速すると考える）
#if USE_NEW_VMAX_LEARN
    // 推定の速さが v_max_mps にどれだけ左右されているか（感度）と、それ以外の誤差を進める（config.h の説明）
    if (a == a_free) {
        vmax_sens += (power / 100.0f - vmax_sens) * dt / MOTOR_TAU_S;
    } else {
        speed_err_sd += SPEED_GRIP_ERR * fabsf(a) * dt;      // グリップで頭打ち：v_max には左右されないが、グリップの誤差が積もる
    }
    {
        const float d = VMAX_DRIFT_SD * v_max_mps;
        vmax_var += d * d * dt;
    }
#endif
    v = v + a * dt;
    if (!finite_f(v)) v = 0.0f;                               // config.h の値がおかしいときに、計算が壊れたままにならないように
    speed_mps = v;
    dist_m = dist_m + v * dt;
}

//**********************************************************************
// ラインの通過時間から速さを測り直す
//   width_m : ラインの進行方向の幅 [m]（0 なら何もしない）
//**********************************************************************
void speed_measure_by_line(float width_m, int pass_ms) {
    if (width_m <= 0.0f) return;
    float t = (pass_ms + LINE_PASS_OFFSET_MS) * 0.001f;
    if (t <= 0.0f) return;
    float v_meas = width_m / t;
    // 測った速さが推定と大きく違う（0.6倍より小さい、1.67倍より大きい）ときは、斜めに横切った・線を見落とした などの
    // 測り間違いか、config.h の車の値が実際と大きく違うかのどちらか。どちらか分からないので、速い方の値を使う
    // （速めに見ておけば、ブレーキが遅れることはない）。2回続けて大きく違ったら、推定は当てにならないとして
    // 元の表の動きに戻す（model_ok = 0。この走行の間は戻さない）
    if (speed_mps > 0.3f) {
        float q = v_meas / speed_mps;
        if ((q < 0.6f || q > 1.67f) && !vmax_plausible(v_meas, t)) {
            model_bad_cnt++;
            if (model_bad_cnt >= 2) model_ok = 0;
            if (speed_mps > v_meas) v_meas = speed_mps;
            speed_mps = v_meas;
            return;
        }
    }
    model_bad_cnt = 0;
#if USE_NEW_LANE_WIDTH_INTERVAL
    const bool half_line = pattern == PT_LANE_L_PASS || pattern == PT_LANE_R_PASS;
    const float sample_s = 0.001f;
    if (half_line && model_ok && t > sample_s && speed_mps > 0.5f) {
        // 通過中の加減速と、入口・出口を次のサンプルで検出する遅れを見込む。
        const float error = GRIP_LONG_MPS2 * (0.5f * (t + sample_s) + sample_s);
        const float low = width_m / (t + sample_s) - error;
        const float high = width_m / (t - sample_s) + error;
        if (speed_mps >= low && speed_mps <= high) return;

        // 区間端について反転した値までに限り、既存のゲインで混ぜる。
        const float edge = speed_mps < low ? low : high;
        const float limit = 2.0f * edge - speed_mps;
        v_meas = speed_mps < low ? fminf(v_meas, limit) : fmaxf(v_meas, limit);
    }
#endif
#if USE_NEW_VMAX_LEARN
    // 線の幅が1つに決まっているときは、測った速さで v_max_mps と速さを両方直す（vmax_observe）
    // 線の幅が1つに決まらないとき（一番広い幅で代えている）は、測った速さが速めに出るので、v_max は直さず速さだけ混ぜる
    // （v_max を直すと、その後ずっと出力を少なめに出して、思ったより遅く走り、曲がり始めの位置もずれる。模擬の crank2 で確認）
    {
        float rq = 0.41f / (pass_ms + LINE_PASS_OFFSET_MS);
        float r_rel = sqrtf(rq * rq + LINE_WIDTH_SD * LINE_WIDTH_SD);
        if (line_w_exact) {
            vmax_observe(v_meas, r_rel);
            return;
        }
        float pv = vmax_sens * vmax_sens * vmax_var + speed_err_sd * speed_err_sd;
        float r = (r_rel * v_meas) * (r_rel * v_meas);
        float k = (pv + r > 0.0f) ? pv / (pv + r) : 1.0f;
        speed_mps = speed_mps + k * (v_meas - speed_mps);
        vmax_sens *= (1.0f - k);
        speed_err_sd = sqrtf((1.0f - k) * (1.0f - k) * speed_err_sd * speed_err_sd + k * k * r);
        return;
    }
#else
    // 推定（モデル）と測定のどちらにも誤差があるので、2つを誤差の大きさに応じて混ぜる（カルマンフィルタ）
    //   K = P ÷ (P + R)、新しい速さ = 推定 × (1 + K × (測定 ÷ 推定 − 1))
    float v_new = v_meas;
    if (speed_mps > 0.5f) {
        float ratio = v_meas / speed_mps;
        float rq = 0.41f / (pass_ms + LINE_PASS_OFFSET_MS);
        float r = rq * rq + LINE_WIDTH_SD * LINE_WIDTH_SD;
        float k = 0.01f / (0.01f + r);
        v_new = speed_mps * (1.0f + k * (ratio - 1.0f));
    }
    speed_mps = v_new;
#endif
}

//**********************************************************************
// 測った速さが、v_max_mps の誤差で説明できる範囲か（3σ 以内）。
//   V_MAX_MPS が大きくずれているうちは、推定と大きく違っても測り間違いとはみなさない
//**********************************************************************
int vmax_plausible(float v_meas, float t) {
#if USE_NEW_VMAX_LEARN
    float rq = 0.00041f / t;
    float r_rel2 = rq * rq + LINE_WIDTH_SD * LINE_WIDTH_SD;
    float s2 = vmax_sens * vmax_sens * vmax_var + speed_err_sd * speed_err_sd + r_rel2 * v_meas * v_meas;
    float e = v_meas - speed_mps;
    return e * e <= 9.0f * s2;
#else
    (void)v_meas; (void)t;
    return 0;
#endif
}

//**********************************************************************
// 測った速さ v_meas（誤差の大きさは v_meas の r_rel 倍）で、v_max_mps と速さ speed_mps を直す（USE_NEW_VMAX_LEARN）
//   推定の速さの誤差 = 感度 s × v_max の誤差 ＋ それ以外の誤差（大きさ speed_err_sd）と考えて、
//   速さと v_max の2つを一緒に直す（カルマンフィルタ）。
//   スタートから全開（s ≒ 1）なら、測った速さとの差はほぼそのまま v_max の誤差になる。
//   ブレーキの後に p% で一定の速さを保っている（s ≒ p/100）なら、差の 100/p 倍が v_max の誤差になる
//**********************************************************************
void vmax_observe(float v_meas, float r_rel) {
#if USE_NEW_VMAX_LEARN
    const float s = vmax_sens;
    const float P = vmax_var;
    const float Vi = speed_err_sd * speed_err_sd;
    const float R = (r_rel * v_meas) * (r_rel * v_meas);
    const float pvv = s * s * P + Vi;                 // 推定の速さの誤差の2乗
    const float pvm = s * P;                          // 速さと v_max の誤差の関係
    const float S = pvv + R;
    if (!(S > 0.0f)) { speed_mps = v_meas; return; }
    const float e = v_meas - speed_mps;
    const float kv = pvv / S;
    const float km = pvm / S;
    float vm = v_max_mps + km * e;
    const float lo = V_MAX_MPS * VMAX_MIN_RATIO;
    const float hi = V_MAX_MPS * VMAX_MAX_RATIO;
    if (vm < lo) vm = lo;
    if (vm > hi) vm = hi;
    v_max_mps = vm;
    speed_mps = speed_mps + kv * e;
    // 直した後の誤差（2×2 の共分散を更新し、感度とそれ以外の誤差に分け直す）
    const float P2 = P - pvm * pvm / S;
    const float pvv2 = pvv - pvv * pvv / S;
    const float pvm2 = pvm - pvv * pvm / S;
    vmax_var = P2 > 1.0e-8f ? P2 : 1.0e-8f;
    vmax_sens = pvm2 / vmax_var;
    float vi2 = pvv2 - vmax_sens * vmax_sens * vmax_var;
    speed_err_sd = vi2 > 0.0f ? sqrtf(vi2) : 0.0f;
#else
    speed_mps = v_meas;
    (void)r_rel;
#endif
}

// ハーフライン退出後の積算距離を、中央線の切れ目で一度だけ照合する。
void correct_lane_speed_by_distance(void) {
    if (!lane_to_exact) return;                   // このレーンチェンジの距離が分からない（一番短い値で代えている）
    if (!model_ok || !(dist_m > 0.0f && speed_mps > 0.5f && lane_to_m > 0.0f)) return;
    const float margin = 2.0f * v_max_mps * 0.001f;
    const float measured = fminf(fmaxf(dist_m, lane_to_m - margin),
                                 lane_to_m + margin);
    const float ratio = measured / dist_m;
    if (!(ratio >= 0.8f && ratio <= 1.25f) || ratio == 1.0f) return;
    if (USE_NEW_LANE_DOWN_ONLY && ratio > 1.0f) return;

    speed_mps *= ratio;
#if USE_NEW_VMAX_LEARN
    v_max_mps = fminf(V_MAX_MPS * VMAX_MAX_RATIO, fmaxf(V_MAX_MPS * VMAX_MIN_RATIO, v_max_mps * ratio));
#endif
}

//**********************************************************************
// クランク・レーンチェンジごとの寸法（config.h の COURSE_LIST / COURSE_START_ROWS。issue #2）
//   クロスライン・ハーフラインを見つけた時に、これから来るクランク・レーンチェンジの値を
//   crank_to_m / crank_line_w_m、lane_to_m / lane_line_w_m に入れる。減速の計算と、通過時間から速さを測るのに使う
//   スタートの位置がまだ1つに決まっていないときは、あり得る行の中で一番短い距離・一番広い線の幅を使い、
//   曲がった向き・ハーフラインの向きと合わない位置を外していく
//**********************************************************************
#define SPOT_N   ((int)(sizeof(COURSE_LIST) / sizeof(COURSE_LIST[0])))
#define START_N  ((int)(sizeof(COURSE_START_ROWS) / sizeof(COURSE_START_ROWS[0])))

// スタート位置 i から数えて、今のクランク・レーンチェンジは表の何行目か
int spot_row(int i) {
    return (COURSE_START_ROWS[i] + spot_no) % SPOT_N;
}

// kind ('C' / 'H') の行のうち、cand の位置から見て今あり得るものの一番短い距離と一番広い線の幅を求める。
// dir が 0 でないときは向きも合うものだけ。合う位置を bit で返す（0 なら1つもない）
unsigned int spot_match(unsigned int cand, char kind, char dir, float* to_m, float* w_m) {
    unsigned int m = 0;
    float to = 1.0e6f, w = 0.0f;
    for (int i = 0; i < START_N; i++) {
        if (!(cand & (1u << i))) continue;
        const CourseSpot& s = COURSE_LIST[spot_row(i)];
        if (s.kind != kind || (dir != 0 && s.dir != dir)) continue;
        m |= 1u << i;
        if (s.to_m < to) to = s.to_m;
        if (s.line_w_m > w) w = s.line_w_m;
    }
    *to_m = to;
    *w_m = w;
    return m;
}

// cand の位置から見て今あり得る kind の行の線の幅が、全部 w と同じなら 1
int spot_width_same(unsigned int cand, char kind, float w) {
    for (int i = 0; i < START_N; i++) {
        if (!(cand & (1u << i))) continue;
        const CourseSpot& s = COURSE_LIST[spot_row(i)];
        if (s.kind == kind && s.line_w_m != w) return 0;
    }
    return 1;
}

// どの行とも合わないときの値：表の中で kind の一番短い距離と一番広い線の幅。その種類の行が無ければ 0 を返す
int spot_fallback(char kind, float* to_m, float* w_m) {
    float to = 1.0e6f, w = 0.0f;
    int n = 0;
    for (int r = 0; r < SPOT_N; r++) {
        const CourseSpot& s = COURSE_LIST[r];
        if (s.kind != kind) continue;
        n++;
        if (s.to_m < to) to = s.to_m;
        if (s.line_w_m > w) w = s.line_w_m;
    }
    *to_m = to;
    *w_m = w;
    return n;
}

int spot_ok;                                      // 1: COURSE_LIST / COURSE_START_ROWS が正しく書かれている

// COURSE_LIST / COURSE_START_ROWS が正しく書かれているか（正しくなければ表を使わない。model_ok も 0 になる）
int course_list_check(void) {
    if (!(LINE_SEEN_EXTRA_M >= 0.0f && LINE_SEEN_EXTRA_M < 0.05f)) return 0;
    for (int r = 0; r < SPOT_N; r++) {
        const CourseSpot& s = COURSE_LIST[r];
        if (!(s.kind == 'C' || s.kind == 'H')) return 0;
        if (!(s.to_m > 0.0f && s.to_m < 5.0f && s.line_w_m >= 0.0f && s.line_w_m < 0.2f)) return 0;
        if (!(s.dir == 'R' || s.dir == 'L')) return 0;
    }
    if (START_N > 16) return 0;
    for (int i = 0; i < START_N; i++) {
        if (!(COURSE_START_ROWS[i] >= 0 && COURSE_START_ROWS[i] < SPOT_N)) return 0;
    }
    return 1;
}

// setup から呼ぶ：数を0に戻し、すべてのスタート位置をあり得るものにする
void course_list_setup(void) {
    spot_ok = course_list_check();
    spot_no = 0;
    spot_cand = (START_N >= 16) ? 0xffffu : ((1u << START_N) - 1u);
    spot_pend = 0;
    crank_spot_select();
    lane_spot_select(true);
}

// クロスラインを見つけた時（pattern 100）に呼ぶ
void crank_spot_select(void) {
    float to, w;
    line_w_exact = 1;
    if (COURSE_LIST_USE && spot_ok) {
        unsigned int m = spot_match(spot_cand, 'C', 0, &to, &w);
        if (m) {
            line_w_exact = spot_width_same(m, 'C', w);
        } else {
            line_w_exact = 0;
        }
        if (m || spot_fallback('C', &to, &w)) {
            crank_to_m = to;
            crank_line_w_m = w + LINE_SEEN_EXTRA_M;
            return;
        }
    }
    crank_to_m = CROSS_TO_CRANK_M;                 // 表を使わない、または表にクランクが無い
    crank_line_w_m = CROSS_LINE_WIDTH_M;
}

// クランクの曲がり角を見つけて、曲がる向きが決まった時に呼ぶ。向きの合わないスタート位置を外し、次へ数を進める
void crank_spot_done(bool right) {
    if (!(COURSE_LIST_USE && spot_ok)) return;
    float to, w;
    spot_cand = spot_match(spot_cand, 'C', right ? 'R' : 'L', &to, &w);
    spot_no = (spot_no + 1) % SPOT_N;              // 表の最後の次は最初の行（1周）
}

// ハーフラインを見つけた時（pattern 200 / 500）に呼ぶ。right: 右レーンチェンジのハーフライン
//   後でクロスラインだったと分かることがあるので、スタート位置を外すのは lane_spot_done で行う
void lane_spot_select(bool right) {
    lane_to_exact = 1;
    line_w_exact = 1;
    float to, w;
    if (COURSE_LIST_USE && spot_ok) {
        spot_pend = spot_match(spot_cand, 'H', right ? 'R' : 'L', &to, &w);
        if (spot_pend) {
            // あり得る行の距離が全部同じときだけ、距離で速さを直してよい
            for (int i = 0; i < START_N; i++) {
                if (!(spot_pend & (1u << i))) continue;
                if (COURSE_LIST[spot_row(i)].to_m != to) lane_to_exact = 0;
            }
            line_w_exact = spot_width_same(spot_pend, 'H', w);
        } else {
            lane_to_exact = 0;
            line_w_exact = 0;
            if (!spot_fallback('H', &to, &w)) {
                to = HALF_TO_LANE_M;               // 表にレーンチェンジが無い
                w = HALF_LINE_WIDTH_M - LINE_SEEN_EXTRA_M;
            }
        }
        lane_to_m = to;
        lane_line_w_m = w + LINE_SEEN_EXTRA_M;
        return;
    }
    lane_to_m = HALF_TO_LANE_M;
    lane_line_w_m = HALF_LINE_WIDTH_M;
}

// ハーフラインを通過し終えて、クロスラインではなくレーンチェンジだと決まった時（pattern 202 / 502）に呼ぶ
void lane_spot_done(void) {
    if (!(COURSE_LIST_USE && spot_ok)) return;
    spot_cand = spot_pend;
    spot_pend = 0;
    spot_no = (spot_no + 1) % SPOT_N;              // 表の最後の次は最初の行（1周）
}

//**********************************************************************
// USE_NEW_CRANK_ANY_COURSE でクロスラインを通過し終えた時に呼ぶ（USE_NEW_VMAX_LEARN）
//   クロスラインの幅は1つに決まらないが、あり得る幅の範囲（COURSE_LIST の候補、無ければ規則の 20〜40mm）の
//   どれかとして、推定の速さがあり得ない（範囲の外）ときだけ v_max_mps を直す（V_MAX_MPS が大きくずれているときに効く）
//**********************************************************************
void cross_line_vmax_observe(int pass_ms) {
#if USE_NEW_VMAX_LEARN
    float w_lo = CROSS_LINE_RULE_MIN_M, w_hi = CROSS_LINE_RULE_MAX_M;
    if (COURSE_LIST_USE && spot_ok && spot_cand) {
        float lo = 1.0e6f, hi = 0.0f;
        for (int i = 0; i < START_N; i++) {
            if (!(spot_cand & (1u << i))) continue;
            const CourseSpot& c = COURSE_LIST[spot_row(i)];
            if (c.kind != 'C') continue;
            if (c.line_w_m < lo) lo = c.line_w_m;
            if (c.line_w_m > hi) hi = c.line_w_m;
        }
        if (hi > 0.0f) { w_lo = lo; w_hi = hi; }
    }
    w_lo += LINE_SEEN_EXTRA_M;
    w_hi += LINE_SEEN_EXTRA_M;
    float t = (pass_ms + LINE_PASS_OFFSET_MS) * 0.001f;
    if (!(t > 0.0f) || speed_mps < 0.3f) return;
    // 本当の速さは「一番狭い幅 ÷ 時間」から「一番広い幅 ÷ 時間」の間のどこか（その中ならどれも同じくらいあり得る）。
    // 推定の速さがこの間に入っていれば、この測定からは何も分からないので直さない。
    // 外れていたら、範囲の真ん中の幅で求めた速さを、範囲の広さの分の誤差がある測定として使って直す（V_MAX_MPS が大きくずれているときだけ働く）
    float rq = 0.41f / (pass_ms + LINE_PASS_OFFSET_MS);
    float v_lo = w_lo / t * (1.0f - rq);
    float v_hi = w_hi / t * (1.0f + rq);
    float v_meas = 0.5f * (w_lo + w_hi) / t;                 // 範囲の真ん中の幅で求めた速さ
    float rw = (w_hi - w_lo) / (w_hi + w_lo) / 1.7320508f;   // 範囲の中のどれか（一様）とみたときの誤差の大きさ
    float r_rel = sqrtf(rq * rq + rw * rw + LINE_WIDTH_SD * LINE_WIDTH_SD);
    // 推定と比べて大きく違いすぎるとき（3σ より外）は、斜めに横切った・見落とした などとみて使わない
    float e = v_meas - speed_mps;
    float s2 = vmax_sens * vmax_sens * vmax_var + speed_err_sd * speed_err_sd + (r_rel * v_meas) * (r_rel * v_meas);
    if (e * e > 9.0f * s2) return;
    vmax_observe(v_meas, r_rel);
#else
    (void)pass_ms;
#endif
}

// 距離を0に戻す（ラインを通過し終わった時などに呼ぶ）。ブレーキの状態も最初に戻す
void reset_distance(void) {
    dist_m = 0.0f;
    brake_started = 0;
}

//**********************************************************************
// 出力 brake_power で減速し続けたとき、v から v_target まで落ちるのに進む距離 [m]
//   モデルの減速度は min(GRIP_LONG_USE, (v − vb) ÷ τ)。vb = brake_power/100 × v_max_mps
//   ① 速いうちはグリップで決まる一定の減速度 G：距離 = (v² − v₁²) ÷ 2G
//      （v₁ = vb + τ·G より遅くなると、モータの力の方が小さくなる）
//   ② その後は v(t) = vb + (v − vb)·e^(−t/τ) で、v_target になる時刻 t* = τ·ln((v − vb) ÷ (v_target − vb))
//      その間の距離 = vb·t* + τ·(v − v_target)
//**********************************************************************
float braking_distance(float v, float v_target, int brake_power) {
    if (v <= v_target) return 0.0f;
    float vb = brake_power / 100.0f * v_max_mps;
    if (vb >= v_target) return 1.0e6f;                // このブレーキでは目標の速さまで落ちない
    float s = 0.0f;
    float v1 = vb + MOTOR_TAU_S * GRIP_LONG_USE;      // これより速いと減速度はグリップで決まる
    if (v > v1) {
        float v_end = (v_target > v1) ? v_target : v1;
        s += (v * v - v_end * v_end) / (2.0f * GRIP_LONG_USE);
        if (v_target >= v1) return s;
        v = v1;
    }
    float t_star = MOTOR_TAU_S * logf((v - vb) / (v_target - vb));
    return s + vb * t_star + MOTOR_TAU_S * (v - v_target);
}

//**********************************************************************
// v から v_target へ最短で近づけるための「指令の速さ」[m/s] を返す（出力% = 指令の速さ ÷ v_max_mps × 100）
//   モデル v' = v + (u − v)·dt/τ で、次の1msにちょうど v_target になる u を求め、
//   加速は摩擦円で残っている分まで、出力は ±100% までに制限する
//   k_abs : 曲率の大きさ [1/m]（摩擦円の計算に使う）
//**********************************************************************
float deadbeat_speed(float v, float v_target, float k_abs) {
    float u = v + (v_target - v) * (MOTOR_TAU_S / 0.001f);
    float u_hi = v + MOTOR_TAU_S * accel_up_limit(v, k_abs);
    if (u > u_hi) u = u_hi;
    // ブレーキ側は制限しない。推定した速さが実際より速いとき、ブレーキが弱くなるのを避けるため
    // （グリップを超える分はタイヤがすべり、減速度は GRIP_LONG_USE 程度になる。モデルもそう計算する）
    if (u > v_max_mps) u = v_max_mps;
    if (u < -v_max_mps) u = -v_max_mps;
    return u;
}

// 指令の速さ [m/s] を出力 [%] に直す
int power_of_speed(float u) {
    float p = u / v_max_mps * 100.0f;
    if (!finite_f(p)) return 0;                                // 計算が壊れたときは出力しない
    int i = (int)(p >= 0.0f ? p + 0.5f : p - 0.5f);
    return clamp_int(i, -100, 100);
}

//**********************************************************************
// 残りの距離と目標の速さから、今出すべきモータ出力を決める
//   remain_m : 目標地点までの残り距離 [m]
//**********************************************************************
int brake_power_for(float remain_m, float v_target, int brake_power, float margin_m) {
    float v = speed_mps;
    if (v > v_target + SPEED_HOLD_BAND_MPS) {
        if (!brake_started) {
            // 次の1msも全開で進んだとして、その先で止まりきれるかを調べる
            float a = (v_max_mps - v) / MOTOR_TAU_S;
            if (a > GRIP_LONG_MPS2) a = GRIP_LONG_MPS2;
            float next_v = v + a * 0.001f;
            float need = next_v * 0.001f + braking_distance(next_v, v_target, brake_power);
            if (remain_m - margin_m > need) {
                return 100;                             // まだ全開でよい
            }
            brake_started = 1;  // 一度ブレーキを始めたら、目標の速さになるまで続ける（全開と交互にならないように）
        }
        return brake_power;                             // ブレーキ
    }
    // 目標の速さの近く：その速さに保つ。
    // 前は「1msで目標にぴったり合わせる出力」（deadbeat_speed）を出していたが、出力を整数に丸めた分だけ目標を少し超え、
    // 超えるとブレーキ（-60）→ 戻すために +62 … を繰り返していた（LOG0019：-60 / +36 / +62 の繰り返しで、
    // 推定は 1.065 m/s のまま、実際の出力の平均は約13% で、クランク手前を約2.3秒のろのろ走った。
    // 推定ではブレーキの効きを GRIP_LONG_USE で頭打ちにしているので、-60 の本当の効きより弱く見ていたため）。
    // そこで、保つのに要る出力に、ずれに比例した分だけを足す（SPEED_HOLD_S で目標に近づく。ブレーキの側に振り切れない）
    float u = v_target + (v_target - v) * (MOTOR_TAU_S / SPEED_HOLD_S);
    int p = power_of_speed(u);
    return p < brake_power ? brake_power : p;
}

//**********************************************************************
// クロスライン通過後（pattern 102, 103）の新しい減速
//**********************************************************************
void crank_speed_control_new(void) {
#if USE_NEW_CRANK_ANY_COURSE
    // 曲がり角までの距離を使わない：曲がり角の線を見てからでも間に合う速さ（計算できなければクランクに入る速さ）まで、すぐに落とす
    float v_app = crank_entry_mps;
    if (CRANK_APPROACH_MPS > 0.0f) v_app = CRANK_APPROACH_MPS;
    else if (CRANK_APPROACH_MPS < 0.0f && crank_app_mps > 0.0f) v_app = crank_app_mps;
    int pa = brake_power_for(0.0f, v_app, CRANK_BRAKE_POWER, 0.0f);
    sp(pa, pa);
    return;
#endif
    float remain = crank_to_m - dist_m;
#if USE_NEW_BRAKE_RANGE
    remain = remain + crank_extra_m;                // 曲がり角の線を見つけた後、円弧に入るまでにも進む
#endif
    int p = brake_power_for(remain, crank_entry_mps, CRANK_BRAKE_POWER, CRANK_BRAKE_MARGIN_M);
    sp(p, p);
}

//**********************************************************************
// ハーフライン通過後（pattern 202, 300〜310 / 502, 600〜610）の新しい減速
//**********************************************************************
void lane_speed_control_new(void) {
    float remain = lane_to_m - dist_m;
#if USE_NEW_BRAKE_RANGE
    remain = remain + lane_extra_m;                 // 中央線の切れ目を見つけた後、S字に入るまでにも進む
#endif
    int p = brake_power_for(remain, lane_entry_mps, LANE_BRAKE_POWER, LANE_BRAKE_MARGIN_M);
    sp(p, p);
}

//**********************************************************************
// ハンドル角度 angle で曲がりながら、車の中心の速さを v_target に近づける
//   左右の後輪は、曲がりの内側と外側で回る距離が違う。
//   半径 R のとき 外輪 = 中心 × (R + TREAD/2) ÷ R、内輪 = 中心 × (R − TREAD/2) ÷ R
//   外輪が100%を超えるときは、比を保ったまま両方を下げる
//**********************************************************************
void drive_at(float v_target, float angle) {
    float k = curvature_of_angle(angle);
    float u = deadbeat_speed(speed_mps, v_target, fabsf(k));
    drive_wheels(u, k);
}

// 車の中心の指令の速さ u [m/s] と曲率 k [1/m] から、左右のモータ出力を決めて出す
void drive_wheels(float u, float k) {
    float half = k * TREAD_M * 0.5f;              // R の代わりに曲率で書く：外輪 = u(1 + k·T/2)
    float ul = u * (1.0f + half);                 // 右に曲がる(k>0)とき左が外輪
    float ur = u * (1.0f - half);
    float big = fabsf(ul) > fabsf(ur) ? fabsf(ul) : fabsf(ur);
    if (big > v_max_mps) {
        ul = ul * (v_max_mps / big);
        ur = ur * (v_max_mps / big);
    }
    int pl = power_of_speed(ul);
    int pr = power_of_speed(ur);
    if (pl < TRACE_MIN_POWER) pl = TRACE_MIN_POWER;
    if (pr < TRACE_MIN_POWER) pr = TRACE_MIN_POWER;
    sp(pl, pr);
}

//**********************************************************************
// 曲率 k_abs の円を、横すべりせずに回れる一番速い速さ v = √(GRIP_LAT_USE ÷ 曲率)
//**********************************************************************
float corner_speed(float k_abs) {
    if (k_abs <= 0.0f) return v_max_mps;
    float v = sqrtf(GRIP_LAT_USE / k_abs);
    return v > v_max_mps ? v_max_mps : v;
}

//**********************************************************************
// 通常走行・大曲がりの新しいモータ出力（ハンドル角度から計算）
//   angle : ハンドル角度[度]（プラスが右）
//   ハンドル角度で決まる半径を、横すべりしない一番速い速さで回る
//**********************************************************************
void trace_motor_new(int angle) {
    // センサは 10mm おきなので、カーブの途中でもハンドル角度は2つの値を行き来する。
    // 速さは、ハンドル角度から求めた曲率を TRACE_CURV_FILTER_S の時定数でならした値で決める
    float k = fabsf(curvature_of_angle(angle));
    trace_k_filt = trace_k_filt + (k - trace_k_filt) * (0.001f / TRACE_CURV_FILTER_S);
    drive_at(corner_speed(trace_k_filt), angle);
}

//**********************************************************************
// 加速でタイヤが空回りしないように、左右のモータ出力を制限する（USE_NEW_TRACTION。sp から呼ぶ）
//   各タイヤの速さ（車の速さと曲がり具合から計算）に対して、加速は摩擦円で残っている分までにする
//   出力% と加速度の関係：加速度 = (出力/100 × v_max_mps − タイヤの速さ) ÷ MOTOR_TAU_S
//   ブレーキは制限しない（推定した速さが実際より速いと、ブレーキが効かなくなるため）
//**********************************************************************
void traction_limit(int* power_l, int* power_r) {
    float v = speed_mps;
    float k = curvature_of_angle(log_raw.ang);
    float a_up = accel_up_limit(v, fabsf(k));
    float wheel[2] = { v * (1.0f + k * TREAD_M * 0.5f), v * (1.0f - k * TREAD_M * 0.5f) };
    int* power[2] = { power_l, power_r };
    for (int i = 0; i < 2; i++) {
        float hi = (wheel[i] + MOTOR_TAU_S * a_up) / v_max_mps * 100.0f;
        int p = *power[i];
        if (p > hi) p = (int)floorf(hi);
        *power[i] = clamp_int(p, -100, 100);
    }
}


//**********************************************************************
// ここから下は USE_NEW_LINE_SHAPE / USE_NEW_CURVE_PREVIEW / USE_NEW_BRAKE_RANGE で使う
//**********************************************************************

// 数として正しい値か（計算が壊れて NaN や無限大になっていないか）
bool finite_f(float x) {
    return (x == x) && x < 1.0e30f && x > -1.0e30f;
}

//**********************************************************************
// サーボの本当の角度の推定（1msごとに speed_model_update から呼ぶ）
//   指令を出してから SERVO_DEAD_S 遅れて動き始め、SERVO_SPEED_DPS の速さで指令の角度へ向かうとする。
//   ハンドルを戻している途中は、指令（0度）より本当の角度の方が大きい。曲がり終わりで加速してよいかは、
//   本当の角度で決まる横向きの加速度で判断する
//**********************************************************************
#define SERVO_HIST 32                             // 指令の履歴の数（2のべき乗。SERVO_DEAD_S は 31ms まで）
float servo_hist[SERVO_HIST];
int servo_hist_pos;

// 前の1msの終わりに出した、ハンドル角度の指令 [度]
float servo_last_cmd(void) {
    return servo_hist[(servo_hist_pos - 1) & (SERVO_HIST - 1)];
}

void servo_model_update(void) {
    servo_hist[servo_hist_pos] = steer_cmd_deg;
    int lag = (int)(SERVO_DEAD_S * 1000.0f + 0.5f);
    if (lag < 0) lag = 0;
    if (lag > SERVO_HIST - 1) lag = SERVO_HIST - 1;
    float target = servo_hist[(servo_hist_pos - lag + SERVO_HIST) & (SERVO_HIST - 1)];
    servo_hist_pos = (servo_hist_pos + 1) & (SERVO_HIST - 1);
    const float step = SERVO_SPEED_DPS * 0.001f;
    float a = servo_est_deg;
    if (target > a + step) a = a + step;
    else if (target < a - step) a = a - step;
    else a = target;
    servo_est_deg = a;
}

//**********************************************************************
// 曲がりながら加速してよい加速度 [m/s²]（摩擦円）
//   (前後 ÷ GRIP_LONG_MPS2)² + (横 ÷ GRIP_LAT_USE)² ≦ 1  →  前後 = GRIP_LONG_MPS2 × √(1 − (v²k ÷ GRIP_LAT_USE)²)
//   横は余裕をみた値（GRIP_LAT_USE）で考え、曲がれる速さで回っている間は加速しない。
//   前後は測った値そのまま（真っすぐなら、タイヤが空回りしない一番強い加速をする）
//**********************************************************************
float long_avail_use(float v, float k_abs) {
    float r = v * v * k_abs / GRIP_LAT_USE;
    if (r >= 1.0f) return 0.0f;
    return GRIP_LONG_MPS2 * sqrtf(1.0f - r * r);
}

// 速さ v から、距離 d の間 GRIP_LONG_USE で減速したときの速さ
float speed_after_brake(float v, float d) {
    float q = v * v - 2.0f * GRIP_LONG_USE * d;
    return q > 0.0f ? sqrtf(q) : 0.0f;
}

//**********************************************************************
// 通常走行・大曲がりの新しいモータ出力（USE_NEW_CURVE_PREVIEW）
//   k_ahead : この先の曲率 [1/m]（推定したセンサの所の曲がり具合 obs_z3）
//   速さの目標は次の3つの小さい方
//    ① 今曲がっている曲率（サーボの本当の角度の推定から求め、PREVIEW_CURV_FILTER_S でならす）を、横すべりせずに回れる速さ
//    ② この先の曲率を回れる速さ v_a に、そこまでの距離 d = PREVIEW_RATIO × SENSOR_TO_REAR_AXLE_M の間に
//       減速して間に合う速さ √(v_a² + 2·GRIP_LONG_USE·d)。カーブの入口では、センサの所の曲がり具合が
//       後輪が着く前に大きくなるので、ハンドルが切れるより前に減速が始まる
//    ③ 直線の速さの上限（TRACE_SAFE_RADIUS_M）
//   出力は「次の1msでちょうど目標の速さになる値」（デッドビート）。ただし
//    ・加速は摩擦円の残りまで（long_avail_use。曲がれる速さで回っている間は加速しない）
//    ・ブレーキは GRIP_LONG_USE の減速度になる出力まで（それより強く逆転させても、タイヤがすべって減速は増えない。
//      すべらない強さにしておくと、曲がる力が残る）
//   カーブの出口では、ハンドルが戻るにつれて本当の角度の曲率が小さくなり、①が上がって自然に全開へ向かう
//**********************************************************************
void trace_motor_preview(float k_ahead) {
    float v = speed_mps;
    float k_act = curvature_of_angle(servo_est_deg);
    // 今の曲がり具合：本当の角度の曲率を PREVIEW_CURV_FILTER_S でならした値（センサのきざみでハンドルが揺れる分をならす）。
    // 横すべりして線から外へずれると、ハンドルを大きく切るのでここが大きくなり、減速する（グリップを大きく見ていた時の守り）
    trace_k_filt = trace_k_filt + (fabsf(k_act) - trace_k_filt) * (0.001f / PREVIEW_CURV_FILTER_S);
    float kn = trace_k_filt;
    float vt = corner_speed(kn);                                     // ①
    float va = corner_speed(fabsf(k_ahead));
    float env = sqrtf(va * va + 2.0f * GRIP_LONG_USE * PREVIEW_RATIO * SENSOR_TO_REAR_AXLE_M);
    if (env < vt) vt = env;                                          // ②
    if (vt > trace_v_cap) vt = trace_v_cap;                          // ③
    float u = v + (vt - v) * (MOTOR_TAU_S / 0.001f);
    float u_hi = v + MOTOR_TAU_S * long_avail_use(v, fabsf(k_act));
    float u_lo = v - MOTOR_TAU_S * GRIP_LONG_USE;
    if (u > u_hi) u = u_hi;
    if (u < u_lo) u = u_lo;
    if (u > v_max_mps) u = v_max_mps;
    if (u < -v_max_mps) u = -v_max_mps;
    drive_center(u, k_act);
}

//**********************************************************************
// 車の中心の指令の速さ u と曲率 k から左右の出力を決める（速さを優先する。USE_NEW_CURVE_PREVIEW）
//   車の中心の速さは左右の出力の平均で決まり、左右の差は内輪と外輪の回る速さの違いを合わせるだけのもの。
//   drive_wheels は外輪が100%を超えるとき両方を下げるので、全開で曲がると中心の速さが落ちる（半径1mで約7%）。
//   ここでは平均（中心の速さ）を保ち、左右の差の方を ±100% に収まるまで小さくする
//**********************************************************************
void drive_center(float u, float k) {
    float d = u * k * TREAD_M * 0.5f;             // 外輪と内輪の差の半分（右に曲がる k>0 とき左が外輪）
    float room = v_max_mps - fabsf(u);
    if (room < 0.0f) room = 0.0f;
    if (d > room) d = room;
    if (d < -room) d = -room;
    int pl = power_of_speed(u + d);
    int pr = power_of_speed(u - d);
    if (pl < TRACE_MIN_POWER) pl = TRACE_MIN_POWER;
    if (pr < TRACE_MIN_POWER) pr = TRACE_MIN_POWER;
    sp(pl, pr);
}

//**********************************************************************
// config.h の車の値が、計算に使える範囲にあるかを調べる（setup で1回）
//   範囲の外なら、速度の推定を使う新しい走行アルゴリズムは使わず、元の表の動きにする（model_ok = 0）
//**********************************************************************
int model_check_config(void) {
    if (!(V_MAX_MPS > 0.3f && V_MAX_MPS < 20.0f)) return 0;
    if (!(MOTOR_TAU_S > 0.005f && MOTOR_TAU_S < 2.0f)) return 0;
    if (!(GRIP_LAT_MPS2 > 0.5f && GRIP_LONG_MPS2 > 0.5f)) return 0;
    if (!(PLAN_GRIP_RATIO_LAT > 0.1f && PLAN_GRIP_RATIO_LAT <= 1.5f)) return 0;
    if (!(PLAN_GRIP_RATIO_LONG > 0.1f && PLAN_GRIP_RATIO_LONG <= 1.5f)) return 0;
    if (!(WHEELBASE_M > 0.02f && TREAD_M > 0.02f && SENSOR_TO_REAR_AXLE_M > 0.02f)) return 0;
    if (!(SERVO_SPEED_DPS > 10.0f && SERVO_DEAD_S >= 0.0f && SERVO_DEAD_S < 0.1f)) return 0;
    if (!(SERVO_MAX_ANGLE > 5 && SERVO_MAX_ANGLE < 80)) return 0;
    if (!(CROSS_LINE_WIDTH_M >= 0.0f && HALF_LINE_WIDTH_M >= 0.0f)) return 0;
    if (COURSE_LIST_USE && !course_list_check()) return 0;   // クランク・レーンチェンジごとの寸法の表
    return 1;
}
