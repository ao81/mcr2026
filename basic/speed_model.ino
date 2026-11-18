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
    float power = (log_raw.ml + log_raw.mr) * 0.5f;          // DIPスイッチで縮小した後の出力
    float target = power / 100.0f * v_max_mps;
    float v = speed_mps;
    float a = (target - v) / MOTOR_TAU_S;
    float a_up = accel_up_limit(v, fabsf(curvature_of_angle(log_raw.ang)));
    if (a > a_up) a = a_up;                                   // タイヤがすべるほどの加速はできない
    if (a < -GRIP_LONG_USE) a = -GRIP_LONG_USE;               // ブレーキは余裕をみた値（遅めに減速すると考える）
    v = v + a * dt;
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
#if USE_NEW_VMAX_LEARN
    // 推定していた速さが測った速さより速いなら、出力100%の速さを実際より大きく見ていたことになる。
    // モデルは出力に比例するので、比の分だけ v_max_mps を直す（1回の測定の誤差が大きいので VMAX_LEARN_GAIN 分ずつ）
    if (speed_mps > 0.5f) {
        float ratio = v_meas / speed_mps;
        float vm = v_max_mps * (1.0f + VMAX_LEARN_GAIN * (ratio - 1.0f));
        float lo = V_MAX_MPS * (1.0f - VMAX_LEARN_RANGE);
        float hi = V_MAX_MPS * (1.0f + VMAX_LEARN_RANGE);
        v_max_mps = vm < lo ? lo : (vm > hi ? hi : vm);
    }
#endif
    speed_mps = v_meas;
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
    int i = (int)(p >= 0.0f ? p + 0.5f : p - 0.5f);
    return clamp_int(i, -100, 100);
}

//**********************************************************************
// 残りの距離と目標の速さから、今出すべきモータ出力を決める
//   remain_m : 目標地点までの残り距離 [m]
//**********************************************************************
int brake_power_for(float remain_m, float v_target, int brake_power, float margin_m) {
    float v = speed_mps;
    if (v > v_target) {
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
    // 目標の速さ以下：その速さに合わせる
    return power_of_speed(deadbeat_speed(v, v_target, 0.0f));
}

//**********************************************************************
// クロスライン通過後（pattern 102, 103）の新しい減速
//**********************************************************************
void crank_speed_control_new(void) {
    int p = brake_power_for(CROSS_TO_CRANK_M - dist_m, crank_entry_mps, CRANK_BRAKE_POWER, CRANK_BRAKE_MARGIN_M);
    sp(p, p);
}

//**********************************************************************
// ハーフライン通過後（pattern 202, 300〜310 / 502, 600〜610）の新しい減速
//**********************************************************************
void lane_speed_control_new(void) {
    int p = brake_power_for(HALF_TO_LANE_M - dist_m, lane_entry_mps, LANE_BRAKE_POWER, LANE_BRAKE_MARGIN_M);
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
