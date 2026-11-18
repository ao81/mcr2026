//**************************************************************************
// speed_model.ino  速度の推定と、計算による減速・速度の決め方（新しい走行アルゴリズム）
//
//  Basicマイコンカーには速度を測るセンサ（エンコーダ）が無い。そこで次の2つで速さを求める。
//   1. モータに出した出力から、車の速さを毎ms計算する（一次遅れのモデル）
//        v ← v + (出力/100 × V_MAX_MPS − v) × 0.001 ÷ MOTOR_TAU_S
//   2. 幅の分かっているライン（クロスライン・ハーフライン）を通過した時間で、v を測り直す
//        v = ラインの幅 ÷ 通過時間
//
//  クランク・レーンチェンジ手前の減速は「最短時間で目標の速さまで落とす」問題になる。
//  加速と減速の強さに上限がある場合、最短時間になるのは
//    全開で進む → 止まりきれる限界の地点から最大ブレーキ → 目標の速さを保つ
//  という切り替え方（バン・バン制御）であることが知られている。
//  ここでは「今ブレーキをかけたら何m で目標の速さまで落ちるか」を毎ms計算し、
//  それが残りの距離に達した瞬間にブレーキへ切り替える。
//**************************************************************************

// 使う変数 speed_mps（速さ）と dist_m（距離）は basic.ino で宣言している

//**********************************************************************
// 1msごとに呼ぶ：最後に出したモータ出力から速さと距離を更新
//**********************************************************************
void speed_model_update(void) {
    const float dt = 0.001f;
    float power = (log_raw.ml + log_raw.mr) * 0.5f;          // DIPスイッチで縮小した後の出力
    float target = power / 100.0f * V_MAX_MPS;
    speed_mps = speed_mps + (target - speed_mps) * (dt / MOTOR_TAU_S);
    dist_m = dist_m + speed_mps * dt;
}

//**********************************************************************
// ラインの通過時間から速さを測り直す
//   width_m : ラインの進行方向の幅 [m]（0 なら何もしない）
//**********************************************************************
void speed_measure_by_line(float width_m, int pass_ms) {
    if (width_m <= 0.0f) return;
    float t = (pass_ms + LINE_PASS_OFFSET_MS) * 0.001f;
    if (t <= 0.0f) return;
    speed_mps = width_m / t;
}

// 距離を0に戻す（ラインを通過し終わった時に呼ぶ）。ブレーキの状態も最初に戻す
void reset_distance(void) {
    dist_m = 0.0f;
    brake_started = 0;
}

//**********************************************************************
// 出力 brake_power で減速し続けたとき、v から v_target まで落ちるのに進む距離 [m]
//   一次遅れのモデルでは、出力 p のときの行き着く速さ vb = p/100 × V_MAX_MPS に向かって
//     v(t) = vb + (v − vb)·e^(−t/τ)
//   となるので、v_target になる時刻 t* = τ·ln((v − vb) ÷ (v_target − vb))、
//   その間の距離 = vb·t* + τ·(v − v_target)
//**********************************************************************
float braking_distance(float v, float v_target, int brake_power) {
    if (v <= v_target) return 0.0f;
    float vb = brake_power / 100.0f * V_MAX_MPS;
    if (vb >= v_target) return 1.0e6f;                // このブレーキでは目標の速さまで落ちない
    float t_star = MOTOR_TAU_S * logf((v - vb) / (v_target - vb));
    return vb * t_star + MOTOR_TAU_S * (v - v_target);
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
            float next_v = v + (V_MAX_MPS - v) * (0.001f / MOTOR_TAU_S);
            float need = next_v * 0.001f + braking_distance(next_v, v_target, brake_power);
            if (remain_m - margin_m > need) {
                return 100;                             // まだ全開でよい
            }
            brake_started = 1;  // 一度ブレーキを始めたら、目標の速さになるまで続ける（全開と交互にならないように）
        }
        return brake_power;                             // ブレーキ
    }
    // 目標の速さ以下：その速さを保つ出力（モータの出力と速さは比例すると考える）
    int hold = (int)(v_target / V_MAX_MPS * 100.0f + 0.5f);
    return clamp_int(hold, 0, 100);
}

//**********************************************************************
// クロスライン通過後（pattern 102, 103）の新しい減速
//**********************************************************************
void crank_speed_control_new(void) {
    int p = brake_power_for(CROSS_TO_CRANK_M - dist_m, CRANK_ENTRY_MPS, CRANK_BRAKE_POWER, CRANK_BRAKE_MARGIN_M);
    sp(p, p);
}

//**********************************************************************
// ハーフライン通過後（pattern 202, 300〜310 / 502, 600〜610）の新しい減速
//**********************************************************************
void lane_speed_control_new(void) {
    int p = brake_power_for(HALF_TO_LANE_M - dist_m, LANE_ENTRY_MPS, LANE_BRAKE_POWER, LANE_BRAKE_MARGIN_M);
    sp(p, p);
}

//**********************************************************************
// 通常走行の新しいモータ出力（ハンドル角度から計算）
//   angle : ハンドル角度[度]（プラスが右）
//**********************************************************************
void trace_motor_new(int angle) {
    if (angle == 0) {
        sp(100, 100);
        return;
    }
    float th = fabsf((float)angle) * 3.14159265f / 180.0f;
    float r = WHEELBASE_M / tanf(th);                         // 後輪の軸の中心が描く円の半径
    float v_limit = sqrtf(LATERAL_ACCEL * r);                 // 横すべりしない速さの上限
    int outer = clamp_int((int)(v_limit / V_MAX_MPS * 100.0f), TRACE_MIN_POWER, 100);
    float ratio = (r - TREAD_M * 0.5f) / (r + TREAD_M * 0.5f); // 内輪 ÷ 外輪
    if (ratio < 0.0f) ratio = 0.0f;
    int inner = (int)(outer * ratio + 0.5f);
    if (angle > 0) {
        sp(outer, inner);        // 右に曲がる：右が内輪
    } else {
        sp(inner, outer);        // 左に曲がる：左が内輪
    }
}
