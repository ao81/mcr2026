//**************************************************************************
// run_lane.ino  右・左レーンチェンジ
//   right = true が右レーンチェンジ（pattern 200〜450）、false が左（500〜750）
//   段階のある pattern は「先頭の番号 + 段階(0〜10)」。段階はハーフラインの通過時間で決まる
//**************************************************************************

// 右/左で pattern 番号を切り替えるための関数
int lane_pt(bool right, int pt_right, int pt_left) {
    return right ? pt_right : pt_left;
}

// 右レーンチェンジ中に新しい車線の線（右端）が見えたか / 左なら左端
bool lane_new_line_found(unsigned char c, bool right) {
    if (right) return (c == 0x01 || c == 0x02 || c == 0x03);
    return (c == 0x80 || c == 0x40 || c == 0xc0);
}

// ハーフラインの後にクロスラインだったと分かったか（右と左で判定が違うのは元のプログラムどおり）
int lane_cross_recheck(bool right) {
    return right ? check_crossline() : check_hidarikakunin();
}

//**********************************************************************
// pattern 200 / 500 : ハーフラインを検出した
//**********************************************************************
void run_lane_found(bool right) {
    lane_offset_m = 0.0f;
    led_out(right ? 0x1 : 0x2);
    ha(0);
    line_pass_state = PASS_WAIT;
    cnt1 = 0;
    pattern = lane_pt(right, PT_LANE_R_PASS, PT_LANE_L_PASS);
    lane_spot_select(right);                       // これから来るレーンチェンジの距離・ハーフラインの幅（config.h の COURSE_LIST）
#if USE_NEW_BRAKE_RANGE && USE_NEW_LANE_BRAKE
    // センサはハーフラインの始まりにいる。距離をラインの幅の分マイナスから数え、すぐに減速の計算を始める
    if (model_ok) {
        reset_distance();
        dist_m = -lane_line_w_m;
        lane_speed_control_new();
    }
#endif
}

//**********************************************************************
// pattern 201 / 501 : ハーフラインの通過時間を測る
//**********************************************************************
void run_lane_pass(bool right) {
    if (lane_cross_recheck(right)) {
        cnt1 = 0;
        pattern = PT_CROSS_FOUND;
        return;
    }
#if USE_NEW_BRAKE_RANGE && USE_NEW_LANE_BRAKE
    if (model_ok) lane_speed_control_new();        // 通過中も減速の計算を続ける
#endif
    measure_line_pass(right ? check_shasenhenkoum() : check_shasenhenkouh());
    if (line_pass_state == PASS_DONE) {
        speed_measure_by_line(lane_line_w_m, line_pass_ms);    // 通過時間から速さを測り直す
        reset_distance();                                          // ここから車線変更地点までの距離を数える
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_SELECT, PT_LANE_L_SELECT);
        return;
    }
    apply_table(LINE_STEER_SENSOR, LINE_STEER_ACT, LINE_STEER_N, sensor_inp(MASK4_4));
}

//**********************************************************************
// pattern 202 / 502 : 通過時間から段階 0〜10 を決める
//**********************************************************************
void run_lane_select(bool right) {
    if (lane_cross_recheck(right)) {
        cnt1 = 0;
        pattern = PT_CROSS_FOUND;
        return;
    }
    line_pass_state = PASS_WAIT;
    lane_spot_done();                              // クロスラインではなかった。向きを表と比べ、次のレーンチェンジへ数を進める
    lane_speed_control();

    int stage = LANE_STAGES - 1;
    for (int i = 0; i < LANE_STAGES - 1; i++) {
        if (line_pass_ms <= LANE_STAGE_LIMIT[i]) {
            stage = i;
            break;
        }
    }
    pattern = lane_pt(right, PT_LANE_R_APPROACH, PT_LANE_L_APPROACH) + stage;
}

//**********************************************************************
// pattern 300〜310 / 600〜610 : 減速しながら直進し、中央線が切れたらハンドルを切る
//**********************************************************************
void run_lane_approach(int stage, bool right) {
    lane_speed_control();
    if (cnt1 > LANE_START_WAIT_MS) {
        if (sensor_inp(MASK4_4) == 0x00) {   // 中央線が切れた → 車線変更開始
#if USE_NEW_LANE_DISTANCE && (USE_NEW_LANE_STEER || USE_NEW_LANE_SNAP)
            correct_lane_speed_by_distance();
#endif
#if USE_NEW_LANE_SNAP
            if (lane_start_snap(right)) return;           // グイッと曲がり、新しい線が見えたら戻す
#endif
#if USE_NEW_LANE_STEER
            if (lane_start_scurve(stage, right)) return;   // 計算した2つの円弧で曲がる
#endif
            line_pass_state = PASS_WAIT;
            led_out(right ? 0x1 : 0x2);
            lane_steer_output(stage, right);
            cnt1 = 0;
            pattern = lane_pt(right, PT_LANE_R_STEER, PT_LANE_L_STEER) + stage;
            return;
        }
    }
    unsigned char c = sensor_inp(MASK4_4);
#if USE_NEW_LANE_STEER
    // 計算した経路は元の線の真上から曲がり始めるのが一番緩くなるので、線の中央を走る
    // （元の表は、車線を変える側へ少し寄せる）。線に対する車の位置も記録しておく
    apply_table(LINE_STEER_SENSOR, LINE_STEER_ACT, LINE_STEER_N, c);
    if (c != 0x00) lane_offset_m = -line_position_m(c);
    (void)stage;
#else
    if (right) {
        apply_table(LANE_R_APPROACH_SENSOR, LANE_R_APPROACH_ACT[stage], LANE_R_APPROACH_N, c);
    } else {
        apply_table(LANE_L_APPROACH_SENSOR, LANE_L_APPROACH_ACT[stage], LANE_L_APPROACH_N, c);
    }
#endif
}

// 車線変更でハンドルを切っている間の出力
void lane_steer_output(int stage, bool right) {
    if (right) {
        ha(LANE_STEER_ANGLE);
        sp(LANE_STEER_OUTER[stage], LANE_STEER_INNER[stage]);
    } else {
        ha(-LANE_STEER_ANGLE);
        sp(LANE_STEER_INNER[stage], LANE_STEER_OUTER[stage]);
    }
}

//**********************************************************************
// pattern 320〜330 / 620〜630 : ハンドルを切って隣の車線へ
//**********************************************************************
void run_lane_steer(int stage, bool right) {
    lane_steer_output(stage, right);
    unsigned char c = sensor_inp(MASK4_4);
    if (cnt1 > LANE_STEER_MS[stage]) {       // 切り終わった → 直進へ
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_STRAIGHT, PT_LANE_L_STRAIGHT) + stage;
        return;
    }
    if (lane_new_line_found(c, right)) {
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_ALIGN, PT_LANE_L_ALIGN) + stage;
    }
}

//**********************************************************************
// pattern 340〜350 / 640〜650 : ハンドルを少し戻して、新しい線が見えるまで直進
//**********************************************************************
void run_lane_straight(int stage, bool right) {
    ha(right ? -LANE_STRAIGHT_ANGLE : LANE_STRAIGHT_ANGLE);
    sp(LANE_STRAIGHT_SPEED[stage], LANE_STRAIGHT_SPEED[stage]);
    unsigned char c = sensor_inp(MASK4_4);
    if (lane_new_line_found(c, right)) {
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_ALIGN, PT_LANE_L_ALIGN) + stage;
    }
}

//**********************************************************************
// pattern 400〜410 / 700〜710 : 新しい線に車体の向きを合わせる
//**********************************************************************
void run_lane_align(int stage, bool right) {
    unsigned char c = sensor_inp(MASK4_4);
    if (right) {
        apply_table(LANE_R_ALIGN_SENSOR, LANE_R_ALIGN_ACT[stage], LANE_R_ALIGN_N, c);
    } else {
        apply_table(LANE_L_ALIGN_SENSOR, LANE_L_ALIGN_ACT[stage], LANE_L_ALIGN_N, c);
    }
    if (right ? (c == 0x70 || c == 0x60) : (c == 0x0e || c == 0x06)) {
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_FINISH, PT_LANE_L_FINISH) + stage;
    }
}

//**********************************************************************
// pattern 420〜430 / 720〜730 : 線が中央に来たら通常走行へ
//**********************************************************************
void run_lane_finish(int stage, bool right) {
    unsigned char c = sensor_inp(MASK4_4);
    if (c == 0x3c || c == 0x1c || c == 0x18 || c == 0x38) {
        led_out(0x0);
        cnt1 = 0;
        pattern = PT_TRACE;
        return;
    }
    if (right) {
        apply_table(LANE_R_FINISH_SENSOR, LANE_R_FINISH_ACT[stage], LANE_R_FINISH_N, c);
    } else {
        apply_table(LANE_L_FINISH_SENSOR, LANE_L_FINISH_ACT[stage], LANE_L_FINISH_N, c);
    }
    if (right ? (c == 0xc0) : (c == 0x03)) {  // 外へふくらんだ
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_RECOVER, PT_LANE_L_RECOVER) + stage;
    }
}

//**********************************************************************
// pattern 440〜450 / 740〜750 : 外へふくらんだ時の立て直し
//**********************************************************************
void run_lane_recover(int stage, bool right) {
    unsigned char c = sensor_inp(MASK4_4);
    if (right ? (c == 0x60 || c == 0xe0) : (c == 0x06 || c == 0x07)) {
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_FINISH, PT_LANE_L_FINISH) + stage;
        return;
    }
    if (right) {
        apply_table(LANE_R_RECOVER_SENSOR, LANE_R_RECOVER_ACT[stage], LANE_R_RECOVER_N, c);
    } else {
        apply_table(LANE_L_RECOVER_SENSOR, LANE_L_RECOVER_ACT[stage], LANE_L_RECOVER_N, c);
    }
}

//**********************************************************************
// 中央線が切れた時に、計算した2つの円弧で車線変更できるか調べて、できるなら 360 / 660 + 段階 へ（USE_NEW_LANE_STEER）
//   戻り値 true: 円弧で曲がる  false: 元の方法で曲がる
//**********************************************************************
bool lane_start_scurve(int stage, bool right) {
    if (!model_ok) return false;                   // 速さの推定が当てにならないときは元の方法で曲がる
    float r, len, wait;
    float v = speed_mps;
    float x_start = right ? lane_offset_m : -lane_offset_m;   // 車線を変える側へずれている分
    float x_used = x_start;
    // 経路は setup で計算した表から引く（planner.ino）。表で足りないときだけ、ここで計算する
    if (!lane_table_plan(v, x_start, &r, &len, &wait, &x_used)) {
        if (!lane_plan(v, x_start, &r, &len, &wait)) return false;
    }
    x_start = x_used;
    float v_turn = corner_speed(1.0f / r);
    float v_chk = v;
#if USE_NEW_BRAKE_RANGE && USE_NEW_LANE_BRAKE
    // S字に入るまでの直進とサーボの遅れの間も減速を続けるので、S字に入る時の速さで判断する
    v_chk = speed_after_brake(v, pre_turn_m(v, r, wait));
#endif
    if (v_chk > v_turn * PLAN_OVERSPEED) return false; // 速すぎて、この円弧では横すべりする

    float d = LANE_SHIFT_M - x_start;              // 横に動く距離
    if (LANE_QUICK_LAT_MPS2 > 0.0f) {
        // 一番緩い経路の代わりに、横の加速度が LANE_QUICK_LAT_MPS2 になる短い経路で、早く横に動き終える
        len = lane_quick_len(v_chk, d, x_start, r, len, wait);
        r = lane_r_for(len, d);
        v_turn = corner_speed(1.0f / r);
    }
    float phi = 2.0f * atanf(d / len);             // 円弧1つで回る角度 [rad]
    float ang = angle_for_radius(r);
    plan_wait_m = wait;
    plan_len_m = r * phi;                          // 円弧1つの長さ
    // サーボが +角度 から −角度 へ動き終わるまでの真ん中で、曲がる向きが変わったとみなせる。
    // その分だけ早く切り返すと、1つ目と2つ目の円弧の長さがそろう（終わりの位置は 2 × 円弧の長さ のまま）
    plan_switch_m = plan_len_m - v_turn * ang / (2.0f * SERVO_SPEED_DPS);
    lane_end_m = 2.0f * plan_len_m;
    lane_sw0_m = plan_switch_m;
    lane_cross_m = (LANE_CROSS_ADJUST_M > 0.0f) ? lane_cross_plan(r, phi, d, v_turn, ang) : -1.0f;
    plan_speed_mps = v_turn;
    plan_angle = ang;
    reset_distance();                              // ここから距離を数える
    line_pass_state = PASS_WAIT;
    led_out(right ? 0x1 : 0x2);
    cnt1 = 0;
    pattern = lane_pt(right, PT_LANE_R_SCURVE, PT_LANE_L_SCURVE) + stage;
    run_lane_scurve(stage, right);                 // この1msの出力もすぐ出す
    return true;
}

//**********************************************************************
// S字の1つ目の円弧で、センサが新しい車線の中央線の真上を通る所（曲がり始めてからの距離 [m]）。通らなければ −1
//   1つ目の円弧で向きが α 変わった時、センサ（後輪の軸の SENSOR_TO_REAR_AXLE_M 前）の横のずれは
//     r(1 − cos α) + SENSOR_TO_REAR_AXLE_M·sin α
//   これが新しい線までの横の距離 d になる α を二分法で求める。サーボが効くまでの遅れの間に進む分を足す
//**********************************************************************
float lane_cross_plan(float r, float phi, float d, float v, float ang) {
    const float sf = SENSOR_TO_REAR_AXLE_M;
    if (r * (1.0f - cosf(phi)) + sf * sinf(phi) < d) return -1.0f;
    float lo = 0.0f, hi = phi;
    for (int i = 0; i < 12; i++) {
        float mid = (lo + hi) * 0.5f;
        if (r * (1.0f - cosf(mid)) + sf * sinf(mid) < d) lo = mid; else hi = mid;
    }
    lane_cross_lag_m = v * servo_delay_s(ang);
    return r * lo + lane_cross_lag_m;
}

//**********************************************************************
// S字の1つ目の円弧の間に、センサが新しい中央線の真上を通った所を見て、切り返す所と終わりをずらす（LANE_CROSS_ADJUST_M）
//   実際の曲がり方が計算より弱い（サーボの遊び・タイヤの横すべりなど）と、新しい線を通るのが計算より遅れる。
//   遅れた距離 Δ だけ切り返す所と終わりを後ろへずらし、残りはハンドル角度を直して計算どおりの半径で曲がる。
//   計算より早く通ったとき（曲がり方が強い）は、前へずらして角度を小さくする。ずらすのは ±LANE_CROSS_ADJUST_M まで
//   2026-11-26：実車でレーンチェンジを直進してコースアウトした。S字は約 6°の小さなハンドルで、センサを見ずに 1.3m 走るので、
//   模擬では、曲がり方が 15% 弱いだけで7回とも外に出た
//**********************************************************************
void lane_cross_check(float s, bool right) {
    unsigned char c = sensor_inp(MASK4_4);
    bool crossed = false;
    if (c != 0x00 && s > lane_cross_m * 0.5f) {
        float pos = line_position_m(c);                            // プラスが右
        crossed = right ? (pos <= 0.0f) : (pos >= 0.0f);           // 線が真ん中まで来た（右へ移るときは、線はセンサの右から左へ流れる）
    }
    if (!crossed && s < lane_cross_m + LANE_CROSS_ADJUST_M) {
        // まだ通っていない。計算の切り返しの所まで来ても、通るまで（最大 LANE_CROSS_ADJUST_M 遅れまで）1つ目の円弧を続ける
        if (s >= plan_switch_m - 0.001f) plan_switch_m = s + 0.002f;
        return;
    }
    float delta = s - lane_cross_m;                                // プラス：計算より遅れて通った
    if (delta > LANE_CROSS_ADJUST_M) delta = LANE_CROSS_ADJUST_M;
    if (delta < -LANE_CROSS_ADJUST_M) delta = -LANE_CROSS_ADJUST_M;
    // 新しい線を通った所では、車の向きは計算の通る所と同じで、後輪の横のずれは計算以上になっている（センサの横のずれは
    // ほとんど SENSOR_TO_REAR_AXLE_M·sin α で決まるため）。つまり計算の経路を Δ 遅れて走っているのと同じなので、
    // 切り返す所と終わりを Δ 後ろへずらし、ここからは計算どおりの半径で曲がるように、ハンドル角度を効きの比 g で直す。
    //   g ≒ 計算で通る距離 ÷ 実際に通った距離（サーボが効くまでに進む分は除く）。0.6〜1.4 倍まで
    float lag = lane_cross_lag_m;
    float g = (lane_cross_m - lag) / (lane_cross_m + delta - lag);
    if (g < 0.6f) g = 0.6f;
    if (g > 1.4f) g = 1.4f;
    plan_angle = angle_for_radius(radius_of_angle(plan_angle) * g);
    float sw = lane_sw0_m + delta;
    if (sw < s) sw = s;                                            // 早く通ったときは、すぐ切り返す
    delta = sw - lane_sw0_m;
    plan_switch_m = sw;
    lane_end_m = 2.0f * plan_len_m + delta;
    lane_cross_m = 0.0f;                                           // 1回だけ
}

//**********************************************************************
// pattern 360〜370 / 660〜670 : 計算した2つの円弧で隣の車線へ（USE_NEW_LANE_STEER）
//   plan_wait_m だけ直進 → 1つ目の円弧（車線を変える向き）→ 2つ目の円弧（逆向き）で、向きを元に戻しながら横にずれる
//   この間はセンサを見ない（新しい線が斜めに見えても、経路を最後まで走る）
//**********************************************************************
void run_lane_scurve(int stage, bool right) {
    const int dir = right ? 1 : -1;
    float s = dist_m - plan_wait_m;
    if (s >= 0.0f && lane_cross_m > 0.0f) lane_cross_check(s, right);
    if (s < 0.0f) {
        ha(0);
        drive_at(plan_speed_mps, 0);
    } else if (s < plan_switch_m) {
        ha_deg(dir * plan_angle);
        drive_at(plan_speed_mps, dir * plan_angle);
    } else if (s < lane_end_m) {
        ha_deg(-dir * plan_angle);
        drive_at(plan_speed_mps, -dir * plan_angle);
    } else {
        ha(0);
        drive_at(plan_speed_mps, 0);
        reset_distance();                          // ここからは線を探した距離を数える
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_SPOST, PT_LANE_L_SPOST) + stage;
    }
}

//**********************************************************************
// pattern 380〜390 / 680〜690 : 円弧を走り終えた後、新しい線を探す（USE_NEW_LANE_STEER）
//   線が通常走行の表にある形で見えたら通常走行へ。手前側の端にだけ見えたら（横に動き足りない）元の処理 400 / 700 へ
//   それ以外の形で見えたら（速さの推定の誤差で、行き過ぎた場合など）、線の方へハンドルを切る。
//   角度は線の位置から計算し（ピュアパシュート）、今の速さで横すべりしない角度までにする
//   LANE_SEARCH_M 走っても見えなければ、新しい車線の側へ LANE_STRAIGHT_ANGLE だけハンドルを切る
//**********************************************************************
void run_lane_spost(int stage, bool right) {
    unsigned char c = sensor_inp(MASK4_4);
    if (find_sensor(TRACE_SENSOR, TRACE_N, c) >= 0) {
        led_out(0x0);
        cnt1 = 0;
        pattern = PT_TRACE;
        return;
    }
    if (lane_new_line_found(c, right)) {
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_ALIGN, PT_LANE_L_ALIGN) + stage;
        return;
    }
    float ang = 0.0f;
    if (c != 0) {
        float lim = angle_for_radius(plan_speed_mps * plan_speed_mps / GRIP_LAT_USE);
        ang = (float)pursuit_angle(c);
        if (ang > lim) ang = lim;
        if (ang < -lim) ang = -lim;
    } else if (dist_m > LANE_SEARCH_M) {
        ang = right ? LANE_STRAIGHT_ANGLE : -LANE_STRAIGHT_ANGLE;
    }
    ha_deg(ang);
    drive_at(plan_speed_mps, ang);
}

//**********************************************************************
// ハーフライン通過後のモータ出力（元: shasenhenkou）
//   通過時間 line_pass_ms と経過時間 cnt1 で決める。詳しくは config.h の説明を参照
//**********************************************************************
void lane_speed_control(void) {
#if USE_NEW_LANE_BRAKE
    if (model_ok) {                                // 速さの推定が当てにならないときは元の表で減速する
        lane_speed_control_new();
        return;
    }
#endif
    int i = clamp_int(line_pass_ms - LANE_K_MIN, 0, LANE_STEPS - 1);
    long t = (long)cnt1;
    int power;
    if (t < LANE_FULL_MS[i]) {
        power = 100;
    } else if (t < LANE_HOLD_END_MS[i]) {
        power = LANE_RAMP[i];
    } else {
        long j = i + 1 + (t - LANE_HOLD_END_MS[i]) / LANE_RAMP_STEP_MS;
        if (j > LANE_STEPS - 1) j = LANE_STEPS - 1;
        power = LANE_RAMP[j];
    }
    sp(power, power);
}

//**********************************************************************
// グイッと曲がるレーンチェンジ（USE_NEW_LANE_SNAP）
//   中央線の切れ目を見つけたら、すぐに変える側へハンドルを切る（pattern 460 / 760）。
//   角度は LANE_SNAP_ANGLE まで。速いときは、横の加速度 v²·曲率 が LANE_SNAP_LAT_MPS2 以下になる角度に小さくする。
//   戻す側へ切るとき（pattern 470 / 770）も同じ上限まで切る。
//
//   新しい線を見てから戻し始めると、サーボが反対へ動き切るまで（約 0.1 秒）曲がり続け、
//   向きが付きすぎて、センサが新しい線を越えて外へ出る（LANE0052：線を見た時に向き 47°、戻しは 17.3° で頭打ち）。
//   そこで、切れ目からの車の向き ψ と後輪の横の位置 y を、ハンドル角度と速さから積み上げて計算し、
//   後輪から LANE_SNAP_POINT_M 前の点の横の位置 yc = y + Lc·sin ψ が新しい線（LANE_SHIFT_M 横）に着く前から戻し始める：
//     ・サーボの遅れ（反対へ動き切るまでの時間）の分だけ先の ψ と yc を予測する
//     ・その点が線に近づく速さを、ずれ ÷ LANE_SNAP_AHEAD_M にする曲がり具合
//         κ = (ずれ ÷ A − sin ψ) ÷ (Lc·cos ψ)
//       で曲がる（ずれが大きいうちは上限まで変える側へ切り、近づくと自然に反対へ切る）
//     ・Lc をセンサの距離より短くするのは、後輪を早く新しい車線に入れるため。センサの距離の点を線に乗せると、
//       車が斜めのまま長く走り、合流部分の終わりの角（棒）に後ろが当たる。代わりにセンサは少し線の向こうへふくらむ
//   新しい線がセンサに見えたら、その位置で y を直す（積み上げの誤差を消す。見え続けている間は向き ψ も少しずつ直す）。
//   線から離れすぎた白（新しい車線の向こう側の路肩の線など）は使わない（一度見る前は LANE_SNAP_GATE_M の 2.5 倍まで使う）。
//   ψ が LANE_SNAP_END_DEG 以下で、線が通常走行の表の形に見えたら通常走行へ
//**********************************************************************

static float lane_snap_cmd_deg;   // USE_NEW_LANE_SNAP：前のハンドルの指令 [度]（変える側がプラス）

static float lane_snap_angle(float v) {
    float ang = (float)LANE_SNAP_ANGLE;
    if (LANE_SNAP_LAT_MPS2 > 0.0f && v > 0.1f) {
        float a = angle_for_radius(v * v / LANE_SNAP_LAT_MPS2);
        if (a < ang) ang = a;
    }
    return ang;
}

bool lane_start_snap(bool right) {
    if (!model_ok) return false;                   // 速さの推定が当てにならないときは元の方法で曲がる
    plan_angle = lane_snap_angle(speed_mps);
    plan_speed_mps = speed_mps;
    lane_yaw = 0.0f;
    lane_snap_y = right ? lane_offset_m : -lane_offset_m;   // 切れ目の直前に線からずれていた分（変える側がプラス）
    lane_snap_seen = 0;
    lane_snap_cmd_deg = 0.0f;
    reset_distance();
    line_pass_state = PASS_WAIT;
    led_out(right ? 0x1 : 0x2);
    cnt1 = 0;
    pattern = lane_pt(right, PT_LANE_R_SNAP, PT_LANE_L_SNAP);
    run_lane_snap(right);
    return true;
}

// ψ と y を 1ms 分進める
//   速さの推定 speed_mps は V_MAX_MPS が本当の最高速度より小さいと遅く出るので、LANE_SNAP_DIST_RATIO で割って本当の速さにする
static float lane_snap_real_v(void) {
    float v = speed_mps > 0.0f ? speed_mps : 0.0f;
    return LANE_SNAP_DIST_RATIO > 0.1f ? v / LANE_SNAP_DIST_RATIO : v;
}

// 棒の上の白の塊のうち、位置 e_pred [m] に一番近い塊の中心 [m]（右がプラス）。
//   新しい線と向こう側の路肩の線が両端に同時に見えたとき（10000001 など）、平均すると真ん中に線があることになってしまうため
static float lane_snap_blob(unsigned char c, float e_pred) {
    float best = 0.0f, best_d = 1e9f;
    int b = 0;
    while (b < 8) {
        if (!(c & (0x80 >> b))) { b++; continue; }
        long sum = 0;
        int n = 0;
        while (b < 8 && (c & (0x80 >> b))) { sum += SENSOR_X_MM[b]; n++; b++; }
        float e = (float)sum / n * 0.001f;
        float d = fabsf(e - e_pred);
        if (d < best_d) { best_d = d; best = e; }
    }
    return best;
}

static void lane_snap_integrate(bool right) {
    float ds = lane_snap_real_v() * 0.001f;
    lane_snap_y += ds * sinf(lane_yaw);
    lane_yaw += ds * curvature_of_angle(servo_est_deg) * (right ? 1.0f : -1.0f);
}

// 1ms ごとのハンドル角度の計算（460 と 470 で共通）。戻り値：計算した角度（変える側がプラス、上限で切る前）
//   turning = 1（460）：まだ上限まで切っている間。予測の時間は、今の角度から反対の上限まで動く時間の半分 ＋ 動き始めの遅れ。
//                       計算した角度が上限より小さくなったら、戻し始める時（470 へ）
//   turning = 0（470）：予測の時間は、サーボがまだ変える側に切れている間は、その角度から反対へ同じだけ切るまでの時間（戻し遅れを防ぐ）。
//                       戻した後は、今の指令まで動き切るまでの遅れだけ（長く予測すると、先の点を見すぎて車が斜めのまま進む）
static float lane_snap_control(bool right, int turning) {
    const float dir = right ? 1.0f : -1.0f;
    const float L = SENSOR_TO_REAR_AXLE_M;
    lane_snap_integrate(right);
    float psi = lane_yaw;
    unsigned char c = sensor_inp(MASK4_4);
    // 新しい線が見えたら、センサの横の位置を直す。センサの棒は車の向きに直角なので、
    //   棒の上で e（右がプラス）の所に線があれば、線の横の位置 LANE_SHIFT_M = ys + e·dir·cos ψ
    //   切れ目のすぐ後（古い線の端）と、計算した位置から離れすぎた白は使わない
    if (c != 0x00 && dist_m > 0.03f) {
        float ys = lane_snap_y + L * sinf(psi);
        float cq = cosf(psi);
        if (cq < 0.3f) cq = 0.3f;
        float e_pred = (LANE_SHIFT_M - ys) * dir / cq;     // 計算の位置から見た、棒の上の線の位置
        float ys_meas = LANE_SHIFT_M - lane_snap_blob(c, e_pred) * dir * cosf(psi);
        float gap = ys_meas - ys;
        if (!lane_snap_seen && fabsf(gap) < 2.5f * LANE_SNAP_GATE_M) {
            lane_snap_y += gap;                    // 初めて見えた：位置をそのまま合わせる
            lane_snap_seen = 1;
        } else if (lane_snap_seen && fabsf(gap) < LANE_SNAP_GATE_M) {
            // 見え続けている：位置を少しずつ直し、ずれ続ける分は向きの誤差（曲がり方・速さの見積もりの誤差）とみて向きも直す
            lane_snap_y += LANE_SNAP_FIX_Y * gap;
            lane_yaw += LANE_SNAP_FIX_YAW * gap / L;
        }
    }
    // サーボの遅れの分だけ先を予測する（今の角度から反対の上限まで動く時間の半分 ＋ 動き始めの遅れ）
    float lim = lane_snap_angle(speed_mps > 0.1f ? speed_mps : 0.1f);   // 角度の上限は、ao が調整した値のまま推定の速さで決める
    float v = lane_snap_real_v();
    if (v < 0.1f) v = 0.1f;
    float servo = servo_est_deg * dir;
    float k_now = curvature_of_angle(servo_est_deg) * dir;
    float tp = turning ? LANE_SNAP_LEAD * (SERVO_DEAD_S + (fabsf(servo) + lim) / (2.0f * SERVO_SPEED_DPS))
                       : SERVO_DEAD_S + fmaxf(fmaxf(servo, 0.0f) / SERVO_SPEED_DPS, fabsf(lane_snap_cmd_deg - servo) / (2.0f * SERVO_SPEED_DPS));
    float psi_p = psi + v * tp * k_now;
    const float Lc = LANE_SNAP_POINT_M;
    float yc_p = lane_snap_y + v * tp * sinf(0.5f * (psi + psi_p)) + Lc * sinf(psi_p);
    float cp = cosf(psi_p);
    if (cp < 0.3f) cp = 0.3f;
    float kd = ((LANE_SHIFT_M - yc_p) / LANE_SNAP_AHEAD_M - sinf(psi_p)) / (Lc * cp);
    float raw = atanf(WHEELBASE_M * kd) / DEG_TO_RAD_F;   // 変える側がプラス
    float ang = turning ? lim : raw;                      // 460 の間は上限まで切り続ける
    if (ang > lim) ang = lim;
    if (ang < -lim) ang = -lim;
    lane_snap_cmd_deg = ang;
    ha_deg(dir * ang);
    drive_at(plan_speed_mps, dir * ang);
    return raw;
}

void run_lane_snap(bool right) {
    float raw = lane_snap_control(right, 1);
    if (raw < lane_snap_angle(speed_mps > 0.1f ? speed_mps : 0.1f) || lane_snap_seen) {   // 戻し始める時
        cnt1 = 0;
        pattern = lane_pt(right, PT_LANE_R_SNAPBACK, PT_LANE_L_SNAPBACK);
    }
}

void run_lane_snapback(bool right) {
    unsigned char c = sensor_inp(MASK4_4);
    bool done = c != 0x00 && fabsf(lane_yaw) < LANE_SNAP_END_DEG * DEG_TO_RAD_F && find_sensor(TRACE_SENSOR, TRACE_N, c) >= 0;
    if (done || (!lane_snap_seen && dist_m > LANE_SNAP_SEARCH_M)) {   // 線が一度も見えないまま長く走ったら、通常走行に任せる
        led_out(0x0);
        cnt1 = 0;
        pattern = PT_TRACE;
        return;
    }
    lane_snap_control(right, 0);
}
