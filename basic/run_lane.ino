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
    led_out(right ? 0x1 : 0x2);
    ha(0);
    line_pass_state = PASS_WAIT;
    cnt1 = 0;
    pattern = lane_pt(right, PT_LANE_R_PASS, PT_LANE_L_PASS);
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
    measure_line_pass(right ? check_shasenhenkoum() : check_shasenhenkouh());
    if (line_pass_state == PASS_DONE) {
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
            line_pass_state = PASS_WAIT;
            led_out(right ? 0x1 : 0x2);
            lane_steer_output(stage, right);
            cnt1 = 0;
            pattern = lane_pt(right, PT_LANE_R_STEER, PT_LANE_L_STEER) + stage;
            return;
        }
    }
    unsigned char c = sensor_inp(MASK4_4);
    if (right) {
        apply_table(LANE_R_APPROACH_SENSOR, LANE_R_APPROACH_ACT[stage], LANE_R_APPROACH_N, c);
    } else {
        apply_table(LANE_L_APPROACH_SENSOR, LANE_L_APPROACH_ACT[stage], LANE_L_APPROACH_N, c);
    }
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
// ハーフライン通過後のモータ出力（元: shasenhenkou）
//   通過時間 line_pass_ms と経過時間 cnt1 で決める。詳しくは config.h の説明を参照
//**********************************************************************
void lane_speed_control(void) {
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
