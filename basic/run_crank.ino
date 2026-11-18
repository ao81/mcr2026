//**************************************************************************
// run_crank.ino  クロスライン、クランク
//**************************************************************************

//**********************************************************************
// pattern 100 : クロスラインを検出した
//**********************************************************************
void run_cross_found(void) {
    led_out(0x3);
    ha(0);
    line_pass_state = PASS_WAIT;
    cnt1 = 0;
    pattern = PT_CROSS_PASS;
}

//**********************************************************************
// pattern 101 : クロスラインの通過時間を測る（速いほど短い）
//**********************************************************************
void run_cross_pass(void) {
    measure_line_pass(check_crossline());
    if (line_pass_state == PASS_DONE) {
        line_pass_state = PASS_WAIT;
        cnt1 = 0;
        pattern = PT_CROSS_BRAKE;
        return;
    }
    apply_table(LINE_STEER_SENSOR, LINE_STEER_ACT, LINE_STEER_N, sensor_inp(MASK4_4));
}

//**********************************************************************
// pattern 102 : 減速。CROSS_STRAIGHT_MS の間はクランクの判定をしない
//**********************************************************************
void run_cross_brake(void) {
    line_pass_state = PASS_WAIT;
    crank_speed_control();
    if (cnt1 > CROSS_STRAIGHT_MS) {
        pattern = PT_CRANK_SEARCH;
        return;
    }
    apply_table(LINE_STEER_SENSOR, LINE_STEER_ACT, LINE_STEER_N, sensor_inp(MASK4_4));
}

//**********************************************************************
// pattern 103 : 減速を続けながら、クランク（コース端の白線）を探す
//**********************************************************************
void run_crank_search(void) {
    crank_speed_control();
    unsigned char c = sensor_inp(MASK4_4);
    apply_table(LINE_STEER_SENSOR, LINE_STEER_ACT, LINE_STEER_N, c);

    if (c == 0xe0 || c == 0xf0 || c == 0xf8 || c == 0xfc) {        // 左クランク
        led_out(0x1);
        ha(-CRANK_ANGLE);
        cnt1 = 0;
        pattern = PT_CRANK_L_ENTER;
    } else if (c == 0x07 || c == 0x0f || c == 0x1f || c == 0x3f) { // 右クランク
        led_out(0x2);
        ha(CRANK_ANGLE);
        cnt1 = 0;
        pattern = PT_CRANK_R_ENTER;
    }
}

//**********************************************************************
// pattern 110 / 150 : クランク突入（ハンドルを最大に切ったまま、センサでモータを決める）
//**********************************************************************
void run_crank_enter(bool right) {
    ha(right ? CRANK_ANGLE : -CRANK_ANGLE);
    if (cnt1 > CRANK_ENTER_MS) {
        cnt1 = 0;
        pattern = right ? PT_CRANK_R_SLOW : PT_CRANK_L_SLOW;
        return;
    }
    unsigned char c = sensor_inp(MASK4_4);
    if (right) {
        apply_table(CRANK_R_ENTER_SENSOR, CRANK_R_ENTER_ACT, CRANK_R_ENTER_N, c);
    } else {
        apply_table(CRANK_L_ENTER_SENSOR, CRANK_L_ENTER_ACT, CRANK_L_ENTER_N, c);
    }
    crank_check_lost(c, right);
}

//**********************************************************************
// pattern 111 / 151 : クランク低速旋回。中央線が見えたら通常走行へ
//**********************************************************************
void run_crank_slow(bool right) {
    ha(right ? CRANK_ANGLE : -CRANK_ANGLE);
    unsigned char c = sensor_inp(MASK4_4);
    if (right) {
        apply_table(CRANK_R_SLOW_SENSOR, CRANK_R_SLOW_ACT, CRANK_R_SLOW_N, c);
    } else {
        apply_table(CRANK_L_SLOW_SENSOR, CRANK_L_SLOW_ACT, CRANK_L_SLOW_N, c);
    }
    if (right ? (c == 0x0e || c == 0x0c || c == 0x1c) : (c == 0x70 || c == 0x30 || c == 0x38)) {
        led_out(0x0);
        cnt1 = 0;
        pattern = PT_TRACE;
        return;
    }
    crank_check_lost(c, right);
}

//**********************************************************************
// pattern 120 / 160 : 中央線を見失った後の復帰
//**********************************************************************
void run_crank_return(bool right) {
    unsigned char c = sensor_inp(MASK4_4);
    if (right) {
        apply_table(CRANK_R_RETURN_SENSOR, CRANK_R_RETURN_ACT, CRANK_R_RETURN_N, c);
    } else {
        apply_table(CRANK_L_RETURN_SENSOR, CRANK_L_RETURN_ACT, CRANK_L_RETURN_N, c);
    }
    if (right ? (c == 0x06 || c == 0x0e || c == 0x0c) : (c == 0x60 || c == 0x70 || c == 0x30)) {
        led_out(0x0);
        cnt1 = 0;
        pattern = PT_TRACE;
        return;
    }
    if (right ? (c == 0x80 || c == 0xc0) : (c == 0x01 || c == 0x03)) {   // 外側の白線が見えた
        cnt1 = 0;
        pattern = right ? PT_CRANK_R_OUTSIDE : PT_CRANK_L_OUTSIDE;
    }
}

//**********************************************************************
// pattern 121 / 161 : 曲がり切れず外側にはみ出した時（外輪を逆転させて戻る）
//**********************************************************************
void run_crank_outside(bool right) {
    ha(right ? CRANK_ANGLE : -CRANK_ANGLE);
    unsigned char c = sensor_inp(MASK4_4);
    if (right) {
        apply_table(CRANK_R_OUTSIDE_SENSOR, CRANK_R_OUTSIDE_ACT, CRANK_R_OUTSIDE_N, c);
    } else {
        apply_table(CRANK_L_OUTSIDE_SENSOR, CRANK_L_OUTSIDE_ACT, CRANK_L_OUTSIDE_N, c);
    }
    // 中央線が見えたら復帰処理へ（cnt1 は0にしない）
    if (c == 0x83 || c == 0x81 || c == 0xc1) {
        pattern = right ? PT_CRANK_R_RETURN : PT_CRANK_L_RETURN;
    }
}

//**********************************************************************
// クランク旋回中に線を見失ったか外にはみ出したかを調べ、復帰処理へ移る（110/111/150/151 で共通）
//**********************************************************************
void crank_check_lost(unsigned char c, bool right) {
    if (c == 0x00) {                                                   // 線が見えない
        cnt1 = 0;
        pattern = right ? PT_CRANK_R_RETURN : PT_CRANK_L_RETURN;
    } else if (right ? (c == 0x80 || c == 0xc0) : (c == 0x01 || c == 0x03)) {  // 外側の白線
        cnt1 = 0;
        pattern = right ? PT_CRANK_R_OUTSIDE : PT_CRANK_L_OUTSIDE;
    }
}

//**********************************************************************
// クロスライン通過後のモータ出力（元: newcrank）
//   通過時間 line_pass_ms と経過時間 cnt1 で決める。詳しくは config.h の説明を参照
//**********************************************************************
void crank_speed_control(void) {
    int i = clamp_int(line_pass_ms - CRANK_K_MIN, 0, CRANK_STEPS - 1);
    long t = (long)cnt1;
    int power;
    if (t < CRANK_FULL_MS[i]) {
        power = 100;
    } else if (t < CRANK_HOLD_END_MS[i]) {
        power = CRANK_RAMP[i];
    } else {
        long j = i + 1 + (t - CRANK_HOLD_END_MS[i]) / CRANK_RAMP_STEP_MS;
        if (j > CRANK_STEPS - 1) j = CRANK_STEPS - 1;
        power = CRANK_RAMP[j];
    }
    sp(power, power);
}
