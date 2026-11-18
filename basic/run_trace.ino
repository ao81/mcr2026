//**************************************************************************
// run_trace.ino  スタート、通常走行、大曲がり、停止
//**************************************************************************

//**********************************************************************
// pattern 0 : プッシュスイッチ待ち
//**********************************************************************
void run_wait_switch(void) {
    if (pushsw_get() == 1) {
        log_st = LOG_INIT;           // ログファイルの作成を始める
        led_out_m(0);
        pattern = PT_WAIT_START;
        cnt1 = 0;
        course_out_ms = 0;
        run_ms = 0;
        return;
    }
    // LED2 と LED3 を 0.25秒ずつ交互に点灯
    led_out((cnt1 % 500 < 250) ? 0x1 : 0x2);

    // スタートバーを検出している間は、マイコンボードのD13を点滅させる
    if (startbar_get() == 1) {
        led_out_m((cnt0 % 200 < 100) ? 1 : 0);
    } else {
        led_out_m(0);
    }
}

//**********************************************************************
// pattern 1 : スタートバーが開くのを待つ
//**********************************************************************
void run_wait_start(void) {
    if (startbar_get() == 0) {       // スタートバーが開いた → スタート
        led_out(0x0);
        cnt_start = 0;
        log_st = LOG_WRITE;          // ログの記録を始める
        pattern = PT_TRACE;
        cnt1 = 0;
        course_out_ms = 0;
        run_ms = 0;
        return;
    }
    // LED2 と LED3 を 0.05秒ずつ交互に点灯
    led_out((cnt1 % 100 < 50) ? 0x1 : 0x2);
}

//**********************************************************************
// pattern 20〜25 : 通常走行
//   stage : 速度段階 0〜5。直線が続くほど段階が上がり、カーブで内輪を強く減速する
//**********************************************************************
void run_trace(int stage) {
    // 特殊なラインの検出
    if (check_crossline())    { cnt1 = 0; pattern = PT_CROSS_FOUND;  return; }
    if (check_shasenhenkoum()) { cnt1 = 0; pattern = PT_LANE_R_FOUND; return; }
    if (check_shasenhenkouh()) { cnt1 = 0; pattern = PT_LANE_L_FOUND; return; }

    unsigned char c = sensor_inp(MASK4_4);

    // 直線が TRACE_STAGE_UP_MS 続いたら、次の段階へ
    if (stage < TRACE_STAGES - 1) {
        if (c == 0x18 || c == 0x1c || c == 0x38) {
            if (cnt1 >= TRACE_STAGE_UP_MS[stage]) {
                pattern = PT_TRACE + stage + 1;
                return;
            }
        } else {
            cnt1 = 0;
        }
    }

    // 表に従ってハンドルとモータを出力
    apply_table(TRACE_SENSOR, TRACE_ACT[stage], TRACE_N, c);

    // 外側のセンサだけが白線 → 大曲がりへ（同じ段階の表を使う）
    if (c == 0x03) { cnt1 = 0; pattern = PT_CURVE_R + stage; }
    if (c == 0xc0) { cnt1 = 0; pattern = PT_CURVE_L + stage; }
}

//**********************************************************************
// pattern 30〜35 / 40〜45 : 大曲がり（コースアウト回避）
//   stage : 段階 0〜5（通常走行の段階を引き継ぐ）
//   right : true=右カーブ  false=左カーブ
//**********************************************************************
void run_curve(int stage, bool right) {
    const int base = right ? PT_CURVE_R : PT_CURVE_L;

    // 一定時間たったら1つ下の段階の表へ（cnt1 は0にしない）
    if (stage > 0 && cnt1 >= CURVE_STAGE_DOWN_MS[stage]) {
        pattern = base + stage - 1;
        return;
    }

    unsigned char c = sensor_inp(MASK4_4);

    // 線が内側に戻ってきたら通常走行（段階0）へ
    if (right ? (c == 0x06 || c == 0x07) : (c == 0x60 || c == 0xe0)) {
        cnt1 = 0;
        pattern = PT_TRACE;
        return;
    }

    if (right) {
        apply_table(CURVE_R_SENSOR, CURVE_R_ACT[stage], CURVE_R_N, c);
    } else {
        apply_table(CURVE_L_SENSOR, CURVE_L_ACT[stage], CURVE_L_N, c);
    }
}

//**********************************************************************
// pattern 800 : 予定の走行時間がたったので停止（ハンドルは線を追い続ける）
//**********************************************************************
void run_stop(void) {
    sp(0, 0);
    if (cnt1 > STOP_STEER_MS) {
        cnt1 = 0;
        pattern = PT_STOP_HOLD;
        return;
    }
    apply_table(STOP_STEER_SENSOR, STOP_STEER_ACT, STOP_STEER_N, sensor_inp(MASK4_4));
}

//**********************************************************************
// pattern 801 : 完全停止（LEDをゆっくり点滅）
//   ※ ログファイルはまだ閉じていない。プッシュスイッチを押すと 1001 で閉じる
//**********************************************************************
void run_stop_hold(void) {
    sp(0, 0);
    if (cnt1 < 450) {
        led_out(0x1);
    } else if (cnt1 < 900) {
        led_out(0x2);
    } else {
        cnt1 = 0;
    }
}

//**********************************************************************
// pattern 1001 : 走行とログ記録の終了
//**********************************************************************
void run_end(void) {
    R_GPT1->GTCR_b.CST = 0;          // サーボのPWMを止めて、ハンドルを手で動かせるようにする
    log_st = LOG_CLOSE;              // ログファイルを閉じる
    sp(0, 0);
    led_out((cnt1 % 1000 < 100) ? 0x1 : 0x0);
}
