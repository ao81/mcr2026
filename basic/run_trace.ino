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
        start_sel_begin();           // スタート位置の選択を受け付け始める（今押している分は数えない）
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
//   この間にプッシュスイッチを押すと、スタート位置（config.h の COURSE_START_ROWS）を選べる（start_sel_update）
//**********************************************************************
void run_wait_start(void) {
    if (startbar_get() == 0) {       // スタートバーが開いた → スタート
        course_list_start(start_sel);   // 選んだスタート位置で、クランク・レーンチェンジの表の数え方を始める
        led_out(0x0);
        cnt_start = 0;
        log_st = LOG_WRITE;          // ログの記録を始める
        pattern = PT_TRACE;
        cnt1 = 0;
        course_out_ms = 0;
        run_ms = 0;
        return;
    }
    start_sel_update();
    if (start_sel == 0) {
        // LED2 と LED3 を 0.05秒ずつ交互に点灯（スタート位置は走りながら判断する）
        led_out((cnt1 % 100 < 50) ? 0x1 : 0x2);
    } else {
        // スタート位置を選んだ：COURSE_START_ROWS の1つ目なら LED2、2つ目なら LED3、3つ目なら両方を点けたままにする
        led_out((unsigned char)(start_sel & 0x3));
    }
}

//**********************************************************************
// スタート位置の選択（pattern 1 の間）
//   start_sel = 0 : 選ばない。走りながら、クランクで曲がった向きなどから判断する（1つ目のクランクは安全側の値で遅くなる）
//   start_sel = k : COURSE_START_ROWS の k 番目（1から）からスタートする。1つ目のクランクから表の値どおりに走る
//   プッシュスイッチを押すたびに 0 → 1 → 2 → … → 0 と変わる（COURSE_LIST_USE が 1 で、位置が2つ以上あるとき）
//**********************************************************************
void start_sel_begin(void) {
    start_sel = 0;
    start_sw_prev = 1;
    start_sw_ms = cnt0;
}

void start_sel_update(void) {
    int sw = pushsw_get();
    // 押した瞬間に1つ進める。チャタリングで何回も数えないよう、前に数えてから 200ms は数えない
    if (sw == 1 && start_sw_prev == 0 && cnt0 - start_sw_ms >= 200) {
        int n = course_start_count();
        if (n >= 2) start_sel = (start_sel + 1) % (n + 1);
        start_sw_ms = cnt0;
    }
    start_sw_prev = sw;
}

//**********************************************************************
// pattern 20〜25 : 通常走行
//   stage : 速度段階 0〜5。直線が続くほど段階が上がり、カーブで内輪を強く減速する
//**********************************************************************
void run_trace(int stage) {
    // 特殊なラインの検出
    bool check_lines = true;
#if USE_NEW_LINE_SHAPE || USE_NEW_CURVE_PREVIEW
    // クロスライン・ハーフラインは直線にある。急なカーブの途中で線を斜めに見ていると、
    // 線1本がハーフラインと同じ並びに見えることがあるので、推定した線の向きが LINE_CHECK_MAX_ANGLE より
    // 大きく、しかもハンドルを LINE_CHECK_MIN_STEER より大きく切っている間だけ探さない（両方が外れることはまずない）
    if (cnt0 - line_obs_ms <= 20 && fabsf(obs_z2) > LINE_CHECK_MAX_ANGLE &&
        fabsf(servo_est_deg) > LINE_CHECK_MIN_STEER) check_lines = false;
#endif
    if (check_lines) {
        if (check_crossline())    { cnt1 = 0; pattern = PT_CROSS_FOUND;  return; }
        if (check_shasenhenkoum()) { cnt1 = 0; pattern = PT_LANE_R_FOUND; return; }
        if (check_shasenhenkouh()) { cnt1 = 0; pattern = PT_LANE_L_FOUND; return; }
    }

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
    int i = apply_table(TRACE_SENSOR, TRACE_ACT[stage], TRACE_N, c);
#if USE_NEW_LINE_SHAPE || USE_NEW_CURVE_PREVIEW
    line_obs_update(c);                      // 線の位置・向き・曲がり具合を推定（planner.ino）
#endif
#if USE_NEW_STEER || USE_NEW_TRACE_SPEED
    if (i >= 0) {
        float ang = TRACE_ACT[stage][i].ang;
#if USE_NEW_STEER
        ang = pursuit_angle(c);              // ハンドル角度を線の位置から計算して置き換える
#if USE_NEW_LINE_SHAPE
        ang = line_obs_angle();              // 推定した線の形と、サーボの遅れの分だけ先の位置から決める
        ha_deg(ang);
#else
        ha((int)ang);
#endif
#endif
#if USE_NEW_TRACE_SPEED
        if (model_ok) {
#if USE_NEW_CURVE_PREVIEW
            // 推定したセンサの所の曲がり具合（この先の曲がり具合）でも速さを決める（speed_model.ino）
            trace_motor_preview(obs_z3);
#else
            trace_motor_new((int)ang);       // モータ値をハンドル角度から計算して置き換える
#endif
        }
#endif
        (void)ang;
    }
#else
    (void)i;
#endif

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

    int i;
    if (right) {
        i = apply_table(CURVE_R_SENSOR, CURVE_R_ACT[stage], CURVE_R_N, c);
    } else {
        i = apply_table(CURVE_L_SENSOR, CURVE_L_ACT[stage], CURVE_L_N, c);
    }
#if USE_NEW_LINE_SHAPE || USE_NEW_CURVE_PREVIEW
    // 大曲がりの間も線の形の推定を続ける（通常走行に戻った時に、カーブの途中から推定し直さなくてよいように）
    if (c == 0x03 || c == 0xc0 || c == 0x07 || c == 0xe0) line_obs_update(c);
    else line_obs_update(0x00);              // 線が端より外にあるかもしれない形のときは、予測だけ進める
#endif
#if USE_NEW_CURVE_SPEED
    if (i >= 0 && model_ok) {                // モータ値をハンドル角度から計算して置き換える
#if USE_NEW_CURVE_PREVIEW
        trace_motor_preview(obs_z3);
#else
        trace_motor_new(right ? CURVE_R_ACT[stage][i].ang : CURVE_L_ACT[stage][i].ang);
#endif
    }
#else
    (void)i;
#endif
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
