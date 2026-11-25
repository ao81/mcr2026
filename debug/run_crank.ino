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
    crank_spot_select();                           // これから来るクランクの距離・クロスラインの幅（config.h の COURSE_LIST）
    crank_cross_v0 = speed_mps;                    // クロスラインに入った時の速さ（CRANK_LINE_SAME_WIDTH）
    crank_seen_w = 0.0f;
    crank_cross_ms = 1;                            // この1msも白を見ている
#if USE_NEW_BRAKE_RANGE && USE_NEW_CRANK_BRAKE
    // センサはクロスラインの始まりにいる。ラインの終わりまでは crank_line_w_m あるので、距離をその分マイナスから数え、
    // 通過時間を測り終わるのを待たずに、今の速さの推定で減速を始める（通過し終えたら測った速さで計算し直す）
    if (model_ok) {
        reset_distance();
        dist_m = -crank_line_w_m;
        crank_speed_control_new();
    }
#endif
}

//**********************************************************************
// CRANK_LINE_SAME_WIDTH：クロスラインがセンサに見える幅を求める（pattern 101・102 で毎ms呼ぶ）
//   両端の3つずつのセンサ（11100111 の所）のどれかが白の間の時間を数え、白が消えたら
//   クロスラインを通る間の速さ（入った時と出た時の推定の平均）× 時間 = センサに見えた幅 とする。
//   曲がり角の線（中央線）も同じ幅とみなして、そこを通る時間で速さを測る（中央線の幅が 20/30/40mm のどれでも合う）。
//   曲がり角の線も「曲がる側の3つのセンサのどれかが白の間」を数えるので、同じ数え方になる
//**********************************************************************
static void crank_cross_count(unsigned char c) {
#if USE_NEW_CRANK_ANY_COURSE && USE_NEW_CRANK_QUICK && CRANK_LINE_SAME_WIDTH
    if (crank_cross_ms <= 0) return;               // 数え終わった
    if ((c & 0xe7) != 0) {
        crank_cross_ms++;
        if (crank_cross_ms > 200) crank_cross_ms = 0;   // 長すぎる（線ではない）。幅は求めない
        return;
    }
    float vc = (crank_cross_v0 + speed_mps) * 0.5f;
    float w = vc * (crank_cross_ms + LINE_PASS_OFFSET_MS) * 0.001f;   // 曲がり角の線の cnt1 と同じ数え方
    w = w * CRANK_SEEN_W_SCALE;                    // 速さの推定が低めだと幅も狭く出る。広めにして、曲がり角の速さを多めに見積もる側にする
    crank_seen_w = (model_ok && w >= CRANK_SEEN_W_MIN_M && w <= CRANK_SEEN_W_MAX_M) ? w : 0.0f;
    crank_cross_ms = 0;
#else
    (void)c;
#endif
}

//**********************************************************************
// pattern 101 : クロスラインの通過時間を測る（速いほど短い）
//**********************************************************************
void run_cross_pass(void) {
#if USE_NEW_BRAKE_RANGE && USE_NEW_CRANK_BRAKE
    if (model_ok) crank_speed_control_new();       // 通過中も減速の計算を続ける
#endif
    crank_cross_count(sensor_inp(MASK4_4));
    measure_line_pass(check_crossline());
    if (line_pass_state == PASS_DONE) {
#if !USE_NEW_CRANK_ANY_COURSE
        speed_measure_by_line(crank_line_w_m, line_pass_ms);   // 通過時間から速さを測り直す（ANY_COURSE では幅が分からないので測らない）
#endif
        reset_distance();                                          // ここからクランクまでの距離を数える
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
    crank_cross_count(sensor_inp(MASK4_4));
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
        crank_spot_done(false);                                     // 曲がる向きを表と比べ、次のクランクへ数を進める
#if USE_NEW_CRANK_TURN
        if (crank_start_arc(false)) return;                         // 計算した円弧で曲がる
#endif
        led_out(0x1);
        ha(-CRANK_ANGLE);
        cnt1 = 0;
        pattern = PT_CRANK_L_ENTER;
    } else if (c == 0x07 || c == 0x0f || c == 0x1f || c == 0x3f) { // 右クランク
        crank_spot_done(true);
#if USE_NEW_CRANK_TURN
        if (crank_start_arc(true)) return;
#endif
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
// 円弧（152 / 112）から 161 / 121 に移った後も、車の向きが変わった角度を積み上げる。
// （161 / 121 はハンドルをいつも CRANK_ANGLE まで切るので、角度は必ず増え、ここで止まったままにはならない）
// 向きがまだ CRANK_EXIT_MIN_YAW_DEG まで変わっていなければ false（中央線が見えても通常走行へ移らない）
//   CRK0099：2つ目のクランクで、向きが 44°ほどしか変わっていないのに 161 で 00111000 を見て通常走行に移った。
//   通常走行は線が左にあるので左へハンドルを切り、線は右へ流れて見えなくなり、コースから出た。
//   うまく曲がれた CRK0095・CRK0100 は、通常走行に移ったときに 93°・105° 変わっていた
//**********************************************************************
static bool crank_turned_enough() {
#if USE_NEW_CRANK_QUICK
    if (!crank_yaw_track) return true;             // 円弧で曲がり始めていない（元の方法）ときは今までどおり
    crank_yaw = crank_yaw + speed_mps * 0.001f * fabsf(curvature_of_angle(servo_est_deg));
    return crank_yaw >= CRANK_EXIT_MIN_YAW_DEG * DEG_TO_RAD_F;
#else
    return true;
#endif
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
    // 中央線がセンサの真ん中付近に見えたら通常走行へ。
    //   センサが後輪から 0.46m 前にあり、大きくハンドルを切るとセンサは横へ速く動くので、線は真ん中を 1〜2ms で通り過ぎる。
    //   右クランクなら 00011000 / 00010000 / 00001000 / 00011100 も通常走行へ移る形に加える（CRK0081）
    if (right ? (c == 0x06 || c == 0x0e || c == 0x0c || c == 0x18 || c == 0x10 || c == 0x08 || c == 0x1c)
              : (c == 0x60 || c == 0x70 || c == 0x30 || c == 0x18 || c == 0x08 || c == 0x10 || c == 0x38)) {
        led_out(0x0);
        cnt1 = 0;
        crank_yaw_track = 0;
        line_ignore_until = cnt0 + CRANK_EXIT_LINE_IGNORE_MS;   // 斜めの中央線をハーフラインと見間違えない（CRK0086・0096）
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
static int bit_count4(unsigned char m) {
    int n = 0;
    for (; m; m &= (unsigned char)(m - 1)) n++;
    return n;
}

void run_crank_outside(bool right) {
    ha(right ? CRANK_ANGLE : -CRANK_ANGLE);
    unsigned char c = sensor_inp(MASK4_4);
    bool turned = crank_turned_enough();
    if (right) {
        apply_table(CRANK_R_OUTSIDE_SENSOR, CRANK_R_OUTSIDE_ACT, CRANK_R_OUTSIDE_N, c);
    } else {
        apply_table(CRANK_L_OUTSIDE_SENSOR, CRANK_L_OUTSIDE_ACT, CRANK_L_OUTSIDE_N, c);
    }
    // 真ん中の2つのセンサのどちらかが線を見て、外側の端のセンサ（右クランクなら左端）が白でなければ、中央線の上に来ているので通常走行へ。
    //   CRK0081：00010111 で中央線を見たのに、この表に形がないので 55° のまま曲がり続けた
    //   CRK0088・0096：01111100（コースの外の白い床が広く見えている）でも通常走行に移り、左ハーフラインと見てレーンチェンジに入った。
    //   外側の2つが白でないこと、真ん中の4つのうち白は3つまで（線の太さ）であることも条件にする
    //   向きがまだ十分に変わっていないときは移らず、ハンドルを切ったまま曲がり続ける（CRK0099）
    if (turned && (c & 0x18) != 0 && (c & (right ? 0xc0 : 0x03)) == 0 && bit_count4(c & 0x3c) <= 3) {
        led_out(0x0);
        cnt1 = 0;
        crank_yaw_track = 0;
        line_ignore_until = cnt0 + CRANK_EXIT_LINE_IGNORE_MS;   // 斜めの中央線をハーフラインと見間違えない（CRK0086・0096）
        pattern = PT_TRACE;
        return;
    }
    // 中央線が見えたら復帰処理へ（cnt1 は0にしない）
    // 曲がる側の端だけに線が見えたときも、線はもうセンサの下を通り過ぎて曲がる側にあるので、復帰処理で線へ寄せる
    //   （この形は上の表にないので、前の出力のまま止まらずに回り続けていた。CRK0054 は 00000001 のまま 2 秒以上）
    if (c == 0x83 || c == 0x81 || c == 0xc1 || c == (right ? 0x01 : 0x80) || c == (right ? 0x03 : 0xc0)) {
        pattern = right ? PT_CRANK_R_RETURN : PT_CRANK_L_RETURN;
    }
}

// 曲がり角の線がセンサに見える幅 [m]
float crank_corner_seen_w(void) {
#if CRANK_LINE_SAME_WIDTH
    if (crank_seen_w > 0.0f) return crank_seen_w;  // クロスラインと同じ幅とみなす
#endif
    return CENTER_LINE_WIDTH_M + LINE_SEEN_EXTRA_M;
}

//**********************************************************************
// クランクを見つけた時に、計算した円弧で曲がれるか調べて、曲がれるなら pattern 112 / 152 へ（USE_NEW_CRANK_TURN）
//   戻り値 true: 円弧で曲がる  false: 曲がれないので元の方法で曲がる
//**********************************************************************
bool crank_start_arc(bool right) {
    if (!model_ok) return false;                   // 速さの推定が当てにならないときは元の方法で曲がる
    float r, wait;
    float v = speed_mps;
#if USE_NEW_CRANK_QUICK
    // 曲がり始めるまでの直進の間に、曲がる速さにする（速ければ減速、遅ければ加速）。サーボが効くまでに進む距離は、その速さで計算する
    if (crank_q_r > 0.0f) v = corner_speed(1.0f / crank_q_r);
#endif
    if (!crank_plan(v, &r, &wait)) return false;
    v = speed_mps;
    float v_turn = corner_speed(1.0f / r);
    float v_chk = v;
#if USE_NEW_BRAKE_RANGE && USE_NEW_CRANK_BRAKE
    // 円弧に入るまでの直進とサーボの遅れの間も減速を続けるので、円弧に入る時の速さで判断する
    v_chk = speed_after_brake(v, pre_turn_m(v, r, wait));
#endif
    if (v_chk > v_turn * PLAN_OVERSPEED) return false; // 速すぎて、この円弧では横すべりする

    plan_wait_m = wait;
    plan_len_m = r * PI_F * 0.5f;                  // 90°分の円弧の長さ
    plan_speed_mps = v_turn;
    plan_angle = angle_for_radius(r);
    reset_distance();                              // ここから距離を数える
    crank_yaw = 0.0f;
    crank_yaw_track = 1;
    crank_unwind = 0;
    crank_unwind_m = 0.0f;
    crank_measuring = 1;
    led_out(right ? 0x2 : 0x1);
    cnt1 = 0;
    pattern = right ? PT_CRANK_R_ARC : PT_CRANK_L_ARC;
    run_crank_arc(right);                          // この1msの出力もすぐ出す
    return true;
}

//**********************************************************************
// pattern 112 / 152 : 計算した円弧で曲がる（USE_NEW_CRANK_TURN）
//   ① plan_wait_m だけ直進 → ② plan_angle で plan_len_m（90°分）走る → ③ ハンドルを戻して線を見つけたら通常走行へ
//   円弧の CRANK_ARC_EXIT_RATIO 以上を走った後に新しい線が中央付近に見えたら、その時点で通常走行へ
//   外側の白線が見えたら（はみ出した）元の処理 121 / 161 へ
//**********************************************************************
void run_crank_arc(bool right) {
    unsigned char c = sensor_inp(MASK4_4);
    const int dir = right ? 1 : -1;
#if USE_NEW_CRANK_QUICK
    // 曲がり角の線（曲がる側へ延びる中央線）を通り過ぎる時間で、速さを測り直す。
    // クロスラインは速いときに通るので 1ms の数え方で ±10% ほどずれるが、ここは減速した後なので、ずれは数%で済む。
    // 曲がり始める位置は、ここで測った速さから決め直す
    if (crank_measuring) {
        unsigned char side = right ? 0x07 : 0xe0;   // 曲がる側のセンサ3つ
        if ((c & side) == 0) {
            crank_measuring = 0;
            float wl = crank_corner_seen_w();
            float vm = wl / ((cnt1 + LINE_PASS_OFFSET_MS) * 0.001f);
            // 減速の後は推定が大きくずれていることがあるので、クロスラインのとき（0.6〜1.67倍）より広く受け付ける。
            // それより大きく違うときは測り間違いとみて使わない
            if (vm > speed_mps * 0.4f && vm < speed_mps * 2.5f) {
                speed_mps = vm;
                dist_m = wl;
                float r, wait;
                float vq = corner_speed(1.0f / crank_q_r);
                if (crank_plan(vq, &r, &wait)) plan_wait_m = wait;
            }
        }
    }
#endif
    float s = dist_m - plan_wait_m;                // 円弧に入ってからの距離
#if USE_NEW_CRANK_QUICK
    // 曲がり角の線を通り過ぎた後、曲がり始めるまでは、センサの前に白い線はないはず。
    // それでも何か見えたら、思ったより先まで進んでいる（速さを少なく見積もっている）ので、すぐ曲がり始める
    //   CRK0056：曲がり角の線の 5〜8cm 先で 11111111 を見たのに、そのまま 13cm 直進してから曲がり、曲がりきれなかった
    if (s < 0.0f && !crank_measuring && c != 0x00) {
        plan_wait_m = dist_m;
        s = 0.0f;
    }
#endif

    if (s < 0.0f) {                                // ① まだ曲がり始めない
        ha(0);
        drive_at(plan_speed_mps, 0);
        return;
    }
    if (right ? (c == 0x80 || c == 0xc0) : (c == 0x01 || c == 0x03)) {   // 外側の白線
        cnt1 = 0;
        pattern = right ? PT_CRANK_R_OUTSIDE : PT_CRANK_L_OUTSIDE;
        return;
    }
#if USE_NEW_CRANK_QUICK
    // 車の向きが変わった角度を、サーボの本当の角度の推定から積み上げる
    crank_yaw = crank_yaw + speed_mps * 0.001f * fabsf(curvature_of_angle(servo_est_deg));
    // 今ハンドルを戻し始めても、戻りきるまでに向きはさらに変わる。その分を足して 90°になったら戻し始める
    // （90°を過ぎてから戻すと、行き過ぎた向きを直すのに時間がかかる）
    if (!crank_unwind && crank_yaw + crank_unwind_yaw(speed_mps, servo_est_deg) >= PI_F * 0.5f - CRANK_UNWIND_EARLY_DEG * DEG_TO_RAD_F) {
        crank_unwind = 1;
        crank_unwind_m = dist_m;
    }
    if (!crank_unwind) {
        ha_deg(dir * plan_angle);                  // ② 円弧
        drive_at(plan_speed_mps, dir * plan_angle);
        return;
    }
    // ③ ハンドルを戻す。戻るにつれて曲がる力が要らなくなるので、その分だけ加速する
    ha(0);
    drive_at(corner_speed(fabsf(curvature_of_angle(servo_est_deg))), servo_est_deg);
    if (find_sensor(TRACE_SENSOR, TRACE_N, c) >= 0) {
        led_out(0x0);
        cnt1 = 0;
        crank_yaw_track = 0;
        line_ignore_until = cnt0 + CRANK_EXIT_LINE_IGNORE_MS;
        pattern = PT_TRACE;                        // 線が見えた → 通常走行
        return;
    }
    // 次のどれかなら、元の復帰処理（線の見えない側＝曲がる側へハンドルを切って線を探す）へ移る
    //   ・向きが変わりすぎても線が見えない
    //   ・ハンドルが戻りきった後に、曲がる側の端のセンサ1つだけが線を見た（曲がり始めが遅れ、新しい線を少し通り越した）
    //     ※ うまく曲がれたときも、ハンドルを戻している途中にこの形が一瞬見えるので、戻りきるまでは使わない
    //   ・ハンドルを戻し始めてから CRANK_EXIT_SEARCH_M 進んでも線が見えない
    //     （ハンドルが 0°のままなので crank_yaw は増えず、上の条件だけでは全開の直進が止まらない。FULL0053）
    bool edge = fabsf(servo_est_deg) < 1.0f && c == (right ? 0x01 : 0x80);
    if (crank_yaw > PI_F * 0.75f || edge || dist_m - crank_unwind_m >= CRANK_EXIT_SEARCH_M) {
        cnt1 = 0;
        pattern = right ? PT_CRANK_R_RETURN : PT_CRANK_L_RETURN;
    }
    return;
#endif
    if (s >= plan_len_m * CRANK_ARC_EXIT_RATIO) {
        bool found = right ? (c == 0x0e || c == 0x0c || c == 0x1c || c == 0x18)
                           : (c == 0x70 || c == 0x30 || c == 0x38 || c == 0x18);
        if (found) {
            led_out(0x0);
            cnt1 = 0;
            crank_yaw_track = 0;
            line_ignore_until = cnt0 + CRANK_EXIT_LINE_IGNORE_MS;
            pattern = PT_TRACE;
            return;
        }
    }
    if (s < plan_len_m) {
        ha_deg(dir * plan_angle);                  // ② 円弧
        drive_at(plan_speed_mps, dir * plan_angle);
        return;
    }
    // ③ 90°分の指令を出し終えた。サーボが戻りきるまでは車はまだ曲がっているので、その間は線を待つ
    ha(0);
    drive_at(plan_speed_mps, 0);
    if (find_sensor(TRACE_SENSOR, TRACE_N, c) >= 0) {
        led_out(0x0);
        cnt1 = 0;
        crank_yaw_track = 0;
        line_ignore_until = cnt0 + CRANK_EXIT_LINE_IGNORE_MS;
        pattern = PT_TRACE;                        // 線が見えた → 通常走行
        return;
    }
    if (s >= plan_len_m + plan_speed_mps * servo_delay_s(plan_angle) * 2.0f) {
        cnt1 = 0;
        pattern = right ? PT_CRANK_R_RETURN : PT_CRANK_L_RETURN;   // それでも見えない → 元の復帰処理
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
#if USE_NEW_CRANK_BRAKE
    if (model_ok) {                                // 速さの推定が当てにならないときは元の表で減速する
        crank_speed_control_new();
        return;
    }
#endif
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
