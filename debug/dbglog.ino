//**************************************************************************
// dbglog.ino  デバッグモードの詳しいログ（debug/ だけにあるファイル。sync.sh で上書きされない）
//   設定は dbg_config.h。読み方と使い方は debug/README.md
//
//   dbg_sample() : 1ms 割り込みの最後（speed_model_update の後）に毎回呼ぶ。RAM のリングバッファに貯める
//   dbg_task()   : loop から毎回呼ぶ。記録が止まり、通常のログが閉じられた後に dbgNNNN.csv へ書き出す
//**************************************************************************
#if DBG_ENABLE
static_assert(DBG_INTERVAL_MS >= 1 && DBG_SAMPLES > 0, "debug interval and samples must be positive");
static_assert(DBG_TARGET >= 1 && DBG_TARGET <= 6, "unknown debug target");
static_assert(DBG_TARGET != 3 || DBG_TO_MS > DBG_FROM_MS, "empty time window");
static_assert(!DBG_AUTO_STOP || DBG_TARGET == 3 || DBG_TARGET_NO >= 1, "select a numbered event");
static_assert(DBG_CHANGE_MAX >= 0 && DBG_CHANGE_MAX < DBG_SAMPLES, "DBG_CHANGE_MAX must be smaller than DBG_SAMPLES");

// 通常走行・総合（5・6）は走り全体を1つのバッファに入れるので、変化した瞬間の行は足さない
#define DBG_CHG_ON (DBG_CHANGE_ROWS && DBG_TARGET != 5 && DBG_TARGET != 6)

typedef struct {
    uint16_t t;        // スタートからの時間 [ms]（下16ビット。書き出す時に戻す）
    int16_t ptn;       // pattern
    int16_t steer;     // ハンドル角度の指令 [0.1度]
    int16_t servo;     // サーボの本当の角度の推定 [0.1度]
    int16_t spd;       // 推定した速さ [mm/s]
    int16_t dist;      // ラインの終わり（または曲がり始め）からの距離 [mm]
    int16_t vmax;      // 出力100%の速さ [mm/s]
    int16_t yaw;       // 曲がり始めてから向きが変わった角度 [0.1度]（クランクは crank_yaw、レーンチェンジ（DBG_TARGET 2）は lane_yaw）
    int16_t lined;     // 線の向きの推定 [0.1度]（obs_z2。USE_NEW_LINE_SHAPE のとき通常走行の間だけ変わる）
    int16_t lat;       // 横の加速度の推定 [cm/s²]（速さ² × 車が本当に曲がっている曲率 car_k_eff。右がプラス）
    uint8_t sen;       // センサ 8 個
    int8_t ml;         // 左モータ出力 [%]
    int8_t mr;         // 右モータ出力 [%]
    uint8_t flags;     // bit0 brake_started, bit1 model_ok, bit2 crank_unwind, bit3 crank_measuring, bit4 変化した瞬間の行
    uint8_t lpass;     // ラインの通過時間 [ms]（最後に測った値）
    int8_t linemm;     // 線の横の位置の推定 [mm]（obs_z1。右がプラス）
} dbg_t;               // 26 バイト

static dbg_t dbg_buf[DBG_SAMPLES];
static volatile int dbg_head;             // 次に書く位置
static volatile int dbg_count;            // 貯まっている数（DBG_SAMPLES まで）
static volatile int dbg_frozen;           // 1: 記録を止めた
static volatile int dbg_reason;           // 止めた理由 1:区間の終わり 2:停止 3:時間指定の終わり 4:模擬の終わり
static volatile int dbg_in_event;         // 1: 今、選んだ区間の中
static volatile int dbg_events;           // 区間に入った回数
static volatile unsigned long dbg_trigger_ms;  // 記録したい区間に入った時間 [ms]
static volatile unsigned long dbg_end_ms;      // その区間を出た時間 [ms]（0 はまだ）
static volatile unsigned long dbg_stop_ms;     // 記録を止めた時間 [ms]
static volatile int dbg_written;          // 1: microSD に書き終えた
static volatile int dbg_after_trigger = -1;// 選んだ区間に入ってから貯めた数（-1 は区間の外）
static volatile int dbg_chg_on;           // 1: 変化した瞬間の行を足す（選んだ区間に入ってから記録を止めるまで）
static volatile int dbg_chg_count;        // 足した「変化した瞬間の行」の数（DBG_CHANGE_MAX まで）
static int dbg_chg_total;                 // 同上（書き出し用。区間に入り直しても数え続ける）

// 同じ標識列とDIPで走ったCSVだけを比較する。仮判定中のハーフラインは数えない。
static uint32_t dbg_route = 2166136261u;
static int dbg_dip = -1;
static int dbg_previous = PT_WAIT_START;
static unsigned long dbg_trace_start;

static int dbg_marker(int p) {
    if (p == PT_CROSS_BRAKE) return 1;
    if (p >= PT_LANE_R_APPROACH && p < PT_LANE_R_APPROACH + LANE_STAGES) return 2;
    if (p >= PT_LANE_L_APPROACH && p < PT_LANE_L_APPROACH + LANE_STAGES) return 3;
    return 0;
}

// 区間に入った時・出た時の値（書き出す時にファイルの先頭へ）
static float dbg_p_cross_to, dbg_p_cross_w, dbg_p_crank_entry, dbg_p_crank_extra;
static float dbg_p_lane_to, dbg_p_lane_w, dbg_p_lane_entry, dbg_p_lane_extra;
static float dbg_p_vmax_in, dbg_p_vmax_out;
static float dbg_p_wait, dbg_p_len, dbg_p_speed, dbg_p_angle;
static int dbg_p_spot_no;
// 区間を出た時（記録を止めた時）の、計算した経路の値。レーンチェンジの切り返しの位置などは走りながら直すので、直した後の値が残る
static float dbg_p_switch, dbg_p_lane_end, dbg_p_lane_sw0, dbg_p_lane_cross, dbg_p_lane_lag, dbg_p_lane_offset;
static float dbg_p_unwind_m, dbg_p_crank_app;
static int dbg_p_crank_wide;

static int16_t dbg_i16(float x) {
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

// 記録したい区間の pattern か
static bool dbg_is_event(int p) {
#if DBG_TARGET == 1
    return p >= PT_CROSS_FOUND && p <= PT_CRANK_R_OUTSIDE;
#elif DBG_TARGET == 2
    return (p >= PT_LANE_R_FOUND && p <= PT_LANE_R_SELECT) || (p >= PT_LANE_R_APPROACH && p <= 470)
        || (p >= PT_LANE_L_FOUND && p <= PT_LANE_L_SELECT) || (p >= PT_LANE_L_APPROACH && p <= 770);
#else
#if DBG_TARGET == 4
    // 緩いカーブは通常トレースのまま曲がる。舵の一瞬の揺れを除く。
    static int turning_ms, straight_ms;
    static bool active;
    if (p < PT_TRACE || p >= PT_CROSS_FOUND) {
        turning_ms = straight_ms = 0;
        active = false;
    } else if ((p >= PT_CURVE_R && p < PT_CURVE_L + TRACE_STAGES) || fabsf(servo_est_deg) >= 5.0f) {
        if (turning_ms < 30) ++turning_ms;
        straight_ms = 0;
        if (turning_ms >= 30) active = true;
    } else {
        turning_ms = 0;
        if (fabsf(servo_est_deg) <= 3.0f) {
            if (straight_ms < 70) ++straight_ms;
            if (straight_ms >= 70) active = false;
        } else straight_ms = 0;
    }
    return active;
#else
    return cnt_start >= DBG_FROM_MS && cnt_start < DBG_TO_MS;
#endif
#endif
}

static void dbg_take_params_in(void) {
    dbg_p_cross_to = crank_to_m;    dbg_p_cross_w = crank_line_w_m;
    dbg_p_crank_entry = crank_entry_mps; dbg_p_crank_extra = crank_extra_m;
    dbg_p_lane_to = lane_to_m;      dbg_p_lane_w = lane_line_w_m;
    dbg_p_lane_entry = lane_entry_mps;   dbg_p_lane_extra = lane_extra_m;
    dbg_p_vmax_in = v_max_mps;
    dbg_p_spot_no = spot_no;
}

static void dbg_take_params_out(void) {
    // 区間に入った時（ラインを見つけた時）にはまだ決まっていない値もあるので、出た時にもう一度取る
    dbg_p_cross_to = crank_to_m;    dbg_p_cross_w = crank_line_w_m;
    dbg_p_lane_to = lane_to_m;      dbg_p_lane_w = lane_line_w_m;
    dbg_p_crank_entry = crank_entry_mps;
    dbg_p_lane_entry = lane_entry_mps;
    dbg_p_vmax_out = v_max_mps;
    dbg_p_wait = plan_wait_m;  dbg_p_len = plan_len_m;
    dbg_p_speed = plan_speed_mps;  dbg_p_angle = plan_angle;
    dbg_p_switch = plan_switch_m;
    dbg_p_lane_end = lane_end_m;  dbg_p_lane_sw0 = lane_sw0_m;  dbg_p_lane_cross = lane_cross_m;
    dbg_p_lane_lag = lane_cross_lag_m;  dbg_p_lane_offset = lane_offset_m;
    dbg_p_unwind_m = crank_unwind_m;  dbg_p_crank_app = crank_app_mps;  dbg_p_crank_wide = crank_wide;
}

static void dbg_freeze(int reason) {
    if (dbg_frozen) return;
    dbg_take_params_out();
    dbg_reason = reason;
    dbg_stop_ms = cnt_start;
    dbg_frozen = 1;
#if DBG_AUTO_STOP
    if (reason != 4) {
        pattern = PT_END;
        run_end();  // モータ出力を0にし、通常ログにも終了を要求する
    }
#endif
}

//**********************************************************************
// 1ms 割り込みから毎回呼ぶ
//**********************************************************************
void dbg_sample(void) {
    if (dbg_frozen) return;
    const int p = pattern;
    if (p < PT_TRACE) return;                    // スタート前は記録しない

    if (dbg_dip < 0) dbg_dip = dipsw_get();
    int marker = dbg_marker(p);
    if (marker && marker != dbg_marker(dbg_previous)) {
        dbg_route = (dbg_route ^ (uint32_t)marker) * 16777619u;
    }
    // クロスラインの後に実際に入ったクランクの左右も区別する。
    if (p != dbg_previous && (p == PT_CRANK_L_ENTER || p == PT_CRANK_R_ENTER ||
        p == PT_CRANK_L_ARC || p == PT_CRANK_R_ARC) &&
        dbg_previous >= PT_CROSS_FOUND && dbg_previous <= PT_CRANK_SEARCH) {
        dbg_route = (dbg_route ^ (uint32_t)(p < PT_CRANK_R_ENTER ? 4 : 5)) * 16777619u;
    }
#if DBG_TARGET == 5 || DBG_TARGET == 6
    if (p < PT_CROSS_FOUND && dbg_previous >= PT_CROSS_FOUND) dbg_trace_start = cnt_start;
    if (marker && marker != dbg_marker(dbg_previous)) {
        ++dbg_events;
        if (!DBG_TARGET_NO || dbg_events == DBG_TARGET_NO) {
            dbg_trigger_ms = DBG_TARGET == 5 ? dbg_trace_start : 0;
            dbg_end_ms = cnt_start;
            dbg_take_params_in();
        }
    }
#else
    // 選んだ区間に入った・出たの判断
    const bool ev = dbg_is_event(p);
    if (ev && !dbg_in_event) {
        dbg_in_event = 1;
        dbg_events++;
#if DBG_TARGET == 4
        dbg_route = (dbg_route ^ (uint32_t)(servo_est_deg >= 0.0f ? 6 : 7)) * 16777619u;
#endif
        if (DBG_TARGET_NO == 0 || dbg_events == DBG_TARGET_NO) {
            dbg_trigger_ms = cnt_start;
            dbg_end_ms = 0;
            dbg_after_trigger = 0;
            dbg_chg_on = DBG_CHG_ON;
            dbg_chg_count = 0;
            dbg_take_params_in();
        }
    } else if (!ev && dbg_in_event && p < PT_STOP) {
        dbg_in_event = 0;
#if DBG_TARGET != 3
        // ハーフラインの仮判定がクロスに訂正された場合などは完走として数えない。
        if (p < PT_TRACE || p >= PT_CROSS_FOUND) {
            if (dbg_events == DBG_TARGET_NO) { dbg_trigger_ms = 0; dbg_chg_on = 0; }
            --dbg_events;
            dbg_after_trigger = -1;
        } else
#endif
        if (DBG_TARGET_NO == 0 || dbg_events == DBG_TARGET_NO) dbg_end_ms = cnt_start;
        dbg_after_trigger = -1;
    }

#endif
    dbg_previous = p;

    // この 1ms の値
    dbg_t now;
    now.t = (uint16_t)cnt_start;
    now.ptn = (int16_t)p;
    now.sen = sensor_inp(MASK4_4);
    now.ml = log_raw.ml;
    now.mr = log_raw.mr;
    now.flags = (brake_started ? 1 : 0) | (model_ok ? 2 : 0) | (crank_unwind ? 4 : 0) | (crank_measuring ? 8 : 0);
    now.steer = dbg_i16(steer_cmd_deg * 10.0f);
    now.lpass = (uint8_t)clamp_int(line_pass_ms, 0, 255);

    bool take = (cnt_start % DBG_INTERVAL_MS == 0);
#if DBG_CHG_ON
    // 決まった間隔の間でも、pattern・センサ・ブレーキなどの状態・ラインの通過時間が変わった ms、
    // モータ出力が DBG_CHG_MOTOR 以上（または前進とブレーキが入れ替わった）、ハンドルの指令が DBG_CHG_STEER10 以上
    // 変わった ms は、その ms の行を足す（最後に記録した行と比べる）。変わった時刻が 1ms 単位で分かる
    static dbg_t last;
    bool chg = false;
    if (!take && dbg_chg_on && dbg_count > 0 && dbg_chg_count < DBG_CHANGE_MAX) {
        chg = now.ptn != last.ptn || now.sen != last.sen || now.flags != (last.flags & 0x0f) || now.lpass != last.lpass
           || abs(now.ml - last.ml) >= DBG_CHG_MOTOR || abs(now.mr - last.mr) >= DBG_CHG_MOTOR
           || (now.ml < 0) != (last.ml < 0) || (now.mr < 0) != (last.mr < 0)
           || abs(now.steer - last.steer) >= DBG_CHG_STEER10;
        if (chg) {
            take = true;
            dbg_chg_count++;
            dbg_chg_total++;
            now.flags |= 0x10;
        }
    }
#endif
    if (take) {
        now.servo = dbg_i16(servo_est_deg * 10.0f);
        now.spd = dbg_i16(speed_mps * 1000.0f);
        now.dist = dbg_i16(dist_m * 1000.0f);
        now.vmax = dbg_i16(v_max_mps * 1000.0f);
#if DBG_TARGET == 2
        now.yaw = dbg_i16(lane_yaw * (1800.0f / PI_F));
#else
        now.yaw = dbg_i16(crank_yaw * (1800.0f / PI_F));
#endif
        now.lined = dbg_i16(obs_z2 * (1800.0f / PI_F));
        now.lat = dbg_i16(speed_mps * speed_mps * car_k_eff * 100.0f);
        const int lm = (int)(obs_z1 * 1000.0f + (obs_z1 >= 0.0f ? 0.5f : -0.5f));
        now.linemm = (int8_t)clamp_int(lm, -127, 127);
        dbg_buf[dbg_head] = now;
#if DBG_CHG_ON
        last = now;
#endif
        dbg_head = (dbg_head + 1 == DBG_SAMPLES) ? 0 : dbg_head + 1;
        if (dbg_count < DBG_SAMPLES) dbg_count++;
#if DBG_TARGET != 3 && DBG_TARGET != 5 && DBG_TARGET != 6
        // 区間が長く続いて（復帰処理から抜けられない など）区間の始まりが上書きされそうになったら、そこで止める。
        // 始まりが消えると、なぜそうなったかが分からない（CRK0054 / CRK0055 は止まっている 2.2 秒だけが残った）。
        // 区間の前の様子を DBG_PRE_KEEP 個だけ残す。止めた理由は 2（途中で止まった）として書く
        if (dbg_after_trigger >= 0 && DBG_TARGET_NO != 0) {
            dbg_after_trigger++;
            if (dbg_after_trigger >= DBG_SAMPLES - DBG_PRE_KEEP) {
                dbg_freeze(2);
                return;
            }
        }
#endif
    }

    // 記録を止める
    if (p >= PT_STOP) {                          // 自動停止・コースアウト・プッシュスイッチ
        dbg_freeze(2);
        return;
    }
#if DBG_TARGET == 3
    if (cnt_start >= DBG_TO_MS) dbg_freeze(3);
#else
    if (DBG_TARGET_NO != 0 && dbg_end_ms != 0 && cnt_start >= dbg_end_ms + DBG_POST_MS) dbg_freeze(1);
#endif
}

//**********************************************************************
// 書き出し（microSD と模擬の両方で使う）
//**********************************************************************
// 小数を "12.345" の形にする（printf の %f が使えない環境があるため）
static void dbg_fmt_f(char* out, int size, float x) {
    long m = (long)(x * 1000.0f + (x >= 0.0f ? 0.5f : -0.5f));
    const char* sign = "";
    if (m < 0) { sign = "-"; m = -m; }
    snprintf(out, size, "%s%ld.%03ld", sign, m / 1000, m % 1000);
}

// ファイルの先頭の行（# で始まる）を n 行目まで作る。行がなくなったら 0 を返す
static int dbg_header_line(int n, char* buf, int size) {
    char a[16];
    switch (n) {
    case 0: snprintf(buf, size, "#dbg,1,trial,%lu,run,%d,dip,%d,route,%lu", (unsigned long)DBG_TRIAL_ID, run_log_no, dbg_dip, (unsigned long)dbg_route); return 1;
    case 1: snprintf(buf, size, "#target,%d,target_no,%d,interval_ms,%d,samples,%d,post_ms,%d,change_rows,%d,change_max,%d,change_used,%d",
                     DBG_TARGET, DBG_TARGET_NO, DBG_INTERVAL_MS, DBG_SAMPLES, DBG_POST_MS, (int)DBG_CHG_ON, DBG_CHANGE_MAX, dbg_chg_total); return 1;
    case 2: snprintf(buf, size, "#reason,%d,events,%d,trigger_ms,%lu,end_ms,%lu",
                     dbg_reason, dbg_events, dbg_trigger_ms, dbg_end_ms); return 1;
    case 3: snprintf(buf, size, "#switch,TRACTION,%d,TRACE_SPEED,%d,CURVE_SPEED,%d,STEER,%d,CRANK_BRAKE,%d,CRANK_TURN,%d,CRANK_QUICK,%d,BRAKE_RANGE,%d",
                     USE_NEW_TRACTION, USE_NEW_TRACE_SPEED, USE_NEW_CURVE_SPEED, USE_NEW_STEER,
                     USE_NEW_CRANK_BRAKE, USE_NEW_CRANK_TURN, USE_NEW_CRANK_QUICK, USE_NEW_BRAKE_RANGE); return 1;
    case 4: snprintf(buf, size, "#switch,LANE_BRAKE,%d,LANE_STEER,%d,VMAX_LEARN,%d,LINE_SHAPE,%d,CURVE_PREVIEW,%d,LANE_SNAP,%d",
                     USE_NEW_LANE_BRAKE, USE_NEW_LANE_STEER, USE_NEW_VMAX_LEARN, USE_NEW_LINE_SHAPE, USE_NEW_CURVE_PREVIEW, USE_NEW_LANE_SNAP); return 1;
    }
    // 小数の値は "#param,名前,値" で1行ずつ
    static const char* const names[] = {
        "CURVE_DRIVE_PERCENT", "V_MAX_MPS", "MOTOR_TAU_S", "GRIP_LAT_MPS2", "GRIP_LONG_MPS2", "PLAN_GRIP_RATIO_LAT", "PLAN_GRIP_RATIO_LONG",
        "CRANK_BRAKE_POWER", "CRANK_BRAKE_MARGIN_M", "CRANK_QUICK_WINDOW_M", "PLAN_OVERSPEED",
        "LANE_BRAKE_POWER", "LANE_BRAKE_MARGIN_M", "SENSOR_TO_REAR_AXLE_M", "WHEELBASE_M",
        "crank_to_m", "crank_line_w_m", "crank_entry_mps", "crank_extra_m", "crank_q_r", "crank_q_d", "crank_q_dlo",
        "lane_to_m", "lane_line_w_m", "lane_entry_mps", "lane_extra_m",
        "vmax_at_trigger", "vmax_at_end", "plan_wait_m", "plan_len_m", "plan_speed_mps", "plan_angle", "spot_no",
        // ここから下は詳細ログで足した値（クランクのブレーキ位置、レーンチェンジの経路の直し）
        "CRANK_ENTRY_MPS", "CRANK_BRAKE_BEFORE_M", "CRANK_DIST_RATIO", "CRANK_WIDE_SEEN_M", "CRANK_WIDE_TO_M", "CRANK_EXIT_MIN_YAW_DEG",
        "LINE_SEEN_EXTRA_M", "LINE_PASS_OFFSET_MS", "CENTER_LINE_WIDTH_M", "BRAKE_DECEL_MPS2", "TREAD_M",
        "LANE_PRE_M", "LANE_SECTION_M", "LANE_QUICK_LAT_MPS2", "LANE_CROSS_ADJUST_M", "SENSOR_HALF_WIDTH_M",
        "LANE_SNAP_ANGLE", "LANE_SNAP_LAT_MPS2", "LANE_SNAP_MPS", "LANE_SNAP_AHEAD_M", "LANE_SNAP_END_DEG", "LANE_SNAP_SEARCH_M",
        "crank_wide", "crank_app_mps", "crank_unwind_m",
        "plan_switch_m", "lane_end_m", "lane_sw0_m", "lane_cross_m", "lane_cross_lag_m", "lane_offset_m",
        "CRANK_BRAKE_DECEL_MPS2", "SPEED_MEAS_MAX_MPS"
    };
    const float vals[] = {
        (float)CURVE_DRIVE_PERCENT, V_MAX_MPS, MOTOR_TAU_S, GRIP_LAT_MPS2, GRIP_LONG_MPS2, PLAN_GRIP_RATIO_LAT, PLAN_GRIP_RATIO_LONG,
        (float)CRANK_BRAKE_POWER, CRANK_BRAKE_MARGIN_M, CRANK_QUICK_WINDOW_M, PLAN_OVERSPEED,
        (float)LANE_BRAKE_POWER, LANE_BRAKE_MARGIN_M, SENSOR_TO_REAR_AXLE_M, WHEELBASE_M,
        dbg_p_cross_to, dbg_p_cross_w, dbg_p_crank_entry, dbg_p_crank_extra, crank_q_r, crank_q_d, crank_q_dlo,
        dbg_p_lane_to, dbg_p_lane_w, dbg_p_lane_entry, dbg_p_lane_extra,
        dbg_p_vmax_in, dbg_p_vmax_out, dbg_p_wait, dbg_p_len, dbg_p_speed, dbg_p_angle, (float)dbg_p_spot_no,
        CRANK_ENTRY_MPS, CRANK_BRAKE_BEFORE_M, CRANK_DIST_RATIO, CRANK_WIDE_SEEN_M, CRANK_WIDE_TO_M, CRANK_EXIT_MIN_YAW_DEG,
        LINE_SEEN_EXTRA_M, (float)LINE_PASS_OFFSET_MS, CENTER_LINE_WIDTH_M, BRAKE_DECEL_MPS2, TREAD_M,
        LANE_PRE_M, LANE_SECTION_M, LANE_QUICK_LAT_MPS2, LANE_CROSS_ADJUST_M, SENSOR_HALF_WIDTH_M,
        (float)LANE_SNAP_ANGLE, LANE_SNAP_LAT_MPS2, LANE_SNAP_MPS, LANE_SNAP_AHEAD_M, LANE_SNAP_END_DEG, LANE_SNAP_SEARCH_M,
        (float)dbg_p_crank_wide, dbg_p_crank_app, dbg_p_unwind_m,
        dbg_p_switch, dbg_p_lane_end, dbg_p_lane_sw0, dbg_p_lane_cross, dbg_p_lane_lag, dbg_p_lane_offset,
        CRANK_BRAKE_DECEL_MPS2, SPEED_MEAS_MAX_MPS
    };
    static_assert(sizeof(names) / sizeof(names[0]) == sizeof(vals) / sizeof(vals[0]), "param names and values differ");
    const int np = sizeof(vals) / sizeof(vals[0]);
    int k = n - 5;
    if (k < np) {
        dbg_fmt_f(a, sizeof(a), vals[k]);
        snprintf(buf, size, "#param,%s,%s", names[k], a);
        return 1;
    }
    if (k == np) {
        snprintf(buf, size, "ms,pattern,sensor,steer_deg10,motor_l,motor_r,servo_deg10,speed_mm_s,dist_mm,vmax_mm_s,yaw_deg10,line_pass_ms,brake,model_ok,unwind,measuring,line_mm,line_deg10,lat_cm_s2,chg");
        return 1;
    }
    return 0;
}

// 古い方から i 番目（0 から）の記録を1行にする。t_full は時間の上位ビットを戻すのに使う
static void dbg_data_line(int i, char* buf, int size, unsigned long* t_full) {
    int idx = dbg_head - dbg_count + i;
    if (idx < 0) idx += DBG_SAMPLES;
    const dbg_t& e = dbg_buf[idx];
    // 下16ビットだけ持っているので、前の行より小さくなったら 65536 を足す
    unsigned long t = (*t_full & ~0xffffUL) | e.t;
    if (i > 0 && t < *t_full) t += 0x10000UL;
    *t_full = t;
    snprintf(buf, size, "%lu,%d,'%08lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             t, (int)e.ptn, convertBCD_CharToLong(e.sen), (int)e.steer, (int)e.ml, (int)e.mr, (int)e.servo,
             (int)e.spd, (int)e.dist, (int)e.vmax, (int)e.yaw, (int)e.lpass,
             e.flags & 1, (e.flags >> 1) & 1, (e.flags >> 2) & 1, (e.flags >> 3) & 1,
             (int)e.linemm, (int)e.lined, (int)e.lat, (e.flags >> 4) & 1);
}

// 最初の行の時間の上位ビット（止めた時の時間から、残っている行の時間の差を足し戻して求める。
// 変化した瞬間の行があるので、行の間隔は一定ではない）
static unsigned long dbg_first_t_hint(void) {
    if (dbg_count <= 0) return 0;
    int li = dbg_head - 1;
    if (li < 0) li += DBG_SAMPLES;
    // 最後の記録の時間を、止めた時間から戻して求める
    unsigned long last = dbg_stop_ms;   // 最後の記録はこの時間か、それより少し前
    unsigned long t_last = (last & ~0xffffUL) | dbg_buf[li].t;
    if (t_last > last && t_last >= 0x10000UL) t_last -= 0x10000UL;
    unsigned long span = 0;
    for (int i = 1; i < dbg_count; i++) {
        int b = li - i + 1, a = li - i;
        if (b < 0) b += DBG_SAMPLES;
        if (a < 0) a += DBG_SAMPLES;
        span += (uint16_t)(dbg_buf[b].t - dbg_buf[a].t);
    }
    return t_last >= span ? t_last - span : 0;
}

//**********************************************************************
// loop から毎回呼ぶ：記録が止まり、通常のログを閉じた後に microSD へ書き出す
//**********************************************************************
void dbg_task(void) {
    // 通常ログのcloseを待つ。
    if (dbg_written || err_sd != 0) return;
    if (!dbg_frozen) {
        // 区間の前にプッシュスイッチで止めた場合も、ここで止めて書き出す
        if (log_st == LOG_CLOSE) dbg_freeze(2);
        return;
    }
    if (!run_log_closed) return;
    int no = run_log_no;
    char buf[240];
    char name[16];
#if DBG_AUTO_STOP
    static const char* const prefix[] = {"", "CRK", "LANE", "TIME", "CURV", "TRCE", "FULL"};
    snprintf(name, sizeof(name), "%s%04d.csv", prefix[DBG_TARGET], no);
#else
    snprintf(name, sizeof(name), "dbg%04d.csv", no);
#endif
    led_out_m(1);                                  // 書いている間はマイコンボードの LED を点ける
    if (SD.exists(name)) {
        Serial.println("同じ番号の詳細ログが既にあります。SDを確認してください。");
        err_sd = 1;
        return;
    }
    File out = SD.open(name, FILE_WRITE);
    if (out) {
        for (int n = 0; dbg_header_line(n, buf, sizeof(buf)); n++) out.println(buf);
        unsigned long t_full = dbg_first_t_hint();
        for (int i = 0; i < dbg_count; i++) {
            dbg_data_line(i, buf, sizeof(buf), &t_full);
            out.println(buf);
        }
        snprintf(buf, sizeof(buf), "#complete,%d", (int)dbg_count);
        out.println(buf);
        out.close();
        Serial.print(name);
        Serial.println("に詳しいログを書きました。");
    } else {
        err_sd = 1;
        Serial.println("詳細ログを作成できません。SDを確認してください。");
        return;
    }
    led_out_m(0);                                  // 消えたら電源を切ってよい
    dbg_written = 1;
}

#ifdef DBG_HOST
// 模擬（debug/tools/sim_compare.sh）で使う：模擬が終わった時に、記録をファイル DBG_HOST_OUT へ書く
#include <cstdio>
#include <cstdlib>
static void dbg_host_dump(void) {
    const char* path = getenv("DBG_HOST_OUT");
    if (!path) return;
    dbg_freeze(4);
    FILE* fp = fopen(path, "w");
    if (!fp) return;
    char buf[240];
    for (int n = 0; dbg_header_line(n, buf, sizeof(buf)); n++) fprintf(fp, "%s\n", buf);
    unsigned long t_full = dbg_first_t_hint();
    for (int i = 0; i < dbg_count; i++) {
        dbg_data_line(i, buf, sizeof(buf), &t_full);
        fprintf(fp, "%s\n", buf);
    }
    fprintf(fp, "#complete,%d\n", (int)dbg_count);
    fclose(fp);
}
static int dbg_host_reg = atexit(dbg_host_dump);
#endif

#else
void dbg_sample(void) {}
void dbg_task(void) {}
#endif
