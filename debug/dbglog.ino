//**************************************************************************
// dbglog.ino  デバッグモードの詳しいログ（debug/ だけにあるファイル。sync.sh で上書きされない）
//   設定は dbg_config.h。読み方と使い方は debug/README.md
//
//   dbg_sample() : 1ms 割り込みの最後（speed_model_update の後）に毎回呼ぶ。RAM のリングバッファに貯める
//   dbg_task()   : loop から毎回呼ぶ。記録が止まり、通常のログが閉じられた後に dbgNNNN.csv へ書き出す
//**************************************************************************
#if DBG_ENABLE

typedef struct {
    uint16_t t;        // スタートからの時間 [ms]（下16ビット。書き出す時に戻す）
    int16_t ptn;       // pattern
    uint8_t sen;       // センサ 8 個
    int8_t ml;         // 左モータ出力 [%]
    int8_t mr;         // 右モータ出力 [%]
    uint8_t flags;     // bit0 brake_started, bit1 model_ok, bit2 crank_unwind, bit3 crank_measuring
    int16_t steer;     // ハンドル角度の指令 [0.1度]
    int16_t servo;     // サーボの本当の角度の推定 [0.1度]
    int16_t spd;       // 推定した速さ [mm/s]
    int16_t dist;      // ラインの終わり（または曲がり始め）からの距離 [mm]
    int16_t vmax;      // 出力100%の速さ [mm/s]
    int16_t yaw;       // クランクで曲がり始めてから向きが変わった角度 [0.1度]（USE_NEW_CRANK_QUICK）
    uint8_t lpass;     // ラインの通過時間 [ms]（最後に測った値）
    uint8_t pad;
} dbg_t;               // 24 バイト

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

// 区間に入った時・出た時の値（書き出す時にファイルの先頭へ）
static float dbg_p_cross_to, dbg_p_cross_w, dbg_p_crank_entry, dbg_p_crank_extra;
static float dbg_p_lane_to, dbg_p_lane_w, dbg_p_lane_entry, dbg_p_lane_extra;
static float dbg_p_vmax_in, dbg_p_vmax_out;
static float dbg_p_wait, dbg_p_len, dbg_p_speed, dbg_p_angle;
static int dbg_p_spot_no;

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
    return (p >= PT_LANE_R_FOUND && p <= PT_LANE_R_SELECT) || (p >= PT_LANE_R_APPROACH && p <= 450)
        || (p >= PT_LANE_L_FOUND && p <= PT_LANE_L_SELECT) || (p >= PT_LANE_L_APPROACH && p <= 750);
#else
    return cnt_start >= DBG_FROM_MS && cnt_start < DBG_TO_MS;
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
    dbg_p_vmax_out = v_max_mps;
    dbg_p_wait = plan_wait_m;  dbg_p_len = plan_len_m;
    dbg_p_speed = plan_speed_mps;  dbg_p_angle = plan_angle;
}

static void dbg_freeze(int reason) {
    if (dbg_frozen) return;
    dbg_take_params_out();
    dbg_reason = reason;
    dbg_stop_ms = cnt_start;
    dbg_frozen = 1;
}

//**********************************************************************
// 1ms 割り込みから毎回呼ぶ
//**********************************************************************
void dbg_sample(void) {
    if (dbg_frozen) return;
    const int p = pattern;
    if (p < PT_TRACE) return;                    // スタート前は記録しない

    // 選んだ区間に入った・出たの判断
    const bool ev = dbg_is_event(p);
    if (ev && !dbg_in_event) {
        dbg_in_event = 1;
        dbg_events++;
        if (DBG_TARGET_NO == 0 || dbg_events == DBG_TARGET_NO) {
            dbg_trigger_ms = cnt_start;
            dbg_end_ms = 0;
            dbg_take_params_in();
        }
    } else if (!ev && dbg_in_event) {
        dbg_in_event = 0;
        if (DBG_TARGET_NO == 0 || dbg_events == DBG_TARGET_NO) dbg_end_ms = cnt_start;
    }

    if (cnt_start % DBG_INTERVAL_MS == 0) {
        dbg_t& e = dbg_buf[dbg_head];
        e.t = (uint16_t)cnt_start;
        e.ptn = (int16_t)p;
        e.sen = sensor_inp(MASK4_4);
        e.ml = log_raw.ml;
        e.mr = log_raw.mr;
        e.flags = (brake_started ? 1 : 0) | (model_ok ? 2 : 0) | (crank_unwind ? 4 : 0) | (crank_measuring ? 8 : 0);
        e.steer = dbg_i16(steer_cmd_deg * 10.0f);
        e.servo = dbg_i16(servo_est_deg * 10.0f);
        e.spd = dbg_i16(speed_mps * 1000.0f);
        e.dist = dbg_i16(dist_m * 1000.0f);
        e.vmax = dbg_i16(v_max_mps * 1000.0f);
        e.yaw = dbg_i16(crank_yaw * (1800.0f / PI_F));
        e.lpass = (uint8_t)clamp_int(line_pass_ms, 0, 255);
        e.pad = 0;
        dbg_head = (dbg_head + 1 == DBG_SAMPLES) ? 0 : dbg_head + 1;
        if (dbg_count < DBG_SAMPLES) dbg_count++;
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
    case 0: snprintf(buf, size, "#dbg,1"); return 1;
    case 1: snprintf(buf, size, "#target,%d,target_no,%d,interval_ms,%d,samples,%d,post_ms,%d",
                     DBG_TARGET, DBG_TARGET_NO, DBG_INTERVAL_MS, DBG_SAMPLES, DBG_POST_MS); return 1;
    case 2: snprintf(buf, size, "#reason,%d,events,%d,trigger_ms,%lu,end_ms,%lu",
                     dbg_reason, dbg_events, dbg_trigger_ms, dbg_end_ms); return 1;
    case 3: snprintf(buf, size, "#switch,TRACTION,%d,TRACE_SPEED,%d,CURVE_SPEED,%d,STEER,%d,CRANK_BRAKE,%d,CRANK_TURN,%d,CRANK_QUICK,%d,BRAKE_RANGE,%d",
                     USE_NEW_TRACTION, USE_NEW_TRACE_SPEED, USE_NEW_CURVE_SPEED, USE_NEW_STEER,
                     USE_NEW_CRANK_BRAKE, USE_NEW_CRANK_TURN, USE_NEW_CRANK_QUICK, USE_NEW_BRAKE_RANGE); return 1;
    case 4: snprintf(buf, size, "#switch,LANE_BRAKE,%d,LANE_STEER,%d,VMAX_LEARN,%d,LINE_SHAPE,%d,CURVE_PREVIEW,%d",
                     USE_NEW_LANE_BRAKE, USE_NEW_LANE_STEER, USE_NEW_VMAX_LEARN, USE_NEW_LINE_SHAPE, USE_NEW_CURVE_PREVIEW); return 1;
    }
    // 小数の値は "#param,名前,値" で1行ずつ
    static const char* const names[] = {
        "V_MAX_MPS", "MOTOR_TAU_S", "GRIP_LAT_MPS2", "GRIP_LONG_MPS2", "PLAN_GRIP_RATIO_LAT", "PLAN_GRIP_RATIO_LONG",
        "CRANK_BRAKE_POWER", "CRANK_BRAKE_MARGIN_M", "CRANK_QUICK_WINDOW_M", "PLAN_OVERSPEED",
        "LANE_BRAKE_POWER", "LANE_BRAKE_MARGIN_M", "SENSOR_TO_REAR_AXLE_M", "WHEELBASE_M",
        "crank_to_m", "crank_line_w_m", "crank_entry_mps", "crank_extra_m", "crank_q_r", "crank_q_d", "crank_q_dlo",
        "lane_to_m", "lane_line_w_m", "lane_entry_mps", "lane_extra_m",
        "vmax_at_trigger", "vmax_at_end", "plan_wait_m", "plan_len_m", "plan_speed_mps", "plan_angle", "spot_no"
    };
    const float vals[] = {
        V_MAX_MPS, MOTOR_TAU_S, GRIP_LAT_MPS2, GRIP_LONG_MPS2, PLAN_GRIP_RATIO_LAT, PLAN_GRIP_RATIO_LONG,
        (float)CRANK_BRAKE_POWER, CRANK_BRAKE_MARGIN_M, CRANK_QUICK_WINDOW_M, PLAN_OVERSPEED,
        (float)LANE_BRAKE_POWER, LANE_BRAKE_MARGIN_M, SENSOR_TO_REAR_AXLE_M, WHEELBASE_M,
        dbg_p_cross_to, dbg_p_cross_w, dbg_p_crank_entry, dbg_p_crank_extra, crank_q_r, crank_q_d, crank_q_dlo,
        dbg_p_lane_to, dbg_p_lane_w, dbg_p_lane_entry, dbg_p_lane_extra,
        dbg_p_vmax_in, dbg_p_vmax_out, dbg_p_wait, dbg_p_len, dbg_p_speed, dbg_p_angle, (float)dbg_p_spot_no
    };
    const int np = sizeof(vals) / sizeof(vals[0]);
    int k = n - 5;
    if (k < np) {
        dbg_fmt_f(a, sizeof(a), vals[k]);
        snprintf(buf, size, "#param,%s,%s", names[k], a);
        return 1;
    }
    if (k == np) {
        snprintf(buf, size, "ms,pattern,sensor,steer_deg10,motor_l,motor_r,servo_deg10,speed_mm_s,dist_mm,vmax_mm_s,yaw_deg10,line_pass_ms,brake,model_ok,unwind,measuring");
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
    snprintf(buf, size, "%lu,%d,'%08lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             t, (int)e.ptn, convertBCD_CharToLong(e.sen), (int)e.steer, (int)e.ml, (int)e.mr, (int)e.servo,
             (int)e.spd, (int)e.dist, (int)e.vmax, (int)e.yaw, (int)e.lpass,
             e.flags & 1, (e.flags >> 1) & 1, (e.flags >> 2) & 1, (e.flags >> 3) & 1);
}

// 最初の行の時間の上位ビット（止めた時の時間から、残っている長さを引いて求める）
static unsigned long dbg_first_t_hint(void) {
    unsigned long span = (unsigned long)(dbg_count > 0 ? dbg_count - 1 : 0) * DBG_INTERVAL_MS;
    unsigned long last = dbg_stop_ms;   // 最後の記録はこの時間か、それより少し前
    int li = dbg_head - 1;
    if (li < 0) li += DBG_SAMPLES;
    // 最後の記録の時間を、止めた時間から戻して求める
    unsigned long t_last = (last & ~0xffffUL) | dbg_buf[li].t;
    if (t_last > last) t_last -= 0x10000UL;
    return t_last >= span ? t_last - span : 0;
}

//**********************************************************************
// loop から毎回呼ぶ：記録が止まり、通常のログを閉じた後に microSD へ書き出す
//**********************************************************************
void dbg_task(void) {
    static unsigned long ready_ms = 0;
    if (dbg_written || err_sd != 0) return;
    if (!dbg_frozen) {
        // 区間の前にプッシュスイッチで止めた場合も、ここで止めて書き出す
        if (log_st == LOG_CLOSE) dbg_freeze(2);
        return;
    }
    if (log_st != LOG_CLOSE || log_write != log_read) return;   // 通常のログを書き終えるまで待つ
    if (ready_ms == 0) { ready_ms = cnt0 + 200; return; }       // 通常のログを閉じるのを待つ
    if (cnt0 < ready_ms) return;

    // 通常のログと同じ番号にする（renban.txt には今のログの番号が入っている）
    int no = 0;
    char buf[200];
    File f = SD.open("renban.txt", FILE_READ);
    if (f) {
        int n = f.read(buf, 8);
        buf[n > 0 ? n : 0] = '\0';
        sscanf(buf, "%d", &no);
        f.close();
    }
    char name[16];
    snprintf(name, sizeof(name), "dbg%04d.csv", no);
    led_out_m(1);                                  // 書いている間はマイコンボードの LED を点ける
    File out = SD.open(name, FILE_WRITE);
    if (out) {
        for (int n = 0; dbg_header_line(n, buf, sizeof(buf)); n++) out.println(buf);
        unsigned long t_full = dbg_first_t_hint();
        for (int i = 0; i < dbg_count; i++) {
            dbg_data_line(i, buf, sizeof(buf), &t_full);
            out.println(buf);
        }
        out.close();
        Serial.print(name);
        Serial.println("に詳しいログを書きました。");
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
    char buf[200];
    for (int n = 0; dbg_header_line(n, buf, sizeof(buf)); n++) fprintf(fp, "%s\n", buf);
    unsigned long t_full = dbg_first_t_hint();
    for (int i = 0; i < dbg_count; i++) {
        dbg_data_line(i, buf, sizeof(buf), &t_full);
        fprintf(fp, "%s\n", buf);
    }
    fclose(fp);
}
static int dbg_host_reg = atexit(dbg_host_dump);
#endif

#else
void dbg_sample(void) {}
void dbg_task(void) {}
#endif
