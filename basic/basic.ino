//**************************************************************************
// 対象マイコン  Renesas Electronics RMC_RA4M1(Rev.2.0)(R7FA4M1AB3CFP9 100pin)
// ファイル内容  Basicマイコンカー走行プログラム（AQM1248液晶なし、microSDにログ記録）
// 元プログラム  ジャパンマイコンカーラリー実行委員会改編版_ＴＰＦ監修 2024.09.01
//               2024.10.15 R.K 版を、動作を変えずに読みやすく作り直したもの
// ライセンス    This software is released under the MIT License.
//               http://opensource.org/licenses/mit-license.php
//
// ファイル構成（Arduino IDE では上部のタブとして表示される）
//   basic.ino      … 初期化、loop、1ms割り込み、pattern の振り分け（このファイル）
//   config.h       … 調整用の数値（時間、速度、段階の切り替え条件など）
//   tables.h       … センサの状態ごとのハンドル角度・モータ値の表
//   hardware.ino   … センサ、モータ、サーボ、LED、スイッチの入出力
//   sensor.ino     … クロスライン・ハーフラインの判定、表の検索
//   run_trace.ino  … スタート、通常走行、大曲がり、停止
//   run_crank.ino  … クロスライン、クランク
//   run_lane.ino   … 右・左レーンチェンジ
//   sdlog.ino      … microSD へのログ保存
//**************************************************************************

//========================================
// インクルード
//========================================
#include "mcr_gpt_lib.h"     // GPTを使用するためのライブラリ
#include <SPI.h>             // microSDとSPI通信するためのライブラリ
#include <SD.h>              // microSD読み書きライブラリ
#include "config.h"
#include "tables.h"

#define timer(x) delay(x)    // timer関数=delay関数

//========================================
// センサのマスク値  ×：マスクあり(無効)　○：マスク無し(有効)
//========================================
#define MASK3_3       0xe7   // ○○○××○○○
#define MASK0_3       0x07   // ×××××○○○
#define MASK3_0       0xe0   // ○○○×××××
#define MASK4_4       0xff   // ○○○○○○○○

//========================================
// pattern 番号（ログの pattern 列にもこの番号が記録される）
//========================================
enum {
    PT_WAIT_SWITCH   = 0,    // プッシュスイッチ待ち
    PT_WAIT_START    = 1,    // スタートバーが開くのを待つ
    PT_TRACE         = 20,   // 通常走行 20〜25（+速度段階）
    PT_CURVE_R       = 30,   // 右大曲がり 30〜35（+段階）
    PT_CURVE_L       = 40,   // 左大曲がり 40〜45（+段階）
    PT_CROSS_FOUND   = 100,  // クロスライン検出
    PT_CROSS_PASS    = 101,  // クロスラインの通過時間を測る
    PT_CROSS_BRAKE   = 102,  // 減速（まだクランク判定しない）
    PT_CRANK_SEARCH  = 103,  // 減速しながらクランクを探す
    PT_CRANK_L_ENTER = 110,  // 左クランク 突入
    PT_CRANK_L_SLOW  = 111,  // 左クランク 低速旋回
    PT_CRANK_L_RETURN  = 120,  // 左クランク 中央線へ復帰
    PT_CRANK_L_OUTSIDE = 121,  // 左クランク 外側にはみ出した
    PT_CRANK_R_ENTER = 150,  // 右クランク 突入
    PT_CRANK_R_SLOW  = 151,  // 右クランク 低速旋回
    PT_CRANK_R_RETURN  = 160,  // 右クランク 中央線へ復帰
    PT_CRANK_R_OUTSIDE = 161,  // 右クランク 外側にはみ出した
    PT_LANE_R_FOUND    = 200,  // 右ハーフライン検出
    PT_LANE_R_PASS     = 201,  // 右ハーフラインの通過時間を測る
    PT_LANE_R_SELECT   = 202,  // 通過時間から段階を決める
    PT_LANE_R_APPROACH = 300,  // 300〜310 白線が切れるまで直進
    PT_LANE_R_STEER    = 320,  // 320〜330 ハンドルを切る
    PT_LANE_R_STRAIGHT = 340,  // 340〜350 切り終わって直進
    PT_LANE_R_ALIGN    = 400,  // 400〜410 新しい線に合わせる
    PT_LANE_R_FINISH   = 420,  // 420〜430 終了処理
    PT_LANE_R_RECOVER  = 440,  // 440〜450 外へふくらんだ時
    PT_LANE_L_FOUND    = 500,  // 以下、左レーンチェンジ（右と同じ並び）
    PT_LANE_L_PASS     = 501,
    PT_LANE_L_SELECT   = 502,
    PT_LANE_L_APPROACH = 600,
    PT_LANE_L_STEER    = 620,
    PT_LANE_L_STRAIGHT = 640,
    PT_LANE_L_ALIGN    = 700,
    PT_LANE_L_FINISH   = 720,
    PT_LANE_L_RECOVER  = 740,
    PT_STOP            = 800,  // 自動停止（ハンドルで線を追う）
    PT_STOP_HOLD       = 801,  // 完全停止（自動停止の後、またはコースアウト）
    PT_END             = 1001  // 走行とログ記録の終了
};

// 自動停止・コースアウト判定を行う pattern の範囲
#define PT_RUN_FIRST   PT_TRACE
#define PT_RUN_LAST    750
// プッシュスイッチで終了できる pattern の範囲
#define PT_PUSH_LAST   1000

// ライン通過時間の計測状態
#define PASS_WAIT      0   // まだラインに乗っていない
#define PASS_ON_LINE   1   // ラインの上
#define PASS_DONE      2   // ラインを通り過ぎた

// ログの状態
#define LOG_NON        0
#define LOG_INIT       1
#define LOG_WRITE      2
#define LOG_CLOSE      3
#define LOG_SLOTS      (LOG_MAX + 1)   // 空と満杯を区別するため1枠追加

//========================================
// プロトタイプ宣言
//========================================
// hardware.ino
unsigned char sensor_inp(unsigned char mask);
unsigned char dipsw_get(void);
unsigned char pushsw_get(void);
unsigned char startbar_get(void);
void led_out(unsigned char led);
void led_out_m(unsigned char led);
void sp(int accele_l, int accele_r);
void ha(int angle);
// sensor.ino
int check_crossline(void);
int check_shasenhenkoum(void);
int check_shasenhenkouh(void);
int check_hidarikakunin(void);
int find_sensor(const uint8_t* list, int n, unsigned char sensor);
void apply_act(const Act& act);
int apply_table(const uint8_t* list, const Act* acts, int n, unsigned char sensor);
void measure_line_pass(int on_line);
int clamp_int(int v, int lo, int hi);
// run_trace.ino
void run_wait_switch(void);
void run_wait_start(void);
void run_trace(int stage);
void run_curve(int stage, bool right);
void run_stop(void);
void run_stop_hold(void);
void run_end(void);
// run_crank.ino
void run_cross_found(void);
void run_cross_pass(void);
void run_cross_brake(void);
void run_crank_search(void);
void run_crank_enter(bool right);
void run_crank_slow(bool right);
void run_crank_return(bool right);
void run_crank_outside(bool right);
void crank_check_lost(unsigned char c, bool right);
void crank_speed_control(void);
// run_lane.ino
void run_lane_found(bool right);
void run_lane_pass(bool right);
void run_lane_select(bool right);
void run_lane_approach(int stage, bool right);
void run_lane_steer(int stage, bool right);
void run_lane_straight(int stage, bool right);
void run_lane_align(int stage, bool right);
void run_lane_finish(int stage, bool right);
void run_lane_recover(int stage, bool right);
void lane_speed_control(void);
int lane_pt(bool right, int pt_right, int pt_left);
bool lane_new_line_found(unsigned char c, bool right);
int lane_cross_recheck(bool right);
void lane_steer_output(int stage, bool right);
// basic.ino
void run_pattern(void);
// sdlog.ino
void log_setup(void);
void log_sample(void);
void log_task(void);
void set_microSD_dateTime(uint16_t* date, uint16_t* time);
unsigned long convertBCD_CharToLong(unsigned char hex);
int getCompileYear(const char* p);
int getCompileMonth(const char* p);
int getCompileDay(const char* p);
int getCompileHour(const char* p);
int getCompilerMinute(const char* p);
int getCompilerSecond(const char* p);
void AGTCallback(timer_callback_args_t __attribute((unused))* p_args);

//========================================
// グローバル変数
//========================================
volatile int pattern;                  // 現在の状態（pattern 番号）
volatile unsigned long cnt0;           // 1msごとに増える（LED点滅用、リセットしない）
volatile unsigned long cnt1;           // 1msごとに増える（各 pattern の経過時間。必要な所で0にする）
volatile unsigned long cnt_start;      // スタートからの時間（ログの時刻）
volatile unsigned long cnt_print;      // シリアル出力の間隔用
volatile unsigned long course_out_ms;  // コースアウト状態が続いている時間（元: daturin_cnt）
volatile unsigned long run_ms;         // 走行時間（自動停止用。元: kansou_cnt）
volatile int line_pass_state;          // ライン通過時間の計測状態（元: kyori_flug）
volatile int line_pass_ms;             // ラインの通過時間[ms]（元: kyoritime）
volatile int f_setup_end;              // setup が終わったら 1

FspTimer fsp_timer;                    // 1ms割り込み用タイマ

// microSD・ログ関係
ArduinoSPI SPI(MISO1, MOSI1, SCK1, FORCE_SPI1_MODE); // RMC-RA4M1のmicroSD用SPI(各端子はpins_arduino.hで定義)
//ArduinoSPI SPI(MISO, MOSI, SCK, FORCE_SPI_MODE);  // Arduino UNO R4のSPIを使う場合(MISO=12,MOSI=11,SCK=13)
File microSD;                          // microSDのファイルアクセス用
const char* C_DATE = __DATE__;         // ファイルの日付用（コンパイルした日付）
const char* C_TIME = __TIME__;         //               （コンパイルした時刻）

volatile int err_sd;                   // 1: microSDが使えない
volatile int log_st;                   // ログの状態 LOG_NON / LOG_INIT / LOG_WRITE / LOG_CLOSE
volatile int log_write;                // リングバッファの書き込み位置（割り込み側）
volatile int log_read;                 // リングバッファの読み出し位置（loop側）
typedef struct {
    unsigned int time;   // スタートからの時間[ms]
    int ptn;             // pattern
    unsigned char sen;   // センサ
    int8_t ang;          // ハンドル角度
    int8_t ml;           // 左モータ（DIPスイッチで縮小した後の値）
    int8_t mr;           // 右モータ
} log_t;
volatile log_t log_buff[LOG_SLOTS];
volatile log_t log_raw;                // 最後に出力したハンドル・モータの値

//***********************************************************************
// setup：電源投入時に1回だけ実行
//***********************************************************************
void setup() {
    // PWM出力の設定
    setGPTterminal(4, 6);                        // P406(GTIOC1B):サーボPWM
    setGPTterminal(4, 1);                        // P401(GTIOC6B):左モータPWM
    setGPTterminal(4, 3);                        // P403(GTIOC3A):右モータPWM
    GTIOC1B = SERVO_CENTER;                      // サーボPWMのON幅
    startPWM_GPT1(GTIOCB, DIV16, PWM_CYCLE);     // サーボPWMの周期(GPT1 GTIOCB)
    GTIOC6B = 0;                                 // 左モータPWMのON幅
    startPWM_GPT6(GTIOCB, DIV16, PWM_CYCLE_M);   // 左モータPWMの周期(GPT6 GTIOCB)
    GTIOC3A = 0;                                 // 右モータPWMのON幅
    startPWM_GPT3(GTIOCA, DIV16, PWM_CYCLE_M);   // 右モータPWMの周期(GPT3 GTIOCA)

    // RMC-RA4M1基板上のDIPスイッチ、LED
    pinMode(13, OUTPUT);
    pinMode(25, INPUT);
    pinMode(26, INPUT);
    led_out_m(0);

    // CN5 モータドライブ基板（29,30,32 はPWM出力なので設定しない）
    pinMode(27, OUTPUT);    // LED2
    pinMode(28, OUTPUT);    // LED3
    pinMode(31, OUTPUT);    // 右モータ方向
    pinMode(33, OUTPUT);    // 左モータ方向
    pinMode(34, INPUT);     // プッシュスイッチ

    // CN7 センサ基板
    for (int pin = 35; pin <= 42; pin++) {
        pinMode(pin, INPUT);
    }

    // スタート前の状態
    ha(0);
    sp(0, 0);
    cnt1 = 0;
    course_out_ms = 0;
    run_ms = 0;
    line_pass_state = PASS_WAIT;
    line_pass_ms = 0;

    // AGT 1msごとの割り込み  PCLKB=24MHz, 1分周 → 24000カウントで1ms（設定値は1小さい23999）
    fsp_timer.begin(TIMER_MODE_PERIODIC, AGT_TIMER, 1, 23999, 1, (timer_source_div_t)TIMER_SOURCE_DIV_1, AGTCallback);
    IRQManager::getInstance().addPeripheral(IRQ_AGT, (void*)fsp_timer.get_cfg());
    fsp_timer.open();

    Serial.begin(9600);
    log_setup();            // microSD の確認（sdlog.ino）

    f_setup_end = 1;
}

//***********************************************************************
// loop：走行とは直接関係ない、優先順位の低い処理（シリアル出力、microSDへの書き込み）
//***********************************************************************
void loop() {
    static int loopcnt = 0;

    // スイッチ待ちの間、0.5秒ごとにセンサの状態をシリアル出力
    if (pattern == PT_WAIT_SWITCH && cnt_print >= 500) {
        cnt_print = 0;
        loopcnt++;
        unsigned char b = sensor_inp(MASK4_4);
        Serial.print(loopcnt);
        Serial.print(" : sensor = ");
        if ((b & 0xf0) == 0x00) Serial.print("0");
        Serial.print(b, HEX);
        Serial.print("  startbar = ");
        Serial.println(startbar_get());
    }

    log_task();             // microSD への書き込み（sdlog.ino）
}

//***********************************************************************
// 1msごとの割り込み：走行の判断はすべてここで行う
//***********************************************************************
void AGTCallback(timer_callback_args_t __attribute((unused))* p_args) {
    cnt0++;
    cnt1++;
    course_out_ms++;
    run_ms++;
    cnt_start++;
    cnt_print++;

    if (f_setup_end == 0) {              // セットアップ実行中は何もしない
        return;
    }

    log_sample();                        // 10msごとにログをRAMに貯める（sdlog.ino）

    // スタートから一定時間後は、プッシュスイッチで走行とログ記録を終了する
    if (pattern >= PT_RUN_FIRST && pattern <= PT_PUSH_LAST
        && cnt_start >= PUSH_STOP_GUARD_MS && pushsw_get() == 1) {
        pattern = PT_END;
    }

    // 走行中の安全装置
    if (pattern >= PT_RUN_FIRST && pattern <= PT_RUN_LAST) {
        // 予定の走行時間がたったら停止
        if (run_ms >= AUTO_STOP_MS) {
            cnt1 = 0;
            pattern = PT_STOP;
        }
        // コースアウト（全センサが白）が続いたら停止
        if (sensor_inp(MASK4_4) == COURSE_OUT_SENSOR) {
            if (course_out_ms >= COURSE_OUT_MS) {
                pattern = PT_STOP_HOLD;
            }
        } else {
            course_out_ms = 0;
        }
    }

    run_pattern();
}

//***********************************************************************
// pattern の振り分け
//   段階のある pattern は「先頭の番号 + 段階」で表す（例: 23 = 通常走行の段階3）
//***********************************************************************
void run_pattern(void) {
    const int p = pattern;

    if (p == PT_WAIT_SWITCH)                                     { run_wait_switch(); return; }
    if (p == PT_WAIT_START)                                      { run_wait_start(); return; }
    if (p >= PT_TRACE   && p < PT_TRACE   + TRACE_STAGES)        { run_trace(p - PT_TRACE); return; }
    if (p >= PT_CURVE_R && p < PT_CURVE_R + TRACE_STAGES)        { run_curve(p - PT_CURVE_R, true); return; }
    if (p >= PT_CURVE_L && p < PT_CURVE_L + TRACE_STAGES)        { run_curve(p - PT_CURVE_L, false); return; }

    switch (p) {
    case PT_CROSS_FOUND:     run_cross_found();         return;
    case PT_CROSS_PASS:      run_cross_pass();          return;
    case PT_CROSS_BRAKE:     run_cross_brake();         return;
    case PT_CRANK_SEARCH:    run_crank_search();        return;
    case PT_CRANK_L_ENTER:   run_crank_enter(false);    return;
    case PT_CRANK_L_SLOW:    run_crank_slow(false);     return;
    case PT_CRANK_L_RETURN:  run_crank_return(false);   return;
    case PT_CRANK_L_OUTSIDE: run_crank_outside(false);  return;
    case PT_CRANK_R_ENTER:   run_crank_enter(true);     return;
    case PT_CRANK_R_SLOW:    run_crank_slow(true);      return;
    case PT_CRANK_R_RETURN:  run_crank_return(true);    return;
    case PT_CRANK_R_OUTSIDE: run_crank_outside(true);   return;
    case PT_LANE_R_FOUND:    run_lane_found(true);      return;
    case PT_LANE_R_PASS:     run_lane_pass(true);       return;
    case PT_LANE_R_SELECT:   run_lane_select(true);     return;
    case PT_LANE_L_FOUND:    run_lane_found(false);     return;
    case PT_LANE_L_PASS:     run_lane_pass(false);      return;
    case PT_LANE_L_SELECT:   run_lane_select(false);    return;
    case PT_STOP:            run_stop();                return;
    case PT_STOP_HOLD:       run_stop_hold();           return;
    case PT_END:             run_end();                 return;
    default:                 break;
    }

    // レーンチェンジ（段階 0〜10）
    for (int side = 0; side < 2; side++) {
        const bool right = (side == 0);
        const int base = right ? PT_LANE_R_APPROACH : PT_LANE_L_APPROACH;   // 300 または 600
        const int stage = (p - base) % 20;
        if (p < base || p >= base + 160 || stage >= LANE_STAGES) continue;
        switch ((p - base) / 20) {
        case 0: run_lane_approach(stage, right); return;   // 300〜310 / 600〜610
        case 1: run_lane_steer(stage, right);    return;   // 320〜330 / 620〜630
        case 2: run_lane_straight(stage, right); return;   // 340〜350 / 640〜650
        case 5: run_lane_align(stage, right);    return;   // 400〜410 / 700〜710
        case 6: run_lane_finish(stage, right);   return;   // 420〜430 / 720〜730
        case 7: run_lane_recover(stage, right);  return;   // 440〜450 / 740〜750
        default: break;
        }
    }
}
