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
//   planner.ino    … クランク・レーンチェンジの経路の計算、ピュアパシュート（新しい走行アルゴリズム）
//   speed_model.ino… 速度の推定と、計算による減速・速さの決め方（新しい走行アルゴリズム）
//   sdlog.ino      … microSD へのログ保存
//**************************************************************************

//========================================
// インクルード
//========================================
#include "mcr_gpt_lib.h"     // GPTを使用するためのライブラリ
#include <SPI.h>             // microSDとSPI通信するためのライブラリ
#include <SD.h>              // microSD読み書きライブラリ
#include "config.h"
#include "dbg_config.h"       // デバッグモードの設定（debug/ だけ）
void dbg_sample(void);           // dbglog.ino
void dbg_task(void);             // dbglog.ino
#include "tables.h"

// 新しい走行アルゴリズムのスイッチの組み合わせの確認（config.h）
//   元の車線変更は、元の減速のあとの速さに合わせた時間でハンドルを切る。減速だけ新しくすると時間が合わず、
//   コースの模擬では、どの速さでもコースアウトした
static_assert(!(USE_NEW_LANE_BRAKE && !USE_NEW_LANE_STEER),
              "USE_NEW_LANE_BRAKE は USE_NEW_LANE_STEER と一緒に 1 にしてください");
//   クランクに入る速さの自動計算（0）は、円弧で曲がる場合の値。元の曲がり方で使うときは、うまく曲がれる速さを入れる
static_assert(!(USE_NEW_CRANK_BRAKE && !USE_NEW_CRANK_TURN) || (CRANK_ENTRY_MPS > 0.0f),
              "USE_NEW_CRANK_TURN が 0 のときは、CRANK_ENTRY_MPS に元の曲がり方でうまく曲がれる速さ（例 1.4）を入れてください");
//   線の形の推定は USE_NEW_STEER のハンドル角度を、先読みは USE_NEW_TRACE_SPEED の速さを変えるもの。単独では働かない
static_assert(!(USE_NEW_LINE_SHAPE && !USE_NEW_STEER), "USE_NEW_LINE_SHAPE は USE_NEW_STEER と一緒に 1 にしてください");
static_assert(!(USE_NEW_CURVE_PREVIEW && !USE_NEW_TRACE_SPEED), "USE_NEW_CURVE_PREVIEW は USE_NEW_TRACE_SPEED と一緒に 1 にしてください");
static_assert(!USE_NEW_CRANK_QUICK || (USE_NEW_CRANK_TURN && USE_NEW_CRANK_BRAKE), "USE_NEW_CRANK_QUICK は USE_NEW_CRANK_TURN・USE_NEW_CRANK_BRAKE と一緒に 1 にしてください");

static_assert(!USE_NEW_LANE_DISTANCE || USE_NEW_LANE_STEER, "USE_NEW_LANE_DISTANCE には USE_NEW_LANE_STEER が必要です");
static_assert(!USE_NEW_LANE_DOWN_ONLY || USE_NEW_LANE_DISTANCE, "USE_NEW_LANE_DOWN_ONLY には USE_NEW_LANE_DISTANCE が必要です");

#define timer(x) delay(x)    // timer関数=delay関数

//========================================
// センサのマスク値  ×：マスクあり(無効)　○：マスク無し(有効)
//========================================
#define MASK3_3       0xe7   // ○○○××○○○
#define MASK0_3       0x07   // ×××××○○○
#define MASK3_0       0xe0   // ○○○×××××
#define MASK4_4       0xff   // ○○○○○○○○

#define PI_F          3.14159265f
#define DEG_TO_RAD_F  0.01745329252f   // 度 → ラジアン
// 新しい走行アルゴリズムの計算に使うグリップ（測った値 × 余裕の割合。config.h）
#define GRIP_LAT_USE  (GRIP_LAT_MPS2 * PLAN_GRIP_RATIO_LAT)
#define GRIP_LONG_USE (GRIP_LONG_MPS2 * PLAN_GRIP_RATIO_LONG)

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
    PT_CRANK_L_ARC   = 112,  // 左クランク 計算した円弧で曲がる（USE_NEW_CRANK_TURN）
    PT_CRANK_L_RETURN  = 120,  // 左クランク 中央線へ復帰
    PT_CRANK_L_OUTSIDE = 121,  // 左クランク 外側にはみ出した
    PT_CRANK_R_ENTER = 150,  // 右クランク 突入
    PT_CRANK_R_SLOW  = 151,  // 右クランク 低速旋回
    PT_CRANK_R_ARC   = 152,  // 右クランク 計算した円弧で曲がる（USE_NEW_CRANK_TURN）
    PT_CRANK_R_RETURN  = 160,  // 右クランク 中央線へ復帰
    PT_CRANK_R_OUTSIDE = 161,  // 右クランク 外側にはみ出した
    PT_LANE_R_FOUND    = 200,  // 右ハーフライン検出
    PT_LANE_R_PASS     = 201,  // 右ハーフラインの通過時間を測る
    PT_LANE_R_SELECT   = 202,  // 通過時間から段階を決める
    PT_LANE_R_APPROACH = 300,  // 300〜310 白線が切れるまで直進
    PT_LANE_R_STEER    = 320,  // 320〜330 ハンドルを切る
    PT_LANE_R_STRAIGHT = 340,  // 340〜350 切り終わって直進
    PT_LANE_R_SCURVE   = 360,  // 360〜370 計算した2つの円弧で隣の車線へ（USE_NEW_LANE_STEER）
    PT_LANE_R_SPOST    = 380,  // 380〜390 円弧を走り終えて、新しい線を探す（USE_NEW_LANE_STEER）
    PT_LANE_R_ALIGN    = 400,  // 400〜410 新しい線に合わせる
    PT_LANE_R_FINISH   = 420,  // 420〜430 終了処理
    PT_LANE_R_RECOVER  = 440,  // 440〜450 外へふくらんだ時
    PT_LANE_L_FOUND    = 500,  // 以下、左レーンチェンジ（右と同じ並び）
    PT_LANE_L_PASS     = 501,
    PT_LANE_L_SELECT   = 502,
    PT_LANE_L_APPROACH = 600,
    PT_LANE_L_STEER    = 620,
    PT_LANE_L_STRAIGHT = 640,
    PT_LANE_L_SCURVE   = 660,
    PT_LANE_L_SPOST    = 680,
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
void ha_deg(float angle);
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
void run_crank_arc(bool right);
bool crank_start_arc(bool right);
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
void run_lane_scurve(int stage, bool right);
void run_lane_spost(int stage, bool right);
bool lane_start_scurve(int stage, bool right);
void lane_speed_control(void);
int lane_pt(bool right, int pt_right, int pt_left);
bool lane_new_line_found(unsigned char c, bool right);
int lane_cross_recheck(bool right);
void lane_steer_output(int stage, bool right);
// basic.ino
void run_pattern(void);
// planner.ino
float radius_of_angle(float angle);
float angle_for_radius(float r);
float servo_delay_s(float angle);
float crank_r_far(float d);
float crank_r_in(float d);
float crank_room(float d);
int crank_plan_delay(float v, float delay, float* r_out, float* wait_out);
int crank_plan(float v, float* r_out, float* wait_out);
void crank_quick_setup(void);
float crank_unwind_yaw(float v, float angle);
float scurve_x(float r, float len, float f, float d);
float scurve_sin(float r, float len, float f);
float lane_r_for(float len, float d);
int scurve_corner_x(float r, float len, float d, float f_line, float fwd, float lat, float* x_out);
int lane_old_ok(float r, float len, float d, float x_start, float s0);
int lane_new_ok(float r, float len, float d, float x_start, float f_end);
int lane_plan_delay(float v, float delay, float x_start, float* r_out, float* len_out, float* wait_out);
int lane_plan(float v, float x_start, float* r_out, float* len_out, float* wait_out);
float crank_turn_speed_auto(void);
float lane_turn_speed_auto(void);
void plan_setup(void);
float line_position_m(unsigned char c);
float pursuit_lookahead_m(void);
int pursuit_angle(unsigned char c);
void line_obs_update(unsigned char c);
float line_obs_angle(void);
void lane_table_setup(void);
int lane_table_plan(float v, float x_start, float* r_out, float* len_out, float* wait_out, float* x_used);
int lane_new_ok_robust(float len, float d, float x_start, float f_end, float u);
float pre_turn_m(float v, float r, float wait);
// speed_model.ino
float curvature_of_angle(float angle);
float accel_up_limit(float v, float k_abs);
void speed_model_update(void);
void speed_measure_by_line(float width_m, int pass_ms);
void vmax_observe(float v_meas, float r_rel);
int vmax_plausible(float v_meas, float t);
void cross_line_vmax_observe(int pass_ms);
void reset_distance(void);
void correct_lane_speed_by_distance(void);
float braking_distance(float v, float v_target, int brake_power);
float deadbeat_speed(float v, float v_target, float k_abs);
int power_of_speed(float u);
int brake_power_for(float remain_m, float v_target, int brake_power, float margin_m);
void crank_speed_control_new(void);
void lane_speed_control_new(void);
void course_list_setup(void);
int course_list_check(void);
void crank_spot_select(void);
void crank_spot_done(bool right);
void lane_spot_select(bool right);
void lane_spot_done(void);
void drive_at(float v_target, float angle);
float corner_speed(float k_abs);
void trace_motor_new(int angle);
void traction_limit(int* power_l, int* power_r);
void servo_model_update(void);
float servo_last_cmd(void);
float long_avail_use(float v, float k_abs);
float speed_after_brake(float v, float d);
void drive_wheels(float u, float k);
void drive_center(float u, float k);
void trace_motor_preview(float k_ahead);
bool finite_f(float x);
int model_check_config(void);
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
volatile float speed_mps;              // 推定した車の速さ [m/s]（speed_model.ino）
volatile float v_max_mps;              // 出力100%のときの速さ [m/s]。初めは V_MAX_MPS、USE_NEW_VMAX_LEARN で走りながら直す
float vmax_var;                        // v_max_mps の誤差の大きさの2乗 [(m/s)²]（USE_NEW_VMAX_LEARN）
float vmax_sens;                       // 推定の速さが v_max_mps にどれだけ左右されているか（感度 ∂v/∂v_max。USE_NEW_VMAX_LEARN）
float speed_err_sd;                    // 推定の速さの、v_max_mps と関係のない誤差の大きさ [m/s]（USE_NEW_VMAX_LEARN）
volatile float dist_m;                 // ラインの終わりからの走行距離 [m]（speed_model.ino）
volatile int brake_started;            // 1: 計算による減速でブレーキを始めた（speed_model.ino）
float crank_entry_mps;                 // クランクに入る速さ [m/s]（planner.ino の plan_setup で決める）
float lane_entry_mps;                  // 車線変更を始める速さ [m/s]（同上）
volatile float plan_wait_m;            // 計算した経路：曲がり始めるまでに直進する距離 [m]
volatile float plan_len_m;             // 計算した経路：円弧1つの長さ [m]
volatile float plan_switch_m;          // 計算した経路：レーンチェンジで逆向きにハンドルを切る所 [m]
volatile float plan_speed_mps;         // 計算した経路：走る速さ [m/s]
volatile float plan_angle;             // 計算した経路：ハンドル角度 [度]（向きは呼ぶ側で付ける）
volatile float trace_k_filt;           // 通常走行の速さを決める曲率をならした値 [1/m]（speed_model.ino）
volatile float lane_offset_m;          // 中央線が切れる直前の、線に対する車の横の位置 [m]（右がプラス）
volatile int model_ok;                 // 1: 速度の推定を使ってよい。config.h の値がおかしいときや、ラインで測った速さと推定が大きく違い続けたときは 0 にして、元の表の動きに戻す
int line_w_exact;                      // 1: これから通るライン（crank_line_w_m / lane_line_w_m）の幅が1つに決まっている（スタート位置が決まらず、一番広い幅で代えているときは 0）
volatile unsigned long line_ignore_until;  // cnt0 がこの値になるまで、通常走行でクロスライン・ハーフラインを探さない（クランクを曲がり終えた直後）
volatile int model_bad_cnt;            // ラインで測った速さと推定が大きく違った回数（続けて）
volatile float steer_cmd_deg;          // 最後に出したハンドル角度の指令 [度]（小数。ha / ha_deg で記録）
volatile float servo_est_deg;          // サーボの本当の角度の推定 [度]（speed_model.ino の servo_model_update）
volatile float car_k_eff;              // 車が本当に曲がっている曲率の推定 [1/m]（摩擦円を超えた分は小さくなる。speed_model.ino）
volatile float obs_z1;                 // センサの所での線の横の位置の推定 [m]（右がプラス。planner.ino の line_obs_update）
volatile float obs_z2;                 // センサの所での線の向きの推定 [rad]（車の向きに対して。右へ向かうとプラス）
volatile float obs_z3;                 // センサの所での線の曲がり具合（曲率）の推定 [1/m]（右に曲がるとプラス）
volatile unsigned long line_obs_ms;    // 最後に推定した時の cnt0（1ms あいたら推定をやり直す）
float pursuit_ld;                      // ピュアパシュートで線を見る距離 [m]（plan_setup で計算しておく）
float obs_k_lim = 10.0f;               // 線の曲がり具合の推定の上限 [1/m]（plan_setup で計算しておく）
float trace_v_cap;                     // 直線の速さの上限 [m/s]（TRACE_SAFE_RADIUS_M。0 なら上限なし）
float crank_q_r;                       // USE_NEW_CRANK_QUICK：曲がる半径 [m]（plan_setup で決める。0 なら曲がれる半径がない）
float crank_q_d;                       // USE_NEW_CRANK_QUICK：曲がり始める所の、後輪の軸から新しい道の手前の端までの距離 [m]
float crank_q_dlo;                     // USE_NEW_CRANK_QUICK：その距離の下限（これより奥まで進んでから曲がると外へ出る）
volatile float crank_yaw;              // USE_NEW_CRANK_QUICK：曲がり始めてから車の向きが変わった角度の推定 [rad]
volatile int crank_measuring;           // USE_NEW_CRANK_QUICK：1 なら曲がり角の線の通過時間を測っている
volatile int crank_yaw_track;          // USE_NEW_CRANK_QUICK：1 なら円弧で曲がり始めたので、160 / 161 でも crank_yaw を積み上げる
volatile int crank_unwind;             // USE_NEW_CRANK_QUICK：1 ならハンドルを戻し始めた
volatile float crank_unwind_m;         // USE_NEW_CRANK_QUICK：ハンドルを戻し始めた所の dist_m
float crank_app_mps;                  // USE_NEW_CRANK_ANY_COURSE：曲がり角の線を見てからブレーキをかけても間に合う一番速い速さ [m/s]（0 は計算できなかった）
float crank_extra_m;                   // クランクの線を見つけてから円弧に入るまでに進む距離 [m]（USE_NEW_BRAKE_RANGE）
float lane_extra_m;                    // 中央線の切れ目を見つけてからS字に入るまでに進む距離 [m]（同上）
// これから来るクランク・レーンチェンジの寸法（speed_model.ino の crank_spot_select / lane_spot_select で決める）
int spot_no;                           // スタートしてから始めたクランク・レーンチェンジの数（COURSE_LIST の行数で割った余り）
unsigned int spot_cand;                // まだあり得るスタート位置（COURSE_START_ROWS の i 番目が bit i）。0 は「どれとも合わない」
unsigned int spot_pend;                // 今のハーフラインの向きと合うスタート位置（レーンチェンジと決まったら spot_cand にする）
float crank_to_m;                      // 今のクランク：クロスラインの終わりから曲がり角まで [m]
float crank_line_w_m;                  // 今のクランク：クロスラインの幅 [m]
float lane_to_m;                       // 今のレーンチェンジ：ハーフラインの終わりから中央線が切れる所まで [m]
float lane_line_w_m;                   // 今のレーンチェンジ：ハーフラインの幅 [m]
int lane_to_exact;                     // 1: lane_to_m はそのレーンチェンジの値（0 は、あり得る値の一番短いもので代えている。距離で速さを直すのに使わない）

FspTimer fsp_timer;                    // 1ms割り込み用タイマ

// microSD・ログ関係
ArduinoSPI SPI(MISO1, MOSI1, SCK1, FORCE_SPI1_MODE); // RMC-RA4M1のmicroSD用SPI(各端子はpins_arduino.hで定義)
//ArduinoSPI SPI(MISO, MOSI, SCK, FORCE_SPI_MODE);  // Arduino UNO R4のSPIを使う場合(MISO=12,MOSI=11,SCK=13)
File microSD;                          // microSDのファイルアクセス用
const char* C_DATE = __DATE__;         // ファイルの日付用（コンパイルした日付）
const char* C_TIME = __TIME__;         //               （コンパイルした時刻）

volatile int err_sd;                   // 1: microSDが使えない
int run_log_no;
bool run_log_closed;
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
    int16_t spd;         // 推定した速さ [mm/s]（LOG_SPEED_MODEL が 1 のときだけ書き出す）
    int16_t dist;        // ラインの終わりからの距離 [mm]（同上）
    int16_t vmax;        // 出力100%のときの速さ v_max_mps [mm/s]（同上。USE_NEW_VMAX_LEARN で変わる）
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
    v_max_mps = V_MAX_MPS;  // 出力100%のときの速さ（speed_model.ino）
    vmax_var = (VMAX_LEARN_SD * V_MAX_MPS) * (VMAX_LEARN_SD * V_MAX_MPS);
    vmax_sens = 0.0f;
    speed_err_sd = 0.0f;

    // AGT 1msごとの割り込み  PCLKB=24MHz, 1分周 → 24000カウントで1ms（設定値は1小さい23999）
    fsp_timer.begin(TIMER_MODE_PERIODIC, AGT_TIMER, 1, 23999, 1, (timer_source_div_t)TIMER_SOURCE_DIV_1, AGTCallback);
    IRQManager::getInstance().addPeripheral(IRQ_AGT, (void*)fsp_timer.get_cfg());
    fsp_timer.open();

    plan_setup();           // 新しい走行アルゴリズムで使う速さを計算しておく（planner.ino）

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
    dbg_task();             // デバッグモード：走り終わった後に dbgNNNN.csv へ書き出す（dbglog.ino）
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

    speed_model_update();                // 出したモータ出力から速さと距離、サーボの角度を計算（speed_model.ino）
    dbg_sample();                        // デバッグモード：詳しいログを RAM に貯める（dbglog.ino）
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
    case PT_CRANK_L_ARC:     run_crank_arc(false);      return;
    case PT_CRANK_R_ENTER:   run_crank_enter(true);     return;
    case PT_CRANK_R_SLOW:    run_crank_slow(true);      return;
    case PT_CRANK_R_RETURN:  run_crank_return(true);    return;
    case PT_CRANK_R_OUTSIDE: run_crank_outside(true);   return;
    case PT_CRANK_R_ARC:     run_crank_arc(true);       return;
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
        case 3: run_lane_scurve(stage, right);   return;   // 360〜370 / 660〜670
        case 4: run_lane_spost(stage, right);    return;   // 380〜390 / 680〜690
        case 5: run_lane_align(stage, right);    return;   // 400〜410 / 700〜710
        case 6: run_lane_finish(stage, right);   return;   // 420〜430 / 720〜730
        case 7: run_lane_recover(stage, right);  return;   // 440〜450 / 740〜750
        default: break;
        }
    }
}
