//**************************************************************************
// 対象ﾏｲｺﾝ     Renesas Electronics RMC_RA4M1(Rev.2.0)(R7FA4M1AB3CFP9 100pin)
// ﾌｧｲﾙ内容     AQM1248液晶不可.SDｶｰﾄﾞ記録のみのBasicマイコンカー走行プログラム
// Copyright    ジャパンマイコンカーラリー実行委員会改編版_ＴＰＦ監修 2024.09.01
// ﾗｲｾﾝｽ        This software is released under the MIT License.
//              http://opensource.org/licenses/mit-license.php
//2024.10.15
//R.K
//**************************************************************************
//========================================
// インクルード
//========================================
#include "mcr_gpt_lib.h"                // GPTを使用するためのライブラリ
#include <SPI.h>                        // microSDとSPI通信するためのライブラリ
#include <SD.h>                         // microSD読み書きライブラリ

//========================================
// シンボル定義
//========================================
const char *C_DATE = __DATE__;          // microSDのファイル日付用 コンパイルした日付
const char *C_TIME = __TIME__;          //                         コンパイルした時間
#define timer(x) delay(x)               // timer関数=delay関数

#define KANSOU_TIME 50000

// クロスライン通過時間15ms以下で、減速後に再加速する際の最終出力（DIP補正前）。
const int CRANK_FAST_APPROACH_POWER = 83;
// クロスライン通過が11ms以下の場合、通過後18～58msに使う初段の逆転出力。
const int CRANK_FAST_PEAK_BRAKE = -32;

// 右レーンチェンジ。時間は元のレーンの白線が途切れた時点から数える。
const int RIGHT_LANE_TURN_DEG = 19;                 // 渡り始めの右舵
const int RIGHT_LANE_SEARCH_DEG = 10;               // 右レーンの中央線を探す間の右舵
const unsigned int RIGHT_LANE_FULL_TURN_MS = 220;   // 速度区分に応じて0～10msを加算
const unsigned int RIGHT_LANE_SEARCH_LIMIT_MS = 450; // この時刻までに確認できなければ停止
const unsigned int RIGHT_LANE_CENTER_CONFIRM_MS = 2; // 右内側の反応後、中央寄りに連続して白が見えた時間

// 右レーンチェンジ：ハーフライン通過後の減速。cnt1 はハーフラインを通り過ぎた時点から数える。
// 減速表 shasenhenkou() は幅20mmのハーフライン用。40mmのコースでは通過時間が約2倍になり「遅い」と判断されるので換算する。
const int HALF_LINE_WIDTH_MM = 40;                  // このコースのハーフラインの幅[mm]（20にすると換算しない）
const unsigned int RIGHT_LANE_FULL_MS = 0;          // 通過後も100%を続ける時間
const int RIGHT_LANE_BRAKE_POWER = -30;             // ブレーキの出力
const unsigned int RIGHT_LANE_BRAKE_MS = 80;        // ブレーキを続ける時間
const int RIGHT_LANE_HOLD_POWER = 65;               // ブレーキの後、中心線が切れるまで保つ出力（0にすると元の減速表）

// 復帰中に両端の白が続いた場合にだけ、戻し舵と通常走行への移行を抑える。
const unsigned int RIGHT_LANE_SPLIT_MS = 16;
const int RIGHT_LANE_SPLIT_STEER_DEG = -12;
const unsigned int RIGHT_LANE_SETTLE_MS = 20;

// モータ、サーボのPWM関係
#define PWM_CYCLE     32999             //  11ms/(1/HOCO*DIV16) = 0.011 /(1/48e6*16) = 33000
#define PWM_CYCLE_M   14999             //   5ms/(1/HOCO*DIV16) = 0.005 /(1/48e6*16) = 15000
#define SERVO_CENTER  4500              // 1.5ms/(1/HOCO*DIV16) = 0.0015/(1/48e6*16) =  4500
#define HANDLE_STEP   23                // １度あたりの値

// マスク値設定 ×：マスクあり(無効)　○：マスク無し(有効)
#define MASK2_2       0x66              // ×○○××○○×
#define MASK2_0       0x60              // ×○○×××××
#define MASK0_2       0x06              // ×××××○○×
#define MASK3_3       0xe7              // ○○○××○○○
#define MASK0_3       0x07              // ×××××○○○
#define MASK3_0       0xe0              // ○○○×××××
#define MASK4_0       0xf0              // ○○○○××××
#define MASK0_4       0x0f              // ××××○○○○
#define MASK4_4       0xff              // ○○○○○○○○

// ログ処理関係
#define LOG_MAX       (100)             // 10ms周期と変化点を共有するリング。終了行用に1枠を予約
#define LOG_NON       (0)
#define LOG_INIT      (1)
#define LOG_WRITE     (2)
#define LOG_CLOSE     (3)

//========================================
// プロトタイプ宣言
//========================================
void AGTCallback( timer_callback_args_t __attribute((unused)) * p_args );
unsigned char sensor_inp( unsigned char mask );
int check_crossline( void );
int check_shasenhenkoum( void );
int check_migikakunin( void );
int check_shasenhenkouh( void );
int check_hidarikakunin( void );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
unsigned char startbar_get( void );
void led_out( unsigned char led );
void led_out_m( unsigned char led );
void sp( int accele_l, int accele_r );
void ha( int angle );
void set_microSD_dateTime( uint16_t* date, uint16_t* time );
void newcrank( void );                  // クランク時の減速処理宣言
void shasenhenkou( void );              // 車線変更時の減速処理宣言
bool right_shift_left_steer( void );    // 右寄せ走行中、車体が中央線より左にいる時の処理
unsigned long convertBCD_CharToLong( unsigned char hex );
int getCompileYear( const char *p );
int getCompileMonth( const char *p );
int getCompileDay( const char *p );
int getCompileHour( const char *p );
int getCompilerMinute( const char *p );
int getCompilerSecond( const char *p );

//========================================
// グローバル変数の宣言
//========================================
// 共通
volatile int kyori_flug;                // 各ラインの通過検出宣言
volatile int kyoritime;                 // 各ラインの通過時間宣言
volatile int f_setup_end;
volatile int pattern;
// 右寄せ走行（パターン300～310）での、元のレーンの中央線に対する車体の位置
enum RightShiftSide { SHIFT_ON_OR_RIGHT, SHIFT_LEFT, SHIFT_FAR_LEFT };
volatile RightShiftSide right_shift_side;
volatile unsigned char right_shift_last;   // 最後に白が見えたセンサの形
volatile unsigned long cnt0;
volatile unsigned long cnt1;
volatile unsigned long daturin_cnt;     // 脱輪確認タイマ用
volatile unsigned long kansou_cnt;      // 完走確認タイマ用
volatile unsigned long cnt_start;
volatile unsigned long cnt_print;

// 割り込み関係
FspTimer fsp_timer;

// microSD、ログ関係
ArduinoSPI SPI(MISO1, MOSI1, SCK1, FORCE_SPI1_MODE); // RMC-RA4M1のmicroSD用SPIを選択(SPIの各端子はpins_arduino.hで定義)
//ArduinoSPI SPI(MISO, MOSI, SCK, FORCE_SPI_MODE); // Arduino UNO R4のSPIを使用(MISO=12,MOSI=11,SCK=13)
File microSD;           // microSDのファイルアクセス用

volatile int err_sd;
volatile int log_st;
volatile int log_write;
volatile int log_read;
struct MotorCommand {
    int8_t ang, ml, mr; // 舵・左右PWM指令。負の値も記録する
};
struct LogRow {
    unsigned long time, timer_ms, white_ms, dropped;
    int marker_ms;
    uint16_t before, after, center_ms;
    int16_t decision_sensor; // 右車線変更の判定に使った実値。未使用は-1
    uint8_t sensor, marker_phase, inner_seen, dip, stop_reason;
    MotorCommand prev, next;
};
enum LogStopReason { LOG_STOP_RUNNING, LOG_STOP_TIME_LIMIT, LOG_STOP_FULL_WHITE, LOG_STOP_RIGHT_LANE_TIMEOUT, LOG_STOP_PUSH_BUTTON };
volatile uint8_t log_last_dip;
volatile MotorCommand log_raw;
volatile LogRow log_buff[LOG_MAX];
unsigned long log_dropped; // 割り込み側だけが更新

int nextLogIndex(int index) {
    return index + 1 == LOG_MAX ? 0 : index + 1;
}

void writeLogRow() {
    // log_read を進めるまでは割り込み側がこの行を上書きしない。
    const volatile LogRow &row = log_buff[log_read];
    static const char *const stop_names[] = {"", "time_limit", "full_white", "right_lane_timeout", "button"};
    char line[192];
    snprintf(line, sizeof(line),
             "%lu,%u,'%08lu,%d,%d,%d,%u,%lu,%d,%u,%lu,%d,%u,%u,%d,%d,%d,%lu,%u,%s",
             row.time, (unsigned)row.before, convertBCD_CharToLong(row.sensor),
             row.prev.ang, row.prev.ml, row.prev.mr, (unsigned)row.after,
             row.timer_ms, row.marker_ms, (unsigned)row.marker_phase, row.white_ms,
             row.decision_sensor, (unsigned)row.inner_seen, (unsigned)row.center_ms,
             row.next.ang, row.next.ml, row.next.mr, row.dropped, (unsigned)row.dip, stop_names[row.stop_reason]);
    microSD.println(line);
    log_read = nextLogIndex(log_read);
}

//***********************************************************************
// setup
//***********************************************************************
void setup()
{
  setGPTterminal( 4, 6 );                        // P406(GTIOC1B):サーボPWM
  setGPTterminal( 4, 1 );                        // P401(GTIOC6B):左モータPWM
  setGPTterminal( 4, 3 );                        // P403(GTIOC3A):右モータPWM
  GTIOC1B = SERVO_CENTER;                        // サーボPWMのON幅の設定
  startPWM_GPT1( GTIOCB , DIV16, PWM_CYCLE );    // サーボPWMの周期設定(GPT1 GTIOCBを使用)
  GTIOC6B = 0;                                   // 左モータPWMのON幅の設定
  startPWM_GPT6( GTIOCB , DIV16, PWM_CYCLE_M );  // 左モータPWMの周期設定(GPT6 GTIOCBを使用)
  GTIOC3A = 0;                                   // 右モータPWMのON幅の設定
  startPWM_GPT3( GTIOCA , DIV16, PWM_CYCLE_M );  // 右モータPWMの周期設定(GPT3 GTIOCAを使用)

  // RMC-RA4M1基板上のDIPスイッチ、LED
  pinMode( 13, OUTPUT );
  pinMode( 25, INPUT );
  pinMode( 26, INPUT );
  led_out_m( 0 );

  // CN5 モータドライブ基板
  pinMode( 27 , OUTPUT );               // LED2
  pinMode( 28 , OUTPUT );               // LED3
  //pinMode( 29 , OUTPUT );             // サーボPWMなので設定しない
  //pinMode( 30 , OUTPUT );             // 右モータPWMなので設定しない
  pinMode( 31, OUTPUT );                // 右モータ方向
  //pinMode( 32 , OUTPUT );             // 左モータPWMなので設定しない
  pinMode( 33, OUTPUT );                // 左モータ方向
  pinMode( 34, INPUT );                 // プッシュスイッチ

  // CN7 センサ基板
  pinMode( 35, INPUT );
  pinMode( 36, INPUT );
  pinMode( 37, INPUT );
  pinMode( 38, INPUT );
  pinMode( 39, INPUT );
  pinMode( 40, INPUT );
  pinMode( 41, INPUT );
  pinMode( 42, INPUT );

  // マシン初期状態
  ha(   0 );                            // スタート前のハンドル角度
  sp(    0,    0 );                     // スタート前のモータ出力値
  cnt1          = 0;                    // スタート前の初期値設定
  daturin_cnt   = 0;                    // スタート前の初期値設定
  kansou_cnt    = 0;                    // スタート前の初期値設定
  kyori_flug    = 0;                    // スタート前の初期値設定
	kyoritime     = 0;                    // スタート前の初期値設定

  // AGT 1msごとの割り込み処理の設定 PCLKB=24MHz ∴TIMER_SOURCE_DIV_1(1分周)なら、1/(24e6*1) * 24000 = 1ms  設定は１小さい値である23999を設定する
  fsp_timer.begin( TIMER_MODE_PERIODIC, AGT_TIMER, 1, 23999, 1, (timer_source_div_t)TIMER_SOURCE_DIV_1, AGTCallback );
  IRQManager::getInstance().addPeripheral( IRQ_AGT, (void*)fsp_timer.get_cfg() );
  fsp_timer.open();

  Serial.begin( 9600 );                 // シリアル通信 速度9600bps
#if 0
  // シリアルが接続されるまで３秒待つ、３秒経っても接続されない場合はタイムアウト
  int i = 0;
  while ( !Serial ) {
          timer( 100 );
          i++;
          if( i >= 30 ) {
          break;
          }
  }
#endif
  // microSDの認識
  Serial.print( "microSDを確認中です... " );
         if( !SD.begin( 2.4e6, CS1 ) ) {       // 通信速度Hz(e6=10の6乗),microSDのCS1
             // microSD認識エラー
             err_sd = 1;
             Serial.println( "microSDが認識されません!" );
             for( int i=0; i<20; i++ ) {         // マイコンボードD13のLEDを点滅させる
                  led_out_m( 1 );
                  timer( 50 );
                  led_out_m( 0 );
                  timer( 50 );
             }
  } else {
             // microSD認識OK!
             SdFile::dateTimeCallback( &set_microSD_dateTime );  // コールバック関数の定義（日付と時刻を返す関数を登録）
             Serial.println( "microSDを認識しました。" );
  }

  // setup完了
  f_setup_end = 1;
}

//***********************************************************************
// loop：シリアル出力（デバッグ用）、microSDへのログ保存処理など、走行とは直接関係ない優先順位が低い処理
//***********************************************************************
void loop()
{
  static int log_pattern = 0;
  static int loopcnt = 0;
  char buf[16];
  int i;

#if 1
  // センサの状態をシリアル出力
  if( pattern == 0 && cnt_print >= 500 ) {
      cnt_print = 0;
      loopcnt++;
      unsigned char b = sensor_inp(MASK4_4);
      Serial.print( loopcnt );            // センサの状態の確認 シリアル出力
      Serial.print( " : sensor = " );
      if( (b & 0xf0) == 0x00 ) Serial.print( "0" );
      Serial.print( b, HEX );
      Serial.print( "  startbar = " );
      Serial.println( startbar_get() );
  }
#endif

  // microSDへのログ保存処理
  if( err_sd == 0 ) {
      switch( log_pattern ) {
              case   0:
                       // 待機
                       if( log_st != LOG_NON ) {
                           log_pattern = 991;
                       }
                       break;

              case 991:
                       // 連番の読み込み
                       i = 0;
                       microSD = SD.open( "renban.txt", FILE_READ );
                              if( microSD != 0 ) {
                                  int length = microSD.available();
                                  if( length > 8 ){
                                      length = 8;
                                  }
                                  microSD.read( buf, length );
                                  sscanf( buf, "%d", &i );
                                  if( i < 0 || i >= 9999 ) {
                                      i = 0;
                                  }
                                  microSD.close();
                       } else {
                                  Serial.print( "renban.txtが開けませんので「log0001.csv」でファイルを作成します。" );
                       }

                       // 連番ファイルを一度、削除
                       if( SD.exists( "renban.txt" ) ) {
                           SD.remove( "renban.txt" );
                       }

                       // 連番ファイルを新規に作成
                       microSD = SD.open( "renban.txt", FILE_WRITE );
                       if( microSD != 0 ) {
                           sprintf( buf, "%d", i + 1 );
                           microSD.println( buf );
                           microSD.close();
                       }

                       // 新しい番号でログファイル作成
                       sprintf( buf, "log%04d.csv", i + 1 );
                       microSD = SD.open( buf, FILE_WRITE );
                              if( microSD != 0 ) {
                                  Serial.print( buf );
                                  Serial.println( "でログファイルを作成します。" );
                                  log_pattern = 992;
                       } else {
                                  err_sd = 1;
                                  Serial.println( "ログファイルが作成できません。" );
                       }
                       break;

              case 992:
                       // 最初に書き込む内容
                       microSD.println( "ms,pattern,sensor,handle,motor_l,motor_r,next_pattern,cnt1_ms,"
                                        "kyoritime_ms,kyori_flag,daturin_cnt_ms,decision_sensor,"
                                        "right_inner_seen,right_center_ms,handle_after,motor_l_after,motor_r_after,dropped_total,dip,stop_reason" );
                       log_pattern = 993;
                       break;

              case 993:
                       if( log_st == LOG_WRITE || log_st == LOG_CLOSE ) log_pattern = 994;
                       break;

              case 994:
                       if( log_st == LOG_CLOSE && log_read == log_write ) {
                           log_pattern = 995;
                       } else if( log_read != log_write ) {
                           writeLogRow();
                       }
                       break;

              case 995:
                       microSD.close();  // 必ずcloseしないと、ファイルが残らない
                       log_pattern = 999;
                       break;

              case 999:
                       // 何もしない
                       break;
      }
  }
}

//***********************************************************************
// 1msごとの割り込み（通常走行、1msごとに実行する処理）
//***********************************************************************
void AGTCallback( timer_callback_args_t __attribute((unused)) * p_args )
{
  static bool right_lane_inner_seen;
  static unsigned int right_lane_center_ms;
  enum RightLaneRecovery { LANE_RETURN, LANE_SPLIT_SEEN, LANE_CENTERING };
  static RightLaneRecovery right_lane_recovery = LANE_RETURN;
  static unsigned int right_lane_split_ms;
  static unsigned int right_lane_settle_ms;
  static LogStopReason stop_reason = LOG_STOP_RUNNING;
  static int previous_sensor = -1;
  static int previous_decision_sensor = -1;

  cnt0++;
  cnt1++;
  daturin_cnt++;
  kansou_cnt++;
  cnt_start++;
  cnt_print++;

  if( f_setup_end == 0 ) {              // セットアップ実行中はここで終わり
      return;
  }

  // 先頭6列は従来と同じ制御前スナップショット。SD出力は loop() で行う。
  const bool logging = log_st == LOG_WRITE;
  const int before = pattern;
  const unsigned long timer_before = cnt1;
  const MotorCommand prev = {log_raw.ang, log_raw.ml, log_raw.mr};
  const int sensor_before = logging ? sensor_inp(MASK4_4) : -1;
  int decision_sensor = -1;

  // ★SDｶｰﾄﾞへの保存方法 → 走行後または走行中にﾓｰﾀﾄﾞﾗｲﾊﾞｰ基盤のプッシュスイッチを押したら記録開始
  if( pattern >= 20 && pattern <=1000 && cnt_start >= 1000 && pushsw_get() == 1 ) {
      if( stop_reason == LOG_STOP_RUNNING ) stop_reason = LOG_STOP_PUSH_BUTTON;
      pattern = 1001;
  }

//***********************************************************************
//***********************************************************************
// ここから追加プログラム  ★完走自動停止時間の設定★ 1秒あたり → 1000カウント
//***********************************************************************
//***********************************************************************
  unsigned char c;
  if( pattern >= 20 && pattern <= 750 ) {
      // 完走自動停止プログラム
      if( kansou_cnt >= KANSOU_TIME ) {     // ★完走自動停止時間の設定★
          cnt1 = 0;
          stop_reason = LOG_STOP_TIME_LIMIT;
          pattern = 800;
      }

      // ｺｰｽｱｳﾄ時の緊急停止追加プログラム
      c = sensor_inp( MASK4_4 );
             if( c == 0xff ) {        // ★床の色に注意してセンサーの反応条件と緊急停止時間を設定★
                 if( daturin_cnt >= 300 ) {    // ★ｺｰｽｱｳﾄ緊急停止時間の設定★
                     stop_reason = LOG_STOP_FULL_WHITE;
                     pattern = 801;
				         }
		  } else {
                 daturin_cnt = 0;
      }
  }

//***********************************************************************
// マイコンカー走行処理プログラム  case:0 から case:1001 まで
//***********************************************************************
switch( pattern ) {

    //*******************************************************************
    // パターンについて
    //   0：スイッチ入力待ち
    //   1：スタートバーが開いたかチェック
    //  20：通常トレース
    //  30：右へ大曲げの終わりのチェック
    //  40：左へ大曲げの終わりのチェック
    // 100：クロスライン検出時の処理
    // 102：クロスラインを読み飛ばす
    // 103：クロスライン後のトレース、クランク検出
    // 110：左クランククリア処理　安定するまで少し待つ
    // 111：左クランククリア処理　曲げ終わりのチェック
    // 121：左クランククリア処理　外側の白線と見間違わないようにする
    // 150：右クランククリア処理　安定するまで少し待つ
    // 151：右クランククリア処理　曲げ終わりのチェック
    // 161：右クランククリア処理　外側の白線と見間違わないようにする
    // 200：右ハーフライン検出時の処理
    // 202：右ハーフラインを読み飛ばす
    // 300：右ハーフライン後のトレース、レーンチェンジ
    // 420：右レーンチェンジ終了のチェック
    // 500：左ハーフライン検出時の処理
    // 502：左ハーフラインを読み飛ばす
    // 600：左ハーフライン後のトレース、レーンチェンジ
    // 720：左レーンチェンジ終了のチェック
    // 800：走行終了処理
    //1001：SDカードへの走行記録開始処理
    //*******************************************************************

 case   0:           /* モータードライブ基盤のプッシュスイッチ入力待ち */
          if( pushsw_get() == 1 ) {
              log_st = LOG_INIT;   /* ログ処理開始 (ファイルオープン) */
              led_out_m( 0 );
              pattern = 1;
              cnt1 = 0;
              daturin_cnt = 0;             /* スタート前の初期値設定 */
              kansou_cnt = 0;              /* スタート前の初期値設定 */
              break;
          }
                 if( cnt1 % 500 < 250 ) {  /* LEDを点滅させる       */
                     led_out( 0x1 );
          } else {
                     led_out( 0x2 );
          }
                 if( startbar_get() == 1 ) {  /* スタートバーセンサが反応したらマイコンボードD13を点滅させる */
                            if( cnt0 % 200 < 100 ) {
                                led_out_m( 1 );
                     } else {
                                led_out_m( 0 );
                     }
          } else {
                     led_out_m( 0 );
          }
          break;

 case   1:              /* スタートバーが開いたかチェック */
          if( startbar_get() == 0 ) { /* スタートバーなし(開) -> スタート！！ */
              led_out( 0x0 );
              cnt_start = 0;
              log_st = LOG_WRITE;          /* ログ保存開始          */
              pattern = 20;                /* LEDを点滅させる       */
              cnt1 = 0;
              daturin_cnt = 0;             /* スタート前の初期値設定 */
              kansou_cnt = 0;              /* スタート前の初期値設定 */
          break;
          }
                 if( cnt1 % 100 < 50 ) {   /* LEDを素早く点滅させる  */
                     led_out( 0x1 );
          } else {
                     led_out( 0x2 );
          }
          break;

 case  20:                  /* 通常走行処理  ★一定の変化量で調整する */
          
          if( check_crossline() )     {cnt1 = 0;pattern = 100;break;} /* クロスラインチェック      */
          if( check_shasenhenkoum() ) {cnt1 = 0;pattern = 200;break;} /* 右車線変更ラインチェック  */
          if( check_shasenhenkouh() ) {cnt1 = 0;pattern = 500;break;} /* 左車線変更ラインチェック  */
             
          c = sensor_inp( MASK4_4 );
          if( c == ( 0x18 )||( 0x1c )||( 0x38 ) ) {
              if( cnt1 >=  350 ) {pattern = 21;break;}
		      } else {
              cnt1 = 0;
          }
    	    switch( sensor_inp( MASK4_4 ) ) {
            case 0x18:ha(   0 );sp(  100 ,  100 );break;                      /*     直  進      */

            case 0x1c:ha(   2 );sp(  100 ,  100 );break;                      /* 車体の右方向補正 */
            case 0x0c:ha(   7 );sp(  100 ,   97 );break;                      /* 微程度の右曲がり */
            case 0x0e:ha(  11 );sp(  100 ,   89 );break;                      /* 微程度の右曲がり */
            case 0x06:ha(  15 );sp(  100 ,   81 );break;                      /* 小程度の右曲がり */
            case 0x07:ha(  18 );sp(  100 ,   75 );break;                      /* 中程度の右曲がり */
            case 0x03:ha(  21 );sp(  100 ,   65 );cnt1 = 0;pattern = 30;break;/* 大程度の右曲がり */
                      
            case 0x38:ha(  -2 );sp(  100 ,  100 );break;                      /* 車体の左方向補正 */
            case 0x30:ha(  -7 );sp(   97 ,  100 );break;                      /* 微程度の左曲がり */
            case 0x70:ha( -11 );sp(   89 ,  100 );break;                      /* 微程度の左曲がり */
            case 0x60:ha( -15 );sp(   81 ,  100 );break;                      /* 小程度の左曲がり */
            case 0xe0:ha( -18 );sp(   75 ,  100 );break;                      /* 中程度の左曲がり */ 
            case 0xc0:ha( -21 );sp(   65 ,  100 );cnt1 = 0;pattern = 40;break;/* 大程度の左曲がり */
            default:break;
          }
          break;

 case  21:                  /* 通常走行処理  ★一定の変化量で調整する */
          
          if( check_crossline() )     {cnt1 = 0;pattern = 100;break;} /* クロスラインチェック      */
          if( check_shasenhenkoum() ) {cnt1 = 0;pattern = 200;break;} /* 右車線変更ラインチェック  */
          if( check_shasenhenkouh() ) {cnt1 = 0;pattern = 500;break;} /* 左車線変更ラインチェック  */

          c = sensor_inp( MASK4_4 );
                 if( c == ( 0x18 )||( 0x1c )||( 0x38 ) ) {
                     if( cnt1 >=  650 ) {
                         pattern = 22;
						         break;
				             }
		      } else {
                     cnt1 = 0;
          }
    	    switch( sensor_inp( MASK4_4 ) ) {
            case 0x18:ha(   0 );sp(  100 ,  100 );break;                      /*     直  進      */

            case 0x1c:ha(   5 );sp(  100 ,  100 );break;                      /* 車体の右方向補正 */
            case 0x0c:ha(   9 );sp(  100 ,   90 );break;                      /* 微程度の右曲がり */       
            case 0x0e:ha(  11 );sp(  100 ,   86 );break;                      /* 微程度の右曲がり */
            case 0x06:ha(  15 );sp(  100 ,   77 );break;                      /* 小程度の右曲がり */
            case 0x07:ha(  19 );sp(  100 ,   68 );break;                      /* 中程度の右曲がり */
            case 0x03:ha(  22 );sp(  100 ,   59 );cnt1 = 0;pattern = 31;break;/* 大程度の右曲がり */
                
            case 0x38:ha(  -2 );sp(  100 ,  100 );break;                      /* 車体の左方向補正 */
            case 0x30:ha(  -7 );sp(   95 ,  100 );break;                      /* 微程度の左曲がり */
            case 0x70:ha( -11 );sp(   86 ,  100 );break;                      /* 微程度の左曲がり */
            case 0x60:ha( -15 );sp(   77 ,  100 );break;                      /* 小程度の左曲がり */
            case 0xe0:ha( -19 );sp(   68 ,  100 );break;                      /* 中程度の左曲がり */                           
            case 0xc0:ha( -22 );sp(   59 ,  100 );cnt1 = 0;pattern = 41;break;/* 大程度の左曲がり */             
            default:break;
          }

          break;

 case  22:                  /* 通常走行処理  ★一定の変化量で調整する */
          
          if( check_crossline() )     {cnt1 = 0;pattern = 100;break;} /* クロスラインチェック      */
          if( check_shasenhenkoum() ) {cnt1 = 0;pattern = 200;break;} /* 右車線変更ラインチェック  */
          if( check_shasenhenkouh() ) {cnt1 = 0;pattern = 500;break;} /* 左車線変更ラインチェック  */
          
          c = sensor_inp( MASK4_4 );
                 if( c == ( 0x18 )||( 0x1c )||( 0x38 ) ) {
                     if( cnt1 >=  900 ) {
                         pattern = 23;
						         break;
				             }
		      } else {
                     cnt1 = 0;
          }
    	    switch( sensor_inp( MASK4_4 ) ) {

            case 0x18:ha(   0 );sp(  100 ,  100 );break;                      /*     直  進      */

            case 0x1c:ha(   5 );sp(  100 ,  100 );break;                      /* 車体の右方向補正 */           
            case 0x0c:ha(   9 );sp(  100 ,   90 );break;                      /* 微程度の右曲がり */          
            case 0x0e:ha(  11 );sp(  100 ,   82 );break;                      /* 微程度の右曲がり */       
            case 0x06:ha(  15 );sp(  100 ,   72 );break;                      /* 小程度の右曲がり */       
            case 0x07:ha(  19 );sp(  100 ,   62 );break;                      /* 中程度の右曲がり */       
            case 0x03:ha(  22 );sp(  100 ,   53 );cnt1 = 0;pattern = 32;break;/* 大程度の右曲がり */

            case 0x38:ha(  -2 );sp(  100 ,  100 );break;                      /* 車体の左方向補正 */                           
            case 0x30:ha(  -7 );sp(   93 ,  100 );break;                      /* 微程度の左曲がり */                            
            case 0x70:ha( -11 );sp(   82 ,  100 );break;                      /* 微程度の左曲がり */                            
            case 0x60:ha( -15 );sp(   72 ,  100 );break;                      /* 小程度の左曲がり */
            case 0xe0:ha( -19 );sp(   62 ,  100 );break;                      /* 中程度の左曲がり */                            
            case 0xc0:ha( -22 );sp(   53 ,  100 );cnt1 = 0;pattern = 42;break;/* 大程度の左曲がり */          
            default:break;
          }
          break;

 case  23:                  /* 通常走行処理  ★一定の変化量で調整する */
          
          if( check_crossline() )     {cnt1 = 0;pattern = 100;break;} /* クロスラインチェック      */
          if( check_shasenhenkoum() ) {cnt1 = 0;pattern = 200;break;} /* 右車線変更ラインチェック  */
          if( check_shasenhenkouh() ) {cnt1 = 0;pattern = 500;break;} /* 左車線変更ラインチェック  */
          
          c = sensor_inp( MASK4_4 );
                 if( c == ( 0x18 )||( 0x1c )||( 0x38 ) ) {
                     if( cnt1 >= 1100 ) {
                         pattern = 24;
						             break;
				             }
		      } else {
                     cnt1 = 0;
          }
    	    switch( sensor_inp( MASK4_4 ) ) {

                  case 0x18:ha(   0 );sp(  100 ,  100 );break;                      /*     直  進      */

                  case 0x1c:ha(   2 );sp(  100 ,  100 );break;                      /* 車体の右方向補正 */                            
                  case 0x0c:ha(   7 );sp(  100 ,   91 );break;                      /* 微程度の右曲がり */                          
                  case 0x0e:ha(  11 );sp(  100 ,   79 );break;                      /* 微程度の右曲がり */                           
                  case 0x06:ha(  15 );sp(  100 ,   67 );break;                      /* 小程度の右曲がり */                            
                  case 0x07:ha(  19 );sp(  100 ,   57 );break;                      /* 中程度の右曲がり */                            
				          case 0x03:ha(  23 );sp(  100 ,   47 );cnt1 = 0;pattern = 33;break;/* 大程度の右曲がり */
                            
                  case 0x38:ha(  -2 );sp(  100 ,  100 );break;                      /* 車体の左方向補正 */                           
				          case 0x30:ha(  -7 );sp(   91 ,  100 );break;                      /* 微程度の左曲がり */                            
				          case 0x70:ha( -11 );sp(   79 ,  100 );break;                      /* 微程度の左曲がり */                           
				          case 0x60:ha( -15 );sp(   67 ,  100 );break;                      /* 小程度の左曲がり */                            
                  case 0xe0:ha( -19 );sp(   57 ,  100 );break;                      /* 中程度の左曲がり */                            
                  case 0xc0:ha( -23 );sp(   47 ,  100 );cnt1 = 0;pattern = 43;break;/* 大程度の左曲がり */         
                  default:break;
          }
          break;

 case  24:                  /* 通常走行処理  ★一定の変化量で調整する */
          
          if( check_crossline() )     {cnt1 = 0;pattern = 100;break;} /* クロスラインチェック      */
          if( check_shasenhenkoum() ) {cnt1 = 0;pattern = 200;break;} /* 右車線変更ラインチェック  */
          if( check_shasenhenkouh() ) {cnt1 = 0;pattern = 500;break;} /* 左車線変更ラインチェック  */
          
          c = sensor_inp( MASK4_4 );
                 if( c == ( 0x18 )||( 0x1c )||( 0x38 ) ) {
                     if( cnt1 >= 1250 ) {
                         pattern = 25;
						             break;
				             }
		      } else {
                     cnt1 = 0;
          }
    	    switch( sensor_inp( MASK4_4 ) ) {

                  case 0x18:ha(   0 );sp(  100 ,  100 );break;                      /*     直  進      */ 

                  case 0x1c:ha(   2 );sp(  100 ,  100 );break;                      /* 車体の右方向補正 */                           
                  case 0x0c:ha(   7 );sp(  100 ,   89 );break;                      /* 微程度の右曲がり */                            
                  case 0x0e:ha(  11 );sp(  100 ,   76 );break;                      /* 微程度の右曲がり */                            
                  case 0x06:ha(  15 );sp(  100 ,   64 );break;                      /* 小程度の右曲がり */                          
                  case 0x07:ha(  19 );sp(  100 ,   52 );break;                      /* 中程度の右曲がり */                            
				          case 0x03:ha(  23 );sp(  100 ,   41 );cnt1 = 0;pattern = 34;break;/* 大程度の右曲がり */
                           
                  case 0x38:ha(  -2 );sp(  100 ,  100 );break;                      /* 車体の左方向補正 */                           
				          case 0x30:ha(  -7 );sp(   89 ,  100 );break;                      /* 微程度の左曲がり */                            
				          case 0x70:ha( -11 );sp(   76 ,  100 );break;                      /* 微程度の左曲がり */                            
				          case 0x60:ha( -15 );sp(   64 ,  100 );break;                      /* 小程度の左曲がり */                            
                  case 0xe0:ha( -19 );sp(   52 ,  100 );break;                      /* 中程度の左曲がり */                            
                  case 0xc0:ha( -23 );sp(   41 ,  100 );cnt1 = 0;pattern = 44;break;/* 大程度の左曲がり */
                  default:break;
          }
          break;

 case  25:                  /* 通常走行処理  ★一定の変化量で調整する */
          
          if( check_crossline() )     {cnt1 = 0;pattern = 100;break;} /* クロスラインチェック      */
          if( check_shasenhenkoum() ) {cnt1 = 0;pattern = 200;break;} /* 右車線変更ラインチェック  */
          if( check_shasenhenkouh() ) {cnt1 = 0;pattern = 500;break;} /* 左車線変更ラインチェック  */
          
          c = sensor_inp( MASK4_4 );
          // 単独点灯でもセンサー位置に応じた既存の操舵・内輪出力を使う。
          switch( sensor_inp( MASK4_4 ) ) {

                  case 0x18:ha(   0 );sp(  100 ,  100 );break;                      /*     直  進      */                            
                  case 0x08:
                  case 0x1c:ha(   2 );sp(  100 ,  100 );break;                      /* 車体の右方向補正 */                            
                  case 0x0c:ha(   7 );sp(  100 ,   87 );break;                      /* 微程度の右曲がり */                           
                  case 0x04:
                  case 0x0e:ha(  12 );sp(  100 ,   74 );break;                      /* 微程度の右曲がり */                           
                  case 0x06:ha(  16 );sp(  100 ,   61 );break;                      /* 小程度の右曲がり */                            
                  case 0x02:
                  case 0x07:ha(  20 );sp(  100 ,   48 );break;                      /* 中程度の右曲がり */                            
				          case 0x03:ha(  24 );sp(  100 ,   35 );cnt1 = 0;pattern = 35;break;/* 大程度の右曲がり */
                            
                  case 0x10:
                  case 0x38:ha(  -2 );sp(  100 ,  100 );break;                      /* 車体の左方向補正 */                            
				          case 0x30:ha(  -7 );sp(   87 ,  100 );break;                      /* 微程度の左曲がり */                            
				          case 0x20:
				          case 0x70:ha( -12 );sp(   74 ,  100 );break;                      /* 微程度の左曲がり */                            
				          case 0x60:ha( -16 );sp(   61 ,  100 );break;                      /* 小程度の左曲がり */                            
                  case 0x40:
                  case 0xe0:ha( -20 );sp(   48 ,  100 );break;                      /* 中程度の左曲がり */                           
                  case 0xc0:ha( -24 );sp(   35 ,  100 );cnt1 = 0;pattern = 45;break;/* 大程度の左曲がり */
                  default:break;
          }
          break;

 case  30:               /* 右へ大曲がりした時のコースアウト回避処理  */
          switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x06:
            case 0x07:cnt1 = 0;pattern = 20;break;/* 右大曲がりの復帰 */                            
            
            case 0x03:ha(  21 );sp(  100 ,   65 );break;                            
            case 0x83:ha(  24 );sp(   95 ,   52 );break;
            case 0x81:ha(  27 );sp(   40 ,   20 );break;/* 急ブレーキの処理 */
            case 0xc1:ha(  30 );sp(   25 ,   10 );break;
            case 0xc0:ha(  33 );sp(   15 ,    0 );break;
            default:break;
          }
          break;
 
 case  31:               /* 右へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >=  25 ) {
              pattern = 30;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x06:
            case 0x07:cnt1 = 0;pattern = 20;break;/* 右大曲がりの復帰 */

            case 0x03:ha(  22 );sp(  100 ,   59 );break;
            case 0x83:ha(  25 );sp(   93 ,   46 );break;
            case 0x81:ha(  28 );sp(   39 ,   16 );break;/* 急ブレーキの処理 */
            case 0xc1:ha(  31 );sp(   25 ,    5 );break;
            case 0xc0:ha(  34 );sp(   14 ,   -5 );break;
            default:break;
          }
          break;
 
 case  32:               /* 右へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >=  50 ) {
              pattern = 31;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x06:
            case 0x07:cnt1 = 0;pattern = 20;break;/* 右大曲がりの復帰 */

            case 0x03:ha(  22 );sp(  100 ,   53 );break;
            case 0x83:ha(  25 );sp(   91 ,   40 );break;
            case 0x81:ha(  28 );sp(   38 ,   12 );break;/* 急ブレーキの処理 */
            case 0xc1:ha(  31 );sp(   25 ,    0 );break;
            case 0xc0:ha(  34 );sp(   13 ,  -10 );break;
            default:break;
          }
          break;
 
 case  33:               /* 右へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >=  75 ) {
              pattern = 32;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x06:
            case 0x07:cnt1 = 0;pattern = 20;break;/* 右大曲がりの復帰 */

            case 0x03:ha(  23 );sp(  100 ,   47 );break;
            case 0x83:ha(  26 );sp(   89 ,   34 );break;
            case 0x81:ha(  29 );sp(   37 ,    8 );break;/* 急ブレーキの処理 */
            case 0xc1:ha(  32 );sp(   25 ,   -5 );break;
            case 0xc0:ha(  35 );sp(   12 ,  -15 );break;
            default:break;
          }
          break;
 
 case  34:               /* 右へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >= 100 ) {
              pattern = 33;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x06:
            case 0x07:cnt1 = 0;pattern = 20;break;/* 右大曲がりの復帰 */

            case 0x03:ha(  23 );sp(  100 ,   41 );break;
            case 0x83:ha(  26 );sp(   87 ,   28 );break;
            case 0x81:ha(  29 );sp(   36 ,    4 );break;/* 急ブレーキの処理 */
            case 0xc1:ha(  32 );sp(   25 ,  -10 );break;
            case 0xc0:ha(  35 );sp(   11 ,  -20 );break;
            default:break;
          }
          break;
 
 case  35:               /* 右へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >= 125 ) {
              pattern = 34;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x06:
            case 0x07:cnt1 = 0;pattern = 20;break; /* 右大曲がりの復帰 */

            case 0x03:ha(  24 );sp(  100 ,   35 );break;
            case 0x83:ha(  27 );sp(   85 ,   22 );break;
            case 0x81:ha(  30 );sp(   35 ,    0 );break;/* 急ブレーキの処理 */
            case 0xc1:ha(  33 );sp(   25 ,  -15 );break;
            case 0xc0:ha(  36 );sp(   10 ,  -25 );break;
            default:break;
          }
          break;
 
 case  40:               /* 左へ大曲がりした時のコースアウト回避処理  */
          switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x60:
            case 0xe0:cnt1 = 0;pattern = 20;break;/* 左大曲がりの復帰 */

            case 0xc0:ha( -21 );sp(   61 ,  100 );break;
            case 0xc1:ha( -24 );sp(   48 ,   95 );break;
            case 0x81:ha( -27 );sp(   16 ,   44 );break;/* 急ブレーキの処理 */
            case 0x83:ha( -30 );sp(    6 ,   29 );break;
            case 0x03:ha( -33 );sp(    0 ,   19 );break;
            default:break;
          }
          break;
		  
 case  41:               /* 左へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >=  25 ) {
              pattern = 40;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x60:
            case 0xe0:cnt1 = 0;pattern = 20;break;/* 左大曲がりの復帰 */

            case 0xc0:ha( -22 );sp(   59 ,  100 );break;
            case 0xc1:ha( -25 );sp(   46 ,   93 );break;
            case 0x81:ha( -28 );sp(   16 ,   39 );break;/* 急ブレーキの処理 */
            case 0x83:ha( -31 );sp(    5 ,   25 );break;
            case 0x03:ha( -34 );sp(   -5 ,   14 );break;
            default:break;
          }
          break;
		  
 case  42:               /* 左へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >=  50 ) {
              pattern = 41;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x60:
            case 0xe0:cnt1 = 0;pattern = 20;break;/* 左大曲がりの復帰 */

            case 0xc0:ha( -22 );sp(   53 ,  100 );break;
            case 0xc1:ha( -25 );sp(   40 ,   91 );break;
            case 0x81:ha( -28 );sp(   12 ,   38 );break; /* 急ブレーキの処理 */
            case 0x83:ha( -31 );sp(    0 ,   25 );break;
            case 0x03:ha( -34 );sp(  -10 ,   13 );break;
            default:break;
          }
          break;
		  
 case  43:               /* 左へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >=  75 ) {
              pattern = 42;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x60:
            case 0xe0:cnt1 = 0;pattern = 20;break;/* 左大曲がりの復帰 */

            case 0xc0:ha( -23 );sp(   47 ,  100 );break;
            case 0xc1:ha( -26 );sp(   34 ,   89 );break;
            case 0x81:ha( -29 );sp(    8 ,   37 );break;/* 急ブレーキの処理 */
            case 0x83:ha( -32 );sp(   -5 ,   25 );break;
            case 0x03:ha( -35 );sp(  -15 ,   12 );break;
            default:break;
          }
          break;
		  
 case  44:               /* 左へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >= 100 ) {
              pattern = 43;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x60:
            case 0xe0:cnt1 = 0;pattern = 20;break;/* 左大曲がりの復帰 */

            case 0xc0:ha( -23 );sp(   41 ,  100 );break;
            case 0xc1:ha( -26 );sp(   28 ,   87 );break;
            case 0x81:ha( -29 );sp(    4 ,   36 );break;/* 急ブレーキの処理 */
            case 0x83:ha( -32 );sp(  -10 ,   25 );break;
            case 0x03:ha( -35 );sp(  -20 ,   11 );break;
            default:break;
          }
          break;
		  
 case  45:               /* 左へ大曲がりした時のコースアウト回避処理  */
          if( cnt1 >= 125 ) {
              pattern = 44;
		          break;
		      }
		      switch( sensor_inp( MASK4_4 ) ) {
          
            case 0x60:
            case 0xe0:cnt1 = 0;pattern = 20;break;/* 左大曲がりの復帰 */

            case 0xc0:ha( -24 );sp(   35 ,  100 );break;
            case 0xc1:ha( -27 );sp(   22 ,   85 );break;
            case 0x81:ha( -30 );sp(    0 ,   35 );break;
            case 0x83:ha( -33 );sp(  -15 ,   25 );break; /* 急ブレーキの処理 */
            case 0x03:ha( -36 );sp(  -25 ,   10 );break;
            default:break;
          }
          break;


 case 100:                              /* クロスライン検出時の処理  */
          led_out( 0x3 );
          ha(   0 );
          kyori_flug = 0;
		      cnt1 = 0;
          pattern = 101;
          break;

 case 101:                       /* クロスライン検出後の通過確認処理  */
                           /* クロスラインテープ幅の通過時間計測[ms]  */
                 if( kyori_flug == 0 && check_crossline() == 1 ){
                     kyori_flug = 1;
          } else if( kyori_flug == 1 && check_crossline() == 0 ){
                     kyori_flug = 2;
		                 kyoritime = cnt1;
		      }
          if( kyori_flug == 2 ){
              kyori_flug = 0;
              cnt1 = 0;
              pattern = 102;
              break;
          }
          switch( sensor_inp( MASK4_4 ) ) {

            case 0x18:ha(   0 );break;  /*   センタ → 直  進   */                                          
            case 0x1c:ha(   2 );break;  /* 小左寄り → 小右曲げ */                                            
            case 0x0c:ha(   4 );break;  /* 中左寄り → 中右曲げ */                                                
            case 0x38:ha(  -2 );break;  /* 小右寄り → 小左曲げ */                                                
            case 0x30:ha(  -4 );break;  /* 中右寄り → 中左曲げ */                                               
            default:break;
          }
          break;

 case 102:        /* クロスライン通過後にハンドルをすぐに曲げない処理 */
          kyori_flug = 0;
          newcrank();
          if( cnt1 > 100 ) {
              pattern = 103;
              break;
          }
          switch( sensor_inp( MASK4_4 ) ) {

            case 0x18:ha(   0 );break;  /*   センタ → 直  進   */                                          
            case 0x1c:ha(   2 );break;  /* 小左寄り → 小右曲げ */                                            
            case 0x0c:ha(   4 );break;  /* 中左寄り → 中右曲げ */                                                
            case 0x38:ha(  -2 );break;  /* 小右寄り → 小左曲げ */                                                
            case 0x30:ha(  -4 );break;  /* 中右寄り → 中左曲げ */                                               
            default:break;
          }
          break;

 case 103:                           /* クロスライン通過後の走行処理 */
          newcrank();
          switch( sensor_inp( MASK4_4 ) ) {

            case 0x18:ha(   0 );break;/*   センタ → 直  進   */                
            case 0x1c:ha(   4 );break;/* 小左寄り → 小右曲げ */                
            case 0x0c:ha(   6 );break;/* 中左寄り → 中右曲げ */                     
            case 0x38:ha(  -4 );break;/* 小右寄り → 小左曲げ */                     
            case 0x30:ha(  -6 );break;/* 中右寄り → 中左曲げ */                     
                      
            case 0xe0:/* 左クランクと判断 → 左クランク旋回処理へ */
            case 0xf0:
            case 0xf8:
            case 0xfc:led_out( 0x1 );ha( -50 );cnt1 = 0;pattern = 110;break;/* ★左クランク最大曲げ角度 */
                    
            case 0x07:/* 右クランクと判断 → 右クランク旋回処理へ */
            case 0x0f:
            case 0x1f:
            case 0x3f:led_out( 0x2 );ha(  50 );cnt1 = 0;pattern = 150;break;/* ★右クランク最大曲げ角度 */
            default:break;
          }
          break;
		  
 case 110:                               /* 左クランク突入時走行処理 */
          ha( -50 );                     /* ★左クランク最大曲げ角度 */
          if( cnt1 >   70 ) {            /* 左クランクライン通過確認 */
              cnt1 = 0;
              pattern = 111;
              break;
          }
          switch( sensor_inp(MASK4_4) ) {

            case 0x80:sp(    5 ,   55 );break;
            case 0xc0:sp(    9 ,   58 );break;
            case 0xe0:sp(   13 ,   61 );break;
            case 0x60:sp(   17 ,   64 );break;
            case 0xf0:sp(   21 ,   67 );break;
            case 0xf8:sp(   25 ,   70 );break;
            case 0xfc:sp(   29 ,   73 );break;
            case 0xfe:sp(   33 ,   76 );break;
            case 0xff:sp(   37 ,   79 );break;
            case 0x70:sp(   41 ,   82 );break;
            case 0x78:sp(   45 ,   84 );break;
            case 0x7c:sp(   49 ,   86 );break;
            case 0x7e:sp(   53 ,   88 );break;
            case 0x7f:sp(   57 ,   90 );break;
            
            case 0x00:cnt1 = 0;pattern = 120;break;/* 中央走行時の処理 */
                      
            case 0x01:
            case 0x03:cnt1 = 0;pattern = 121;break;/* 外側走行時の処理 */
            default:break;
          }
	        break;

 case 111:                               /* 左クランク低速走行時処理 */
          ha( -50 );                     /* ★左クランク最大曲げ角度 */
          switch( sensor_inp(MASK4_4) ) {

            case 0x80:sp(   10 ,   60 );break;
            case 0xc0:sp(   15 ,   63 );break;
            case 0xe0:sp(   20 ,   66 );break;
            case 0x60:sp(   25 ,   69 );break;
            case 0xf0:sp(   30 ,   72 );break;
            case 0xf8:sp(   35 ,   75 );break;
            case 0xfc:sp(   40 ,   78 );break;
            case 0xfe:sp(   45 ,   81 );break;
            case 0xff:sp(   50 ,   84 );break;
            case 0x70:
            case 0x30:
            
            case 0x38:led_out( 0x0 );cnt1 = 0;pattern = 20;break;/* 通常走行へ完全復帰 */
            
            case 0x00:cnt1 = 0;pattern = 120;break;/* 中央走行時の処理 */
            case 0x01:
            case 0x03:cnt1 = 0;pattern = 121;break;/* 外側走行時の処理 */
            default:break;
          }
	        break;

 case 120:             /* 左クランク高速走行後のセンターライン復帰処理 */
          switch( sensor_inp(MASK4_4) ) {

            case 0x00:ha( -50 );sp(    0 ,   70 );break;/* 左クランク曲角度 */
            case 0x83:
            case 0x81:
            case 0xc1:ha( -55 );sp(    5 ,   65 );break;/* 左クランク曲角度 */
            case 0x80:ha( -50 );sp(   20 ,   75 );break;/* 左クランク曲角度 */
            case 0xc0:ha( -45 );sp(   40 ,   90 );break;/* 左クランク曲角度 */
            case 0xe0:ha( -40 );sp(   60 ,  100 );break;/* 左クランク曲角度 */
            case 0x60:
            case 0x70:

            case 0x30:led_out( 0x0 );cnt1 = 0;pattern = 20;break;/* 通常走行へ完全復帰 */

            case 0x01:
            case 0x03:cnt1 = 0;pattern = 121;break;/* 外側走行時の処理 */
            default:break;
          }
	        break;

 case 121:                    /* 左クランク高速走行による脱輪回避処理 */
          ha( -50 );                     /* 左クランクの最大曲げ角度 */
          switch( sensor_inp(MASK4_4) ) {

            case 0x01:sp(   -5 ,   70 );break;
            case 0x03:sp(  -11 ,   65 );break;
            case 0x07:sp(  -16 ,   60 );break;
            case 0x0f:sp(  -22 ,   55 );break;
            case 0x1f:sp(  -26 ,   50 );break;
            case 0x3f:sp(  -30 ,   45 );break;
            
            case 0x83:
            case 0x81:
            case 0xc1:pattern = 120;break;/* 中央ライン検出後の処理 */
            default:break;
          }
	        break;

 case 150:                               /* 右クランク突入時走行処理 */
          ha(  50 );                     /* ★右クランク最大曲げ角度 */

          if( cnt1 >   70 ) {            /* 右クランクライン通過確認 */
              cnt1 = 0;
              pattern = 151;
              break;
          }
          switch( sensor_inp(MASK4_4) ) {

            case 0x01:sp(   55 ,    5 );break;
            case 0x03:sp(   58 ,    9 );break;
            case 0x07:sp(   61 ,   13 );break;
            case 0x06:sp(   64 ,   17 );break;
            case 0x0f:sp(   67 ,   21 );break;
            case 0x1f:sp(   80 ,   15 );break;
            case 0x3f:sp(   73 ,   29 );break;
            case 0x7f:sp(   76 ,   33 );break;
            case 0xff:sp(   79 ,   37 );break;
            case 0x0e:sp(   82 ,   41 );break;
            case 0x1e:sp(   84 ,   45 );break;
            case 0x3e:sp(   86 ,   49 );break;
            case 0x7e:sp(   88 ,   53 );break;
            case 0xfe:sp(   90 ,   57 );break;

            case 0x00:cnt1 = 0;pattern = 160;break;/* 中央走行時の処理 */
            
            case 0x80:
            case 0xc0:cnt1 = 0;pattern = 161;break;/* 外側走行時の処理 */
            default:break;
          }
	        break;

 case 151:                               /* 右クランク低速走行時処理 */
          ha(  50 );                     /* ★右クランク最大曲げ角度 */
          switch( sensor_inp(MASK4_4) ) {

            case 0x01:sp(   60 ,   10 );break;
            case 0x03:sp(   63 ,   15 );break;
            case 0x07:sp(   66 ,   20 );break;
            case 0x06:sp(   69 ,   25 );break;
            case 0x0f:sp(   72 ,   30 );break;
            case 0x1f:sp(   75 ,   35 );break;
            case 0x3f:sp(   78 ,   40 );break;
            case 0x7f:sp(   81 ,   45 );break;
            case 0xff:sp(   84 ,   50 );break;
            
            case 0x0e:
            case 0x0c:
            case 0x1c:led_out( 0x0 );cnt1 = 0;pattern = 20;break;/* 通常走行へ完全復帰 */

            case 0x00:cnt1 = 0;pattern = 160;break;/* 中央走行時の処理 */
            
            case 0x80:
            case 0xc0:cnt1 = 0;pattern = 161;break;/* 外側走行時の処理 */
            default:break;
          }
	        break;

 case 160:             /* 右クランク高速走行後のセンターライン復帰処理 */
          switch( sensor_inp(MASK4_4) ) {

            case 0x00:ha(  50 );sp(   70 ,    0 );break;/* 右クランク曲角度 */
            case 0xc1:
            case 0x81:
            case 0x83:ha(  50 );sp(   60 ,    5 );break;/* 右クランク曲角度 */
            case 0x01:ha(  45 );sp(   70 ,   20 );break;/* 右クランク曲角度 */
            case 0x03:ha(  40 );sp(   85 ,   40 );break;/* 右クランク曲角度 */
            case 0x07:ha(  35 );sp(  100 ,   60 );break;/* 右クランク曲角度 */
            
            case 0x06:
            case 0x0e:
            case 0x0c:led_out( 0x0 );cnt1 = 0;pattern = 20;break;/* 通常走行へ完全復帰 */
            
            case 0x80:
            case 0xc0:cnt1 = 0;pattern = 161;break; /* 外側走行時の処理 */
            default:break;
          }
	        break;

 case 161:                    /* 右クランク高速走行による脱輪回避処理 */
          ha(  50 );                     /* 右クランクの最大曲げ角度 */
          switch( sensor_inp(MASK4_4) ) {

            case 0x80:sp(   45 ,   -5 );break;
            case 0xc0:sp(   40 ,  -11 );break;
            case 0xe0:sp(   35 ,  -17 );break;
            case 0xf0:sp(   30 ,  -23 );break;
            case 0xf8:sp(   25 ,  -29 );break;
            case 0xfc:sp(   20 ,  -35 );break;
            
            case 0xc1:
            case 0x81:
            case 0x83:pattern = 160; break;/* 中央ライン検出後の処理 */
            default:break;
          }
	        break;

 case 200:                           /* 右車線変更ライン検出時の処理 */
          led_out( 0x1 );
          ha(   0 );
          kyori_flug = 0;
	        cnt1 = 0;
          pattern = 201;
          break;

 case 201:              /* 右車線変更ライン検出後の通過状況の確認処理 */

          if( check_crossline() )     {cnt1 = 0;pattern = 100;break;} /* クロスライン再確認      */
          
		      /* 右車線変更の通過時間計測[ms] 白線手前から白線終わりまで  */
		             if( kyori_flug == 0 && check_shasenhenkoum() == 1 ){
                     kyori_flug = 1;
	   	    } else if( kyori_flug == 1 && check_shasenhenkoum() == 0 ){
                     kyori_flug = 2;
		  	             kyoritime = cnt1 * 20 / HALF_LINE_WIDTH_MM; /* 20mm幅に換算 */
		      }
          if( kyori_flug == 2 ){
	            cnt1 = 0;
	            pattern = 202;
	            break;
		      }
          switch( sensor_inp( MASK4_4 ) ) {

            case 0x18:ha(   0 );break;/*   センタ → 直  進   */                
            case 0x1c:ha(  13 );break; /* 小左寄り → 小右曲げ */                
            case 0x0c:ha(  16 );break;/* 中左寄り → 中右曲げ */                     
            case 0x38:ha( -13 );break;/* 小右寄り → 小左曲げ */                     
            case 0x30:ha( -16 );break;/* 中右寄り → 中左曲げ */                     
            default:break;
          }
		      break;

 case 202:                        /*右車線変更ライン通過後の減速処理 */

          if( check_crossline() )     {cnt1 = 0;pattern = 100;break;} /* クロスライン再確認      */

		  	  kyori_flug = 0;
          right_shift_side = SHIFT_ON_OR_RIGHT;  /* 右寄せ走行の位置判定を初期化 */
          right_shift_last = 0x18;
	        shasenhenkou();
		       	     if( kyoritime <=  12 ){pattern = 300;break;
		  	  } else if( kyoritime <=  15 ){pattern = 301;break;
		   	  } else if( kyoritime <=  18 ){pattern = 302;break;
		   	  } else if( kyoritime <=  21 ){pattern = 303;break;
		   	  } else if( kyoritime <=  24 ){pattern = 304;break;
		   	  } else if( kyoritime <=  27 ){pattern = 305;break;
		   	  } else if( kyoritime <=  30 ){pattern = 306;break;
		   	  } else if( kyoritime <=  33 ){pattern = 307;break;
		   	  } else if( kyoritime <=  36 ){pattern = 308;break;
		  	  } else if( kyoritime <=  39 ){pattern = 309;break;
	 	  	  } else                       {pattern = 310;break;
		  	  }
		      switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;				          
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 300:                      /* 右車線変更前の車体の右寄せ走行処理 */
		      shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x1 );
			        ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   75 ,   30 );
			        cnt1 = 0;
			        pattern = 320;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   5 );break;
            case 0x18:ha(   4 );break;
            case 0x38:ha(   4 );break;
            case 0x30:ha(   4 );break;
            case 0x70:ha(   3 );break;
            case 0x60:ha(   2 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -2 );break;
            case 0xc1:ha(  -3 );break;
            case 0x81:ha(  -4 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 301:                      /* 右車線変更前の車体の右寄せ走行処理 */
		      shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x1 );
			        ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   76 ,   32 );
			        cnt1 = 0;
			        pattern = 321;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 302:                      /* 右車線変更前の車体の右寄せ走行処理 */
		  	  shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		  	  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		    	    led_out( 0x1 );
			 	  	  ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   77 ,   34 );
			        cnt1 = 0;
			        pattern = 322;
              break;
			    }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 303:                      /* 右車線変更前の車体の右寄せ走行処理 */
		  	  shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		  	  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		     	    led_out( 0x1 );
			  	    ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   78 ,   36 );
			 	      cnt1 = 0;
			  	    pattern = 323;
              break;
		  	  }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 304:                      /* 右車線変更前の車体の右寄せ走行処理 */
		  	  shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		  	  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		     	    led_out( 0x1 );
			 	      ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   79 ,   38 );
			 	      cnt1 = 0;
			 	      pattern = 324;
              break;
		 	    }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 305:                      /* 右車線変更前の車体の右寄せ走行処理 */
		  	  shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		  	  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		      	  led_out( 0x1 );
			 	      ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   80 ,   40 );
			  	    cnt1 = 0;
			 	      pattern = 325;
              break;
		 	    }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 306:                      /* 右車線変更前の車体の右寄せ走行処理 */
		  	  shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		  	  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		      	  led_out( 0x1 );
			  	    ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   81 ,   42 );
			  	    cnt1 = 0;
			  	    pattern = 326;
              break;
		  	  }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 307:                      /* 右車線変更前の車体の右寄せ走行処理 */
		  	  shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		  	  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		      	  led_out( 0x1 );
			 	      ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   82 ,   44 );
			  	    cnt1 = 0;
			  	    pattern = 327;
              break;
		  	  }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 308:                      /* 右車線変更前の車体の右寄せ走行処理 */
		  	  shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		  	  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		      	  led_out( 0x1 );
			  	    ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   83 ,   46 );
			  	    cnt1 = 0;
			  	    pattern = 328;
              break;
		  	  }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 309:                      /* 右車線変更前の車体の右寄せ走行処理 */
		  	  shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		  	  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		      	  led_out( 0x1 );
			  	    ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   84 ,   48 );
			  	    cnt1 = 0;
			  	    pattern = 329;
              break;
		  	  }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 310:                      /* 右車線変更前の車体の右寄せ走行処理 */
		  	  shasenhenkou();
          if( right_shift_left_steer() ) break; /* 車体が中央線より左：右へ戻す */
		  	  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 右車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		      	  led_out( 0x1 );
			  	    ha( RIGHT_LANE_TURN_DEG );        /* ★右車線変更時のハンドルの曲げ角度 */
              sp(   85 ,   50 );
			  	    cnt1 = 0;
			  	    pattern = 330;
              break;
		  	  }
          }
	        switch( sensor_inp(MASK4_4) ) {

            case 0x1c:ha(   3 );break;
            case 0x18:ha(   2 );break;
            case 0x38:ha(   2 );break;
            case 0x30:ha(   2 );break;
            case 0x70:ha(   1 );break;
            case 0x60:ha(   1 );break;
            case 0xe0:ha(   0 );break;
            case 0xc0:ha(  -1 );break;
            case 0xc1:ha(  -2 );break;
            case 0x81:ha(  -3 );break;
            case 0x83:ha(  -4 );break;
            case 0x03:ha(  -5 );break;
            default:break;
          }
          break;

 case 320: case 321: case 322: case 323: case 324: case 325:
 case 326: case 327: case 328: case 329: case 330:
 case 340: case 341: case 342: case 343: case 344: case 345:
 case 346: case 347: case 348: case 349: case 350: {
          const int stage = (pattern <= 330) ? pattern - 320 : pattern - 340;
          // cnt1は白線が途切れた時点からの累積時間。初期旋回中もセンサーの順序を記憶する。
          const bool can_return = cnt1 >= RIGHT_LANE_FULL_TURN_MS + stage;
          if( cnt1 == 1 ) {
              right_lane_recovery = LANE_RETURN;
              right_lane_inner_seen = false;
              right_lane_center_ms = 0;
          }
          if( cnt1 >= RIGHT_LANE_SEARCH_LIMIT_MS ) {
              ha( 0 );
              sp( 0, 0 );
              cnt1 = 0;
              stop_reason = LOG_STOP_RIGHT_LANE_TIMEOUT;
              pattern = 801;
              break;
          }

          // 右内側 (bit 2) から中央寄り (bit 3) に動く白線を候補にする。
          c = sensor_inp( MASK4_4 );
          decision_sensor = c;
          switch( c ) {
            case 0x04: case 0x06: case 0x07:
              right_lane_inner_seen = true;
              right_lane_center_ms = 0;
              break;

            case 0x08: case 0x0c: case 0x0e: case 0x1c: case 0x18:
              if( right_lane_inner_seen ) {
                  right_lane_center_ms++;
              }
              break;

            default:
              // 左端線、全消灯、左右同時反応など。前の候補とはつながない。
              right_lane_inner_seen = false;
              right_lane_center_ms = 0;
              break;
          }
          if( can_return && right_lane_center_ms >= RIGHT_LANE_CENTER_CONFIRM_MS ) {
              // 元の終了処理の最も弱い戻し舵から入り、以降は元の表で追従する。
              ha( -3 );
              sp( 75 + stage, 90 + stage );
              cnt1 = 0;
              pattern = 400 + stage;
          } else {
              if( can_return ) {
                  ha( RIGHT_LANE_SEARCH_DEG );
                  pattern = 340 + stage;
              } else {
                  ha( RIGHT_LANE_TURN_DEG );
              }
              sp( 75 + stage, 30 + 2 * stage );
          }
          break;
 }

 case 400:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   75 ,   90 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   72 ,   88 );  
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   69 ,   86 );  
		                        break;
				          case 0x38:ha( -10 );
                            sp(   66 ,   84 );  
		                        break;
                  case 0x30:ha( -13 );
				                    sp(   63 ,   81 );  
		                        break;
				          case 0x70:ha( -16 );
				                    sp(   59 ,   78 );  
		                        cnt1 = 0;
                            pattern = 420;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   55 ,   75 );  
		                        cnt1 = 0;
                            pattern = 420;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 401:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   76 ,   91 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   73 ,   89 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   70 ,   87 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   67 ,   85 );
		                        break;
                  case 0x30:ha( -13 );
				                    sp(   64 ,   82 );
		                        break;
				          case 0x70:ha( -16 );
				                    sp(   60 ,   79 );
		                        cnt1 = 0;
                            pattern = 421;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   56 ,   76 );
		                        cnt1 = 0;
                            pattern = 421;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 402: /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   77 ,   92 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   74 ,   90 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   71 ,   88 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   68 ,   86 );
		                        break;
                  case 0x30:ha( -13 );
                            sp(   65 ,   83 );
		                        break;
                  case 0x70:ha( -16 );
				                    sp(   61 ,   80 );
		                        cnt1 = 0;
                            pattern = 422;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   57 ,   77 );
		                        cnt1 = 0;
                            pattern = 422;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 403:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   78 ,   93 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   75 ,   91 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   72 ,   89 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   69 ,   87 );
		                        break;
                  case 0x30:ha( -13 );
                            sp(   66 ,   84 );
		                        break;
                  case 0x70:ha( -16 );
				                    sp(   62 ,   81 );
		                        cnt1 = 0;
                            pattern = 423;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   58 ,   78 );
		                        cnt1 = 0;
                            pattern = 423;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 404:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   79 ,   94 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   76 ,   92 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   73 ,   90 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   70 ,   88 );
		                        break;
                  case 0x30:ha( -13 );
                            sp(   67 ,   85 );
		                        break;
                  case 0x70:ha( -16 );
				                    sp(   63 ,   82 );
		                        cnt1 = 0;
                            pattern = 424;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   59 ,   79 );
		                        cnt1 = 0;
                            pattern = 424;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 405:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   80 ,   95 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   77 ,   93 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   74 ,   91 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   71 ,   89 );
		                        break;
                  case 0x30:ha( -13 );
                            sp(   68 ,   86 );
		                        break;
                  case 0x70:ha( -16 );
				                    sp(   64 ,   83 );
		                        cnt1 = 0;
                            pattern = 425;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   60 ,   80 );
		                        cnt1 = 0;
                            pattern = 425;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 406:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   81 ,   96 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   78 ,   94 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   75 ,   92 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   72 ,   90 );
		                        break;
                  case 0x30:ha( -13 );
                            sp(   69 ,   87 );
		                        break;
                  case 0x70:ha( -16 );
				                    sp(   65 ,   84 );
		                        cnt1 = 0;
                            pattern = 426;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   61 ,   81 );
		                        cnt1 = 0;
                            pattern = 426;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 407:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   82 ,   97 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   79 ,   95 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   76 ,   93 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   73 ,   91 );
		                        break;
                  case 0x30:ha( -13 );
                            sp(   70 ,   88 );
		                        break;
                  case 0x70:ha( -16 );
				                    sp(   66 ,   85 );
		                        cnt1 = 0;
                            pattern = 427;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   62 ,   82 );
		                        cnt1 = 0;
                            pattern = 427;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 408:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   83 ,   98 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   80 ,   96 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   77 ,   94 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   74 ,   92 );
		                        break;
                  case 0x30:ha( -13 );
                            sp(   71 ,   89 );
		                        break;
                  case 0x70:ha( -16 );
				                    sp(   67 ,   86 );
		                        cnt1 = 0;
                            pattern = 428;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   63 ,   83 );
		                        cnt1 = 0;
                            pattern = 428;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 409:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   84 ,   99 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   81 ,   97 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   78 ,   95 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   75 ,   93 );
		                        break;
                  case 0x30:ha( -13 );
                            sp(   72 ,   90 );
		                        break;
                  case 0x70:ha( -16 );
				                    sp(   68 ,   87 );
		                        cnt1 = 0;
                            pattern = 429;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   64 ,   84 );
		                        cnt1 = 0;
                            pattern = 429;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 410:   /* 右車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x0c:ha(  -3 );
                            sp(   85 ,  100 );  
		                        break;
			            case 0x1c:ha(  -5 );
                            sp(   82 ,   98 );
		                        break;
			            case 0x18:ha(  -7 );
                            sp(   79 ,   96 );
		                        break;
				          case 0x38:ha( -10 );
                            sp(   76 ,   94 );
		                        break;
                  case 0x30:ha( -13 );
                            sp(   73 ,   91 );
		                        break;
                  case 0x70:ha( -16 );
				                    sp(   69 ,   88 );
		                        cnt1 = 0;
                            pattern = 430;
                            break;
				          case 0x60:ha( -19 );
				                    sp(   65 ,   85 );
		                        cnt1 = 0;
                            pattern = 430;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 420: case 421: case 422: case 423: case 424: case 425:
 case 426: case 427: case 428: case 429: case 430: {
          const int stage = pattern - 420;
          c = sensor_inp( MASK4_4 );
          if( cnt1 == 1 ) {
              right_lane_split_ms = 0;
              right_lane_settle_ms = 0;
          }

          // 両端が同時に白い間は、直前の左舵をいつまでも保持しない。
          if( c == 0x81 || c == 0xc1 ) {
              right_lane_settle_ms = 0;
              if( right_lane_split_ms < RIGHT_LANE_SPLIT_MS ) ++right_lane_split_ms;
              if( right_lane_split_ms == RIGHT_LANE_SPLIT_MS ) {
                  right_lane_recovery = LANE_SPLIT_SEEN;
                  ha( RIGHT_LANE_SPLIT_STEER_DEG );
                  sp( 60 + stage, 70 + stage );
              }
              break;
          }
          right_lane_split_ms = 0;

          const bool center = c == 0x18 || c == 0x38 || c == 0x1c || c == 0x3c;
          const bool single_center = c == 0x10 || c == 0x08;
          if( center || (right_lane_recovery != LANE_RETURN && single_center) ) {
              if( right_lane_recovery != LANE_RETURN ) {
                  right_lane_recovery = LANE_CENTERING;
                  // 両端線を見た走行では、中央付近で小さく舵を合わせてから加速する。
                  int angle = 0;
                  if( c == 0x38 || c == 0x10 ) angle = -2;
                  if( c == 0x1c || c == 0x08 ) angle = 2;
                  ha( angle );
                  sp( 65 + stage, 65 + stage );
                  if( ++right_lane_settle_ms < RIGHT_LANE_SETTLE_MS ) break;
              }
              led_out( 0x0 );
              cnt1 = 0;
              pattern = 20;
              break;
          }
          right_lane_settle_ms = 0;

          // 確認時間より速く白線が中央を横切った場合も、右に見える線に追従する。
          if( right_lane_recovery == LANE_CENTERING ) {
              int angle = 0;
              switch( c ) {
                  case 0x0c:                       angle = 7;  break;
                  case 0x04: case 0x0e:            angle = 11; break;
                  case 0x06:                       angle = 15; break;
                  case 0x02: case 0x07:            angle = 18; break;
                  case 0x01: case 0x03:            angle = 21; break;
              }
              if( angle != 0 ) {
                  ha( angle );
                  sp( 65 + stage, 65 + stage );
                  break;
              }
          }
          switch( c ) {
              case 0x30: ha( -13 ); sp( 63 + stage, 81 + stage ); break;
              case 0x70: ha( -16 ); sp( 59 + stage, 78 + stage ); break;
              case 0x60: ha( -19 ); sp( 55 + stage, 75 + stage ); break;
              case 0xe0: ha( -23 ); sp( 49 + stage, 70 + stage ); break;
              case 0xc0:
                  ha( -27 );
                  sp( 43 + stage, 65 + stage );
                  cnt1 = 0;
                  pattern = 440 + stage;
                  break;
              default: break;
          }
          break;
 }

 case 440:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 420;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   43 ,   65 );  
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   35 ,   60 );  
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   27 ,   50 );  
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   14 ,   40 );  
		                        break;
		              case 0x03:ha( -39 );
				                    sp(    1 ,   30 );  
		                        break;
		              case 0x07:ha( -42 );
				                    sp(  -12 ,   20 );  
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -25 ,   10 );  
		                        break;
		                default:
                            break;
          }
          break;

 case 441:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 421;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   44 ,   66 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   36 ,   61 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   28 ,   51 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   15 ,   41 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(    2 ,   31 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(  -11 ,   21 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -24 ,   11 );
		                        break;
		                default:
                            break;
          }
          break;

 case 442:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 422;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   45 ,   67 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   37 ,   62 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   29 ,   52 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   16 ,   42 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(    3 ,   32 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(  -10 ,   22 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -23 ,   12 );
		                        break;
		                default:
                            break;
          }
          break;

 case 443:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 423;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   46 ,   68 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   38 ,   63 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   30 ,   53 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   17 ,   43 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(    4 ,   33 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(   -9 ,   23 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -22 ,   13 );
		                        break;
		                default:
                            break;
          }
          break;

 case 444:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 424;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   47 ,   69 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   39 ,   64 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   31 ,   54 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   18 ,   44 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(    5 ,   34 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(   -8 ,   24 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -21 ,   14 );
		                        break;
		                default:
                            break;
          }
          break;

 case 445:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 425;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   48 ,   70 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   40 ,   65 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   32 ,   55 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   19 ,   45 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(    6 ,   35 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(   -7 ,   25 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -20 ,   15 );
		                        break;
		                default:
                            break;
          }
          break;

 case 446:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 426;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   49 ,   71 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   41 ,   66 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   33 ,   56 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   20 ,   46 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(    7 ,   36 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(   -6 ,   26 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -19 ,   16 );
		                        break;
		                default:
                            break;
          }
          break;

 case 447:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 427;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   50 ,   72 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   42 ,   67 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   34 ,   57 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   21 ,   47 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(    8 ,   37 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(   -5 ,   27 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -18 ,   17 );
		                        break;
		                default:
                            break;
          }
          break;

 case 448:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 428;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   51 ,   73 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   43 ,   68 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   35 ,   58 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   22 ,   48 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(    9 ,   38 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(   -4 ,   28 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -17 ,   18 );
		                        break;
		                default:
                            break;
          }
          break;

 case 449:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 429;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   52 ,   74 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   44 ,   69 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   36 ,   59 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   23 ,   49 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(   10 ,   39 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(   -3 ,   29 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -16 ,   19 );
		                        break;
		                default:
                            break;
          }
          break;

 case 450:   /* 右車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		              case 0xe0:cnt1 = 0;
                            pattern = 430;
                            break;	  
                  case 0xc0:ha( -27 );
				                    sp(   53 ,   75 );
		                        break;
				          case 0xc1:ha( -30 );
				                    sp(   45 ,   70 );
		                        break;
		              case 0x81:ha( -33 );
				                    sp(   37 ,   60 );
		                        break;
		              case 0x83:ha( -36 );
				                    sp(   24 ,   50 );
		                        break;
		              case 0x03:ha( -39 );
				                    sp(   11 ,   40 );
		                        break;
		              case 0x07:ha( -42 );
				                    sp(   -2 ,   30 );
		                        break;
		              case 0x06:ha( -45 );
				                    sp(  -15 ,   20 );
		                        break;
		                default:
                            break;
          }
          break;

 case 500:                           /* 左車線変更ライン検出時の処理 */
          led_out( 0x2 );
          ha(   0 );
          kyori_flug = 0;
	        cnt1 = 0;
		      pattern = 501;
          break;

 case 501:              /* 左車線変更ライン検出後の通過状況の確認処理 */
	        if( check_hidarikakunin() ) {  /* クロスライン再チェック   */
              cnt1 = 0;
	            pattern = 100;
              break;
          }
		      /* 右車線変更の通過時間計測[ms] 白線手前から白線終わりまで  */
                 if( kyori_flug == 0 && check_shasenhenkouh() == 1 ){
                     kyori_flug = 1;
	   	    } else if( kyori_flug == 1 && check_shasenhenkouh() == 0 ){
                     kyori_flug = 2;
		                 kyoritime = cnt1;
		      }
          if( kyori_flug == 2 ){
	            cnt1 = 0;
	            pattern = 502;
	            break;
		      }
          switch( sensor_inp( MASK4_4 ) ) {

		              case 0x18:ha(   0 );       /*   センタ → 直  進   */
                            break;                
                  case 0x1c:ha(   2 );       /* 小左寄り → 小右曲げ */
                            break;                
                  case 0x0c:ha(   4 );       /* 中左寄り → 中右曲げ */
                            break;                     
                  case 0x38:ha(  -2 );       /* 小右寄り → 小左曲げ */
                            break;                     
                  case 0x30:ha(  -4 );       /* 中右寄り → 中左曲げ */
                            break;                     
                    default:
                            break;
          }
		      break;

 case 502:                        /*左車線変更ライン通過後の減速処理 */
		      if( check_hidarikakunin() ) {  /* クロスラインチェック    */
              cnt1 = 0;
	            pattern = 100;
              break;
          }
		      kyori_flug = 0;
	        shasenhenkou();
		             if( kyoritime <=  12 ){
		                 pattern = 600;
                     break;
		      } else if( kyoritime <=  15 ){
		                 pattern = 601;
                     break;
		      } else if( kyoritime <=  18 ){
		                 pattern = 602;
                     break;
		      } else if( kyoritime <=  21 ){
		                 pattern = 603;
                     break;
		      } else if( kyoritime <=  24 ){
		                 pattern = 604;
                     break;
		      } else if( kyoritime <=  27 ){
		                 pattern = 605;
                     break;
		      } else if( kyoritime <=  30 ){
		                 pattern = 606;
                     break;
		      } else if( kyoritime <=  33 ){
		                 pattern = 607;
                     break;
		      } else if( kyoritime <=  36 ){
		                 pattern = 608;
                     break;
		      } else if( kyoritime <=  39 ){
		                 pattern = 609;
                     break;
		      } else                       {
		                 pattern = 610;
                     break;
		      }
		      switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;

 case 600:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   30 ,   75 );
			        cnt1 = 0;
			        pattern = 620;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -6 );
				                    break;
                  case 0x18:ha(  -5 );
				                    break;
               	  case 0x1c:ha(  -4 );
				                    break;
                  case 0x0c:ha(  -3 );
				                    break;
                  case 0x0e:ha(  -2 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 601:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   32 ,   76 );
			        cnt1 = 0;
			        pattern = 621;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 602:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   34 ,   77 );
			        cnt1 = 0;
			        pattern = 622;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 603:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   36 ,   78 );
			        cnt1 = 0;
			        pattern = 623;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 604:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   38 ,   79 );
			        cnt1 = 0;
			        pattern = 624;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 605:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   40 ,   80 );
			        cnt1 = 0;
			        pattern = 625;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 606:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   42 ,   81 );
			        cnt1 = 0;
			        pattern = 626;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 607:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   44 ,   82 );
			        cnt1 = 0;
			        pattern = 627;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 608:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   46 ,   83 );
			        cnt1 = 0;
			        pattern = 628;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 609:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   48 ,   84 );
			        cnt1 = 0;
			        pattern = 629;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 610:                      /* 左車線変更前の車体の左寄せ走行処理 */
		      shasenhenkou();
		      if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){   /* 左車線変更開始の判断 → 車線変更の開始 */
              kyori_flug = 0;
		          led_out( 0x2 );
			        ha( -16 );        /* ★左車線変更時のハンドルの曲げ角度 */
              sp(   50 ,   85 );
			        cnt1 = 0;
			        pattern = 630;
              break;
		      }
          }
	        switch( sensor_inp(MASK4_4) ) {

		              case 0x38:ha(  -3 );
				                    break;
                  case 0x18:ha(  -2 );
				                    break;
               	  case 0x1c:ha(  -2 );
				                    break;
                  case 0x0c:ha(  -2 );
				                    break;
                  case 0x0e:ha(  -1 );
				                    break;
                  case 0x06:ha(  -1 );
				                    break;
                  case 0x07:ha(   0 );
				                    break;
                  case 0x03:ha(   1 );
				                    break;
                  case 0x83:ha(   2 );
				                    break;
				          case 0x81:ha(   3 );
				                    break;
				          case 0xc1:ha(   4 );
				                    break;
				          case 0xc0:ha(   5 );
				                    break;
				            default:
                            break;
          }
          break;
		  
 case 620:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   30 ,   75 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 155 ) {
              cnt1 = 0;
			        pattern = 640;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 700;
              break;
		      }
		      break;

 case 621:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   32 ,   76 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 156 ) {
              cnt1 = 0;
			        pattern = 641;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 701;
              break;
		      }
		      break;

 case 622:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   34 ,   77 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 157 ) {
              cnt1 = 0;
			        pattern = 642;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 702;
              break;
		      }
		      break;

 case 623:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   36 ,   78 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 158 ) {
              cnt1 = 0;
			        pattern = 643;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 703;
              break;
		      }
		      break;

 case 624:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   38 ,   79 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 159 ) {
              cnt1 = 0;
			        pattern = 644;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 704;
              break;
		      }
		      break;

 case 625:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   40 ,   80 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 160 ) {
              cnt1 = 0;
			        pattern = 645;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 705;
              break;
		      }
		      break;

 case 626:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   42 ,   81 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 161 ) {
              cnt1 = 0;
			        pattern = 646;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 706;
              break;
		      }
		      break;

 case 627:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   44 ,   82 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 162 ) {
              cnt1 = 0;
			        pattern = 647;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 707;
              break;
		      }
		      break;

 case 628:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   46 ,   83 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 163 ) {
              cnt1 = 0;
		      	  pattern = 648;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 708;
              break;
		      }
		      break;

 case 629:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   48 ,   84 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 164 ) {
              cnt1 = 0;
			        pattern = 649;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 709;
              break;
		      }
		      break;

 case 630:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha( -16 );            /* ★左車線変更時のハンドルの曲げ角度 */
          sp(   50 ,   85 );
		      c = sensor_inp( MASK4_4 );
          if( cnt1 > 165 ) {
              cnt1 = 0;
			        pattern = 650;
              break;
		      }
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 710;
              break;
		      }
		      break;

 case 640:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   80 ,   80 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 700;
              break;
		      }
		      break;
		  
 case 641:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   81 ,   81 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 701;
              break;
		      }
		      break;
		  
 case 642:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   82 ,   82 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 702;
              break;
		      }
		      break;
		  
 case 643:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   83 ,   83 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 703;
              break;
		      }
		      break;
		  
 case 644:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   84 ,   84 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 704;
              break;
		      }
		      break;
		  
 case 645:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   85 ,   85 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 705;
              break;
		      }
		      break;
		  
 case 646:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   86 ,   86 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 706;
              break;
		      }
		      break;
		  
 case 647:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   87 ,   87 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 707;
              break;
		      }
		      break;
		  
 case 648:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   88 ,   88 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 708;
              break;
		      }
		      break;
		  
 case 649:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   89 ,   89 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 709;
              break;
		      }
		      break;
		  
 case 650:   /* 左車線変更ラインの通過時間による車線変更中のモータ設定 */
          ha(   2 );              /* 左車線変更中のハンドルの戻し角度 */
          sp(   90 ,   90 );
		      c = sensor_inp( MASK4_4 );
          if( c == ( 0x80 )||( 0x40 )||( 0xc0 ) ) {
              cnt1 = 0;
			        pattern = 710;
              break;
		      }
		      break;
		  
 case 700:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   90 ,   75 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   88 ,   72 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   86 ,   69 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   84 ,   66 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   81 ,   63 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   78 ,   59 );
		                        cnt1 = 0;
                            pattern = 720;
                            break;
				          case 0x06:ha(  19 );
				                    sp(   75 ,   55 );
		                        cnt1 = 0;
                            pattern = 720;
                            break;
		                default:
                            break;
          }
          break;
  
 case 701:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   91 ,   76 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   89 ,   73 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   87 ,   70 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   85 ,   67 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   82 ,   64 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   79 ,   60 );
		                        cnt1 = 0;
                            pattern = 721;
                            break;
				          case 0x06:ha(  19 );
				                    sp(   76 ,   56 );
		                        cnt1 = 0;
                            pattern = 721;
                            break;
		                default:
                            break;
          }
          break;
  
 case 702:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   92 ,   77 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   90 ,   74 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   88 ,   71 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   86 ,   68 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   83 ,   65 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   80 ,   61 );
		                        cnt1 = 0;
                            pattern = 722;
                            break;
			        	  case 0x06:ha(  19 );
				                    sp(   77 ,   57 );
		                        cnt1 = 0;
                            pattern = 722;
                            break;
		                default:
                            break;
          }
          break;
  
 case 703:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   93 ,   78 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   91 ,   75 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   89 ,   72 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   87 ,   69 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   84 ,   66 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   81 ,   62 );
		                        cnt1 = 0;
                            pattern = 723;
                            break;
				          case 0x06:ha(  19 );
				                    sp(   78 ,   58 );
		                        cnt1 = 0;
                            pattern = 723;
                            break;
		                default:
                            break;
          }
          break;
  
 case 704:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   94 ,   79 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   92 ,   76 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   90 ,   73 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   88 ,   70 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   85 ,   67 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   82 ,   63 );
		                        cnt1 = 0;
                            pattern = 724;
                            break;
				          case 0x06:ha(  19 );
				                    sp(   79 ,   59 );
		                        cnt1 = 0;
                            pattern = 724;
                            break;
		                default:
                            break;
          }
          break;
  
 case 705:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   95 ,   80 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   93 ,   77 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   91 ,   74 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   89 ,   71 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   86 ,   68 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   83 ,   64 );
		                        cnt1 = 0;
                            pattern = 725;
                            break;
				          case 0x06:ha(  19 );
				                    sp(   80 ,   60 );
		                        cnt1 = 0;
                            pattern = 725;
                            break;
		                default:
                            break;
          }
          break;
  
 case 706:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   96 ,   81 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   94 ,   78 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   92 ,   75 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   90 ,   72 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   87 ,   69 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   84 ,   65 );
		                        cnt1 = 0;
                            pattern = 726;
                            break;
				          case 0x06:ha(  19 );
				                    sp(   81 ,   61 );
		                        cnt1 = 0;
                            pattern = 726;
                            break;
		                default:
                            break;
          }
          break;
  
 case 707:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   97 ,   82 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   95 ,   79 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   93 ,   76 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   91 ,   73 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   88 ,   70 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   85 ,   66 );
		                        cnt1 = 0;
                            pattern = 727;
                            break;
			        	  case 0x06:ha(  19 );
				                    sp(   82 ,   62 );
		                        cnt1 = 0;
                            pattern = 727;
                            break;
		                default:
                            break;
          }
          break;
  
 case 708:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   98 ,   83 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   96 ,   80 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   94 ,   77 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   92 ,   74 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   89 ,   71 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   86 ,   67 );
		                        cnt1 = 0;
                            pattern = 728;
                            break;
				          case 0x06:ha(  19 );
				                    sp(   83 ,   63 );
		                        cnt1 = 0;
                            pattern = 728;
                            break;
		                default:
                            break;
          }
          break;
  
 case 709:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(   99 ,   84 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   97 ,   81 );
		                        break;
				          case 0x18:ha(   7 );
				                    sp(   95 ,   78 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   93 ,   75 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   90 ,   72 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   87 ,   68 );
		                        cnt1 = 0;
                            pattern = 729;
                            break;
				          case 0x06:ha(  19 );
				                    sp(   84 ,   64 );
		                        cnt1 = 0;
                            pattern = 729;
                            break;
		                default:
                            break;
          }
          break;
  
 case 710:   /* 左車線変更ラインの通過時間による車線変更の終了移行処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x30:ha(   3 );
				                    sp(  100 ,   85 );
		                        break;
				          case 0x38:ha(   5 );
				                    sp(   98 ,   82 );
		                        break;
			        	  case 0x18:ha(   7 );
				                    sp(   96 ,   79 );
		                        break;
				          case 0x1c:ha(  10 );
                            sp(   94 ,   76 );
		                        break;
				          case 0x0c:ha(  13 );
                            sp(   91 ,   73 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   88 ,   69 );
		                        cnt1 = 0;
                            pattern = 730;
                            break;
				          case 0x06:ha(  19 );
				                    sp(   85 ,   65 );
		                        cnt1 = 0;
                            pattern = 730;
                            break;
		                default:
                            break;
          }
          break;
  
 case 720:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   81 ,   63 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   78 ,   59 );
		                        break;
				          case 0x06:ha(  19 );
				                    sp(   75 ,   55 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   70 ,   49 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   65 ,   43 );
		                        cnt1 = 0;
                            pattern = 740;
                            break;
		                default:
                            break;
          }
          break;

 case 721:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   82 ,   64 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   79 ,   60 );
		                        break;
				          case 0x06:ha(  19 );
				                    sp(   76 ,   56 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   71 ,   50 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   66 ,   44 );
		                        cnt1 = 0;
                            pattern = 741;
                            break;
		                default:
                            break;
          }
          break;

 case 722:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   83 ,   65 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   80 ,   61 );
		                        break;
				          case 0x06:ha(  19 );
				                    sp(   77 ,   57 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   72 ,   51 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   67 ,   45 );
		                        cnt1 = 0;
                            pattern = 742;
                            break;
		                default:
                            break;
          }
          break;

 case 723:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   84 ,   66 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   81 ,   62 );
		                        break;
			        	  case 0x06:ha(  19 );
				                    sp(   78 ,   58 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   73 ,   52 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   68 ,   46 );
		                        cnt1 = 0;
                            pattern = 743;
                            break;
		                default:
                            break;
          }
          break;

 case 724:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   85 ,   67 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   82 ,   63 );
		                        break;
				          case 0x06:ha(  19 );
				                    sp(   79 ,   59 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   74 ,   53 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   69 ,   47 );
		                        cnt1 = 0;
                            pattern = 744;
                            break;
		                default:
                            break;
          }
          break;

 case 725:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   86 ,   68 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   83 ,   64 );
		                        break;
				          case 0x06:ha(  19 );
				                    sp(   80 ,   60 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   75 ,   54 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   70 ,   48 );
		                        cnt1 = 0;
                            pattern = 745;
                            break;
		                default:
                            break;
          }
          break;

 case 726:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   87 ,   69 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   84 ,   65 );
		                        break;
				          case 0x06:ha(  19 );
				                    sp(   81 ,   61 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   76 ,   55 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   71 ,   49 );
		                        cnt1 = 0;
                            pattern = 746;
                            break;
		                default:
                            break;
          }
          break;

 case 727:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   88 ,   70 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   85 ,   66 );
		                        break;
			        	  case 0x06:ha(  19 );
				                    sp(   82 ,   62 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   77 ,   56 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   72 ,   50 );
		                        cnt1 = 0;
                            pattern = 747;
                            break;
		                default:
                            break;
          }
          break;

 case 728:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   89 ,   71 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   86 ,   67 );
		                        break;
				          case 0x06:ha(  19 );
				                    sp(   83 ,   63 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   78 ,   57 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   73 ,   51 );
		                        cnt1 = 0;
                            pattern = 748;
                            break;
		                default:
                            break;
          }
          break;

 case 729:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   90 ,   72 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   87 ,   68 );
		                        break;
				          case 0x06:ha(  19 );
				                    sp(   84 ,   64 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   79 ,   58 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   74 ,   52 );
		                        cnt1 = 0;
                            pattern = 749;
                            break;
		                default:
                            break;
          }
          break;

 case 730:              /* 左車線変更の終了処理とコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		              case 0x38:          
                  case 0x18:
		              case 0x1c:led_out( 0x0 );
		                        cnt1 = 0;
                            pattern = 20;
                            break;
			            case 0x0c:ha(  13 );
				                    sp(   91 ,   73 );
		                        break;
				          case 0x0e:ha(  16 );
				                    sp(   88 ,   69 );
		                        break;
				          case 0x06:ha(  19 );
				                    sp(   85 ,   65 );
		                        break;
		              case 0x07:ha(  23 );
				                    sp(   80 ,   59 );
		                        break;
		              case 0x03:ha(  27 );
				                    sp(   75 ,   53 );
		                        cnt1 = 0;
                            pattern = 750;
                            break;
		                default:
                            break;
          }
          break;

 case 740:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 720;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   65 ,   43 );
		                        break;
				          case 0x83:ha(  30 );
				                    sp(   60 ,   35 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   50 ,   27 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   40 ,   14 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   30 ,    1 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   20 ,  -12 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   10 ,  -25 );
		                        break;
		                default:
                            break;
          }
          break;

 case 741:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 721;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   66 ,   44 );
		                        break;
				          case 0x83:ha(  30 );
				                    sp(   61 ,   36 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   51 ,   28 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   41 ,   15 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   31 ,    2 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   21 ,  -11 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   11 ,  -24 );
		                        break;
		                default:
                            break;
          }
          break;

 case 742:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 722;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   67 ,   45 );
		                        break;
			        	  case 0x83:ha(  30 );
				                    sp(   62 ,   37 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   52 ,   29 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   42 ,   16 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   32 ,    3 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   22 ,  -10 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   12 ,  -23 );
		                        break;
		                default:
                            break;
          }
          break;

 case 743:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 723;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   68 ,   47 );
		                        break;
				          case 0x83:ha(  30 );
				                    sp(   63 ,   39 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   53 ,   30 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   43 ,   17 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   33 ,    4 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   23 ,   -9 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   13 ,  -22 );
		                        break;
		                default:
                            break;
          }
          break;

 case 744:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 724;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   69 ,   48 );
		                        break;
				          case 0x83:ha(  30 );
				                    sp(   64 ,   40 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   54 ,   31 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   44 ,   18 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   34 ,    5 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   24 ,   -8 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   14 ,  -21 );
		                        break;
		                default:
                            break;
          }
          break;

 case 745:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 725;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   70 ,   48 );
		                        break;
			        	  case 0x83:ha(  30 );
				                    sp(   65 ,   40 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   55 ,   32 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   45 ,   19 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   35 ,    6 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   25 ,   -7 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   15 ,  -20 );
		                        break;
		                default:
                            break;
          }
          break;

 case 746:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 726;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   71 ,   49 );
		                        break;
				          case 0x83:ha(  30 );
				                    sp(   66 ,   41 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   56 ,   33 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   46 ,   20 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   36 ,    7 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   26 ,   -6 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   16 ,  -19 );
		                        break;
		                default:
                            break;
          }
          break;

 case 747:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 727;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   72 ,   50 );
		                        break;
				          case 0x83:ha(  30 );
				                    sp(   67 ,   42 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   57 ,   34 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   47 ,   21 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   37 ,    8 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   27 ,   -5 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   17 ,  -18 );
		                        break;
		                default:
                            break;
          }
          break;

 case 748:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 728;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   73 ,   51 );
		                        break;
			        	  case 0x83:ha(  30 );
				                    sp(   68 ,   43 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   58 ,   35 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   48 ,   22 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   38 ,    9 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   28 ,   -4 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   18 ,  -17 );
		                        break;
		                default:
                            break;
          }
          break;

 case 749:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 729;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   74 ,   52 );
		                        break;
				          case 0x83:ha(  30 );
				                    sp(   69 ,   44 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   59 ,   36 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   49 ,   23 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   39 ,   10 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   29 ,   -3 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   19 ,  -16 );
		                        break;
		                default:
                            break;
          }
          break;

 case 750:   /* 左車線変更終了時に外へ膨らんだ時のコースアウト回避処理 */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		              case 0x07:cnt1 = 0;
                            pattern = 730;
                            break;	  
                  case 0x03:ha(  27 );
				                    sp(   75 ,   53 );
		                        break;
				          case 0x83:ha(  30 );
				                    sp(   70 ,   45 );
		                        break;
		              case 0x81:ha(  33 );
				                    sp(   60 ,   37 );
		                        break;
		              case 0xc1:ha(  36 );
				                    sp(   50 ,   24 );
		                        break;
		              case 0xc0:ha(  39 );
				                    sp(   40 ,   11 );
		                        break;
		              case 0xe0:ha(  42 );
				                    sp(   30 ,   -2 );
		                        break;
		              case 0x60:ha(  45 );
				                    sp(   20 ,  -15 );
		                        break;
		                default:
                            break;
          }
          break;

 case 800:                       /* 予定走行時間の経過による通常停止 */
          sp(    0 ,    0 );
 	        if( cnt1 > 200 ) {                  /* ハンドルの追従処理 */
              cnt1 = 0;
              pattern = 801;
              break;
		      }
          switch( sensor_inp( MASK4_4 ) ) {

                  case 0x18:ha(   0 );          /*      直  進     */
                            break;
                  case 0x1c:ha(   2 );          /* 車体の右方向補正 */
                            break;
                  case 0x0c:ha(   7 );          /* 微程度の右曲がり */
                            break;
                  case 0x0e:ha(  11 );          /* 微程度の右曲がり */
                            break;
                  case 0x06:ha(  15 );          /* 小程度の右曲がり */
                            break;
                  case 0x07:ha(  18 );          /* 中程度の右曲がり */
                            break;
				          case 0x03:ha(  21 );          /* 大程度の右曲がり */
                            break;
          
                  case 0x38:ha(  -2 );          /* 車体の左方向補正 */
                            break;
				          case 0x30:ha(  -7 );          /* 微程度の左曲がり */
                            break;
				          case 0x70:ha( -11 );          /* 微程度の左曲がり */
                            break;
				          case 0x60:ha( -15 );          /* 小程度の左曲がり */
                            break;
                  case 0xe0:ha( -18 );          /* 中程度の左曲がり */
                            break;
                  case 0xc0:ha( -21 );          /* 大程度の左曲がり */
                            break;
                    default:
                            break;
          }
          break;

 case 801:                     /* 完走 or コースアウトによる完全停止 */
          sp(    0 ,    0 );
 	               if( cnt1 < 450 ) {          /* LEDの超低速点滅処理 */
                     led_out( 0x1 );
          } else if( cnt1 < 900 ) {
                     led_out( 0x2 );
          } else {
                     cnt1 = 0;
          }
          break;

 case 1001:                    /* 走行データのSDｶｰﾄﾞへの保存開始処理 */
          /* ★SDｶｰﾄﾞへの保存方法 → 走行後または走行中にﾓｰﾀﾄﾞﾗｲﾊﾞｰ基盤のプッシュスイッチを押す */
          R_GPT1->GTCR_b.CST = 0;          /* コースアウト&完走時にサーボモーターの固定解除  */
          log_st = LOG_CLOSE;
          sp(    0 ,    0 );
 	               if( cnt1 % 1000 < 100 ) {          /* LEDを細かく点滅させて終了を知らせる  */
                     led_out( 0x1 );
          } else {
                     led_out( 0x0 );
          }
          break;
}   //switch( pattern )処理範囲

  if( logging ) {
      const bool changed = before != pattern || sensor_before != previous_sensor ||
                           decision_sensor != previous_decision_sensor ||
                           prev.ang != log_raw.ang || prev.ml != log_raw.ml || prev.mr != log_raw.mr;
      previous_sensor = sensor_before;
      previous_decision_sensor = decision_sensor;
      if( changed || cnt_start % 10 == 0 ) {
          const int next = nextLogIndex(log_write);
          const bool closing = log_st == LOG_CLOSE;
          if( next == log_read || (!closing && nextLogIndex(next) == log_read) ) {
              ++log_dropped;
          } else {
              volatile LogRow &row = log_buff[log_write];
              row.time = cnt_start;
              row.before = before;
              row.sensor = sensor_before;
              row.prev.ang = prev.ang;
              row.prev.ml = prev.ml;
              row.prev.mr = prev.mr;
              row.after = pattern;
              row.timer_ms = timer_before; // 元の制御用cnt1。状態によってリセット位置が異なる
              row.marker_ms = kyoritime;   // 最後に測定したマーカー通過時間
              row.marker_phase = kyori_flug;
              row.white_ms = daturin_cnt;
              row.decision_sensor = decision_sensor;
              row.inner_seen = decision_sensor >= 0 && right_lane_inner_seen;
              row.center_ms = decision_sensor >= 0 ? right_lane_center_ms : 0;
              row.next.ang = log_raw.ang;
              row.next.ml = log_raw.ml;
              row.next.mr = log_raw.mr;
              row.stop_reason = stop_reason;
              row.dip = log_last_dip;
              row.dropped = log_dropped;
              log_write = next;
          }
      }
  }
}   //1ms ごとの割り込み処理範囲

//**********************************************************************
// センサ状態検出
// 引数　 マスク値
// 戻り値 センサ値
//**********************************************************************
unsigned char sensor_inp( unsigned char mask )
{
  unsigned char sensor;

  sensor  = (R_PORT0->PIDR_b.PIDR12) << 7;  // D35
  sensor |= (R_PORT0->PIDR_b.PIDR11) << 6;  // D36
  sensor |= (R_PORT0->PIDR_b.PIDR10) << 5;  // D37
  sensor |= (R_PORT0->PIDR_b.PIDR8 ) << 4;  // D38
  sensor |= (R_PORT0->PIDR_b.PIDR7 ) << 3;  // D39
  sensor |= (R_PORT0->PIDR_b.PIDR6 ) << 2;  // D40
  sensor |= (R_PORT0->PIDR_b.PIDR5 ) << 1;  // D41
  sensor |= (R_PORT0->PIDR_b.PIDR4 ) << 0;  // D42

  sensor = (~sensor) & mask;

  return sensor;
}

//**********************************************************************
// クロスライン検出処理
// 戻り値 0:クロスラインなし 1:あり
//**********************************************************************
int check_crossline( void )
{
  int ret = 0;
  unsigned char b = sensor_inp(MASK3_3);
  if( b==0xe7 || b==0xe5 || b==0xe3 || b==0xe6 ||
      b==0x67 || b==0x65 || b==0x63 || b==0x66 ||
      b==0xc7 || b==0xc5 || b==0xc3 || b==0xc6 ||
      b==0xa7 || b==0xa5 || b==0xa3 || b==0xa6 ) {
      ret = 1;
  }
  return ret;
}

//**********************************************************************
// 右ハーフライン検出処理
// 戻り値 0:なし 1:あり
//**********************************************************************
int check_shasenhenkoum( void )
{
  int ret = 0;
  unsigned char m = sensor_inp(MASK4_4);
  if( m==0x0f || m==0x1f || m==0x1d || m==0x1b || m==0x17 ||
		  m==0x3f || m==0x3d || m==0x3b || m==0x3e || m==0x37 ) {
      ret = 1;
  }
  return ret;
}

//**********************************************************************
// 右ハーフライン検出後のクロスライン検出確認処理     
// 戻り値 0:なし 1:あり                            
//**********************************************************************
int check_migikakunin( void )
{
  int ret = 0;
  unsigned char m = sensor_inp( MASK3_0 );
  if( m==0xe0 || m==0xc0 || m==0x60 || m==0xa0 || m==0x80 || m==0x40 ) {
      ret = 1;
  }
  return ret;
}

//**********************************************************************
// 左ハーフライン検出処理
// 戻り値 0:なし 1:あり
//**********************************************************************
int check_shasenhenkouh( void )
{
  int ret = 0;
  unsigned char h = sensor_inp(MASK4_4);
  if( h==0xf0 || h==0xf8 || h==0xb8 || h==0xd8 || h==0xe8 ||
		  h==0xfc || h==0xbc || h==0xdc || h==0xec || h==0x7c ) {
      ret = 1;
  }
  return ret;
}

//**********************************************************************
// 左ハーフライン検出後のクロスライン検出確認処理
// 戻り値 0:なし 1:あり                         
//**********************************************************************
int check_hidarikakunin( void )
{
  int ret = 0;
  unsigned char h = sensor_inp( MASK0_3 );
  if( h==0x07 || h==0x03 || h==0x06 || h==0x05 || h==0x01 || h==0x02 ) {
      ret = 1;
  }
  return ret;
}

//**********************************************************************
// ディップスイッチ値読み込み
// 戻り値 スイッチ値 0～3
//**********************************************************************
unsigned char dipsw_get( void )
{
  unsigned char sw = ( !(R_PORT3->PIDR_b.PIDR6) << 1 ) + ( !(R_PORT3->PIDR_b.PIDR7) );

  return sw;
}

//**********************************************************************
// プッシュスイッチ値読み込み
// 戻り値 スイッチ値 ON:1 OFF:0
//**********************************************************************
unsigned char pushsw_get( void )
{
  unsigned char sw = !(R_PORT0->PIDR_b.PIDR3);

  return sw;
}

//**********************************************************************
// スタートバー検出センサ読み込み
// 戻り値 センサ値 ON(バーあり):1 OFF(なし):0
//**********************************************************************
unsigned char startbar_get( void )
{
  unsigned char b = !(R_PORT0->PIDR_b.PIDR4);

  return b;
}

//**********************************************************************
// LED制御
// 引数　スイッチ値 LED2:bit1 LED3:bit0  "0":消灯 "1":点灯
// 例)0x3→LED2:ON LED3:ON  0x2→LED2:ON LED3:OFF
//**********************************************************************
void led_out( unsigned char led )
{
  R_PORT4->PODR_b.PODR2 = !(led & 0x02);
  R_PORT4->PODR_b.PODR5 = !(led & 0x01);
}

//**********************************************************************
// LED制御(マイコンボードのD13)
// 引数　"0":消灯 "1":点灯
//**********************************************************************
void led_out_m( unsigned char led )
{
  R_PORT1->PODR_b.PODR11 = !led;
}

//**********************************************************************
// モータ速度制御
// 引数　 左モータ:-100～100、右モータ:-100～100
//        0で停止、100で正転100%、-100で逆転100%
// 戻り値 なし
//**********************************************************************
void sp( int accele_l, int accele_r )
{
  int sw_data = dipsw_get() + 7;         // 7～10
  log_last_dip = sw_data - 7; // このモータ指令に適用した実際のDIP値
  accele_l = accele_l * sw_data / 10;
  accele_r = accele_r * sw_data / 10;

  log_raw.ml = accele_l;
  log_raw.mr = accele_r;

  // 左モータ制御
  if( accele_l >= 0 ) {
    R_PORT4->PODR_b.PODR0 = 0;
    accele_l = (long)(PWM_CYCLE_M + 1) * accele_l / 100;
    GTIOC6B = accele_l;
  } else {
    R_PORT4->PODR_b.PODR0 = 1;
    accele_l = (long)(PWM_CYCLE_M + 1) * (-accele_l) / 100;
    GTIOC6B = accele_l;
  }

  // 右モータ制御
  if( accele_r >= 0 ) {
    R_PORT4->PODR_b.PODR4 = 0;
    accele_r = (long)(PWM_CYCLE_M + 1) * accele_r / 100;
    GTIOC3A = accele_r;
  } else {
    R_PORT4->PODR_b.PODR4 = 1;
    accele_r = (long)(PWM_CYCLE_M + 1) * (-accele_r) / 100;
    GTIOC3A = accele_r;
  }
}

//**********************************************************************
// サーボハンドル操作
// 引数　 サーボ操作角度：-90～90
//        -90で左へ90度、0でまっすぐ、90で右へ90度回転
//**********************************************************************
void ha( int angle )
{
  log_raw.ang = angle;
  // サーボが左右逆に動く場合は、「-」を「+」に替えてください
  GTIOC1B = SERVO_CENTER - angle * HANDLE_STEP;
}

//**********************************************************************
// microSD.close時に、ファイルの日付・時刻を設定
//**********************************************************************
void set_microSD_dateTime( uint16_t* date, uint16_t* time )
{
  // FAT_DATEマクロでフィールドを埋めて日付を返す
  *date = FAT_DATE( (uint16_t)getCompileYear( C_DATE ),
                    (uint8_t)getCompileMonth( C_DATE ),
                    (uint8_t)getCompileDay( C_DATE ) );

  // FAT_TIMEマクロでフィールドを埋めて時間を返す
  *time = FAT_TIME( getCompileHour( C_TIME ),
                    getCompilerMinute( C_TIME ),
                    getCompilerSecond( C_TIME ) );
}

//**********************************************************************
// char型データの値をlong型変数に2進数で変換
// 引数　 unsigned char 変換元の8bitデータ
// 戻り値 unsigned long 変換先の変数(0～11111111) ※0か1しかありません
//**********************************************************************
unsigned long convertBCD_CharToLong( unsigned char hex )
{
  unsigned long ret = 0;

  for( int i=0; i<8; i++ ) {
    ret *= 10;
    if( (hex & 0x80) != 0 ) {
      ret += 1;
    }
    hex <<= 1;
  }

  return ret;
}

//**********************************************************************
// モジュール名 getCompileYear
// 処理概要     コンパイルした時の年を取得
// 引数　       なし
// 戻り値       年
//**********************************************************************
int getCompileYear( const char *p )
{
  int i = atoi( p + 7 );

  if( i < 1980 || i > 2107 ) {
    i = 2023;
  }

  return i;
}

//**********************************************************************
// モジュール名 getCompileMonth
// 処理概要     コンパイルした時の月を取得
// 引数　       なし
// 戻り値       月
//**********************************************************************
int getCompileMonth( const char *p )
{
  const char monthStr[] = { "JanFebMarAprMayJunJulAugSepOctNovDec" };

  for( int i=0; i<12; i++ ) {
    int r = strncmp( monthStr + i * 3, p, 3 );
    if( r == 0 ) {
      return i + 1;
    }
  }

  return 1;
}

//**********************************************************************
// モジュール名 getCompileDay
// 処理概要     コンパイルした時の日を取得
// 引数　       なし
// 戻り値       日
//**********************************************************************
int getCompileDay( const char *p )
{
  int i = atoi( p + 4 );

  if( i < 1 || i > 31 ) {
    i = 1;
  }

  return i;
}

//**********************************************************************
// モジュール名 getCompileHour
// 処理概要     コンパイルした時の時を取得
// 引数　       なし
// 戻り値       時
//**********************************************************************
int getCompileHour( const char *p )
{
  int i = atoi( p );

  if( i < 0 || i > 23 ) {
    i = 0;
  }

  return i;
}

//**********************************************************************
// モジュール名 getCompilerMinute
// 処理概要     コンパイルした時の分を取得
// 引数　       なし
// 戻り値       分
//**********************************************************************
int getCompilerMinute( const char *p )
{
  int i = atoi( p + 3 );

  if( i < 0 || i > 59 ) {
    i = 0;
  }

  return i;
}

//**********************************************************************
// モジュール名 getCompilerSecond
// 処理概要     コンパイルした時の秒を取得
// 引数　       なし
// 戻り値       秒
//**********************************************************************
int getCompilerSecond( const char *p )
{
  int i = atoi( p + 6 );

  if( i < 0 || i > 59 ) {
    i = 0;
  }

  return i;
}

//**********************************************************************
// 各距離のクランク処理の集合	
// 処理パターンについては先頭のグローバル変数の宣言にて行う
//**********************************************************************
void newcrank( void )
{
         if( kyoritime <=  11 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  18 ){sp(  100 ,  100 );}
    else if(cnt1 <  59 ){sp( CRANK_FAST_PEAK_BRAKE , CRANK_FAST_PEAK_BRAKE );}
    else if(cnt1 <  75 ){sp(  -30 ,  -30 );}
    else if(cnt1 <  91 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 107 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 123 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 139 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 171 ){sp(    0 ,    0 );}
    else if(cnt1 < 187 ){sp(    5 ,    5 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
                   else {sp( CRANK_FAST_APPROACH_POWER , CRANK_FAST_APPROACH_POWER );}
    }
    else if( kyoritime <=  12 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  24 ){sp(  100 ,  100 );}
    else if(cnt1 <  75 ){sp(  -30 ,  -30 );}
    else if(cnt1 <  91 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 107 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 123 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 139 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 171 ){sp(    0 ,    0 );}
    else if(cnt1 < 187 ){sp(    5 ,    5 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
                   else {sp( CRANK_FAST_APPROACH_POWER , CRANK_FAST_APPROACH_POWER );}
    }
    else if( kyoritime <=  13 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  30 ){sp(  100 ,  100 );}
    else if(cnt1 <  91 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 107 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 123 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 139 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 171 ){sp(    0 ,    0 );}
    else if(cnt1 < 187 ){sp(    5 ,    5 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
                   else {sp( CRANK_FAST_APPROACH_POWER , CRANK_FAST_APPROACH_POWER );}
    }
    else if( kyoritime <=  14 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  36 ){sp(  100 ,  100 );}
    else if(cnt1 < 107 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 123 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 139 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 171 ){sp(    0 ,    0 );}
    else if(cnt1 < 187 ){sp(    5 ,    5 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
                   else {sp( CRANK_FAST_APPROACH_POWER , CRANK_FAST_APPROACH_POWER );}
    }
    else if( kyoritime <=  15 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  42 ){sp(  100 ,  100 );}
    else if(cnt1 < 123 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 139 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 171 ){sp(    0 ,    0 );}
    else if(cnt1 < 187 ){sp(    5 ,    5 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
                   else {sp( CRANK_FAST_APPROACH_POWER , CRANK_FAST_APPROACH_POWER );}
    }
    else if( kyoritime <=  16 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  48 ){sp(  100 ,  100 );}
    else if(cnt1 < 139 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 171 ){sp(    0 ,    0 );}
    else if(cnt1 < 187 ){sp(    5 ,    5 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  17 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  54 ){sp(  100 ,  100 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 171 ){sp(    0 ,    0 );}
    else if(cnt1 < 187 ){sp(    5 ,    5 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  18 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  60 ){sp(  100 ,  100 );}
    else if(cnt1 < 171 ){sp(    0 ,    0 );}
    else if(cnt1 < 187 ){sp(    5 ,    5 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  19 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  66 ){sp(  100 ,  100 );}
    else if(cnt1 < 187 ){sp(    5 ,    5 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  20 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  72 ){sp(  100 ,  100 );}
    else if(cnt1 < 203 ){sp(   10 ,   10 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  21 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  78 ){sp(  100 ,  100 );}
    else if(cnt1 < 219 ){sp(   15 ,   15 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  22 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  84 ){sp(  100 ,  100 );}
    else if(cnt1 < 235 ){sp(   20 ,   20 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  23 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  90 ){sp(  100 ,  100 );}
    else if(cnt1 < 251 ){sp(   25 ,   25 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  24 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  96 ){sp(  100 ,  100 );}
    else if(cnt1 < 267 ){sp(   30 ,   30 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  25 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 102 ){sp(  100 ,  100 );}
    else if(cnt1 < 283 ){sp(   35 ,   35 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  26 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 108 ){sp(  100 ,  100 );}
    else if(cnt1 < 299 ){sp(   41 ,   41 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  27 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 114 ){sp(  100 ,  100 );}
    else if(cnt1 < 315 ){sp(   47 ,   47 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  28 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 120 ){sp(  100 ,  100 );}
    else if(cnt1 < 331 ){sp(   53 ,   53 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  29 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 126 ){sp(  100 ,  100 );}
    else if(cnt1 < 347 ){sp(   59 ,   59 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  30 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 132 ){sp(  100 ,  100 );}
    else if(cnt1 < 363 ){sp(   65 ,   65 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  31 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 138 ){sp(  100 ,  100 );}
    else if(cnt1 < 379 ){sp(   71 ,   71 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  32 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 144 ){sp(  100 ,  100 );}
    else if(cnt1 < 395 ){sp(   77 ,   77 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  33 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 150 ){sp(  100 ,  100 );}
    else if(cnt1 < 411 ){sp(   83 ,   83 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  34 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 < 156 ){sp(  100 ,  100 );}
    else if(cnt1 < 427 ){sp(   89 ,   89 );}
                   else {sp(   95 ,   95 );}
    }
    else                       {     /* クロスライン通過後の減速制御 */
         if(cnt1 < 162 ){sp(  100 ,  100 );}
                   else {sp(   95 ,   95 );}
	}
}

//**********************************************************************
// 各距離の車線変更処理の集合
// 処理パターンについては先頭のグローバル変数の宣言にて行う
//**********************************************************************
void shasenhenkou( void )
{
    if( RIGHT_LANE_HOLD_POWER > 0 ) {  /* 速さに関係なく、ブレーキの後は一定の出力で中心線の切れ目まで走る */
             if( cnt1 < RIGHT_LANE_FULL_MS ){sp( 100 , 100 );}
        else if( cnt1 < RIGHT_LANE_FULL_MS + RIGHT_LANE_BRAKE_MS ){sp( RIGHT_LANE_BRAKE_POWER , RIGHT_LANE_BRAKE_POWER );}
        else {sp( RIGHT_LANE_HOLD_POWER , RIGHT_LANE_HOLD_POWER );}
        return;
    }
         if( kyoritime <=   5 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 <  75 ){sp(  100 ,  100 );}
    else if(cnt1 <  81 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  87 ){sp(    0 ,    0 );}
    else if(cnt1 <  93 ){sp(    5 ,    5 );}
    else if(cnt1 <  99 ){sp(   10 ,   10 );}
    else if(cnt1 < 105 ){sp(   15 ,   15 );}
    else if(cnt1 < 111 ){sp(   20 ,   20 );}
    else if(cnt1 < 117 ){sp(   25 ,   25 );}
    else if(cnt1 < 123 ){sp(   30 ,   30 );}
    else if(cnt1 < 129 ){sp(   35 ,   35 );}
    else if(cnt1 < 135 ){sp(   40 ,   40 );}
    else if(cnt1 < 141 ){sp(   45 ,   45 );}
    else if(cnt1 < 147 ){sp(   50 ,   50 );}
    else if(cnt1 < 153 ){sp(   55 ,   55 );}
    else if(cnt1 < 159 ){sp(   60 ,   60 );}
    else if(cnt1 < 165 ){sp(   65 ,   65 );}
    else if(cnt1 < 171 ){sp(   70 ,   70 );}
    else if(cnt1 < 177 ){sp(   75 ,   75 );}
    else if(cnt1 < 183 ){sp(   79 ,   79 );}
    else if(cnt1 < 189 ){sp(   83 ,   83 );}
    else if(cnt1 < 195 ){sp(   87 ,   87 );}
    else if(cnt1 < 201 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=   6 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 <  80 ){sp(  100 ,  100 );}
    else if(cnt1 <  86 ){sp(    0 ,    0 );}
    else if(cnt1 <  92 ){sp(    5 ,    5 );}
    else if(cnt1 <  98 ){sp(   10 ,   10 );}
    else if(cnt1 < 104 ){sp(   15 ,   15 );}
    else if(cnt1 < 110 ){sp(   20 ,   20 );}
    else if(cnt1 < 116 ){sp(   25 ,   25 );}
    else if(cnt1 < 122 ){sp(   30 ,   30 );}
    else if(cnt1 < 128 ){sp(   35 ,   35 );}
    else if(cnt1 < 134 ){sp(   40 ,   40 );}
    else if(cnt1 < 140 ){sp(   45 ,   45 );}
    else if(cnt1 < 146 ){sp(   50 ,   50 );}
    else if(cnt1 < 152 ){sp(   55 ,   55 );}
    else if(cnt1 < 158 ){sp(   60 ,   60 );}
    else if(cnt1 < 164 ){sp(   65 ,   65 );}
    else if(cnt1 < 170 ){sp(   70 ,   70 );}
    else if(cnt1 < 176 ){sp(   75 ,   75 );}
    else if(cnt1 < 182 ){sp(   79 ,   79 );}
    else if(cnt1 < 188 ){sp(   83 ,   83 );}
    else if(cnt1 < 194 ){sp(   87 ,   87 );}
    else if(cnt1 < 200 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=   7 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 <  86 ){sp(  100 ,  100 );}
    else if(cnt1 <  92 ){sp(    5 ,    5 );}
    else if(cnt1 <  98 ){sp(   10 ,   10 );}
    else if(cnt1 < 104 ){sp(   15 ,   15 );}
    else if(cnt1 < 110 ){sp(   20 ,   20 );}
    else if(cnt1 < 116 ){sp(   25 ,   25 );}
    else if(cnt1 < 122 ){sp(   30 ,   30 );}
    else if(cnt1 < 128 ){sp(   35 ,   35 );}
    else if(cnt1 < 134 ){sp(   40 ,   40 );}
    else if(cnt1 < 140 ){sp(   45 ,   45 );}
    else if(cnt1 < 146 ){sp(   50 ,   50 );}
    else if(cnt1 < 152 ){sp(   55 ,   55 );}
    else if(cnt1 < 158 ){sp(   60 ,   60 );}
    else if(cnt1 < 164 ){sp(   65 ,   65 );}
    else if(cnt1 < 170 ){sp(   70 ,   70 );}
    else if(cnt1 < 176 ){sp(   75 ,   75 );}
    else if(cnt1 < 182 ){sp(   79 ,   79 );}
    else if(cnt1 < 188 ){sp(   83 ,   83 );}
    else if(cnt1 < 194 ){sp(   87 ,   87 );}
    else if(cnt1 < 200 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=   8 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 <  93 ){sp(  100 ,  100 );}
    else if(cnt1 <  99 ){sp(   10 ,   10 );}
    else if(cnt1 < 105 ){sp(   15 ,   15 );}
    else if(cnt1 < 111 ){sp(   20 ,   20 );}
    else if(cnt1 < 117 ){sp(   25 ,   25 );}
    else if(cnt1 < 123 ){sp(   30 ,   30 );}
    else if(cnt1 < 129 ){sp(   35 ,   35 );}
    else if(cnt1 < 135 ){sp(   40 ,   40 );}
    else if(cnt1 < 141 ){sp(   45 ,   45 );}
    else if(cnt1 < 147 ){sp(   50 ,   50 );}
    else if(cnt1 < 153 ){sp(   55 ,   55 );}
    else if(cnt1 < 159 ){sp(   60 ,   60 );}
    else if(cnt1 < 165 ){sp(   65 ,   65 );}
    else if(cnt1 < 171 ){sp(   70 ,   70 );}
    else if(cnt1 < 177 ){sp(   75 ,   75 );}
    else if(cnt1 < 183 ){sp(   79 ,   79 );}
    else if(cnt1 < 189 ){sp(   83 ,   83 );}
    else if(cnt1 < 195 ){sp(   87 ,   87 );}
    else if(cnt1 < 201 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=   9 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 101 ){sp(  100 ,  100 );}
    else if(cnt1 < 107 ){sp(   15 ,   15 );}
    else if(cnt1 < 113 ){sp(   20 ,   20 );}
    else if(cnt1 < 119 ){sp(   25 ,   25 );}
    else if(cnt1 < 125 ){sp(   30 ,   30 );}
    else if(cnt1 < 131 ){sp(   35 ,   35 );}
    else if(cnt1 < 137 ){sp(   40 ,   40 );}
    else if(cnt1 < 143 ){sp(   45 ,   45 );}
    else if(cnt1 < 149 ){sp(   50 ,   50 );}
    else if(cnt1 < 155 ){sp(   55 ,   55 );}
    else if(cnt1 < 161 ){sp(   60 ,   60 );}
    else if(cnt1 < 167 ){sp(   65 ,   65 );}
    else if(cnt1 < 173 ){sp(   70 ,   70 );}
    else if(cnt1 < 179 ){sp(   75 ,   75 );}
    else if(cnt1 < 185 ){sp(   79 ,   79 );}
    else if(cnt1 < 191 ){sp(   83 ,   83 );}
    else if(cnt1 < 197 ){sp(   87 ,   87 );}
    else if(cnt1 < 203 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  10 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 110 ){sp(  100 ,  100 );}
    else if(cnt1 < 116 ){sp(   20 ,   20 );}
    else if(cnt1 < 122 ){sp(   25 ,   25 );}
    else if(cnt1 < 128 ){sp(   30 ,   30 );}
    else if(cnt1 < 134 ){sp(   35 ,   35 );}
    else if(cnt1 < 140 ){sp(   40 ,   40 );}
    else if(cnt1 < 146 ){sp(   45 ,   45 );}
    else if(cnt1 < 152 ){sp(   50 ,   50 );}
    else if(cnt1 < 158 ){sp(   55 ,   55 );}
    else if(cnt1 < 164 ){sp(   60 ,   60 );}
    else if(cnt1 < 170 ){sp(   65 ,   65 );}
    else if(cnt1 < 176 ){sp(   70 ,   70 );}
    else if(cnt1 < 182 ){sp(   75 ,   75 );}
    else if(cnt1 < 188 ){sp(   79 ,   79 );}
    else if(cnt1 < 194 ){sp(   83 ,   83 );}
    else if(cnt1 < 200 ){sp(   87 ,   87 );}
    else if(cnt1 < 206 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  11 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 120 ){sp(  100 ,  100 );}
    else if(cnt1 < 126 ){sp(   25 ,   25 );}
    else if(cnt1 < 132 ){sp(   30 ,   30 );}
    else if(cnt1 < 138 ){sp(   35 ,   35 );}
    else if(cnt1 < 144 ){sp(   40 ,   40 );}
    else if(cnt1 < 150 ){sp(   45 ,   45 );}
    else if(cnt1 < 156 ){sp(   50 ,   50 );}
    else if(cnt1 < 162 ){sp(   55 ,   55 );}
    else if(cnt1 < 168 ){sp(   60 ,   60 );}
    else if(cnt1 < 174 ){sp(   65 ,   65 );}
    else if(cnt1 < 180 ){sp(   70 ,   70 );}
    else if(cnt1 < 186 ){sp(   75 ,   75 );}
    else if(cnt1 < 192 ){sp(   79 ,   79 );}
    else if(cnt1 < 198 ){sp(   83 ,   83 );}
    else if(cnt1 < 204 ){sp(   87 ,   87 );}
    else if(cnt1 < 210 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  12 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 131 ){sp(  100 ,  100 );}
    else if(cnt1 < 137 ){sp(   30 ,   30 );}
    else if(cnt1 < 143 ){sp(   35 ,   35 );}
    else if(cnt1 < 149 ){sp(   40 ,   40 );}
    else if(cnt1 < 155 ){sp(   45 ,   45 );}
    else if(cnt1 < 161 ){sp(   50 ,   50 );}
    else if(cnt1 < 167 ){sp(   55 ,   55 );}
    else if(cnt1 < 173 ){sp(   60 ,   60 );}
    else if(cnt1 < 179 ){sp(   65 ,   65 );}
    else if(cnt1 < 185 ){sp(   70 ,   70 );}
    else if(cnt1 < 191 ){sp(   75 ,   75 );}
    else if(cnt1 < 197 ){sp(   79 ,   79 );}
    else if(cnt1 < 203 ){sp(   83 ,   83 );}
    else if(cnt1 < 209 ){sp(   87 ,   87 );}
    else if(cnt1 < 215 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  13 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 143 ){sp(  100 ,  100 );}
    else if(cnt1 < 149 ){sp(   35 ,   35 );}
    else if(cnt1 < 155 ){sp(   40 ,   40 );}
    else if(cnt1 < 161 ){sp(   45 ,   45 );}
    else if(cnt1 < 167 ){sp(   50 ,   50 );}
    else if(cnt1 < 173 ){sp(   55 ,   55 );}
    else if(cnt1 < 179 ){sp(   60 ,   60 );}
    else if(cnt1 < 185 ){sp(   65 ,   65 );}
    else if(cnt1 < 191 ){sp(   70 ,   70 );}
    else if(cnt1 < 197 ){sp(   75 ,   75 );}
    else if(cnt1 < 203 ){sp(   79 ,   79 );}
    else if(cnt1 < 209 ){sp(   83 ,   83 );}
    else if(cnt1 < 215 ){sp(   87 ,   87 );}
    else if(cnt1 < 221 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  14 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 156 ){sp(  100 ,  100 );}
    else if(cnt1 < 162 ){sp(   40 ,   40 );}
    else if(cnt1 < 168 ){sp(   45 ,   45 );}
    else if(cnt1 < 174 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   55 ,   55 );}
    else if(cnt1 < 186 ){sp(   60 ,   60 );}
    else if(cnt1 < 192 ){sp(   65 ,   65 );}
    else if(cnt1 < 198 ){sp(   70 ,   70 );}
    else if(cnt1 < 204 ){sp(   75 ,   75 );}
    else if(cnt1 < 210 ){sp(   79 ,   79 );}
    else if(cnt1 < 216 ){sp(   83 ,   83 );}
    else if(cnt1 < 222 ){sp(   87 ,   87 );}
    else if(cnt1 < 228 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  15 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 170 ){sp(  100 ,  100 );}
    else if(cnt1 < 176 ){sp(   45 ,   45 );}
    else if(cnt1 < 182 ){sp(   50 ,   50 );}
    else if(cnt1 < 188 ){sp(   55 ,   55 );}
    else if(cnt1 < 194 ){sp(   60 ,   60 );}
    else if(cnt1 < 200 ){sp(   65 ,   65 );}
    else if(cnt1 < 206 ){sp(   70 ,   70 );}
    else if(cnt1 < 212 ){sp(   75 ,   75 );}
    else if(cnt1 < 218 ){sp(   79 ,   79 );}
    else if(cnt1 < 224 ){sp(   83 ,   83 );}
    else if(cnt1 < 230 ){sp(   87 ,   87 );}
    else if(cnt1 < 236 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  16 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 185 ){sp(  100 ,  100 );}
    else if(cnt1 < 191 ){sp(   50 ,   50 );}
    else if(cnt1 < 197 ){sp(   55 ,   55 );}
    else if(cnt1 < 203 ){sp(   60 ,   60 );}
    else if(cnt1 < 209 ){sp(   65 ,   65 );}
    else if(cnt1 < 215 ){sp(   70 ,   70 );}
    else if(cnt1 < 221 ){sp(   75 ,   75 );}
    else if(cnt1 < 227 ){sp(   79 ,   79 );}
    else if(cnt1 < 233 ){sp(   83 ,   83 );}
    else if(cnt1 < 239 ){sp(   87 ,   87 );}
    else if(cnt1 < 245 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  17 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 201 ){sp(  100 ,  100 );}
    else if(cnt1 < 207 ){sp(   55 ,   55 );}
    else if(cnt1 < 213 ){sp(   60 ,   60 );}
    else if(cnt1 < 219 ){sp(   65 ,   65 );}
    else if(cnt1 < 225 ){sp(   70 ,   70 );}
    else if(cnt1 < 231 ){sp(   75 ,   75 );}
    else if(cnt1 < 237 ){sp(   79 ,   79 );}
    else if(cnt1 < 243 ){sp(   83 ,   83 );}
    else if(cnt1 < 249 ){sp(   87 ,   87 );}
    else if(cnt1 < 255 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  18 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 218 ){sp(  100 ,  100 );}
    else if(cnt1 < 224 ){sp(   60 ,   60 );}
    else if(cnt1 < 230 ){sp(   65 ,   65 );}
    else if(cnt1 < 236 ){sp(   70 ,   70 );}
    else if(cnt1 < 242 ){sp(   75 ,   75 );}
    else if(cnt1 < 248 ){sp(   79 ,   79 );}
    else if(cnt1 < 254 ){sp(   83 ,   83 );}
    else if(cnt1 < 260 ){sp(   87 ,   87 );}
    else if(cnt1 < 266 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  19 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 236 ){sp(  100 ,  100 );}
    else if(cnt1 < 242 ){sp(   65 ,   65 );}
    else if(cnt1 < 248 ){sp(   70 ,   70 );}
    else if(cnt1 < 254 ){sp(   75 ,   75 );}
    else if(cnt1 < 260 ){sp(   79 ,   79 );}
    else if(cnt1 < 266 ){sp(   83 ,   83 );}
    else if(cnt1 < 272 ){sp(   87 ,   87 );}
    else if(cnt1 < 278 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  20 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 255 ){sp(  100 ,  100 );}
    else if(cnt1 < 261 ){sp(   70 ,   70 );}
    else if(cnt1 < 267 ){sp(   75 ,   75 );}
    else if(cnt1 < 273 ){sp(   79 ,   79 );}
    else if(cnt1 < 279 ){sp(   83 ,   83 );}
    else if(cnt1 < 285 ){sp(   87 ,   87 );}
    else if(cnt1 < 291 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  21 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 275 ){sp(  100 ,  100 );}
    else if(cnt1 < 281 ){sp(   75 ,   75 );}
    else if(cnt1 < 287 ){sp(   79 ,   79 );}
    else if(cnt1 < 293 ){sp(   83 ,   83 );}
    else if(cnt1 < 299 ){sp(   87 ,   87 );}
    else if(cnt1 < 305 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  22 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 296 ){sp(  100 ,  100 );}
    else if(cnt1 < 302 ){sp(   79 ,   79 );}
    else if(cnt1 < 308 ){sp(   83 ,   83 );}
    else if(cnt1 < 314 ){sp(   87 ,   87 );}
    else if(cnt1 < 320 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  23 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 318 ){sp(  100 ,  100 );}
    else if(cnt1 < 324 ){sp(   83 ,   83 );}
    else if(cnt1 < 330 ){sp(   87 ,   87 );}
    else if(cnt1 < 336 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  24 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 341 ){sp(  100 ,  100 );}
    else if(cnt1 < 347 ){sp(   87 ,   87 );}
    else if(cnt1 < 353 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else if( kyoritime <=  25 ){   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 365 ){sp(  100 ,  100 );}
    else if(cnt1 < 371 ){sp(   91 ,   91 );}
                   else {sp(   95 ,   95 );}
    }
    else                       {   /* 車線変更ライン通過後の減速制御 */
         if(cnt1 < 390 ){sp(  100 ,  100 );}
                   else {sp(   95 ,   95 );}
    }
}

//**********************************************************************
// 右寄せ走行（パターン300～310）で、車体が元のレーンの中央線より左にいる時の処理
// 右寄せの表は「中央線が中央か左に見える（車体が線より右）」ことが前提で、
// 0x03・0x83・0x81・0xc1 を「中央線が左端、右の路肩が右端」と読んで左へ切る。
// 車体が線より左にいる時は、同じ形が「中央線が右端、左の路肩が左端」なので、
// 左へ切るとさらに左へ出て、左の路肩を中央線として追い、コースの外の黒を切れ目と判断していた（LOG0110・LOG0119）。
// 戻り値 true:この関数でハンドルを決めた（表と切れ目の判断は使わない） false:従来の処理を使う
//**********************************************************************
bool right_shift_left_steer( void )
{
  unsigned char s = sensor_inp( MASK4_4 );
  bool use_old = false;

  switch( right_shift_side ) {
    case SHIFT_ON_OR_RIGHT:
      /* 中央より右の範囲だけに白 → 車体は線より左。直前が端の白（右の路肩が入ってきた時）は除く */
      if( (s & 0xf0) == 0 && (s & 0x0c) != 0 && (right_shift_last & 0x81) == 0 ) {
          right_shift_side = SHIFT_LEFT;
      } else {
          use_old = true;
      }
      break;

    case SHIFT_LEFT:
      if( s == 0x00 ) {
          if( right_shift_last & 0x03 ) {
              right_shift_side = SHIFT_FAR_LEFT;   /* 中央線が右端から外れた */
          } else {
              use_old = true;                      /* 中央付近で消えた → 中央線の切れ目 */
          }
      } else if( (s & 0x10) != 0 && (s & 0x80) == 0 ) {
          right_shift_side = SHIFT_ON_OR_RIGHT;    /* 中央線が中央に戻った */
          use_old = true;
      } else if( (s & 0x80) != 0 && (s & 0x0f) == 0 ) {
          right_shift_side = SHIFT_FAR_LEFT;       /* 左の路肩だけが見える */
      }
      break;

    case SHIFT_FAR_LEFT:
      if( (s & 0x01) != 0 && (s & 0x80) == 0 ) {
          right_shift_side = SHIFT_LEFT;           /* 中央線が右端に戻ってきた */
      }
      break;
  }
  if( s != 0x00 ) right_shift_last = s;
  if( use_old ) return false;

  if( right_shift_side == SHIFT_FAR_LEFT ) {
      ha( 24 );
      return true;
  }
  switch( s ) {                                    /* 通常走行（パターン25）の右曲げと同じ角度 */
    case 0x08: case 0x0c: ha(  7 ); break;
    case 0x04: case 0x0e: ha( 12 ); break;
    case 0x06:            ha( 16 ); break;
    case 0x02: case 0x07: ha( 20 ); break;
    case 0x01: case 0x03: case 0x81: case 0x83:
    case 0xc1: case 0xc3: case 0x87: ha( 24 ); break; /* 中央線が右端、左に路肩 */
    default: break;
  }
  return true;
}

//**********************************************************************
// end of file
//**********************************************************************