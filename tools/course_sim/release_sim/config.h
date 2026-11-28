// release/release.ino を模擬で走らせるときに、course_sim.cpp が使う車とコースの値（模擬専用。release.ino は変えない）
//   release.ino には basic/config.h が無いので、course_sim.cpp が読む名前をここで決める。
//   ここに書いた値が、画面（release-sim）の「コースと車」の初期値になる（build_release.py が読む）。
//   車の寸法は basic/config.h と同じ（同じ車）。速さ・摩擦・ブレーキは実車の記録に合わせた値。
#ifndef MCR_CONFIG_H
#define MCR_CONFIG_H

// release.ino の #define と同じ値（build_release.py・run_release.sh が release.ino から読んで -D で渡す）
#ifndef PWM_CYCLE_M
#define PWM_CYCLE_M     14999
#endif
#ifndef SERVO_CENTER
#define SERVO_CENTER    4500
#endif
#ifndef HANDLE_STEP
#define HANDLE_STEP     23
#endif
#define SERVO_DIR       (-1)    // release.ino の ha() は SERVO_CENTER - 角度 × HANDLE_STEP

// 車の寸法（basic/config.h と同じ）
#define WHEELBASE_M             0.160f  // 前輪の軸と後輪の軸の距離
#define TREAD_M                 0.115f  // 左右の後輪の中心どうしの距離
#define SENSOR_TO_REAR_AXLE_M   0.370f  // 後輪の軸からセンサまで
const int SENSOR_X_MM[8] = { -63, -45, -27, -10, 10, 27, 45, 63 };
#define SENSOR_HALF_WIDTH_M     0.070f  // センサの基板の幅の半分（レーンチェンジでは、センサの腕と先も区間の外に出てはいけない）
#define SERVO_SPEED_DPS         429.0f  // サーボが角度を変える速さ [度/s]
#define SERVO_DEAD_S            0.010f  // 指令からサーボが動き始めるまでの遅れ [s]

// コースアウトの判定に使う点（後輪の軸から前へ CAR_FRONT_M、横へ ±CAR_HALF_WIDTH_M の4つの角）
//   初期値は「前輪の軸の中心と後輪の軸の中心」。車輪の外側の角（basic の模擬と同じ 0.180 / 0.090）にすると、
//   線をなぞって半径 0.60 のカーブを回るだけで内側の車輪がコースの内側の端を越える（センサが後輪から 0.37 前にあるため）。
//   実車はカーブ・クランクを走れているので、模擬では軸の中心で判定する。
#define CAR_FRONT_M             0.160f
#define CAR_HALF_WIDTH_M        0.000f

// 走りの値（実車の記録から：出力100%で約 2.5 m/s、-40 のブレーキで約 10 m/s²、出力 28% 以下では走らない）
#define V_MAX_MPS               2.5f    // 出力100%で走り続けたときの速さ [m/s]
#define MOTOR_TAU_S             0.20f   // 速さが変わるときの時定数 [s]（basic の模擬は 0.15。LOG0124 ではクランク手前・レーンチェンジ手前の
                                        //   減速が模擬より弱く、0.20 の方が近い。-40 のブレーキは 0.20 でも前後のグリップ 11 で頭打ちになる）
#define MOTOR_FRICTION_PERCENT  28.0f   // 出力の大きさがこれ [%] 以下では、モータに電圧がかからない
#define GRIP_LAT_MPS2           7.0f    // 横のグリップ [m/s²]
#define GRIP_LONG_MPS2          11.0f   // 前後のグリップ [m/s²]
#define SIM_DRIVE_DEFAULT       1       // 左右の後輪を別々に計算する（クランクで内側を逆転させるため）

// コース（ao のコース：クロスライン 20mm → 曲がった後の中心線まで 800mm、ハーフライン 40mm）
#define CROSS_TO_CRANK_M        0.800f  // クロスラインの終わりから、曲がった後の車線の中心線まで [m]
#define HALF_TO_LANE_M          0.600f  // ハーフラインの終わりから、古い車線の中心線が切れる所まで [m]（規則の 300〜1000mm にコース図の 400 を使い、合流部分の 200 を足した）
#endif
