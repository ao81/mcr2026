// release/release.ino を模擬で走らせるときの、course_sim.cpp とのつなぎ（模擬専用。release.ino は変えない）
//   course_sim.cpp は basic/ の変数（速さの推定など）を表示に使うので、同じ名前の変数をここに置く。
//   release.ino の AGTCallback は、ビルドのときに -DAGTCallback=release_AGTCallback で名前を変えてあり、
//   ここで呼んだ後に、表示用の列へ release.ino の値を入れる：
//     speed_mps の列 ← kyoritime（ラインの通過時間 [ms]）
//     dist_m の列    ← cnt1（パターンごとのタイマ [ms]）
#include "mock_arduino.h"

volatile float speed_mps, dist_m, plan_wait_m, plan_len_m, plan_speed_mps, plan_switch_m, lane_end_m;
volatile float plan_angle;
volatile float obs_z1, obs_z2, obs_z3, servo_est_deg;
volatile float lane_yaw, lane_snap_y;

extern volatile int kyoritime;
extern volatile unsigned long cnt1;
void release_AGTCallback(timer_callback_args_t* p_args);

void AGTCallback(timer_callback_args_t* p_args) {
    release_AGTCallback(p_args);
    speed_mps = (float)kyoritime;
    dist_m = (float)cnt1;
}
