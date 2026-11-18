//**************************************************************************
// hardware.ino  センサ、モータ、サーボ、LED、スイッチの入出力
//**************************************************************************

//**********************************************************************
// センサ状態の読み込み
//   引数   マスク値（1のビットだけ有効）
//   戻り値 センサ値。bit7(0x80)=左端 … bit0(0x01)=右端、白線を検出したビットが1
//**********************************************************************
unsigned char sensor_inp(unsigned char mask) {
    unsigned char sensor;

    sensor  = (R_PORT0->PIDR_b.PIDR12) << 7;   // D35 左端
    sensor |= (R_PORT0->PIDR_b.PIDR11) << 6;   // D36
    sensor |= (R_PORT0->PIDR_b.PIDR10) << 5;   // D37
    sensor |= (R_PORT0->PIDR_b.PIDR8)  << 4;   // D38
    sensor |= (R_PORT0->PIDR_b.PIDR7)  << 3;   // D39
    sensor |= (R_PORT0->PIDR_b.PIDR6)  << 2;   // D40
    sensor |= (R_PORT0->PIDR_b.PIDR5)  << 1;   // D41
    sensor |= (R_PORT0->PIDR_b.PIDR4)  << 0;   // D42 右端

    sensor = (~sensor) & mask;                 // センサは白で0になるので反転する

    return sensor;
}

//**********************************************************************
// DIPスイッチ（マイコンボード上）  戻り値 0〜3
//**********************************************************************
unsigned char dipsw_get(void) {
    unsigned char sw = (!(R_PORT3->PIDR_b.PIDR6) << 1) + (!(R_PORT3->PIDR_b.PIDR7));
    return sw;
}

//**********************************************************************
// プッシュスイッチ（モータドライブ基板）  戻り値 押している:1 離している:0
//**********************************************************************
unsigned char pushsw_get(void) {
    unsigned char sw = !(R_PORT0->PIDR_b.PIDR3);
    return sw;
}

//**********************************************************************
// スタートバー検出センサ  戻り値 バーあり:1 バーなし:0
//**********************************************************************
unsigned char startbar_get(void) {
    unsigned char b = !(R_PORT0->PIDR_b.PIDR4);
    return b;
}

//**********************************************************************
// LED（モータドライブ基板）  引数 bit1:LED2 bit0:LED3  1で点灯
//   例) 0x3 → LED2:ON LED3:ON   0x2 → LED2:ON LED3:OFF
//**********************************************************************
void led_out(unsigned char led) {
    R_PORT4->PODR_b.PODR2 = !(led & 0x02);
    R_PORT4->PODR_b.PODR5 = !(led & 0x01);
}

//**********************************************************************
// LED（マイコンボードのD13）  引数 1:点灯 0:消灯
//**********************************************************************
void led_out_m(unsigned char led) {
    R_PORT1->PODR_b.PODR11 = !led;
}

//**********************************************************************
// モータ速度
//   引数 左モータ, 右モータ  -100〜100（0で停止、100で正転100%、-100で逆転100%）
//   DIPスイッチの値で 70〜100% に縮小してから出力する
//**********************************************************************
void sp(int accele_l, int accele_r) {
    int sw_data = dipsw_get() + DIP_SPEED_BASE;   // 7〜10
    accele_l = accele_l * sw_data / 10;
    accele_r = accele_r * sw_data / 10;

    log_raw.ml = accele_l;
    log_raw.mr = accele_r;

    // 左モータ
    if (accele_l >= 0) {
        R_PORT4->PODR_b.PODR0 = 0;
        GTIOC6B = (long)(PWM_CYCLE_M + 1) * accele_l / 100;
    } else {
        R_PORT4->PODR_b.PODR0 = 1;
        GTIOC6B = (long)(PWM_CYCLE_M + 1) * (-accele_l) / 100;
    }

    // 右モータ
    if (accele_r >= 0) {
        R_PORT4->PODR_b.PODR4 = 0;
        GTIOC3A = (long)(PWM_CYCLE_M + 1) * accele_r / 100;
    } else {
        R_PORT4->PODR_b.PODR4 = 1;
        GTIOC3A = (long)(PWM_CYCLE_M + 1) * (-accele_r) / 100;
    }
}

//**********************************************************************
// サーボ（ハンドル）
//   引数 角度 -90〜90（マイナスで左、0でまっすぐ、プラスで右）
//**********************************************************************
void ha(int angle) {
    log_raw.ang = angle;
    GTIOC1B = SERVO_CENTER + SERVO_DIR * angle * HANDLE_STEP;
}
