//**************************************************************************
// sdlog.ino  microSD へのログ保存
//   割り込み側（log_sample）が 10ms ごとに RAM のリングバッファへ貯め、
//   loop 側（log_task）が空き時間に microSD へ書き込む。
//   ファイルは log0001.csv, log0002.csv ... と連番で作る（番号は renban.txt に保存）
//**************************************************************************

// log_task の状態
#define LOGTASK_IDLE        0     // 待機
#define LOGTASK_OPEN        991   // 連番を読んでファイルを作る
#define LOGTASK_HEADER      992   // 1行目（列名）を書く
#define LOGTASK_WAIT_START  993   // スタートを待つ
#define LOGTASK_WRITE       994   // 貯めたログを書く
#define LOGTASK_CLOSE       995   // ファイルを閉じる
#define LOGTASK_DONE        999   // 何もしない

//**********************************************************************
// microSD の確認（setup から呼ぶ）
//**********************************************************************
void log_setup(void) {
    Serial.print("microSDを確認中です... ");
    if (!SD.begin(2.4e6, CS1)) {       // 通信速度Hz, microSDのCS1
        err_sd = 1;
        Serial.println("microSDが認識されません!");
        for (int i = 0; i < 20; i++) { // マイコンボードD13のLEDを点滅させる
            led_out_m(1);
            timer(50);
            led_out_m(0);
            timer(50);
        }
    } else {
        SdFile::dateTimeCallback(&set_microSD_dateTime);  // ファイルの日付・時刻を返す関数を登録
        Serial.println("microSDを認識しました。");
    }
}

//**********************************************************************
// ログを RAM に貯める（1ms割り込みから毎回呼ぶ。実際に貯めるのは LOG_INTERVAL_MS ごと）
//**********************************************************************
void log_sample(void) {
    if (log_st != LOG_WRITE || cnt_start % LOG_INTERVAL_MS != 0) {
        return;
    }
    int next_write = log_write + 1;
    if (next_write == LOG_SLOTS) {
        next_write = 0;
    }
    // 満杯のときは新しいログを捨てて、まだ書いていないログを守る
    if (next_write != log_read) {
        log_buff[log_write].time = cnt_start;
        log_buff[log_write].ptn = pattern;
        log_buff[log_write].sen = sensor_inp(MASK4_4);
        log_buff[log_write].ang = log_raw.ang;
        log_buff[log_write].ml = log_raw.ml;
        log_buff[log_write].mr = log_raw.mr;
        log_buff[log_write].spd = (int16_t)(speed_mps * 1000.0f);
        log_buff[log_write].dist = (int16_t)(dist_m > 30.0f ? 30000 : dist_m * 1000.0f);
        log_buff[log_write].vmax = (int16_t)(v_max_mps * 1000.0f);
        log_write = next_write;
    }
}

//**********************************************************************
// microSD への書き込み（loop から毎回呼ぶ）
//**********************************************************************
void log_task(void) {
    static int state = LOGTASK_IDLE;
    char buf[80];
    int i;

    if (err_sd != 0) {
        return;
    }

    switch (state) {
    case LOGTASK_IDLE:
        if (log_st != LOG_NON) {
            state = LOGTASK_OPEN;
        }
        break;

    case LOGTASK_OPEN:
        // 連番の読み込み
        i = 0;
        microSD = SD.open("renban.txt", FILE_READ);
        if (microSD != 0) {
            int length = microSD.available();
            if (length > 8) {
                length = 8;
            }
            int bytes_read = microSD.read(buf, length);
            buf[bytes_read > 0 ? bytes_read : 0] = '\0';
            sscanf(buf, "%d", &i);
            if (i < 0 || i >= 9999) {
                i = 0;
            }
            microSD.close();
        } else {
            Serial.print("renban.txtが開けませんので「log0001.csv」でファイルを作成します。");
        }

        // 連番ファイルを作り直す
        if (SD.exists("renban.txt")) {
            SD.remove("renban.txt");
        }
        microSD = SD.open("renban.txt", FILE_WRITE);
        if (microSD != 0) {
            sprintf(buf, "%d", i + 1);
            microSD.println(buf);
            microSD.close();
        }

        // 新しい番号でログファイルを作る
        run_log_no = i + 1;
        sprintf(buf, "log%04d.csv", run_log_no);
        microSD = SD.open(buf, FILE_WRITE);
        if (microSD != 0) {
            Serial.print(buf);
            Serial.println("でログファイルを作成します。");
            state = LOGTASK_HEADER;
        } else {
            err_sd = 1;
            Serial.println("ログファイルが作成できません。");
        }
        break;

    case LOGTASK_HEADER:
#if LOG_SPEED_MODEL
        microSD.println("ms,pattern,sensor,handle,motor_l,motor_r,speed_mm_s,dist_mm,vmax_mm_s");
#else
        microSD.println("ms,pattern,sensor,handle,motor_l,motor_r");
#endif
        state = LOGTASK_WAIT_START;
        break;

    case LOGTASK_WAIT_START:
        if (log_st == LOG_WRITE || log_st == LOG_CLOSE) {
            state = LOGTASK_WRITE;
        }
        break;

    case LOGTASK_WRITE:
        if (log_write != log_read) {
            const volatile log_t& entry = log_buff[log_read];
#if LOG_SPEED_MODEL
            snprintf(buf, sizeof(buf), "%u,%d,'%08lu,%d,%d,%d,%d,%d,%d",
                entry.time, entry.ptn, convertBCD_CharToLong(entry.sen),
                (int)entry.ang, (int)entry.ml, (int)entry.mr, (int)entry.spd, (int)entry.dist, (int)entry.vmax);
#else
            snprintf(buf, sizeof(buf), "%u,%d,'%08lu,%d,%d,%d",
                entry.time, entry.ptn, convertBCD_CharToLong(entry.sen),
                (int)entry.ang, (int)entry.ml, (int)entry.mr);
#endif
            microSD.println(buf);
            const int next_read = log_read + 1;
            log_read = next_read == LOG_SLOTS ? 0 : next_read;
        } else if (log_st == LOG_CLOSE && log_write == log_read) {
            // 終了要求を確認した後に、未保存のログがないことを再確認してから閉じる
            state = LOGTASK_CLOSE;
        }
        break;

    case LOGTASK_CLOSE:
        microSD.close();   // 必ずcloseしないと、ファイルが残らない
        run_log_closed = true;
        state = LOGTASK_DONE;
        break;

    case LOGTASK_DONE:
        break;
    }
}

//**********************************************************************
// microSD.close時に、ファイルの日付・時刻を設定（コンパイルした日時を使う）
//**********************************************************************
void set_microSD_dateTime(uint16_t* date, uint16_t* time) {
    *date = FAT_DATE((uint16_t)getCompileYear(C_DATE),
        (uint8_t)getCompileMonth(C_DATE),
        (uint8_t)getCompileDay(C_DATE));
    *time = FAT_TIME(getCompileHour(C_TIME),
        getCompilerMinute(C_TIME),
        getCompilerSecond(C_TIME));
}

//**********************************************************************
// 8bitの値を、0と1だけの10進数に変換（例: 0x18 → 11000）。ログのセンサ列用
//**********************************************************************
unsigned long convertBCD_CharToLong(unsigned char hex) {
    unsigned long ret = 0;
    for (int i = 0; i < 8; i++) {
        ret *= 10;
        if ((hex & 0x80) != 0) {
            ret += 1;
        }
        hex <<= 1;
    }
    return ret;
}

//**********************************************************************
// コンパイルした日時（__DATE__ = "Nov 18 2026", __TIME__ = "10:56:32"）から各値を取り出す
//**********************************************************************
int getCompileYear(const char* p) {
    int i = atoi(p + 7);
    if (i < 1980 || i > 2107) i = 2023;
    return i;
}

int getCompileMonth(const char* p) {
    const char monthStr[] = { "JanFebMarAprMayJunJulAugSepOctNovDec" };
    for (int i = 0; i < 12; i++) {
        if (strncmp(monthStr + i * 3, p, 3) == 0) {
            return i + 1;
        }
    }
    return 1;
}

int getCompileDay(const char* p) {
    int i = atoi(p + 4);
    if (i < 1 || i > 31) i = 1;
    return i;
}

int getCompileHour(const char* p) {
    int i = atoi(p);
    if (i < 0 || i > 23) i = 0;
    return i;
}

int getCompilerMinute(const char* p) {
    int i = atoi(p + 3);
    if (i < 0 || i > 59) i = 0;
    return i;
}

int getCompilerSecond(const char* p) {
    int i = atoi(p + 6);
    if (i < 0 || i > 59) i = 0;
    return i;
}
