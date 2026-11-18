// PC上でスケッチを1ms単位で動かし、出力を1行ずつ表示するテストプログラム
//   使い方: sim <seed> <ms>
//   同じ seed なら毎回同じ入力（センサ・スイッチ）を与えるので、
//   元のプログラムと新しいプログラムの出力を比べれば動作が同じか確かめられる。
#include "mock_arduino.h"
#include "mcr_gpt_lib.h"
#include "SD.h"
#include <random>
#include <vector>

MockPort mock_port[5];
MockGpt mock_gpt1;
MockSerial Serial;
SDClass SD;
std::map<std::string, std::string> mock_files;
bool mock_sd_ok = true;
volatile uint32_t GTIOC1B, GTIOC6B, GTIOC3A;

extern volatile int pattern;
void setup();
void loop();
void AGTCallback(timer_callback_args_t* p_args);

static const uint8_t INTERESTING[] = {
    0x00,0x01,0x02,0x03,0x05,0x06,0x07,0x0c,0x0e,0x0f,0x17,0x18,0x1b,0x1c,0x1d,0x1e,0x1f,
    0x30,0x37,0x38,0x3b,0x3c,0x3d,0x3e,0x3f,0x40,0x60,0x63,0x65,0x66,0x67,0x70,0x78,0x7c,
    0x7e,0x7f,0x80,0x81,0x83,0xa0,0xa3,0xa5,0xa6,0xa7,0xb8,0xbc,0xc0,0xc1,0xc3,0xc5,0xc6,
    0xc7,0xd8,0xdc,0xe0,0xe3,0xe5,0xe6,0xe7,0xe8,0xec,0xf0,0xf8,0xfc,0xfe,0x18,0x18,0x1c,0x38};

// pattern に合わせて、その状態で意味のあるセンサ値を選びやすくする（奥の状態まで試すため）
static std::vector<uint8_t> candidates(int p) {
    if (p >= 20 && p <= 25) return {0x18, 0x18, 0x18, 0x1c, 0x38, 0x0c, 0x30, 0x0e, 0x70, 0x06, 0x60, 0x07, 0xe0, 0x03, 0xc0, 0xe7, 0x1f, 0xf8};
    if (p >= 30 && p <= 45) return {0x03, 0x83, 0x81, 0xc1, 0xc0, 0x06, 0x07, 0x60, 0xe0, 0x03, 0x00};
    if (p == 101 || p == 201 || p == 501) return {0x18, 0xe7, 0xff, 0x1f, 0x3f, 0xf8, 0xfc, 0x1c, 0x38, 0x0c, 0x30, 0x07};
    if (p >= 102 && p <= 103) return {0x18, 0x1c, 0x38, 0xe0, 0xf0, 0xf8, 0xfc, 0x07, 0x0f, 0x1f, 0x3f, 0x00};
    if (p >= 110 && p <= 161) return {0x80, 0xc0, 0xe0, 0x60, 0xf0, 0xff, 0x70, 0x7f, 0x00, 0x01, 0x03, 0x07, 0x06, 0x0f, 0x3f, 0x0e,
                                      0x1e, 0x83, 0x81, 0xc1, 0x30, 0x38, 0x0c, 0x1c, 0x7e, 0xfe};
    if ((p >= 202 && p <= 310) || (p >= 502 && p <= 610)) return {0x00, 0x18, 0x1c, 0x38, 0x30, 0x70, 0x0c, 0x0e, 0xc1, 0x83, 0x03, 0xc0, 0xe7, 0x07};
    if ((p >= 320 && p <= 350) || (p >= 620 && p <= 650)) return {0x00, 0x01, 0x02, 0x03, 0x80, 0x40, 0xc0, 0x18};
    if ((p >= 400 && p <= 450) || (p >= 700 && p <= 750)) return {0x0c, 0x1c, 0x18, 0x38, 0x30, 0x70, 0x60, 0xe0, 0xc0, 0xc1, 0x81, 0x83,
                                                               0x03, 0x07, 0x06, 0x0e, 0x3c};
    return {0x18};
}

static void set_sensor(uint8_t s) {   // s: 1 = 白
    const int bitpos[8] = {4, 5, 6, 7, 8, 10, 11, 12};  // bit0..bit7 の PORT0 ビット
    uint16_t v = 0xffff;
    for (int b = 0; b < 8; b++) if (s & (1 << b)) v &= ~(1u << bitpos[b]);
    MockPort& p = mock_port[0];
    p.PIDR_b.PIDR4 = (v >> 4) & 1; p.PIDR_b.PIDR5 = (v >> 5) & 1; p.PIDR_b.PIDR6 = (v >> 6) & 1;
    p.PIDR_b.PIDR7 = (v >> 7) & 1; p.PIDR_b.PIDR8 = (v >> 8) & 1; p.PIDR_b.PIDR10 = (v >> 10) & 1;
    p.PIDR_b.PIDR11 = (v >> 11) & 1; p.PIDR_b.PIDR12 = (v >> 12) & 1;
}

int main(int argc, char** argv) {
    unsigned seed = argc > 1 ? atoi(argv[1]) : 1;
    long ms = argc > 2 ? atol(argv[2]) : 8000;
    std::mt19937 rng(seed);
    auto rnd = [&](int n) { return (int)(rng() % n); };

    int dip = rnd(4);
    mock_port[3].PIDR_b.PIDR6 = !(dip & 2);
    mock_port[3].PIDR_b.PIDR7 = !(dip & 1);
    mock_sd_ok = rnd(10) != 0;
    if (rnd(3) == 0) mock_files["renban.txt"] = std::to_string(rnd(12000) - 100) + "\r\n";
    long push_at = rnd(2) ? 1100 + rnd(9000) : -1;      // 走行中にプッシュスイッチを押す時刻
    int start_mode = rnd(3);
    bool guided = rnd(4) != 0;

    mock_port[0].PIDR_b.PIDR3 = 1;  // プッシュスイッチ OFF
    set_sensor(0x19);               // スタートバーあり
    setup();

    uint8_t s = 0x19; long hold = 0;
    for (long t = 0; t < ms; t++) {
        // 入力を作る
        bool push = (t >= 50 && t < 80) || (push_at >= 0 && t >= push_at && t < push_at + 40);
        if (start_mode == 2 && t >= 200 && t < 230) push = true;
        mock_port[0].PIDR_b.PIDR3 = !push;
        if (t < 400) {
            s = 0x19;
        } else if (--hold <= 0) {
            int r = rnd(100);
            std::vector<uint8_t> c = candidates(pattern);
            if (guided && r < 60) s = c[rnd(c.size())];
            else if (r < 60) s = INTERESTING[rnd(sizeof(INTERESTING))];
            else if (r < 85) { int k = rnd(4); s = k == 0 ? (uint8_t)(s << 1) : k == 1 ? (uint8_t)(s >> 1) : (uint8_t)(s ^ (1 << rnd(8))); }
            else if (r < 90) s = (uint8_t)rnd(256);
            else s = 0x18;
            int h = rnd(4);
            hold = h == 0 ? 1 + rnd(5) : h == 1 ? 1 + rnd(30) : h == 2 ? 1 + rnd(150) : 1 + rnd(600);
        }
        set_sensor(s);

        AGTCallback(0);
        loop();
        loop();
        printf("%ld s=%02x p=%d sv=%u ml=%u mr=%u d=%d%d led=%d%d%d cst=%d\n", t, s, pattern,
               (unsigned)GTIOC1B, (unsigned)GTIOC6B, (unsigned)GTIOC3A,
               mock_port[4].PODR_b.PODR0, mock_port[4].PODR_b.PODR4,
               mock_port[4].PODR_b.PODR2, mock_port[4].PODR_b.PODR5, mock_port[1].PODR_b.PODR11,
               mock_gpt1.GTCR_b.CST);
    }
    for (auto& f : mock_files) printf("FILE %s\n%s\n", f.first.c_str(), f.second.c_str());
    return 0;
}
