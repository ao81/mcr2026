// PC上でスケッチを動かすための、ハードウェアの代わり（テスト専用。実機では使わない）
#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <string>
#include <map>

#define INPUT  0
#define OUTPUT 1
#define HEX    16
inline void pinMode(int, int) {}
inline void delay(unsigned long) {}

struct MockPort {
    struct {
        uint16_t PIDR0 : 1;
        uint16_t PIDR1 : 1;
        uint16_t PIDR2 : 1;
        uint16_t PIDR3 : 1;
        uint16_t PIDR4 : 1;
        uint16_t PIDR5 : 1;
        uint16_t PIDR6 : 1;
        uint16_t PIDR7 : 1;
        uint16_t PIDR8 : 1;
        uint16_t PIDR9 : 1;
        uint16_t PIDR10 : 1;
        uint16_t PIDR11 : 1;
        uint16_t PIDR12 : 1;
        uint16_t PIDR13 : 1;
        uint16_t PIDR14 : 1;
        uint16_t PIDR15 : 1;
    } PIDR_b;
    struct {
        uint16_t PODR0 : 1;
        uint16_t PODR1 : 1;
        uint16_t PODR2 : 1;
        uint16_t PODR3 : 1;
        uint16_t PODR4 : 1;
        uint16_t PODR5 : 1;
        uint16_t PODR6 : 1;
        uint16_t PODR7 : 1;
        uint16_t PODR8 : 1;
        uint16_t PODR9 : 1;
        uint16_t PODR10 : 1;
        uint16_t PODR11 : 1;
        uint16_t PODR12 : 1;
        uint16_t PODR13 : 1;
        uint16_t PODR14 : 1;
        uint16_t PODR15 : 1;
    } PODR_b;
};
extern MockPort mock_port[5];
#define R_PORT0 (&mock_port[0])
#define R_PORT1 (&mock_port[1])
#define R_PORT3 (&mock_port[3])
#define R_PORT4 (&mock_port[4])

struct MockGpt { struct { uint32_t CST : 1; } GTCR_b; };
extern MockGpt mock_gpt1;
#define R_GPT1 (&mock_gpt1)

struct MockSerial {
    void begin(long) {}
    template <class T> void print(T) {}
    template <class T> void print(T, int) {}
    template <class T> void println(T) {}
    void println() {}
};
extern MockSerial Serial;

typedef struct { int dummy; } timer_callback_args_t;
typedef int timer_source_div_t;
#define TIMER_MODE_PERIODIC 0
#define AGT_TIMER 0
#define TIMER_SOURCE_DIV_1 0
#define IRQ_AGT 0
struct FspTimer {
    bool begin(int, int, int, unsigned long, int, timer_source_div_t, void (*)(timer_callback_args_t*)) { return true; }
    void* get_cfg() { return 0; }
    bool open() { return true; }
};
struct IRQManager {
    static IRQManager& getInstance() { static IRQManager m; return m; }
    bool addPeripheral(int, void*) { return true; }
};
#endif
