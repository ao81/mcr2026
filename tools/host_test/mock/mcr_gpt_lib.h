// テスト用の代わり
#ifndef MOCK_GPT_H
#define MOCK_GPT_H
#include "mock_arduino.h"
extern volatile uint32_t GTIOC1B, GTIOC6B, GTIOC3A;
#define GTIOCA 0
#define GTIOCB 1
#define DIV16 16
inline void setGPTterminal(int, int) {}
inline void startPWM_GPT1(int, int, int) {}
inline void startPWM_GPT3(int, int, int) {}
inline void startPWM_GPT6(int, int, int) {}
#endif
