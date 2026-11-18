// テスト用の microSD の代わり（メモリ上にファイルを持つ）
#ifndef MOCK_SD_H
#define MOCK_SD_H
#include "mock_arduino.h"
#define FILE_READ 0
#define FILE_WRITE 1
#define FAT_DATE(y, m, d) (uint16_t)(((y) - 1980) << 9 | (m) << 5 | (d))
#define FAT_TIME(h, m, s) (uint16_t)((h) << 11 | (m) << 5 | (s) >> 1)
extern std::map<std::string, std::string> mock_files;
extern bool mock_sd_ok;
struct File {
    std::string name; bool open_ = false; size_t pos = 0;
    operator bool() const { return open_; }
    int available() { return open_ ? (int)(mock_files[name].size() - pos) : 0; }
    int read(char* buf, int len) {
        int n = 0;
        while (n < len && pos < mock_files[name].size()) buf[n++] = mock_files[name][pos++];
        return n;
    }
    void println(const char* s) { if (open_) mock_files[name] += std::string(s) + "\r\n"; }
    void close() { open_ = false; }
};
struct SDClass {
    bool begin(double, int) { return mock_sd_ok; }
    File open(const char* n, int mode) {
        File f; f.name = n;
        if (mode == FILE_READ && !mock_files.count(n)) return f;
        if (mode == FILE_WRITE) mock_files[n];
        f.open_ = true; return f;
    }
    bool exists(const char* n) { return mock_files.count(n) > 0; }
    bool remove(const char* n) { return mock_files.erase(n) > 0; }
};
extern SDClass SD;
struct SdFile { static void dateTimeCallback(void (*)(uint16_t*, uint16_t*)) {} };
#endif
