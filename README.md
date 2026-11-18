# mcr2026

マイコンカーラリー Basic マイコンカー（RMC-RA4M1）の走行プログラムです。

| 場所 | 内容 |
|---|---|
| `basic/` | 走行プログラム（Arduino IDE で `basic/basic.ino` を開く） |
| `basic/config.h` | 調整する数値と、新しい走行アルゴリズムのスイッチ |
| `basic/tables.h` | センサ値ごとのハンドル角度・モータ値の表 |
| `docs/tuning_guide.md` | 実機での確認と数値調整の手順 |
| `tools/host_test/` | 元のプログラムと動きが同じかを PC で確かめる道具 |
