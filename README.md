# mcr2026

マイコンカーラリー Basic マイコンカー（RMC-RA4M1）の走行プログラムです。

| 場所 | 内容 |
|---|---|
| `basic/` | 走行プログラム（Arduino IDE で `basic/basic.ino` を開く） |
| `basic/config.h` | 調整する数値と、新しい走行アルゴリズムのスイッチ（`USE_NEW_*`、現在の初期値はヘッダー参照） |
| `basic/tables.h` | センサ値ごとのハンドル角度・モータ値の表 |
| `basic/speed_model.ino` | 速さの推定と、計算による減速・速さの決め方（新しい走行アルゴリズム） |
| `basic/planner.ino` | クランク・レーンチェンジの経路の計算と、ピュアパシュート（新しい走行アルゴリズム） |
| `docs/tuning_guide.md` | 新しい走行アルゴリズムの考え方、実機で測る値、確認と数値調整の手順 |
| `tools/host_test/` | 元のプログラムと動きが同じかを PC で確かめる道具 |
| `tools/config_editor.py` | パラメータ編集・区間デバッグ・CSVによる自動調整。Braveでローカルと同期するには `python tools/config_editor.py` を起動（編集ページは public ブランチの `config/` から読む）。[手順](debug/README.md)・[Web版](https://ao81.github.io/mcr2026/config/) |
| `tools/course_sim/` | コースと車を計算で模擬し、新しい走行アルゴリズムを PC で走らせる道具。[Web版のシミュレータ](https://ao81.github.io/mcr2026/simulator/) を開くと、コース・スイッチ・値を画面で選んで走らせ、動きをアニメーションで見られる |
| `public` ブランチ | GitHub Pages で公開するページ（[ao81.github.io/mcr2026](https://ao81.github.io/mcr2026/)）。選択ページ `index.html`、パラメータ編集 `config/`、シミュレータ `simulator/`。シミュレータは main の `basic/` が変わると GitHub Actions が自動で作り直す |
