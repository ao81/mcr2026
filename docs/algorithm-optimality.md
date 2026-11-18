# Basicマイコンカーの走行制御はどこまで最適か

**結論**　12個の新制御をすべて有効にする設定は、全コースで最速にも、全条件で完走できる設定にもならない。改善の第一候補は、レーンチェンジの直前にすでに観測している**ハーフラインの幅と、そこから中央線が切れるまでの距離**を速度推定に使う方法である。走行プログラムのコピーに試作して比べると、9つの物理量・コース条件を同時に変えた256走で、現行新制御の失敗104回が併用案では20回になった。逆に現行法だけが成功した走りが10回ある。置き換えの判断には、この10回を再現する実機測定が必要である。

対象は [mcr2026 の 74fb102](https://github.com/ao81/mcr2026/tree/74fb102b3bee3c39a7871699f0994e57c623c83c)。以下のタイムと失敗件数は、この版の C++ シミュレータを今回実行して得た値である。提案の試作は付録で再現できる。車体への書き込み・実走試験は行っていない。

## 1. 何を「優れている」と判定するか

1. **同じ車体・コース・開始位置で完走できるか。** 失敗はゴール前にシミュレータが終了した走り。失敗を集計から落として平均タイムだけを比較すると、難しい走りで脱落した方式が速く見える。
2. **両方が完走したとき、同じスタートから同じゴールまで何 ms 違うか。** 表のペア差は「候補のタイム − 現行新制御のタイム」。負が短縮を表す。
3. **どの前提に依存するか。** コース形状、車体寸法、使えるグリップ、センサの見える幅、サーボ遅れ、速度推定誤差を含める。競技車が実際に遭遇する各条件の頻度は不明である。以下の件数は選んだ試験集合の成績であり、実機の完走確率に換算できない。

この優先順位は、タイムの付く走りをまず増やすという本稿の評価方針である。2走のベストタイムなどを直接最適化したい場合は、失敗の扱いと条件の発生頻度を別途定める必要がある。

## 2. 現行法で成り立つ最適性と、その限界

| 制御 | ソースで確認した動作 | 数学的に言える範囲 |
|---|---|---|
| 通常走行の操舵 | 白線の横位置からピュアパシュートで操舵する。設定を組み合わせると、線の向き・曲率も推定し、サーボ遅れ後のずれを補正する。 | 観測できた線に追従する局所制御。8本のセンサの観測履歴が同一の間は、次に直線が続くコースと直後に急カーブが来るコースを区別できない。 |
| 通常走行の速度 | 推定曲率から横加速度の上限速度を決め、前方の曲率と比較する。既定の `PREVIEW_RATIO=0` でも前方曲率は使う。その位置までに残る制動距離をゼロとして計算する設定である。 | 曲率が既知で一定なら横力の上限を計算できる。観測遅れ、次の曲率変化、脱輪時の損失まで含めた周回最速の証明にはならない。 |
| クランク | 車幅・前端・余白・舵角・サーボ遅れを使って90°の単円弧を求める。進入が速すぎる場合などは従来処理に戻る。 | モデル内で大きい半径を選ぶ幾何学問題を解く。前後の直線、旋回距離、立ち上がりを合わせた時間は直接比較していない。 |
| レーンチェンジ | 逆向きの等半径円弧2本と待ち距離を選び、推定距離で舵を切り返す。円弧走行中は線による軌道修正を行わない。 | 指定した円弧族で曲率を小さくできる。速度推定の偏りが距離に積算されると、正しく求めた円弧でも、車体が予定地点に着く前後に切り返してしまう。 |
| ブレーキ・速度推定 | 直線では残距離と制動距離を比較する。速度は左右出力の平均から予測し、線幅を通過時間で割った値で補正する。加速制限、最高速学習、速度推定の異常判定も備える。 | 正しい速度・既知の終端速度・既知の制動能力の下では、最大限加速して制動限界で切り替える考え方に根拠がある。実装には余裕距離、出力制限、整数 PWM、線幅の誤差が入る。 |

実装中の分岐と計算式の確認には、[通常走行](../basic/run_trace.ino)、[経路計算](../basic/planner.ino)、[速度モデル](../basic/speed_model.ino)、[クランク](../basic/run_crank.ino)、[レーンチェンジ](../basic/run_lane.ino)、[設定値](../basic/config.h)を使用した。

### 最大半径と最短時間が一致する条件

横加速度の許容値を $a_y$、円弧の半径を $R$ とすると、定常旋回で $v^2/R \le a_y$ である。したがって横力だけで決まる上限速度は $\sqrt{a_y R}$。ところが90°の円弧の長さも $\pi R/2$ に増えるので、円弧だけの所要時間は限界速度でも $(\pi/2)\sqrt{R/a_y}$ となる。

**同じ入口・出口を与えた例**を考える。直交する2本の直線上に、交点までの距離がそれぞれ $L$ の始点・終点を置く。半径 $R$ の接円弧で結ぶと、残る直線は合計 $2(L-R)$。直線速度を $V$、加減速と操舵の切替時間をゼロ、円弧を $\sqrt{a_y R}\le V$ とする点車両では、

$$
T(R)=\frac{2(L-R)}{V}+\frac{\pi}{2}\sqrt{\frac{R}{a_y}},
\qquad
T'(R)=-\frac{2}{V}+\frac{\pi}{4\sqrt{a_y R}} .
$$

$L=1$ m、$V=3$ m/s、$a_y=7.2$ m/s² では、$R=0.15$ m から $0.18$ m に広げると 793.39 ms から 795.03 ms に増える。これは「半径を広げれば必ず区間全体が速くなる」という推論への反例である。実機の最適半径は、有限の制動力、サーボ速度、車体の収まりを含め、同じ境界条件で改めて求める必要がある。

### 現行版をそのまま走らせた結果

左右各7通りの開始位置。両方式で車体・コースは共通。全14走成功した欄は算術平均を ms で示す。クランクの標識距離は 0.50 m、車線変更は 0.30 m である。

| コース | 従来制御（全スイッチ0） | 現行新制御（全スイッチ1） |
|---|---:|---:|
| クランク | 0/14 完走 | 1594.4 |
| レーンチェンジ | 1645.6 | 1637.7 |
| 180°カーブ R=0.45 m | 0/14 完走 | 0/14 完走 |
| 180°カーブ R=0.80 m | 0/14 完走 | 2226.7 |
| 180°カーブ R=1.00 m | 2256.3 | 2322.1 |
| 180°カーブ R=1.50 m | 2777.3 | 2694.9 |
| S字 R=0.80 m | 0/14 完走 | 2059.0 |
| S字 R=1.00 m | 2031.0 | 2091.1 |
| S字 R=1.50 m | 2554.0 | 2481.6 |

同じ1.0 mカーブでも従来法のタイムが短い。モデルはグリップ不足時に縦加速度と曲率を縮めるが、実タイヤの横すべりによる速度損失を独立にモデル化していない。したがって、この順位をそのまま実車に移すことはできない。また、既存ガイドの「概算最短時間の±1%」は、特定の円経路とグリップを仮定した比較である。あらゆる実現可能な経路・制御に対する下界との差を測ったものではない。[車両モデル](../tools/course_sim/course_sim.cpp)・[既存の概算と比較条件](tuning_guide.md)

## 3. 改善案A　1 msの測定と両立する速度範囲で補正量を制限する

線幅を $W$、割り込み周期を $h=0.001$ s、検出処理の既知の1周期遅れを足した時間を $t=(\text{line_pass_ms}+1)h$ とする。各エッジを次のサンプルで初めて観測できたなら、入口と出口の検出遅れはそれぞれ $[0,h]$。両者の差から真の通過時間 $T$ は $t-h\le T\le t+h$ に入る。周期内の位相に確率分布を仮定する必要はない。

線を正面から一度横切り、見える幅が既知で $t>h$ なら、平均速度の範囲は $[W/(t+h),W/(t-h)]$。補正したいのは終了を検出した時点の速度である。通過中から検出まで $|\dot v|\le A$ と仮定すると、退出時と通過平均の差は最大 $AT/2$（各時点との速度差を積分すればよい）。退出検出の最大 $Ah$ も足すと、

$$
E=A((t+h)/2+h),\qquad I=[W/(t+h)-E,\;W/(t-h)+E].
$$

例として $W=0.040$ m、$t=13$ ms なら、平均速度は **2.857〜3.333 m/s** と両立する。点測定値は 3.077 m/s である。予測が 2.92 m/s なら、測定だけを理由に上方へ引き上げる必然性はない。そこで予測 $x$ が $I$ の中なら速度と最高速の学習をその回は見送る。外なら $x$ に近い端点を $e$ とし、点測定 $W/t$ を $x$ と $2e-x$ を両端とする区間に収めてから既存のゲインで混ぜる。大幅な電池電圧の変化など、予測と観測の食い違いが大きいときは点測定まで動かせる。

**保証の範囲**　真値 $y\in I$ なら、この補正の一回で $|x-y|$ は増えない。例えば $x<e\le y$ のとき、$x$ と $2e-x$ は $e$ について対称であり、その間のどの点も $y$ までの距離が $|x-y|$ 以下になる。既存ゲイン $0\le K\le1$ の混合後も同様。上側は符号を反転して同じである。これは一回の速度推定の性質で、将来の最高速学習・軌道・完走の保証には及ばない。見える幅の校正、斜め横断、通過中の欠測、実加速度が仮定を超える場合は別途検証が必要。幅に既知の範囲があれば、下端には $W_{\min}$、上端には $W_{\max}$ を使う拡張が可能である。この拡張は今回の比較に含めていない。

## 4. 改善案B　中央線の切れ目で推定した走行距離を照合する

現行法はハーフラインを抜けた時点で積算距離をゼロに戻す。次に中央線が切れた瞬間には、**既知の標識間距離 $D$ と、積算した距離 $\hat D$ の両方が既にある**。現行のS字経路はそこから距離を再度ゼロにして走り始める。この直前に速度の倍率を見直せる。シミュレータの真の速度・座標は制御に渡さない。

直線区間で推定速度が一貫して真値の $b$ 倍なら、$\hat D=\int b v\,dt=bD$。従って区間終端で $q=D/\hat D$ を掛けると、このモデルでは速度の倍率誤差がちょうど消える。40 mmの線と0.30 mの標識間隔を3 m/sで通過すると、1 ms相当の3 mmが占める割合はそれぞれ7.5%と1.0%。長い観測区間を使う理由である。

試作では $m=2\hat v_{\max}h$ とし、$\hat D$ に最も近い $[D-m,D+m]$ 内の値を $D_o$ とする。$q=D_o/\hat D$。**両方向補正**は $0.8\le q\le1.25$ で速度と最高速推定を $q$ 倍し、最高速推定は既存の学習範囲内に収める。**下方限定**は $0.8\le q<1$ のときだけ同じ操作を行う。これは「推定速度を下げる」操作であり、モーター出力を下げる操作とは限らない。下方限定は線幅を広げた探索条件で両方向補正に失敗が出た後に加えた対照案である。

$m$ はサンプル遅れを想定した許容幅、倍率範囲は試作上の選択である。実速度の厳密な上限が $\hat v_{\max}$ と一致する保証はない。減速中の時定数誤差や車体の蛇行では $b$ が一定という仮定が崩れ、標識間隔の設定誤差も速度誤差と区別できない。また速度誤差を最高速推定まで転写する操作は仮説である。Aの局所的な証明をBには適用できない。

## 5. 同じ条件でのレーンチェンジ比較

表内は**失敗した走りの数**。Aは幅による補正のみ、Bは距離照合のみ。「併用」はAの後にBを使う。すべて同じ新制御・同じグリップ設定で比較し、同じ列の試験では全方式に同じ入力条件を与えた。

| 方式 | ① 標準と個別変更など 154走 | ② 4因子の組合せ 160走 | ③ 設定距離0.50 mの格子 162走 | ④ 新しい9因子の組合せ 256走 |
|---|---:|---:|---:|---:|
| 現行新制御 | 16 | 64 | 48 | 104 |
| Aのみ | 8 | 44 | 42 | 88 |
| Bのみ・両方向 | 20 | 10 | 26 | 28 |
| A＋B・両方向 | 14 | **0** | 26 | **20** |
| A＋B・下方限定 | **0** | 4 | **0** | 38 |

- **①** 設定距離0.30 m。標準、最高速2.4/3.3、横グリップ6.4、線幅0.0256/0.0384、時定数0.18、サーボ480°/s、DIP=0、センサ前方距離0.225、最高速3.3＋縦グリップ4.8＋サーボ遅れ0.012の11条件。左右×開始位置7通り。速度は m/s、加速度は m/s²、長さは m、時間は s。
- **②** 設定距離0.30 m。最高速 {2.70, 3.15}、横グリップ {6.8, 8.4}、サーボ速度 {510, 660}、物理線幅 {0.029, 0.035} の全16組×左右×開始位置5通り。
- **③** 車体側には0.50 mを設定。実際の標識間隔 {0.48, 0.50, 0.52}、最高速 {2.55, 3.00, 3.30}、物理線幅 {0.027, 0.032, 0.037} の27組×左右×開始位置3通り。
- **④** 候補を決めた後、付録に示す9次元のHalton列で異なる128条件を決定論的に生成し、それぞれ左右で評価した。実間隔0.48〜0.52 m、車体側設定0.50 m。最高速、縦横グリップ、モーター時定数、サーボの遅れと速度、線幅、開始位置を同時に変えた。どの組にも実機の発生確率による重みはない。左右には対称性があり、256回を独立な確率標本とは扱わない。

### 改善した条件と、悪化した条件

| ④で現行新制御と比較 | 現行失敗→候補成功 | 現行成功→候補失敗 | 両方成功 | 両方成功時のペア差（平均 / 中央値 / 範囲） |
|---|---:|---:|---:|---|
| Aのみ | 28 | 12 | 140 | −5.63 / 0 / −51〜+49 ms |
| Bのみ・両方向 | 90 | 14 | 138 | −4.78 / −1 / −67〜+61 ms |
| A＋B・両方向 | 94 | 10 | 142 | −3.63 / −1 / −67〜+78 ms |
| A＋B・下方限定 | 72 | 6 | 146 | −8.67 / −1 / −67〜+39 ms |

④の左レーンチェンジの例では、条件番号2で現行は失敗、両方向併用は1814 msで完走した。条件番号51では現行が1630 msで完走し、両方向併用は失敗した。加えて①の物理線幅0.0384 mでは、現行は14/14完走、両方向併用は0/14、下方限定は14/14だった。④で完走数を最大にする選択と、既に完走できていた条件を維持する選択は一致しない。

標準条件の車線変更だけを見ると現行1637.7 ms、併用の両方式1631.4 ms（すべて14/14完走）。7種類の開始位置のうち5つでは併用の方が4〜7 ms遅く、残る2つで35〜37 ms短縮した。**改善の主な価値は、平均タイムの約6 msより、失敗条件がどう入れ替わるか**にある。②で全走成功した両方向併用も、①③④には失敗が残る。

## 6. さらに時間を詰める設計と、実機での採否

| 候補 | 解く問題と評価 |
|---|---|
| A＋Bによる車線変更 | ハードウェア追加を要さず、通常走行の1 msごとに新しい探索を行う必要もない。試験④の完走数なら両方向併用、①〜③で既存の成功条件を失いにくい試験案としては下方限定が有力。実車で物理線幅・センサの見える幅・標識間隔を測り、同じ車体で両方向を比較する。 |
| 固定経路の速度最適化 | センサが標識を検出し、この先の局所経路が既知になった区間が対象。複数の円弧や舵角が連続する経路について、前後の加速・制動を含む通過時間をPCで比較できる。今回のタイム表にはこの方法の性能を含めていない。 |
| 通常カーブの進入速度制限 | 未観測の先の曲率は一意に推定できない。既存の `TRACE_SAFE_RADIUS_M` を対照に、想定最小半径と利用可能な検出距離から速度上限を置く方法がある。0.45 mの失敗はレーンチェンジ用A/Bの適用範囲外である。 |
| コース全体のMPC・軌道最適化 | 先の地図や正確な位置が与えられた条件では上限性能の比較に使える。Basic車の8本のセンサ、実装が使える情報、1 msの締切をそろえて比較する必要がある。今回その方式で速くなるという実測結果はない。 |

固定経路の長さ座標を $s$、曲率を $\kappa(s)$、$x(s)=v(s)^2$ とすれば、$dx/ds=2a_x$、時間は $\int ds/\sqrt{x}$。区分ごとに一定の前後加速度を仮定すると $x_{i+1}=x_i+2a_i\Delta s_i$、区間時間は $2\Delta s_i/(\sqrt{x_i}+\sqrt{x_{i+1}})$。例えば制約には次を入れる。

$$
\left(\frac{a_i}{G_x}\right)^2+
\left(\frac{\kappa_i x_i}{G_y}\right)^2\le1,\qquad
\delta(s)=\arctan(L_w\kappa(s)),\qquad
|\delta'(s)|\sqrt{x(s)}\le\dot\delta_{\max}.
$$

ここで $L_w$ はホイールベース。急に曲率が変わる理想円弧の接続は、有限の操舵速度に合わせて平滑化するか、サーボを動的な状態として扱う。終端の速度条件から後ろ向きに到達できる速度域を求め、入口から前向きに加速可能な速度域と合わせる。モーターの一次遅れに基づく加速上限、PWM飽和、車体の四隅、区間途中の制約も検査する。これを**複数の経路に、同じ入口・出口位置、向き、速度条件で**適用し、通過時間を比較するのが次の研究段階である。固定経路上の速度最適化と、経路まで自由にした大域最適化には異なる保証が必要である。文献では到達可能集合による固定経路の時間最適化、および経路と速度を交互に改善する車両向けの手法が示されている。[Pham & Pham](https://arxiv.org/abs/1707.07239)・[Kapania ほか](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/2015_dscc_kapania_sequential_2step_0.pdf)

**実機で先に判定すること**　ハーフライン退出時と中央線切れ目の割り込みで、イベント時刻、通過カウント、補正前後の速度・最高速、積算距離、倍率、経路採用または従来処理への復帰を記録する。今の通常ログは10 ms周期なので、境界時刻は割り込みで採取し、書き込みは既存のリングバッファ経由とする。外部動画または光電ゲートで同じ区間の実速度・切り返し位置を独立に測る。走行順を交互にし、電池・タイヤ・線幅の条件ごとに、脱輪数、共通完走でのタイム、1 ms割り込みの最長処理時間を確認する。シミュレータの設定値を実測で置き換えた後に、どちらを大会用に採るか判断できる。

## 付録　再現方法

基準版と同じGCC（今回の環境は g++ 14.2.0、`-std=gnu++17 -O1`）とPython 3で、以下のブロックをそのまま実行できる。シミュレータ本体とハードウェアのモックは基準版を使う。各走行は新しいプロセスで開始し、全コースをつないだ連続走行や区間間に残る学習状態の効果は評価していない。

<details>
<summary>速度補正の差分と全条件の再現コード</summary>

Aは `speed_measure_by_line()` 内の異常値判定・カウンタリセットの後、`float v_new = v_meas;` の直前に追加する。ここでは車線変更にだけ有効にする。

<!-- width-update -->
```cpp
    const float h = 0.001f;
    if ((pattern == PT_LANE_L_PASS || pattern == PT_LANE_R_PASS) && t > h && speed_mps > 0.5f) {
        const float dv = GRIP_LONG_MPS2 * (0.5f * (t + h) + h);
        const float lo = width_m / (t + h) - dv;
        const float hi = width_m / (t - h) + dv;
        if (speed_mps >= lo && speed_mps <= hi) return;
        const float edge = speed_mps < lo ? lo : hi;
        const float reflected = 2.0f * edge - speed_mps;
        v_meas = speed_mps < lo ? fminf(v_meas, reflected) : fmaxf(v_meas, reflected);
    }
```

Bは `run_lane_approach()` の中央線切れ目検出後、`lane_start_scurve()` を呼ぶ直前に追加する。下方限定では倍率条件の上限を `ratio < 1.0f` にする。

<!-- distance-update -->
```cpp
            if (model_ok && dist_m > 0.0f && speed_mps > 0.5f) {
                const float h = 0.001f;
                const float margin = 2.0f * v_max_mps * h;
                const float measured = fminf(fmaxf(dist_m, HALF_TO_LANE_M - margin),
                                             HALF_TO_LANE_M + margin);
                const float ratio = measured / dist_m;
                if (ratio >= 0.8f && ratio <= 1.25f) {
                    speed_mps *= ratio;
                    v_max_mps = fminf(V_MAX_MPS * (1.0f + VMAX_LEARN_RANGE),
                                      fmaxf(V_MAX_MPS * (1.0f - VMAX_LEARN_RANGE),
                                            v_max_mps * ratio));
                }
            }
```

本稿を含むリポジトリのルートから以下を実行する。`report` は本稿、`source` は検証する基準版。空いている別ディレクトリに基準版を取り出す。出力CSVの `ms` 欄が空の行は失敗である。

```sh
git worktree add --detach ../mcr-baseline-74fb102 74fb102b3bee3c39a7871699f0994e57c623c83c
python3 - docs/algorithm-optimality.md ../mcr-baseline-74fb102 > mcr-trials.csv <<'PY'
import pathlib, re, sys
report = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
code = re.search(r'<!-- reproduce -->\n```python\n(.*?)\n```', report, re.S).group(1)
exec(compile(code, "reproduce.py", "exec"))
PY
```

<!-- reproduce -->
```python
import csv
import itertools
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

source = pathlib.Path(sys.argv[2]).resolve()
import hashlib

inputs = sorted([*source.glob("basic/*.ino"), *source.glob("basic/*.h"),
                 *source.glob("tools/host_test/mock/*.h"), source / "tools/course_sim/course_sim.cpp"])
digest = hashlib.sha256()
for path in inputs:
    name = path.relative_to(source).as_posix().encode()
    digest.update(name + b"\0" + path.read_bytes().replace(b"\r\n", b"\n") + b"\0")
assert digest.hexdigest() == "e2848c8b7980a3bfdc00017b67f22606924cdadd6c253f22f74e2dfad02cc20f", "走行ソースまたはシミュレータの版が異なります"

def snippet(name):
    return re.search(r"<!-- " + name + r" -->\n```cpp\n(.*?)\n```", report, re.S).group(1)

def insert_before(path, marker, block):
    body = path.read_text(encoding="utf-8")
    assert body.count(marker) == 1, (path, marker)
    path.write_text(body.replace(marker, block + "\n" + marker), encoding="utf-8")

def build(folder, mode, distance):
    folder.mkdir()
    sketch = folder / "basic"
    shutil.copytree(source / "basic", sketch)
    config = sketch / "config.h"
    values = config.read_text(encoding="utf-8")
    if mode != "off":
        values, count = re.subn(r"^(#define USE_NEW_\w+\s+)0 ", r"\g<1>1 ", values, flags=re.M)
        assert count == 12
    values, count = re.subn(r"(#define HALF_TO_LANE_M\s+)0.30f", r"\g<1>" + distance + "f", values)
    assert count == 1
    config.write_text(values, encoding="utf-8")
    if mode in ("A", "AB", "down"):
        insert_before(sketch / "speed_model.ino", "    float v_new = v_meas;", snippet("width-update"))
    if mode in ("B", "AB", "down"):
        block = snippet("distance-update")
        if mode == "down":
            block = block.replace("ratio <= 1.25f", "ratio < 1.0f")
        insert_before(sketch / "run_lane.ino", "            if (lane_start_scurve(stage, right))", block)
    first = sketch / "basic.ino"
    files = [first] + sorted(p for p in sketch.glob("*.ino") if p != first)
    combined = folder / "sketch.cpp"
    combined.write_text('#include "mock_arduino.h"\n' +
                        "\n".join(p.read_text(encoding="utf-8") for p in files), encoding="utf-8")
    binary = folder / "sim"
    subprocess.run(["g++", "-std=gnu++17", "-O1", "-w",
                    "-I" + str(source / "tools/host_test/mock"), "-I" + str(sketch),
                    "-o", binary, combined, source / "tools/course_sim/course_sim.cpp"], check=True)
    return binary

def radical_inverse(n, base):
    result, scale = 0.0, 1.0 / base
    while n:
        n, digit = divmod(n, base)
        result += digit * scale
        scale /= base
    return result

def conditions():
    seven = [1.400, 1.417, 1.433, 1.450, 1.467, 1.483, 1.500]
    for kind in ("lane", "crank", "curve", "s"):
        radii = {"curve": [0.45, 0.8, 1.0, 1.5], "s": [0.8, 1.0, 1.5]}.get(kind, [None])
        for side, radius, ap in itertools.product("lr", radii, seven):
            opts = {"approach": ap}
            if radius:
                opts["curve_r" if kind == "curve" else "s_rad"] = radius
            yield "baseline", "0.30", kind + "_" + side, opts
    scenarios = [{}, {"v_max": 2.4}, {"v_max": 3.3}, {"grip_lat": 6.4},
                 {"line_w": 0.0256}, {"line_w": 0.0384}, {"tau": 0.18}, {"servo_dps": 480},
                 {"dip": 0}, {"sensor_to_axle": 0.225},
                 {"v_max": 3.3, "grip_long": 4.8, "servo_dead": 0.012}]
    for side, env, ap in itertools.product("lr", scenarios, seven):
        yield "1", "0.30", "lane_" + side, dict(env, approach=ap)
    for v, g, s, w, side, ap in itertools.product([2.70, 3.15], [6.8, 8.4], [510, 660],
            [0.029, 0.035], "lr", [1.409, 1.429, 1.449, 1.469, 1.489]):
        yield "2", "0.30", "lane_" + side, dict(v_max=v, grip_lat=g, servo_dps=s, line_w=w, approach=ap)
    for d, v, w, side, ap in itertools.product([0.48, 0.50, 0.52], [2.55, 3.0, 3.30],
            [0.027, 0.032, 0.037], "lr", [1.423, 1.453, 1.493]):
        yield "3", "0.50", "lane_" + side, dict(half_to=d, v_max=v, line_w=w, approach=ap)
    bounds = [("v_max", 2.4, 3.45), ("grip_lat", 6.4, 9.6), ("grip_long", 4.8, 7.2),
              ("tau", 0.12, 0.18), ("servo_dead", 0.008, 0.012), ("line_w", 0.0256, 0.0384),
              ("half_to", 0.48, 0.52), ("approach", 1.40, 1.50), ("servo_dps", 480, 720)]
    for i in range(1, 129):
        opts = {k: f"{lo + (hi-lo)*radical_inverse(i, p):.8f}"
                for (k, lo, hi), p in zip(bounds, [2, 3, 5, 7, 11, 13, 17, 19, 23])}
        for side in "lr":
            yield "4", "0.50", "lane_" + side, opts

writer = csv.writer(sys.stdout)
writer.writerow(["suite", "case", "mode", "course", "settings", "ms"])
with tempfile.TemporaryDirectory() as tmp:
    binaries = {}
    for case, (suite, distance, course, options) in enumerate(conditions(), 1):
        modes = ("off", "all") if suite == "baseline" else ("all", "A", "B", "AB", "down")
        for mode in modes:
            key = (mode, distance)
            if key not in binaries:
                binaries[key] = build(pathlib.Path(tmp) / (mode + distance), mode, distance)
            params = [f"{k}={v}" for k, v in options.items()]
            run = subprocess.run([binaries[key], course, *params], capture_output=True, text=True)
            line = run.stdout.splitlines()[-1]
            match = re.search(r"ゴール\s+(\d+) ms", line)
            assert "横G" in line and run.returncode == (0 if match else 1), line
            writer.writerow([suite, case, mode, course, " ".join(params), match[1] if match else ""])
```

</details>
