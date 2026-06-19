# ML-GINS-Calib 実行手順

LiDAR 外部パラメータ（top / front / back の base_link 基準キャリブレーション）の
環境構築・データ準備・実行・評価の手順をまとめたものです。

---

## 1. ディレクトリ構成

```
experiments/
├── README.md                       ← 本書
├── config/
│   └── params.yaml                 ← 実行構成。config/params.yaml に上書きして使う
├── gt/
│   ├── gt_extrinsic_abs_base_to_lidar.txt  ← GT（base_link基準・絶対値, xyzq）
│   └── gt_extrinsic_rel.txt                 ← GT（top基準・相対値, xyzq）
├── scripts/
│   ├── prepare_data.sh             ← rosbag(.mcap) → 入力データ変換
│   ├── run_calibration.sh          ← 構成を適用してキャリブレーション実行
│   └── evaluate_stages.py          ← ステージ別 絶対誤差レポート
└── output/                         ← 実行結果の出力先（ログ・推定値）
```

---

## 2. 環境構築・ビルド

依存関係（GTSAM / Ceres / manif）のインストールとビルド手順は
リポジトリ直下の [../README.md](../README.md) を参照してください。
ビルド後 `build/bin/ml_gins_calibrator` と `build/bin/ml_gins_evaluator` が生成されます。

`prepare_data.py`（rosbag 変換）を使う場合のみ、追加で ROS 2 Python 環境が必要です:

```bash
source /opt/ros/<distro>/setup.bash        # humble など
python3 -c "import rosbag2_py, rclpy"      # 動作確認
```

> 入力データが生成済みであれば、この手順はスキップできます。

---

## 3. データ準備（rosbag → 入力データ）

rosbag(.mcap) を入力データ形式に変換します。

```bash
source /opt/ros/<distro>/setup.bash
# 第1引数: bag(.mcap) のパス, 第2引数: 出力先データディレクトリ
./experiments/scripts/prepare_data.sh <bag.mcap> <data_dir>
```

このスクリプトは内部で次を実行します（要点）:

| 設定 | 値 | 意味 |
|------|----|----|
| `--gnss_topic` | `/sensing/gnss/pose` | base_link の map 座標ポーズ（= imu frame） |
| `--lidar_topics` | top/front/back の `pointcloud_raw_ex` | 各 LiDAR の生点群 |
| `--frame_step` | 5 | 基準 LiDAR を 1/5 に間引き |
| `--max_time_diff` | 0.05 s | GNSS/LiDAR 最近傍同期の許容時間差 |

出力:

```
<data_dir>/gins_sync/gins_sync.txt   GNSS/INS ポーズ
<data_dir>/top/   *.pcd               同期済み点群
<data_dir>/front/ *.pcd
<data_dir>/back/  *.pcd
```

> bag 内のトピック名は `ros2 bag info <bag>` で確認してください。

### 座標系と imu frame について

- `/sensing/gnss/pose` の child_frame は **base_link**。したがって本ツールの
  「imu frame」= **base_link**、推定対象 `T_imu_lidar` = `T_base_link→LiDAR`。
- TF チェーンは `base_link → sensor_kit_base_link → <lidar>_base_link`。

---

## 4. キャリブレーション実行

`experiments/config/params.yaml` の `data_dir` と `init_ext` を対象データに合わせて
設定してから実行します。

```bash
# リポジトリ直下で
./experiments/scripts/run_calibration.sh
```

このスクリプトは

1. 現在の `config/params.yaml` を `experiments/output/params.yaml.bak` に退避し、
2. `experiments/config/params.yaml` を `config/params.yaml` に上書きし、
3. `build/bin/ml_gins_calibrator` を実行（ログを `experiments/output/calibrator_output.txt` に保存）、
4. 終了時に元の `config/params.yaml` を復元します。

出力:

- `experiments/output/calibrator_output.txt` … 全コンソールログ（ステージ別 4×4 行列を含む）
- `experiments/output/extrinsic_parameters.txt` … top 基準の推定外部パラメータ（xyzq）

> **所要時間**: フル 3-LiDAR + Multi-LiDAR で 4 スレッド数時間程度。

手動で実行する場合:

```bash
cp experiments/config/params.yaml config/params.yaml
./build/bin/ml_gins_calibrator        # リポジトリ直下から
```

---

## 5. 精度評価

### 5.1 ステージ別 絶対誤差（base_link 基準）

```bash
python3 experiments/scripts/evaluate_stages.py
# 別の出力を評価する場合:
#   --log <calibrator_output.txt> --ext <extrinsic_*.txt> --gt <gt_abs.txt>
```

- `Stage1 (Coarse)`  : 各 LiDAR セクションの 1 個目の 4×4 行列
- `Stage1+2 (Joint)` : 2 個目の 4×4 行列
- `Multi-LiDAR`      : top(Stage1+2) × top 相対 front/back。top は anchor なので Stage1+2 と同値
- 誤差: 回転 = `R_est·R_gtᵀ` の角度、`dRx/dRy/dRz` は SO(3) log 成分(°)、並進は成分差(m)
- GT は `experiments/gt/gt_extrinsic_abs_base_to_lidar.txt`（base_link 基準・絶対値）

### 5.2 相対（top 基準）評価

同梱の評価バイナリで top 基準の相対誤差を出すこともできます:

```bash
./build/bin/ml_gins_evaluator \
    experiments/output/extrinsic_parameters.txt \
    experiments/gt/gt_extrinsic_rel.txt \
    experiments/output/eval.txt
```

### 5.3 定性評価（任意）

`config/params.yaml` の `save_maps: true` にして実行すると before/after/aligned の
PCD が出力されます。点群の重なりを目視確認できます:

```bash
pcl_viewer experiments/output/maps/top_before_calibration.pcd \
           experiments/output/maps/top_after_calibration.pcd
pcl_viewer experiments/output/maps/top_aligned.pcd \
           experiments/output/maps/front_aligned.pcd \
           experiments/output/maps/back_aligned.pcd
```
