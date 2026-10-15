# nextage_tutorials

## jsk-nextage tutorials

### Setup

```bash
cd ~
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/Michi-Tsubaki/rtmros_tutorials.git -b ros2
sudo apt update
sudo apt upgrade
rosdep update
rosdep install --from-path src -iry --skip-keys "realsense2_camera realsense2_description"
colcon build --packages-up-to nextage_tutorials --symlink-install
```


### Launch

#### Terminal 1 (`ros1_bridge`)

```bash
cd ~/colcon_ws/src/rtm_tutorials/nextage_tutorials/launch
./bridge.sh
```

#### Terminal 2 (cameras and force torque sensors)

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch nextage_tutorials nextage.launch.xml 
```

### カメラ構成

`nextage.launch.xml` は次の3台とRVizの3つのImage表示を起動します。

| 位置 | 機種 | 画像トピック | 単独起動launch |
| --- | --- | --- | --- |
| 上部 | L515 | `/top_camera/color/image_raw` | `raw/top_camera.launch.xml` |
| 左手 | D405 | `/lhand_camera/color/image_rect_raw` | `raw/d405_camera.launch.xml` |
| 右手 | Innomaker USBカメラ | `/rhand_camera/color/image_raw` | `raw/rhand_camera.launch.xml` |

各カメラは `launch_top_camera:=false`、`launch_lhand_camera:=false`、
`launch_rhand_camera:=false` で個別に無効化できます。
旧引数 `launch_d405_camera` も左手カメラの起動設定として使用できます。
追加の作業カメラ（`/dev/elp01`）は既定で無効です。使用時だけ
`launch_task_camera:=true` を指定してください。
右手USBカメラは固定のby-idパスで識別し、変更時は `rhand_video_device` を指定します。
既存のlaunchを停止してから再起動すると変更が反映されます。

### 実機での力覚センサの自動キャリブレーション

`ft_payload_calibration.launch.xml` は **実機を動かします**。既存の
`nextage.launch.xml` とROS 1ブリッジを起動し、手先を無接触・無負荷にして、
腰と手首の周囲に動作空間を確保してください。他の操作ノードからの指令は止めます。
関節リミットは確認しますが、衝突回避プランニングは行いません。

```bash
cd ~/colcon_ws
colcon build --packages-select nextage_tutorials --symlink-install
source install/setup.bash
# 右腕。左腕は arm:=larm（同時に起動しない）
ros2 launch nextage_tutorials ft_payload_calibration.launch.xml arm:=rarm
```

処理は2つのスクリプトに分かれています。

- `scripts/ft_calibration_motion.py`: `/fullbody_controller/follow_joint_trajectory_action`
  に全15関節の目標を送り、腰と選択腕の手首2軸だけを変えます。頭部と反対腕の
  関節角は開始時の値に保持します（腰を動かすため、空間上の位置は変わります）。
  各移動の成功と2秒間の静止を確認してから、同定ノードに収集を依頼します。
- `scripts/ft_payload_identification.py`: 各姿勢で200サンプルを収集します。
  センサ時刻のTFからセンサ座標系の重力を求め、全姿勢を使って
  `F = m*g + b_force`, `T = (m*r) × g + b_torque` を最小二乗で解きます。
  力・トルクのバイアス6成分、質量、センサ原点からの重心位置を同定します。

既定の9姿勢は `config/ft_calibration_poses.yaml` の開始姿勢からの相対角度です。
腰は最大±0.10 rad、手首2軸は最大±0.35 rad動かします。1移動は最低5秒です。
全部の計測が終わると開始姿勢に戻ってから同定します。途中で失敗した場合は
後続の移動を送らず、実行中のactionのキャンセルを要求します。失敗時に
自動で開始姿勢へ戻る動作は行いません。

正常終了後は次のファイルが保存されます。既存の同名ファイルは置換されます。

- `~/.ros/nextage/rarm_ft_payload.yaml`: 質量[kg]、重心[m]、バイアス[N, Nm]、
  推定残差、姿勢ごとの測定値。左腕では `larm_...`。
- `~/.ros/nextage/rarm_ft_payload.admittance.yaml`: admittance_controller用の
  `gravity_compensation.CoG.pos` と `CoG.force`（質量×9.80665 [N]）。

同定ノードは正常終了後も動作し、右腕なら
`/right_ft/force_torque_compensated` に重力・バイアス補償済みの値を出します。
Ctrl-Cで終了できます。姿勢の多様性不足、静止時のばらつき、モデル残差過大、
非正の質量はエラーになります。負の質量の場合はセンサの符号・座標系を確認します。
センサが反力側の符号を出していることを確認できた場合にのみ `wrench_sign:=-1.0`
を指定します。姿勢や閾値はlaunch引数と設定ファイルで変更できます。

### Admittance controllerへの接続

`admittance.launch.xml` は既存のROS 2 controller_managerに片腕のコントローラを
登録します。既定ではinactiveです。現在のhrpsysのROS 1→ROS 2 actionブリッジだけでは
実行できません。`admittance_controller` と `kinematics_interface_kdl` に加えて、
関節のposition command/stateと力覚6成分のstate interfacesを公開する
ros2_control hardware adapterが必要です。`config/nextage_admittance.yaml` の
FT名も、そのadapterが公開する名前に合わせてください。

```bash
ros2 launch nextage_tutorials admittance.launch.xml arm:=rarm
# hardware adapterがバイアスのみ補正した値を提供する場合:
ros2 launch nextage_tutorials admittance.launch.xml arm:=rarm \
  gravity_params_file:=$HOME/.ros/nextage/rarm_ft_payload.admittance.yaml
```

生成されたadmittance用重力設定は、hardware adapter側でセンサの符号と
バイアスを補正し、**重力成分は残した値**に適用します。補償済みトピックをadapterの
入力に使う場合は、コントローラ側の `CoG.force` を0.0にして二重補償を避けます。
`wrench_reference` は目標の力オフセットであり、測定値の入力先ではありません。
このlaunch群はhardware adapterの代替にはなりません。

1姿勢だけのゼロ点調整には `ft_zero_calibration.launch.xml` もありますが、
姿勢が変わる用途では上記の複数姿勢キャリブレーションを使います。
キャリブレーション中にadmittance制御を同時に有効にしないでください。
