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
手首の周囲に動作空間を確保してください。他の操作ノードからの指令は止めます。
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
  に全15関節の目標を送り、選択腕の手首2軸だけを変えます。腰・頭部と反対腕の
  関節角は開始時の値に保持します。
  各移動の成功と2秒間の静止を確認してから、同定ノードに収集を依頼します。
- `scripts/ft_payload_identification.py`: 各姿勢で200サンプルを収集します。
  センサ時刻のTFからセンサ座標系の重力を求め、全姿勢を使って
  `F = m*g + b_force`, `T = (m*r) × g + b_torque` を最小二乗で解きます。
  力・トルクのバイアス6成分、質量、センサ原点からの重心位置を同定します。

既定の9姿勢は `config/ft_calibration_poses.yaml` の開始姿勢からの相対角度です。
手首2軸は最大±0.35 rad動かし、腰は動かしません。1移動は最低5秒です。
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

### hrpsysトピック経由の実機接続（2026-11-06検証）

`admittance_bringup.launch.py` は `nextage_tutorials/TopicSystem` を使って、
腕の6関節と補償済み力覚6成分を `/admittance/controller_manager` に公開します。
既存の `/joint_states` と `/fullbody_controller/command` のROS 1ブリッジを使用します。
元の `admittance.launch.xml` は既存managerへの登録用として残しています。
このアダプタは片腕ずつの小変位試験用です。50 Hzで制御計算し、軌道送信は最大25 Hz、
目標時間は100 msです。通信を介した制御なのでハードリアルタイムではありません。

#### 左手だけ起動

前提：ROS 1ブリッジ・力センサ起動済み、校正済み。各ターミナルで
`source ~/colcon_ws/install/setup.bash` を実行してください。
Touch teleopと他の腕指令ノードは停止し、左右のbringupを同時起動しないでください。
同じ腕の補償ノードが既に動いている場合はターミナル1を省略します。

```bash
# ターミナル1：保存済みの重力補償
ros2 launch nextage_tutorials ft_compensation.launch.xml arm:=larm
# ターミナル2：左腕のadmittance制御を有効にして起動
ros2 launch nextage_tutorials admittance_bringup.launch.py arm:=larm inactive:=false
# 停止（別ターミナル）
ros2 control switch_controllers -c /admittance/controller_manager --deactivate larm_admittance_controller --strict
```

#### 右手だけ起動

前提は左手と同じです。腕を切り替えるときは、先に起動済みのbringupをCtrl-Cで終了します。

```bash
# ターミナル1：保存済みの重力補償
ros2 launch nextage_tutorials ft_compensation.launch.xml arm:=rarm
# ターミナル2：右腕のadmittance制御を有効にして起動
ros2 launch nextage_tutorials admittance_bringup.launch.py arm:=rarm inactive:=false
# 停止（別ターミナル）
ros2 control switch_controllers -c /admittance/controller_manager --deactivate rarm_admittance_controller --strict
```

#### 再校正が必要な場合

補償ノードとbringupを終了してから実行します（右手は `arm:=rarm`）。
完了後に校正launchをCtrl-Cで終了し、上記の手順で起動します。

```bash
ros2 launch nextage_tutorials ft_payload_calibration.launch.xml arm:=larm wrench_sign:=-1.0
```

現在の取り付けでは両センサとも `wrench_sign:=-1.0` で重力との関係を確認しました。
取り付け・工具・配線を変えた場合は再確認・再校正してください。
保存先は `~/.ros/nextage/{larm,rarm}_ft_payload.yaml` です。
今回の実測値は `config/ft_calibration/2026-11-06/` にも保存しています。
同じ取り付け・工具構成で保存値を直接使う場合は、補償launchに
`calibration_file:=$(ros2 pkg prefix --share nextage_tutorials)/config/ft_calibration/2026-11-06/larm_ft_payload.yaml`
を追加します（右手はファイル名の `larm` を `rarm` に変更）。
補償ノードは保存された座標系・質量・符号・有限値を検証して読み込み、
`/{left,right}_ft/force_torque_compensated` に出力します。
重力はこの段階で除去するため、bringup側のCoG.forceは0に固定しています。
同定に失敗した場合も `.samples.yaml` に測定データを残します。

既定では**センサ座標Xの並進1軸のみ**を有効にします。ハードウェアアダプタは
起動時の関節角から±0.10 rad、指令速度0.10 rad/s、実測からの目標先行0.01 radを上限とし、
関節リミット、腰の移動、過大な力（各軸15 N / 1.5 Nm）、250 msを超えるデータ断、
別のfullbody指令publisherを検出すると停止します。停止後はbringupを再起動してください。
停止時に既に受理された最後の100 ms軌道は実行され得ます。
別経路のaction指令やROS 1側の操作との排他は保証しないので、他の操作を同時に行わないでください。
外部障害物の衝突回避は行いません。

力覚ドライバには再接続時のDTR設定、シリアルの排他open、2秒測定断での終了を追加しています。
既存のrespawn設定により、USB復帰後に再接続します。別launchで同じセンサを重複起動しないでください。

実機では左右それぞれ9姿勢から同定し、推定質量は左0.2567 kg、右0.2321 kgでした。
センサXの目標力オフセット0.5 Nで、左約2.62 mm・右約2.46 mmの手先変位と
ゼロ指令への復帰を確認しました。試験後はinactiveです。
無接触時の補償後平均力は各成分約0.21 N以下ですが、微小力を精密に扱う用途では
残差・温度ドリフト・工具の変化をさらに評価してください。

検証:

```bash
python3 -m unittest discover -s test -p test_ft_payload.py
# 実機のdomainとは別のROS_DOMAIN_ID=179で模擬関節・力を使用。
python3 test/check_admittance_transport.py
```
