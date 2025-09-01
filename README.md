# fix2xyz_packages_ros2
緯度経度[NavSatFix等]-マップ座標[Odometry等]の変換用ツールをまとめたリポジトリです。  
- fix_msgs : NavSatFix型に姿勢の情報を加えた独自メッセージ[FixWithOrientation]を定義するパッケージ  
- fix2xyz  : マップ座標から緯度経度に変換するノード(fix2xyz_node)と緯度経度からマップ座標に変換するノード(xyz2fix_node)を含むパッケージ  

## build
```
cd ~/colcon_ws/src
git clone https://github.com/Hokuyo-RD/fix2xyz_packages_ros2.git
cd ~/colcon_ws
colcon build --symlink-install
```