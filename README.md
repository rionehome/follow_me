# Follow_me on ROS2

## Overview
自己位置推定(カルマンフィルタ)のアルゴリズムを利用した,精度の高いfollow meのプログラムです.

## setup
`https://github.com/rionehome/move`からmoveパッケージをclone&buildしてください.

## Usage
```
ros2 launch follow_me follow.launch.py
```

## Node
**`name` Follow**

### Subscribe Topic

* **`/scan`** ydlidarの情報受け取り（ sensor_msgs/LaserScan ）

* **`/follow_me/control`** follow me 開始・終了のシグナル受け取り ( std_msgs/String )


### Publish Topic

* **`/move/velocity`** 制御パラメータ送信 ( move/velocity )
