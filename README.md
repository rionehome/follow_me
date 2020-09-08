# follow_me
## Overview
自己位置推定のアルゴリズムを利用した、精度の高いfollow meのプログラムです。

## setup
```
https://github.com/rionehome/follow_me
https://github.com/rionehome/move
https://github.com/EAIBOT/ydlidar
https://github.com/rionehome/rione_msgs
https://github.com/rionehome/emergency_stop
```
これらのパッケージをcloneした上でインストール・ビルドを行ってください。

## Usage
```
roslaunch follow_me follow.launch  
```

## Node
**`name` Follow**

### Subscribe Topic

* **`/scan`** ydlidarの情報受け取り（ sensor_msgs/LaserScan ）

* **`/follow_me/control`** follow me 開始・終了のシグナル受け取り ( std_msgs/String )


### Publish Topic

* **`/move/velocity`** 制御パラメータ送信 ( move/velocity )