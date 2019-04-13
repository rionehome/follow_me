# follow_me
## Overview
自己位置推定のアルゴリズムを利用した、精度の高いfollow meのプログラムです。

## setup
`https://github.com/rionehome/move`からmoveパッケージ、`https://github.com/rionehome/ros_posenet`からposenetパッケージをclone&buildしてください。

## Usage
```
roslaunch turtlebot_bringup minimal.launch  
roslaunch followme follow.launch  
```

## Node
**`name` Follow**

### Subscribe Topic

* **`/scan`** ydlidarの情報受け取り（ sensor_msgs/LaserScan ）

* **`/ros_posenet/poses`** ros_posenetからの結果受け取り ( ros_posenet/Poses )

* **`/follow_me_nlp/follow_me`** follow me 開始・終了のシグナル受け取り ( std_msgs/String )


### Publish Topic

* **`/move/velocity`** 制御パラメータ送信 ( std_msgs/Float64MultiArray )