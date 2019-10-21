# rosbot_ekf
The default package for the new ROSbot firmware.

## Installation

First, you need to install robot localization pkg to use the `rosbot_ekf` package:

```
$ sudo apt-get install ros-kinetic-robot-localization
```

After that, clone the `rosbot_ekf` repo to your `ros_ws/src` directory and compile with `catkin_make`.

## Using `rosbot_ekf` package

To start the rosserial communication and EKF run:
```bash
$ roslaunch rosbot_ekf all.launch
```

For PRO version add parameter:

```bash
$ roslaunch rosbot_ekf all.launch rosbot_pro:=true
```

You can also include this launch in your custom launch files using:

```xml
<include file="$(find rosbot_ekf)/launch/all.launch"/>
```

For PRO version it will look like that:

```xml
<include file="$(find rosbot_ekf)/launch/all.launch">
    <arg name="rosbot_pro" value="true"/>
</include>
```

## Authors

* **Adam Krawczyk** - *Initial work* - [adamkrawczyk](https://github.com/adamkrawczyk)
* **Szymon Szantula** - [byq77](https://github.com/byq77)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details