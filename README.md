# pet_ros2_lightbeacon_pkg
ROS2 Python package to toggle PWM(RC) controlled light beacons on/off/strobe/blink.

This is the first "smoking/burning" release...

<table>
   <tr>
      <td>
        <img src="doc/LightBeacon.png" width="350px">
      </td>
      <td>
        <img src="https://user-images.githubusercontent.com/49879749/156254687-591ab6c0-a413-49ae-9e35-8ce395d5ccf8.mp4" width="400px">
      </td>
   </tr>
</table>

```mermaid
flowchart LR
    A(Power Up)-->|On| B(1.Rotating Fast)
    B --> |cycle pwm|C(2.Rotating Slow)
    C --> |cycle pwm|D(3.Flashing)
    D --> |cycle pwm|E(4.Strobing)
    E --> |cycle pwm|F(5.LED Off)
    F --> |cycle pwm|B
```


