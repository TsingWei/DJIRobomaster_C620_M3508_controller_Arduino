# A Motor Controller For C620 On Arudino(2560)
## Hardware:
* Arduno mega 2560
* MCP2515 can module
* DJI RoboMaster C620 motor controller
* DJI RoboMaster M3508 motor
## Library using:
* [TimerOne](https://github.com/PaulStoffregen/TimerOne) 用来调用atmega内部的定时器，可以精确到us
* [MsTimer2](https://github.com/PaulStoffregen/MsTimer2) 第二个定时器，精确到ms
* [Arduino-MCP2515](https://github.com/autowp/arduino-mcp2515) 用来连接can通信板
* [FastPID](https://github.com/mike-matera/FastPID) 一个使用定点数来进行pid计算的库，速度稍微快那么一丢丢
* [Arduino-Kalman](https://github.com/bachagas/Kalman) Kalman滤波的库，方便滤波
## 连线

| MCP2515 | Mega2560  |
| ------- | --------- | 
| SCK   | 52 |  
| MISO    | 50 |  
| MOSI    | 51    | 
| CS    | 53     | 
| INT    | -     | 
## Main:
[/src/main.ino](/src/main.ino)
## 整体框架: 
![flowchart](/img/flowchart.png)
