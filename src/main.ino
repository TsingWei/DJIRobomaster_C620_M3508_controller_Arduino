#include <SPI.h>
#include <mcp2515.h>
#include <TimerOne.h>
#include <Kalman.h>
#include <MsTimer2.h>
#include <FastPID.h>

#define window_size 5
int TEST_SPEED = 800;

struct can_frame canMsgIn;          // 收到的can包
struct can_frame canMsgOut;         // 要发的can包
MCP2515 mcp2515(53);                // CS(SS) to pin 53 on MEGA2560
// Kalman myFilter(0.125,32,1023,0);   // Kalman滤波器
// 0.03 16
Kalman myFilter(0.03,16,1023,0);   // Kalman滤波器
int angle[2] = {0};                 // Suppose 2 motors totally
int angle_last[2] = {0};
int RPM[2] = {0};
int actualCurrent[2] = {0};
int setCurrent[2] = {0};
int T[2] = {0};

int RPM_est[window_size] = {0};
int slide_R = 0;              // RPM的均值滤波的flag
int RAW_RPM;                  // 原始RPM值

const long interval = 500;    // 采样间隔 单位us
int delta_pos;                // 两次采样的转子位置差值

char recv[50];

long set_pos = 15000;
int set_speed = 0;
int set_current = 0;


// 位置环PID参数
const float pos_Kp = 0.15; //.045 
const float pos_Ki = 0.00006;//00006
const int pos_Kd = 0;
long sum_delta_pos = 0;


// 速度环PID参数
const float speed_Kp = 2.0; //1.5
const float speed_Ki = 0; // 0.002
const float speed_Kd = 0;
long sum_delta_speed = 0;
int delta_speed = 0;
FastPID speed_PID(speed_Kp, speed_Ki, speed_Kd, 1000000/interval, 15, 1);

int TAG;

long position = 0; // 当前转子位置
float i1 = 136.53333 / (1000000 / interval);


int calc_Position(const long setPos)
{
  return constrain(
    pos_Kp *(setPos - position) + 0 * sum_delta_pos,
    -16384, 16384);
}

int calc_Speed(const long setSpeed)
{
  return speed_PID.step(setSpeed, RPM[0]);
  // if(setSpeed>1000)
  //   return constrain(
  //     speed_Kp * (setSpeed - RPM[0]) + speed_Ki * sum_delta_speed - speed_Kd * delta_speed ,
  //     -16384, 16384);
  // else
  //   return constrain(
  //     speed_Kp * (setSpeed - RPM[0]) + speed_Ki * sum_delta_speed - speed_Kd * delta_speed ,
  //     -16384, 16384);
}

void TEST(){
  if(TEST_SPEED == 5000)
    TEST_SPEED = 0;
  else
    TEST_SPEED = 5000;
}

void initMotor()
{
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Timer1.initialize(interval);         
  Timer1.attachInterrupt(checkCANmsg); // attach the service routine here
  // MsTimer2::set(3000,TEST);
  // MsTimer2::start();
}


// 更新电机各个参数
void updateInfo(const int motorID)
{

  angle[motorID] = toRealData(canMsgIn.data[0], canMsgIn.data[1]);
  RAW_RPM = toRealData(canMsgIn.data[2],canMsgIn.data[3]);
  // 卡尔曼滤波
  RPM[motorID] = (int)myFilter.getFilteredValue( RAW_RPM );
  // 不滤波
  // RPM[motorID] = toRealData(canMsgIn.data[2], canMsgIn.data[3]);  
  actualCurrent[motorID] = toRealData(canMsgIn.data[4], canMsgIn.data[5]);
  T[motorID] = canMsgIn.data[6];
  delta_speed = set_speed - RPM[0];
  sum_delta_speed += delta_speed;
  sum_delta_speed = constrain(sum_delta_speed,-16384000,16384000);
  // if(set_pos - position > 200)
    sum_delta_pos += set_pos - position;

  delta_pos = angle[motorID] - angle_last[motorID] ;
  if (abs(RPM[motorID]) < 1200)       // 低速状态
  { 
    // 转满一圈
    if (delta_pos < -8000)
      position ++;
    else if (delta_pos > 8000)
      position --;
    // 未转满一圈
    else
      position += delta_pos; 
  }
  else if (abs(RPM[motorID]) < 6000)  // 中速状态
  { 
    if (RPM[motorID] < 0 && delta_pos > 4000)
    {
      position -= 8192;
    }
    else if (-delta_pos > 4000 && RPM[motorID] > 0)
    {
      position += 8192;
    }
  }
  else   // 高速状态，直接取转速近似积分
  { 
    position += (RPM[motorID] * i1)*8192;
  }
  
  angle_last[motorID] = angle[motorID];

  
  // if (abs(set_pos - position) < 400 && RPM[motorID] < 300)
  //   set_speed = 0;
  // else 
  //   set_speed = calc_Position(set_pos);
  set_speed = TEST_SPEED;
  set_current = calc_Speed(set_speed);
  setMotorCurrent(0, set_current);
}

void makeMsg(const int motorID)
{ // make the msg to send
  canMsgOut.can_id = 0x200;
  canMsgOut.can_dlc = 8;
  canMsgOut.data[motorID] = (char)(setCurrent[motorID] / 256);     // High 8 bit
  canMsgOut.data[motorID + 1] = (char)(setCurrent[motorID] % 256); // Low 8 bit
}

void setMotorCurrent(const int motorID, const int current)
{ // set motor current  -16384 ~ 16384 ----> -20A ~ +20A
  setCurrent[motorID] = current;
  makeMsg(motorID);
  mcp2515.sendMessage(&canMsgOut);
}

void printMessage(int motorID)
{
  // Serial.print("ID：");
  // Serial.print("<<<<#########");
  // Serial.print(delta_pos);
  // Serial.print("\t");

  // Serial.print(long(position));
  // Serial.print("\t");
  // Serial.print(0.001 * pos_Kp *(set_pos - position));
  // Serial.print("\t");

  // Serial.print(set_pos);
  // Serial.print("\t");

  // Serial.print("Angle: ");
  Serial.print(RPM[motorID]);
  // Serial.print("\t");
  // Serial.print(set_speed);
  Serial.print("\t");

  // // Serial.print("RPM: ");
  // Serial.print(set_pos);
  // Serial.print("\t");

  // Serial.print("I: ");
  // Serial.print(actualCurrent[motorID]);
  // Serial.print("\t");

  // Serial.print("T: ");
  // Serial.print(T[motorID]);
  // Serial.print("\t");

  // Serial.print("PID: ");
  Serial.print(set_speed);
  Serial.print("\t");

   Serial.print(set_current+set_speed);
  Serial.print("\t");
  // Serial.print("\t#########>>>>");
  Serial.println();
}

int toRealData(unsigned char DataH, unsigned char DataL)
{
  return int(word(DataH, DataL));
}

bool CANmsgComing()
{
  return mcp2515.readMessage(&canMsgIn) == MCP2515::ERROR_OK;
}

void checkCANmsg()
{
  if (CANmsgComing())
  {
    updateInfo(canMsgIn.can_id - 0x201);
  }
}

void setup()
{
  Serial.begin(115200);
  initMotor();
}

void loop()
{

  // delay(500);
  int length;
  if (Serial.available() > 0){
    length = Serial.readBytes(recv, 50);
    recv[length] = '\0';
    TEST_SPEED = atol (recv);

  }
  // delay(100);s
  printMessage(canMsgIn.can_id - 0x201);
}
