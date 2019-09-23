#include <SPI.h>
#include <mcp2515.h>
#include <TimerOne.h>

#define window_size 5

struct can_frame canMsgIn;  // CAN packet received
struct can_frame canMsgOut; // CAN packet to send
MCP2515 mcp2515(53);        // CS(SS) to pin 53 on MEGA2560
int angle[2] = {0};      // Suppose 2 motors totally
int angle_last[2] = {0};
int RPM[2] = {0};
int actualCurrent[2] = {0};
int setCurrent[2] = {0};
int T[2] = {0};

int RPM_est[window_size] = {0};
int slide_R = 0;              // RPM的均值滤波的flag

const long interval = 500;    // 采样间隔 单位ns
int delta_pos;                // 两次采样的转子位置差值

char recv[20];

int set_pos = 1000;
int set_speed = 500;


// 位置环PID参数
const int pos_Kp = 200; 
const int pos_Ki = 0;
const int pos_Kd = 0;

// 速度环PID参数
const int speed_Kp = 5;
const int speed_Ki = 0;
const int speed_Kd = 0;

int TAG;

long position = 0; // 当前转子位置，+1圈 = +10
float i1 = 136.53333 / (1000000 / interval);

int update_RPM_est(int newRPM) // RPM的均值滤波
{
  int total = 0;
  RPM_est[slide_R] = newRPM;
  slide_R++;
  slide_R = slide_R % window_size;
  for (int i = 0; i < window_size; i++)
    total += RPM_est[i];
  return total / window_size;
}

int PIDout(long setPos)
{
  return constrain(pos_Kp * (setPos - position), -16384, 16384);
}

int PID_Speed(long setSpeed)
{
  return constrain(speed_Kp * (setSpeed - RPM[0]), -16384, 16384);
}

void initMotor()
{
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Timer1.initialize(interval);         
  Timer1.attachInterrupt(checkCANmsg); // attach the service routine here
}

// 更新电机各个参数
void updateInfo(int motorID)
{

  angle[motorID] = toRealData(canMsgIn.data[0], canMsgIn.data[1]);
  RPM[motorID] = update_RPM_est( toRealData(canMsgIn.data[2],canMsgIn.data[3]) ) ; // 使用均值滤波
  // RPM[motorID] = toRealData(canMsgIn.data[2], canMsgIn.data[3]);  // 不滤波
  actualCurrent[motorID] = toRealData(canMsgIn.data[4], canMsgIn.data[5]);
  T[motorID] = canMsgIn.data[6];

  delta_pos = angle[motorID] - angle_last[motorID] ;
  if (RPM[motorID] < 1000 && RPM[motorID] > -1000)       // 低速状态
  { 
    // 转满一圈
    if (delta_pos < -7000)
      position += 10;
    else if (delta_pos > 7000)
      position -= 10;
    // 未转满一圈
    else
      position += (delta_pos * 10) / 8192; 
  }
  else if (abs(RPM[motorID]) < 8000)  // 中速状态
  { 
    if (RPM[motorID] < 0 && delta_pos > 5000)
    {
      position -= 10;
    }
    else if (-delta_pos > 5000 && RPM[motorID] > 0)
    {
      position += 10;
    }
  }
  else   // 高速状态，直接取转速近似积分
  { 
    position += (long)(RPM[motorID] * i1)*10;
  }

  angle_last[motorID] = angle[motorID];
  setMotorCurrent(0, PID_Speed(set_speed));
}

void makeMsg(int motorID)
{ // make the msg to send
  canMsgOut.can_id = 0x200;
  canMsgOut.can_dlc = 8;
  canMsgOut.data[motorID] = (char)(setCurrent[motorID] / 256);     // High 8 bit
  canMsgOut.data[motorID + 1] = (char)(setCurrent[motorID] % 256); // Low 8 bit
}

void setMotorCurrent(int motorID, int current)
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

  // Serial.print(position);
  // Serial.print(set_pos);
  // Serial.print("\t");

  // Serial.print("Angle: ");
  Serial.print(RPM[motorID]);
  Serial.print("\t");

  // Serial.print("RPM: ");
  // Serial.print(set_pos);
  // Serial.print("\t");

  // Serial.print("I: ");
  // Serial.print(actualCurrent[motorID]);
  // Serial.print("\t");

  // Serial.print("T: ");
  // Serial.print(T[motorID]);
  // Serial.print("\t");

  // Serial.print("PID: ");
  Serial.print(PID_Speed(set_speed));
  Serial.print("\t");

  //  Serial.print(TAG);
  // Serial.print("\t");
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
    length = Serial.readBytes( recv, 20);
    recv[length] = '\0';
    set_speed = atoi(recv);
    Serial.print("Set speed to");
    Serial.println(recv);
  }
  
  // printMessage(canMsgIn.can_id - 0x201);
}
