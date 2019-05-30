#include <avr/pgmspace.h>
#include <Arduino.h>
#include "Pins.h"
#include "Ticker.h"
#include "PinChangeInt.h"
#include "Rgb.h"
#include <stdio.h>
#include "Music.h"
#include <EEPROM.h>

typedef unsigned long millis_t;

Ticker line_tracking;
Ticker key_mode;
Ticker ir_recevie;
Ticker voltage_measure;

int addr_line_tracking_threshold = 0;

int key_value = 0;

bool is_left_line_tracking = false;
bool is_right_line_tracking = false;
bool is_ir_recevie = false;

#define MAX_CMD_SIZE 96
#define BUFSIZE 4
uint8_t commands_in_queue = 0;
uint8_t cmd_queue_index_r = 0;
uint8_t cmd_queue_index_w = 0;
char command_queue[BUFSIZE][MAX_CMD_SIZE];
static int serial_count;
#define parameter_num_max 6          //最大参数数量
String parameter[parameter_num_max]; //参数值
float get_time = 0;
static millis_t get_time_delay;
int line_tracking_threshold = 150;
double voltage;

class L293
{
public:
  int left_1_pin;
  int left_2_pin;
  int right_1_pin;
  int right_2_pin;
  int enable_left_pin;
  int enable_right_pin;
  int car_speed = 0;
  int turn_speed = 0;
  uint16_t left_speed = 0;
  uint16_t right_speed = 0;

  enum RUN_STATUE
  {
    STOP,
    BACK,
    FORWARD,
    LEFT,
    RIGHT
  } run_statue = STOP;

  void init(int left1pin, int left2pin, int right1pin, int right2pin, int enableleftpin, int enablerightpin)
  {
    left_1_pin = left1pin;
    left_2_pin = left2pin;
    right_1_pin = right1pin;
    right_2_pin = right2pin;
    enable_left_pin = enableleftpin;
    enable_right_pin = enablerightpin;
    pinMode(left_1_pin, OUTPUT);
    pinMode(left_2_pin, OUTPUT);
    pinMode(right_1_pin, OUTPUT);
    pinMode(right_2_pin, OUTPUT);
    pinMode(enable_left_pin, OUTPUT);
    pinMode(enable_right_pin, OUTPUT);
    stop();
  }

  void leftFront(int leftspeed)
  {
    analogWrite(enable_left_pin, leftspeed);
    digitalWrite(left_1_pin, HIGH);
    digitalWrite(left_2_pin, LOW);
  }

  void leftBack(int leftspeed)
  {
    analogWrite(enable_left_pin, leftspeed);
    digitalWrite(left_1_pin, LOW);
    digitalWrite(left_2_pin, HIGH);
  }

  void leftStop()
  {
    analogWrite(enable_left_pin, 0);
    digitalWrite(left_1_pin, LOW);
    digitalWrite(left_2_pin, LOW);
  }

  void rightFront(int rightspeed)
  {
    analogWrite(enable_right_pin, rightspeed);
    digitalWrite(right_1_pin, LOW);
    digitalWrite(right_2_pin, HIGH);
  }

  void rightBack(int rightspeed)
  {
    analogWrite(enable_right_pin, rightspeed);
    digitalWrite(right_1_pin, HIGH);
    digitalWrite(right_2_pin, LOW);
  }

  void rightStop()
  {
    analogWrite(enable_right_pin, 0);
    digitalWrite(right_1_pin, LOW);
    digitalWrite(right_2_pin, LOW);
  }

  void forward(int speed)
  {
    run_statue = FORWARD;
    left_speed = speed;
    right_speed = speed;
    leftFront(speed);
    rightFront(speed);
  }

  void back(int speed)
  {
    run_statue = BACK;
    left_speed = speed;
    right_speed = speed;
    leftBack(speed);
    rightBack(speed);
  }

  void left(int speed)
  {
    run_statue = LEFT;
    left_speed = speed;
    right_speed = speed;
    leftBack(speed);
    rightFront(speed);
  }

  void right(int speed)
  {
    run_statue = RIGHT;
    left_speed = speed;
    right_speed = speed;
    leftFront(speed);
    rightBack(speed);
  }

  void stop()
  {
    run_statue = STOP;
    left_speed = 0;
    right_speed = 0;
    car_speed = 0;
    turn_speed = 0;
    leftStop();
    rightStop();
  }

  void left2(int speed)
  {
    run_statue = LEFT;
    left_speed = speed;
    right_speed = speed;
    leftFront(speed - 100);
    rightFront(speed);
  }

  void right2(int speed)
  {
    run_statue = RIGHT;
    left_speed = speed;
    right_speed = speed;
    leftFront(speed);
    rightFront(speed - 100);
  }

private:
} l293;

enum FUNCTION_MODE
{
  IDLE,
  LINE_TRACKING,
  OBSTACLE_AVOIDANCE,
  FOLLOW,
  BLUETOOTH,
  EXPLORE,
} function_mode = IDLE;

//命令
enum SERIAL_COMMAND
{
  CMD_NULL,  //无
  CMD_OA,    //查询避障数据
  CMD_LT,    //查询循迹数据
  CMD_TURN,  //控制小车运动
  CMD_MOVE,  //控制电机运动
  CMD_MOVES, //控制两个电机运动
  CMD_RGB,   //设置RGB颜色
  CMD_RGBS,  //设置两个RGB颜色
  CMD_RGBB,  //设置RGB亮度
  CMD_BEEP,  //设置蜂鸣器音调
  CMD_BEEPS, //设置蜂鸣器音调及时间
  CMD_KEY,   //设置小车功能模式
} serial_command = CMD_NULL;

void voltageInit()
{
  pinMode(VOL_MEASURE_PIN, INPUT);
  voltage_measure.start(voltageMeasure, 1000);
}
void voltageMeasure()
{
  voltage = analogRead(VOL_MEASURE_PIN) * 4.96 / 1024;
  if (voltage < 3.6)
  {
    rgb.flashRedColorFlag();
  }
  else
  {
    rgb.flashGreedYellowColorFlag();
  }
}

void followMode()
{
  l293.car_speed = 170;
  if (is_ir_recevie)
  {
    l293.forward(l293.car_speed);
  }
  else
  {
    l293.stop();
  }
}

void obstacleAvoidanceMode()
{
  l293.car_speed = 170;
  static millis_t delay_time = millis();
  static bool flag = false;

  if (flag)
  {
    if (millis() - delay_time > 150)
    {
      flag = false;
    }
  }
  else
  {
    if (is_ir_recevie)
    {
      random() % 2 ? l293.right(l293.car_speed) : l293.left(l293.car_speed);
      flag = true;
      delay_time = millis();
    }
    else
    {
      l293.forward(l293.car_speed);
    }
  }
}

void exploreMode()
{
  l293.car_speed = 170;
  static millis_t delay_time = millis();
  static bool flag = false;
  static bool stopFlag = false;
  if (is_left_line_tracking || is_right_line_tracking)
  {
    if (stopFlag)
    {
      stopFlag = false;
      l293.back(255);
      delay(50);
    }
    l293.stop();
  }
  else
  {
    stopFlag = true;
    if (flag)
    {
      if (millis() - delay_time > 150)
      {
        flag = false;
      }
    }
    else
    {
      if (is_ir_recevie)
      {
        random() % 2 ? l293.right(l293.car_speed) : l293.left(l293.car_speed);
        flag = true;
        delay_time = millis();
      }
      else
      {
        l293.forward(l293.car_speed);
      }
    }
  }
}

void getKeyValue()
{
  static bool key_flag = false;
  if (!digitalRead(KEY_MODE))
  {
    key_flag = true;
  }

  if (key_flag && digitalRead(KEY_MODE))
  {
    key_flag = false;
    key_value++;
    if (key_value >= 5)
    {
      key_value = 0;
    }
    switch (key_value)
    {
    case 0:
      function_mode = IDLE;
      l293.stop();
      rgb.lightOff();
      break;
    case 1:
      function_mode = LINE_TRACKING;
      rgb.brightGreenColor();
      break;
    case 2:
      function_mode = OBSTACLE_AVOIDANCE;
      rgb.brightYellowColor();
      break;
    case 3:
      function_mode = FOLLOW;
      rgb.brightBlueColor();
      break;
    case 4:
      function_mode = EXPLORE;
      rgb.brightWhiteColor();
      break;
    default:
      break;
    }
  }
}

void keyInit()
{
  pinMode(KEY_MODE, INPUT_PULLUP);
  key_mode.start(getKeyValue, 40);
}

void irInit()
{
  pinMode(IR_RECEIVE_PIN, INPUT);
  ir_recevie.start(getIrData, 20);
}

void getIrData()
{
  static int ir_recevie_data;
  int ir_recevie_threshold = 1000;
  ir_recevie_data = analogRead(IR_RECEIVE_PIN);
  // Serial.print(ir_recevie_data);
  // Serial.print("\n");
  is_ir_recevie = ir_recevie_data < ir_recevie_threshold ? true : false;
}

void getLineTrackingData()
{
  static int line_tracking_left_data;
  static int line_tracking_right_data;
  line_tracking_left_data = analogRead(LINE_TRACKING_LEFT_PIN);
  line_tracking_right_data = analogRead(LINE_TRACKING_RIGHT_PIN);
  // Serial.print(line_tracking_left_data);
  // Serial.print("\t");
  // Serial.print(line_tracking_right_data);
  // Serial.print("\n");
  is_left_line_tracking = line_tracking_left_data >= line_tracking_threshold ? true : false;
  is_right_line_tracking = line_tracking_right_data >= line_tracking_threshold ? true : false;
}

void lineTrackingInit()
{
  pinMode(LINE_TRACKING_LEFT_PIN, INPUT);
  pinMode(LINE_TRACKING_RIGHT_PIN, INPUT);
  int t = 0;
  EEPROM.get(addr_line_tracking_threshold, t);
  if (t != -1)
  {
    line_tracking_threshold = t;
  }
//  Serial.print("line_tracking_threshold=");
//  Serial.println(line_tracking_threshold);
  line_tracking.start(getLineTrackingData, 20);
}

void lineTrackingMode()
{
  //将电压（电量）转化为电机转速（占空比），实现小车平稳运行。
  l293.car_speed = map(voltage * 10, 3.6 * 10, 4.2 * 10, 200, 150);
  if (is_left_line_tracking && is_right_line_tracking)
  {
    l293.stop();
  }
  else if (is_left_line_tracking)
  {
    l293.left(l293.car_speed);
    // delay(20);
  }
  else if (is_right_line_tracking)
  {
    l293.right(l293.car_speed);
    // delay(20);
  }
  else
  {
    l293.forward(l293.car_speed);
  }
}

inline bool enqueuecommand(const char *cmd)
{
  if (commands_in_queue >= BUFSIZE)
    return false;
  strcpy(command_queue[cmd_queue_index_w], cmd);
  if (++cmd_queue_index_w >= BUFSIZE)
  {
    cmd_queue_index_w = 0;
  }
  commands_in_queue++;
  return true;
}

inline void get_command()
{
  // static millis_t printTime;
  // printTime = millis();
  static char serial_line_buffer[MAX_CMD_SIZE];
  static bool command_start = false;
  int c;
  while (commands_in_queue < BUFSIZE && (c = Serial.read()) >= 0)
  {
    char serial_char = c;
    if (serial_char == '{')
    {
      if (command_start)
      {
        serial_line_buffer[serial_count] = 0;
        serial_count = 0;
      }
      else
      {
        command_start = true;
      }
    }
    else if (command_start)
    {
      if (serial_char == '}')
      {
        if (!serial_count)
        {
          continue;
        }
        serial_line_buffer[serial_count] = 0;
        serial_count = 0;
        if (command_start)
        {
          command_start = false;
          enqueuecommand(serial_line_buffer);
          // Serial.print(millis() - printTime);
        }
      }
      else if (serial_count >= MAX_CMD_SIZE - 1)
      {
      }
      else if (serial_char == '\\')
      {
        if ((c = Serial.read()) >= 0)
          serial_line_buffer[serial_count++] = (char)c;
      }
      else
      {
        serial_line_buffer[serial_count++] = serial_char;
      }
    }
  }
}

void process_parsed_command()
{
  int index_start[parameter_num_max]; //参数开始位置
  int index_end[parameter_num_max];   //参数结束位置
  int parameter_num = 0;              //参数数量
  int car_speed;
  String current_command(command_queue[cmd_queue_index_r]); //将命令转化为String类
  int i = 0;

  // Serial.println(current_command);
  // return;

  //处理第一个参数
  index_start[i] = current_command.indexOf('[');
  if (index_start[i] <= 0)
  {
    return;
  }
  index_end[i] = current_command.indexOf(']');
  if (index_end[i] <= 0 || index_start[i] >= index_end[i])
  {
    return;
  }
  parameter_num++;
  String command_head = current_command.substring(0, index_start[i]); //读取命令头部
  parameter[i] = current_command.substring(index_start[i] + 1, index_end[i]);

  //处理剩余的参数
  for (++i; i < parameter_num_max; i++)
  {
    index_start[i] = current_command.indexOf('[', index_end[i - 1] + 1);
    if (index_start[i] <= 0)
    {
      break;
    }
    index_end[i] = current_command.indexOf(']', index_end[i - 1] + 1);
    if (index_end[i] <= 0 || index_start[i] >= index_end[i])
    {
      index_start[i] = 0;
      index_end[i] = 0;
      break;
    }
    parameter_num++;
    parameter[i] = current_command.substring(index_start[i] + 1, index_end[i]);
  }

  if (command_head == "OA" && parameter_num == 1)
  {
    if (parameter[0] == "?")
    {
      is_ir_recevie ? Serial.print("{true}") : Serial.print("{false}");
    }
    else
    {
      return;
    }
  }
  else if (command_head == "V" && parameter_num == 1)
  {
    if (parameter[0] == "?")
    {
      Serial.print(voltage);
    }
    else
    {
      return;
    }
  }
  else if (command_head == "LT" && parameter_num == 2)
  {
    if (parameter[1] == "?")
    {
      if (parameter[0] == "0")
      {
        is_left_line_tracking ? Serial.print("{true}") : Serial.print("{false}");
      }
      else if (parameter[0] == "1")
      {
        is_right_line_tracking ? Serial.print("{true}") : Serial.print("{false}");
      }
      else
      {
        return;
      }
    }
    else
    {
      return;
    }
  }
  else if (command_head == "TURN" && parameter_num == 2)
  {
    if (parameter[1] == "0")
    {
      car_speed = 0;
    }
    if (parameter[1] == "300")
    {
      car_speed = 300;
    }
    else
    {
      if (parameter[1].toInt() != 0)
      {
        car_speed = parameter[1].toInt();
      }
      else
      {
        return;
      }
    }

    if (car_speed != 300)
    {
      l293.car_speed = car_speed;
    }
    else
    {
      if (parameter[0] != "0" && l293.car_speed == 0)
      {
        l293.car_speed = 255;
      }
    }

    if (parameter[0] == "0")
    {
      l293.stop();
    }
    else if (parameter[0] == "1")
    {
      l293.forward(l293.car_speed);
    }
    else if (parameter[0] == "2")
    {
      l293.back(l293.car_speed);
    }
    else if (parameter[0] == "3")
    {
      l293.left(l293.car_speed);
    }
    else if (parameter[0] == "4")
    {
      l293.right(l293.car_speed);
    }
    else
    {
      return;
    }
    Serial.print("{ok}");
  }
  else if (command_head == "TURNS" && parameter_num == 3)
  {
    if (parameter[1] == "0")
    {
      car_speed = 0;
    }
    if (parameter[1] == "300")
    {
      car_speed = 300;
    }
    else
    {
      if (parameter[1].toInt() != 0)
      {
        car_speed = parameter[1].toInt();
      }
      else
      {
        return;
      }
    }

    if (car_speed != 300)
    {
      l293.car_speed = car_speed;
    }
    else
    {
      if (parameter[0] != "0" && l293.car_speed == 0)
      {
        l293.car_speed = 255;
      }
    }

    if (parameter[0] == "0")
    {
      l293.stop();
    }
    else if (parameter[0] == "1")
    {
      l293.forward(l293.car_speed);
    }
    else if (parameter[0] == "2")
    {
      l293.back(l293.car_speed);
    }
    else if (parameter[0] == "3")
    {
      l293.left2(l293.car_speed);
    }
    else if (parameter[0] == "4")
    {
      l293.right2(l293.car_speed);
    }
    else
    {
      return;
    }

    if (parameter[2] == "0")
    {
      rgb.lightOff();
    }
    else if (parameter[2] == "1")
    {
      rgb.led_rgb_new[0] = 0xA020F0; //紫色
      rgb.led_rgb_old[0] = 0xA020F0;
      rgb.led_rgb_new[1] = 0xA020F0;
      rgb.led_rgb_old[1] = 0xA020F0;
    }
    else if (parameter[2] == "2")
    {
      rgb.led_rgb_new[0] = 0xA020F0; //紫色
      rgb.led_rgb_old[0] = 0xA020F0;
      rgb.led_rgb_new[1] = 0xA020F0;
      rgb.led_rgb_old[1] = 0xA020F0;
    }

    // char *ptr;
    // rgb.led_rgb_new[0] = strtol(&parameter[2][0], &ptr, 16);
    // rgb.led_rgb_new[1] = strtol(&parameter[2][0], &ptr, 16);
    // rgb.led_rgb_old[0] = strtol(&parameter[2][0], &ptr, 16);
    // rgb.led_rgb_old[1] = strtol(&parameter[2][0], &ptr, 16);
    // rgb.brightBlueColor();
    // Serial.println(strtol(&parameter[2][0], &ptr, 16), HEX);
    // Serial.println(commands_in_queue);
  }
  else if (command_head == "MOVE" && parameter_num == 3)
  {
    if (parameter[2] == "0")
    {
      car_speed = 0;
    }
    if (parameter[2] == "300")
    {
      car_speed = 300;
    }
    else
    {
      if (parameter[2].toInt() != 0)
      {
        car_speed = parameter[2].toInt();
      }
      else
      {
        return;
      }
    }

    if (parameter[0] == "0")
    {
      if (car_speed != 300)
      {
        l293.car_speed = car_speed;
      }
      else
      {
        if ((parameter[1] == "1" || parameter[2] == "2") && l293.car_speed == 0)
        {
          l293.car_speed = 255;
        }
      }

      if (parameter[1] == "0")
      {
        l293.stop();
      }
      else if (parameter[1] == "1")
      {
        l293.forward(l293.car_speed);
      }
      else if (parameter[1] == "2")
      {
        l293.back(l293.car_speed);
      }
      else if (parameter[1] == "3")
      {
        switch (l293.run_statue)
        {
        case L293::RUN_STATUE::STOP:
          l293.stop();
          break;
        case L293::RUN_STATUE::BACK:
          l293.back(l293.car_speed);
          break;
        case L293::RUN_STATUE::FORWARD:
          l293.forward(l293.car_speed);
          break;
        case L293::RUN_STATUE::LEFT:
          l293.left(l293.car_speed);
          break;
        case L293::RUN_STATUE::RIGHT:
          l293.right(l293.car_speed);
          break;
        default:
          break;
        }
      }
      else
      {
        return;
      }
    }
    else if (parameter[0] == "1")
    {
      if (car_speed != 300)
      {
        l293.left_speed = car_speed;
      }
      else
      {
        if ((parameter[1] == "1" || parameter[2] == "2") && l293.left_speed == 0)
        {
          l293.left_speed = 255;
        }
      }

      if (parameter[1] == "0")
      {
        l293.leftStop();
        l293.run_statue = L293::RUN_STATUE::STOP;
      }
      else if (parameter[1] == "1")
      {
        l293.run_statue = L293::RUN_STATUE::FORWARD;
        l293.leftFront(l293.left_speed);
      }
      else if (parameter[1] == "2")
      {
        l293.run_statue = L293::RUN_STATUE::BACK;
        l293.leftBack(l293.left_speed);
      }
      else if (parameter[1] == "3")
      {
        switch (l293.run_statue)
        {
        case L293::RUN_STATUE::STOP:
          l293.leftStop();
          l293.run_statue = L293::RUN_STATUE::STOP;
          break;
        case L293::RUN_STATUE::BACK:
          l293.leftBack(l293.left_speed);
          l293.run_statue = L293::RUN_STATUE::BACK;
          break;
        case L293::RUN_STATUE::FORWARD:
          l293.leftFront(l293.left_speed);
          l293.run_statue = L293::RUN_STATUE::FORWARD;
          break;
        case L293::RUN_STATUE::LEFT:
          l293.leftBack(l293.left_speed);
          l293.run_statue = L293::RUN_STATUE::BACK;
          break;
        case L293::RUN_STATUE::RIGHT:
          l293.leftFront(l293.left_speed);
          l293.run_statue = L293::RUN_STATUE::FORWARD;
          break;
        default:
          break;
        }
      }
      else
      {
        return;
      }
    }
    else if (parameter[0] == "2")
    {
      if (car_speed != 300)
      {
        l293.right_speed = car_speed;
      }
      else
      {
        if ((parameter[1] == "1" || parameter[1] == "2") && l293.right_speed == 0)
        {
          l293.right_speed = 255;
        }
      }

      if (parameter[1] == "0")
      {
        l293.rightStop();
        l293.run_statue = L293::RUN_STATUE::STOP;
      }
      else if (parameter[1] == "1")
      {
        l293.rightFront(l293.right_speed);
        l293.run_statue = L293::RUN_STATUE::FORWARD;
      }
      else if (parameter[1] == "2")
      {
        l293.rightBack(l293.right_speed);
        l293.run_statue = L293::RUN_STATUE::BACK;
      }
      else if (parameter[1] == "3")
      {
        switch (l293.run_statue)
        {
        case L293::RUN_STATUE::STOP:
          l293.run_statue = L293::RUN_STATUE::STOP;
          l293.rightStop();
          break;
        case L293::RUN_STATUE::BACK:
          l293.rightBack(l293.right_speed);
          l293.run_statue = L293::RUN_STATUE::BACK;
          break;
        case L293::RUN_STATUE::FORWARD:
          l293.rightFront(l293.right_speed);
          l293.run_statue = L293::RUN_STATUE::FORWARD;
          break;
        case L293::RUN_STATUE::LEFT:
          l293.rightFront(l293.right_speed);
          l293.run_statue = L293::RUN_STATUE::FORWARD;
          break;
        case L293::RUN_STATUE::RIGHT:
          l293.rightBack(l293.right_speed);
          l293.run_statue = L293::RUN_STATUE::RIGHT;
          break;
        default:
          break;
        }
      }
      else
      {
        return;
      }
    }
    else
    {
      return;
    }
    Serial.print("{ok}");
  }
  else if (command_head == "MOVES" && parameter_num == 4)
  {
    if (parameter[1] == "0")
    {
      car_speed = 0;
    }
    if (parameter[1] == "300")
    {
      car_speed = 300;
    }
    else
    {
      if (parameter[1].toInt() != 0)
      {
        car_speed = parameter[1].toInt();
      }
      else
      {
        return;
      }
    }

    if (car_speed != 300)
    {
      l293.left_speed = car_speed;
    }
    else
    {
      if ((parameter[0] == "1" || parameter[0] == "2") && l293.left_speed == 0)
      {
        l293.left_speed = 255;
      }
    }

    if (parameter[0] == "0")
    {
      l293.left_speed = 0;
      l293.leftStop();
    }
    else if (parameter[0] == "1")
    {
      l293.leftFront(l293.left_speed);
    }
    else if (parameter[0] == "2")
    {
      l293.leftBack(l293.left_speed);
    }
    else if (parameter[0] == "3")
    {
    }
    else
    {
      return;
    }

    if (parameter[3] == "0")
    {
      car_speed = 0;
    }
    if (parameter[3] == "300")
    {
      car_speed = 300;
    }
    else
    {
      if (parameter[3].toInt() != 0)
      {
        car_speed = parameter[3].toInt();
      }
      else
      {
        return;
      }
    }

    if (car_speed != 300)
    {
      l293.right_speed = car_speed;
    }
    else
    {
      if ((parameter[2] == "1" || parameter[2] == "2") && l293.right_speed == 0)
      {
        l293.right_speed = 255;
      }
    }

    if (parameter[2] == "0")
    {
      l293.right_speed = 0;
      l293.rightStop();
    }
    else if (parameter[2] == "1")
    {
      l293.rightFront(l293.right_speed);
    }
    else if (parameter[2] == "2")
    {
      l293.rightBack(l293.right_speed);
    }
    else if (parameter[2] == "3")
    {
    }
    else
    {
      return;
    }
    Serial.print("{ok}");
  }
  else if (command_head == "RGB" && parameter_num == 6)
  {
    if (parameter[3] == "0")
    {
      rgb.led_rgb_new[0] = rgb.Color(parameter[0].toInt(), parameter[1].toInt(), parameter[2].toInt());
      rgb.led_rgb_new[1] = rgb.Color(parameter[0].toInt(), parameter[1].toInt(), parameter[2].toInt());
      if (parameter[5] == "0")
      {
        rgb.led_rgb_old[0] = rgb.Color(parameter[0].toInt(), parameter[1].toInt(), parameter[2].toInt());
        rgb.led_rgb_old[1] = rgb.Color(parameter[0].toInt(), parameter[1].toInt(), parameter[2].toInt());
      }
      else
      {
        rgb.led_rgb_old[0] = rgb.Color(0, 0, 0);
        rgb.led_rgb_old[1] = rgb.Color(0, 0, 0);
      }
    }
    else if (parameter[3] == "1")
    {
      rgb.led_rgb_new[1] = rgb.Color(parameter[0].toInt(), parameter[1].toInt(), parameter[2].toInt());
      if (parameter[5] == "0")
      {
        rgb.led_rgb_old[1] = rgb.Color(parameter[0].toInt(), parameter[1].toInt(), parameter[2].toInt());
      }
      else
      {
        rgb.led_rgb_old[1] = rgb.Color(0, 0, 0);
      }
    }
    else if (parameter[3] == "2")
    {
      rgb.led_rgb_new[0] = rgb.Color(parameter[0].toInt(), parameter[1].toInt(), parameter[2].toInt());
      if (parameter[5] == "0")
      {
        rgb.led_rgb_old[0] = rgb.Color(parameter[0].toInt(), parameter[1].toInt(), parameter[2].toInt());
      }
      else
      {
        rgb.led_rgb_old[0] = rgb.Color(0, 0, 0);
      }
    }
    else
    {
      return;
    }

    if (parameter[4] == "0")
    {
      get_time = 0;
    }
    else if (parameter[4].toInt() != 0)
    {
      get_time = parameter[4].toInt();
    }
    else if (parameter[4].toFloat() != 0)
    {
      get_time = parameter[4].toFloat();
    }
    else
    {
      return;
    }

    if (get_time == 0)
    {
      Serial.print("{ok}");
    }
    else
    {
      serial_command = CMD_RGB;
      get_time_delay = millis();
    }
  }
  else if (command_head == "RGBS" && parameter_num == 4)
  {
    char *ptr;
    rgb.led_rgb_new[1] = strtol(&parameter[0][0], &ptr, 16);
    if (parameter[1] == "0")
    {
      rgb.led_rgb_old[1] = strtol(&parameter[0][0], &ptr, 16);
    }
    else
    {
      rgb.led_rgb_old[1] = rgb.Color(0, 0, 0);
    }

    rgb.led_rgb_new[0] = strtol(&parameter[2][0], &ptr, 16);
    if (parameter[3] == "0")
    {
      rgb.led_rgb_old[0] = strtol(&parameter[2][0], &ptr, 16);
    }
    else
    {
      rgb.led_rgb_old[0] = rgb.Color(0, 0, 0);
    }
    Serial.print("{ok}");
  }
  else if (command_head == "RGBB" && parameter_num == 1)
  {
    if (parameter[0] == "0")
    {
      rgb.brightness = 0;
    }
    else if (parameter[0].toInt() != 0)
    {
      if (parameter[0].toInt() > 255)
      {
        rgb.brightness = 255;
      }
      else
      {
        rgb.brightness = parameter[0].toInt();
      }
    }
    rgb.setBrightness(rgb.brightness);
    Serial.print("{ok}");
  }
  else if (command_head == "BEEP" && parameter_num == 1)
  {
    if (parameter[0] == "0")
    {
      noTone(BEEP_PIN);
      Serial.print("{ok}");
    }
    else
    {
      if (parameter[0].toInt() != 0)
      {
        get_time = parameter[0].toInt();
      }
      else if (parameter[0].toFloat() != 0)
      {
        get_time = parameter[0].toFloat();
      }
      else
      {
        return;
      }
      serial_command = CMD_BEEP;
      get_time_delay = millis();
    }
  }
  else if (command_head == "BEEPS" && parameter_num == 2)
  {
    if (parameter[1] == "0")
    {
      get_time = 0;
      Serial.print("{ok}");
    }
    else
    {
      if (parameter[1].toInt() != 0)
      {
        get_time = parameter[1].toInt();
      }
      else if (parameter[1].toFloat() != 0)
      {
        get_time = parameter[1].toFloat();
      }
      else
      {
        return;
      }
      serial_command = CMD_BEEPS;
      get_time_delay = millis();
    }
    tone(BEEP_PIN, parameter[0].toInt());
  }
  else if (command_head == "KEY" && parameter_num == 1)
  {
    if (parameter[0] == "0")
    {
      l293.stop();
      function_mode = IDLE;
      rgb.lightOffAll();
      key_value = 0;
    }
    else if (parameter[0] == "1")
    {
      function_mode = LINE_TRACKING;
      key_value = 1;
      rgb.brightGreenColor();
    }
    else if (parameter[0] == "2")
    {
      function_mode = OBSTACLE_AVOIDANCE;
      key_value = 2;
      rgb.brightYellowColor();
    }
    else if (parameter[0] == "3")
    {
      function_mode = FOLLOW;
      key_value = 3;
      rgb.brightBlueColor();
    }
    else if (parameter[0] == "4")
    {
      function_mode = EXPLORE;
      key_value = 4;
      rgb.brightWhiteColor();
    }
    else
    {
      return;
    }
    Serial.print("{ok}");
  }
  else if (command_head == "CLEAR" && parameter_num == 1)
  {
    if (parameter[0] == "0")
    {
      rgb.lightOff();
      function_mode = IDLE;
      key_value = 0;
      l293.stop();
      noTone(BEEP_PIN);
    }
    else if (parameter[0] == "1")
    {
      rgb.lightOff();
    }
    else if (parameter[0] == "2")
    {
      function_mode = IDLE;
      key_value = 0;
      l293.stop();
    }
    else if (parameter[0] == "3")
    {
      noTone(BEEP_PIN);
    }
    else
    {
      return;
    }
    Serial.print("{ok}");
  }
  else if (command_head == "Tvalue" && parameter_num == 1)
  {
    char *ptr;
    line_tracking_threshold = strtol(&parameter[0][0], &ptr, 10);
    EEPROM.put(addr_line_tracking_threshold, line_tracking_threshold);
//    Serial.print("line_tracking_threshold=");
//    Serial.println(line_tracking_threshold);
    Serial.print("{ok}");
  }
  else
  {
    return;
  }

  if (command_head != "KEY" && command_head != "BEEPS")
  {
    function_mode = BLUETOOTH;
  }
}

void functionMode()
{
  switch (function_mode)
  {
  case IDLE:
    break;
  case LINE_TRACKING:
    lineTrackingMode();
    break;
  case OBSTACLE_AVOIDANCE:
    obstacleAvoidanceMode();
    break;
  case FOLLOW:
    followMode();
    break;
  case BLUETOOTH:
    break;
  case EXPLORE:
    exploreMode();
    break;
  default:
    break;
  }
}

void cmdDelayTime()
{
  switch (serial_command)
  {
  case CMD_NULL:
    break;
  case CMD_RGB:
    if (get_time != 0)
    {
      if (millis() - get_time_delay > (millis_t)(get_time * 1000))
      {
        if (parameter[3] == "0")
        {
          rgb.lightOff();
        }
        else if (parameter[3] == "1")
        {
          rgb.led_rgb_new[1] = rgb.Color(0, 0, 0);
          rgb.led_rgb_old[1] = rgb.Color(0, 0, 0);
        }
        else if (parameter[3] == "2")
        {
          rgb.led_rgb_new[0] = rgb.Color(0, 0, 0);
          rgb.led_rgb_old[0] = rgb.Color(0, 0, 0);
        }
        else
        {
          rgb.lightOff();
        }
        Serial.print("{ok}");
        serial_command = CMD_NULL;
        get_time = 0;
      }
    }
    break;
  case CMD_BEEPS:
    if (get_time != 0)
    {
      if (millis() - get_time_delay > (millis_t)(get_time * 1000))
      {
        noTone(BEEP_PIN);
        Serial.print("{ok}");
        serial_command = CMD_NULL;
        get_time = 0;
      }
    }
    break;
  case CMD_BEEP:
    if (get_time != 0)
    {
      if (get_time == 1)
      {
        play1();
      }
      serial_command = CMD_NULL;
    }
    break;
  default:
    break;
  }
}

void setup()
{
  Serial.begin(9600);
  lineTrackingInit();
  l293.init(L293_LEFT_1_PIN, L293_LEFT_2_PIN, L293_RIGHT_1_PIN, L293_RIGHT_2_PIN, L293_ENABLE_LEFT_PIN, L293_ENABLE_RIGHT_PIN);
  keyInit();
  pinMode(BEEP_PIN, OUTPUT);
  voltageInit();
  irInit();
  rgb.initialize();
}

void loop()
{
  if (commands_in_queue < BUFSIZE)
  {
    get_command();
  }

  if (commands_in_queue)
  {
    process_parsed_command();
    if (commands_in_queue)
    {
      --commands_in_queue;
      if (++cmd_queue_index_r >= BUFSIZE)
        cmd_queue_index_r = 0;
    }
  }

  line_tracking.update();
  key_mode.update();
  ir_recevie.update();
  voltage_measure.update();
  rgb.blink(100);
  functionMode();
  cmdDelayTime();
  // static millis_t print_time = 0;
  // if (millis() - print_time >= 2000)
  // {
  //   print_time = millis();
  //   Serial.println(l293.car_speed);
  //   Serial.println(voltage);
  // }
}
