#include <SoftwareSerial.h>

#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer

#define SERVO_FRAME_HEADER         0x55
#define SERVO_MOVE_TIME_WRITE      1
#define SERVO_MOVE_TIME_READ       2
#define SERVO_MOVE_TIME_WAIT_WRITE 7
#define SERVO_MOVE_TIME_WAIT_READ  8
#define SERVO_MOVE_START           11
#define SERVO_MOVE_STOP            12
#define SERVO_ID_WRITE             13
#define SERVO_ID_READ              14
#define SERVO_ANGLE_OFFSET_ADJUST  17
#define SERVO_ANGLE_OFFSET_WRITE   18
#define SERVO_ANGLE_OFFSET_READ    19
#define SERVO_ANGLE_LIMIT_WRITE    20
#define SERVO_ANGLE_LIMIT_READ     21
#define SERVO_VIN_LIMIT_WRITE      22
#define SERVO_VIN_LIMIT_READ       23
#define SERVO_TEMP_MAX_LIMIT_WRITE 24
#define SERVO_TEMP_MAX_LIMIT_READ  25
#define SERVO_TEMP_READ            26
#define SERVO_VIN_READ             27
#define SERVO_POS_READ             28
#define SERVO_OR_MOTOR_MODE_WRITE  29
#define SERVO_OR_MOTOR_MODE_READ   30
#define SERVO_LOAD_OR_UNLOAD_WRITE 31
#define SERVO_LOAD_OR_UNLOAD_READ  32
#define SERVO_LED_CTRL_WRITE       33
#define SERVO_LED_CTRL_READ        34
#define SERVO_LED_ERROR_WRITE      35
#define SERVO_LED_ERROR_READ       36

//#define DEBUG 1  /*Debug ：print debug value*/

// CONSTANTS
// Right servos
const byte ID1 = 1;
// Left servos
const byte ID2 = 2;
// Front servos
const byte ID3 = 3;
// Back servos
const byte ID4 = 4;
// Servo motor pins
const byte RX_PIN_SERVO = 10;
const byte TX_PIN_SERVO = 11;
// Servo motors SoftwareSerial(rxPin, txPin)
SoftwareSerial serial_servo(RX_PIN_SERVO, TX_PIN_SERVO);

const float ANGLE = 0.24;
const int MIN_SPEED = 0;
const int MAX_SPEED = 1000;
const int MIN_ANGLE = 0;
const int HALF_ANGLE = 90;
const int MAX_ANGLE = 180;
const int TURN_TIME = 1000;

byte CheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++)
  {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

// Servo mode
void SerialServoMove(SoftwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if (position < 0)
    position = 0;
  if (position > 1000)
    position = 1000;
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = CheckSum(buf);
  SerialX.write(buf, 10);
}

void SerialServoStopMove(SoftwareSerial &SerialX, uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_MOVE_STOP;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);
}

void SerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = CheckSum(buf);
  SerialX.write(buf, 7);

#ifdef DEBUG
  Serial.println("SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

// Motor mode
void SerialServoSetMode(SoftwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = CheckSum(buf);

#ifdef DEBUG
  Serial.println("SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  SerialX.write(buf, 10);
}
void SerialServoLoad(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = CheckSum(buf);

  SerialX.write(buf, 7);

#ifdef DEBUG
  Serial.println("SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

void SerialServoUnload(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = CheckSum(buf);

  SerialX.write(buf, 7);

#ifdef DEBUG
  Serial.println("SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

int SerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available())
  {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted)
    {
      if (rxBuf == SERVO_FRAME_HEADER)
      {
        frameCount++;
        if (frameCount == 2)
        {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else
      {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted)
    {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3)
      {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7)
        {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3)
      {

#ifdef DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++)
        {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (CheckSum(recvBuf) == recvBuf[dataCount - 1])
        {

#ifdef DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}

int SerialServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_POS_READ;
  buf[5] = CheckSum(buf);

#ifdef DEBUG
  Serial.println("SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef DEBUG
  Serial.println(ret);
#endif
  return ret;
}
int SerialServoReadVin(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = SERVO_VIN_READ;
  buf[5] = CheckSum(buf);

#ifdef DEBUG
  Serial.println("SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;

#ifdef DEBUG
  Serial.println(ret);
#endif
  return ret;
}

int speed = MIN_SPEED;
int front_wheels_angle = HALF_ANGLE;
int back_wheels_angle = HALF_ANGLE;
float front_wheels_angle_turn = front_wheels_angle / ANGLE; // 90 / 0.24 = 375
float back_wheels_angle_turn = back_wheels_angle / ANGLE;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  // set the data rate for the SoftwareSerial port
  serial_servo.begin(115200);
  delay(1000);
}

void loop()
{
  if (Serial.available() > 0)
  {
    Serial.println("Serial available");

    int character = Serial.read();
    Serial.println(character);
    // q (stop turn)    w (forward)   e (align)
    // a (left)         s (stop)      d (right)
    // z (max left)     x (back)      c (max right)
    switch (character)
    {
    case 'w':
      Serial.println("Rover. Go forward.");
      // Go forward
      // If speed has not reached the max speed limit
      if (abs(speed) < MAX_SPEED)
      {
        speed = speed + 100;
      }
      Serial.println(speed);
      SerialServoSetMode(serial_servo, ID1, 1, -speed);
      SerialServoSetMode(serial_servo, ID2, 1, +speed);
      break;

    case 's':
      Serial.println("Rover. Stop.");
      // Stop
      speed = MIN_SPEED;
      SerialServoSetMode(serial_servo, ID1, 1, speed);
      SerialServoSetMode(serial_servo, ID2, 1, speed);
      break;

    case 'x':
      Serial.println("Rover. Go back.");
      // Go back
      // If speed has not reached the max speed limit
      if (abs(speed) < MAX_SPEED)
      {
        speed = speed - 100;
      }
      Serial.println(speed);
      SerialServoSetMode(serial_servo, ID1, 1, -speed);
      SerialServoSetMode(serial_servo, ID2, 1, +speed);
      break;

    case 'a':
      Serial.println("Rover. Turn left.");
      // Turn left
      // If ANGLE has not reached the min ANGLE (left)
      if (front_wheels_angle > MIN_ANGLE)
      {
        front_wheels_angle = front_wheels_angle - 10;
        front_wheels_angle_turn = front_wheels_angle / ANGLE;
      }
      SerialServoMove(serial_servo, ID3, front_wheels_angle_turn, TURN_TIME);
      break;
   
    case 'd':
      Serial.println("Rover. Turn right.");
      // Turn right
      // If ANGLE has not reached the max ANGLE (right)
      if (front_wheels_angle < MAX_ANGLE)
      {
        front_wheels_angle = front_wheels_angle + 10;
        front_wheels_angle_turn = front_wheels_angle / ANGLE;
      }
      SerialServoMove(serial_servo, ID3, front_wheels_angle_turn, TURN_TIME);
      break;
 
    case 'q':
      Serial.println("Rover. Stop turning.");
      // Stop turn
      SerialServoStopMove(serial_servo, ID3);
      SerialServoStopMove(serial_servo, ID4);
      break;
    
    case 'e':
      Serial.println("Rover. Align wheels.");
      // Center wheels
      front_wheels_angle = HALF_ANGLE;
      front_wheels_angle_turn = front_wheels_angle / ANGLE;
      SerialServoMove(serial_servo, ID3, front_wheels_angle_turn, TURN_TIME);
      // back_wheels_angle = HALF_ANGLE;
      // back_wheels_angle_turn = back_wheels_angle / ANGLE;
      // SerialServoMove(serial_servo, ID4, back_wheels_angle_turn, TURN_TIME);
      break;

    case 'z':
      Serial.println("Rover. Max left angle.");
      // Set max left angle
      front_wheels_angle = MIN_ANGLE;
      front_wheels_angle_turn = front_wheels_angle / ANGLE;
      SerialServoMove(serial_servo, ID3, front_wheels_angle_turn, TURN_TIME);
      // back_wheels_angle = MIN_ANGLE;
      // back_wheels_angle_turn = back_wheels_angle / ANGLE;
      // SerialServoMove(serial_servo, ID4, back_wheels_angle_turn, TURN_TIME);
      break;

    case 'c':
      Serial.println("Rover. Max right angle.");
      // Set max right angle
      front_wheels_angle = MAX_ANGLE;
      front_wheels_angle_turn = front_wheels_angle / ANGLE;
      SerialServoMove(serial_servo, ID3, front_wheels_angle_turn, TURN_TIME);
      // back_wheels_angle = MAX_ANGLE;
      // back_wheels_angle_turn = back_wheels_angle / ANGLE;
      // SerialServoMove(serial_servo, ID4, back_wheels_angle_turn, TURN_TIME);
      break;

    }
  }
}
