#ifndef MODULES_H_
#define MODULES_H_
#define BAUD_RATE       9600
#define IN1             PA0
#define IN2             PA1
#define IN3             PA2
#define IN4             PA3
#define StartSW         PA4
#define DipSW1          PA5
#define DipSW2          PA6
#define Debug_Buzzer    PB15
#define INITIAL_SPEED   50
#define KP              2
#define KD              0.5
#define KI              0
#define OUTPUT_LIMIT    255
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define SHT_LOX1        PA10
#define SHT_LOX2        PA9
#define SHT_LOX3        PA8

void Forward(unsigned int Speed1,unsigned int Speed2);
void Backward(unsigned int Speed1,unsigned int Speed2);
void Right(unsigned int Speed);
void Left(unsigned int Speed);
void Stop(void);
void Hard_Right(unsigned int Speed);
void Hard_Left(unsigned int Speed);

void Forward_PID(void);

void Init(void);

int ReadCompass(void);

void path_save(char direction);

void setID(void);
void read_three_sensors(int* reading1,int* reading2,int* reading3);
void turn_right(double startangle);
void turn_left(double startangle);
#endif
