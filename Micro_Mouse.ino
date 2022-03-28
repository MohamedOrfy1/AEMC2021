#include <Wire.h>
#include "Modules.h"
bool flag=true;
extern double startangle,newangle;
int F_sensor,R_sensor,L_sensor;

void setup() {
  Wire.begin();
  Init();
  setID();
}

void loop() 
{
  while(flag==true)
  {
    Serial.println("Not Calibrated");
    if(digitalRead(StartSW)==0)
    {
      Serial.println("Done Calibration");
      startangle=ReadCompass();
      flag=false;
    }
    else
      ReadCompass();
  }
  read_three_sensors(&R_sensor,&F_sensor,&L_sensor);
//     newangle=ReadCompass();
//  if (L_sensor > 150 ) {
//    turn_left(startangle);
//     Serial.println("Left");
//     //path_save('L');
// }
// else if (F_sensor > 100 ) {
//     Serial.println("Forward");
//     Forward(50,50);
//     //path_save('S');
//   } 
//   else if (R_sensor > 150 ) {
//   
//   turn_right(startangle);
//   path_save('R');
  //}
  //else if (L_sensor < 150 && F_sensor < 100 && R_sensor < 150 ) {
//    Stop();
//    delay(200);
    //startangle+=180;
//    delay(2000);
//    path_save('B');
//}
    //Forward_PID();
    //Serial.println(startangle);
if (L_sensor > 200){
  Hard_Left(70);
  Serial.println("Hard_Right");
  }
  else if (F_sensor > 150){
  Forward(40,40);
  if(L_sensor < 110){
    Right(30);
    Serial.println("Left");
  }
  if(L_sensor > 140){
    Left(30);
    Serial.println("Right");
  }
  //Serial.println("Forward");
  }
  else if (L_sensor < 140&&F_sensor<100 ){
    Hard_Right(70);
    Serial.println("Hard_Left");
  }

}
//double incAngle(startangle){
//  double prevAngle = 0;
//  if(prevAngle != startangle){
//    startangle += 90;
//  }
//  prevAngle = startangle
//}
//
//double decAngle(startangle){
//  double prevAngle = 0;
//  if(prevAngle != startangle){
//    startangle -= 90;
//  }
//  prevAngle = startangle
//}
