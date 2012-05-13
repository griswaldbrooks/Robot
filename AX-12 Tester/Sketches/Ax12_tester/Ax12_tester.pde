#include <ax12.h>

int current_pos;
int faux_pos;
int temp;

void setup(){
  ax12Init(1000000);
  //ax12SetRegister2(1, 32, 10);
//  ax12SetRegister(1, AX_CW_COMPLIANCE_SLOPE, 1);
//  ax12SetRegister(1, AX_CCW_COMPLIANCE_SLOPE, 1);
//  ax12SetRegister(5, AX_CW_COMPLIANCE_SLOPE, 1);
//  ax12SetRegister(5, AX_CCW_COMPLIANCE_SLOPE, 1);
//  ax12SetRegister(9, AX_CW_COMPLIANCE_SLOPE, 1);
//  ax12SetRegister(9, AX_CCW_COMPLIANCE_SLOPE, 1);
//  ax12SetRegister(13, AX_CW_COMPLIANCE_SLOPE, 1);
//  ax12SetRegister(13, AX_CCW_COMPLIANCE_SLOPE, 1);
  Serial.begin(115200);
  //current_pos = ax12GetRegister(1, 36, 2);
  //temp = 512 - current_pos;
  //faux_pos = abs(temp);
}

int get_intpos(int degree){
  return (int)(3.413*(float)degree - 1.0f);
}

void loop(){
//  SetPosition(1, get_intpos(1));  
//   ax12ReadPacket(2);
//  //delay(20);
//  while(current_pos > get_intpos(3)){
////    ax12SetRegister2(1, 32, -2*faux_pos + 1024 + 10);
//    ax12ReadPacket(2);
//    //delay(20);
////Serial.print("Go to 0: ");
//    current_pos = ax12GetRegister(1, 36, 2);
//    while(current_pos == -1){
//      delay(8);
//      current_pos = ax12GetRegister(1, 36, 2);
//    }
////    temp = 512 - current_pos;
////    faux_pos = abs(temp);
////Serial.println(current_pos);
//  }
//  
//  delay(8);
//
//  SetPosition(1, get_intpos(60));  
//  ax12ReadPacket(2);
//  //delay(20);
//    while(current_pos < get_intpos(58)){
////      ax12SetRegister2(1, 32, -2*faux_pos + 1024 + 10);
//      ax12ReadPacket(2);
//      //delay(20);
////      Serial.print("Go to 1023: ");
//      current_pos = ax12GetRegister(1, 36, 2);
//      while(current_pos == -1){
//        delay(8);
//        current_pos = ax12GetRegister(1, 36, 2);
//      }
////      temp = 512 - current_pos;
////      faux_pos = abs(temp);
////      Serial.println(current_pos);
//  }
//  
//  delay(8);

  SetPosition(1, 511);
  SetPosition(5, 511);
  SetPosition(9, 511);
  SetPosition(13, 511);
  
  SetPosition(2, 511);
  SetPosition(6, 511);
  SetPosition(10, 511);
  SetPosition(14, 511);
  
  SetPosition(3, 511);
  SetPosition(7, 511);
  SetPosition(11, 511);
  SetPosition(15, 511);
  
  SetPosition(4, 511);
  SetPosition(8, 511);
  SetPosition(12, 511);
  SetPosition(16, 511);
}

