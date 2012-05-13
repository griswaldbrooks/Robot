
#include <SoR_Utils.h>
#include <ax12.h>


void setup(){
  ax12Init(1000000);
//  Serial.begin(115200);

}

int get_intpos(int degree){
  return (int)(3.413*(float)degree - 1.0f);
}

void loop(){


  SetPosition(1, 511);
  SetPosition(5, 511);
  SetPosition(9, 511);
  SetPosition(13, 511);
  /*
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
  */
}


int main(){
	
	setup();
	
	while(1){
		loop();
	}


}
