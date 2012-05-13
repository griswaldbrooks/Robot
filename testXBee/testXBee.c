#include <SoR_Utils.h>

int main(){
uint32_t counter = 0;
uartInit();
while(1){
	uart1SendByte(counter);
	counter++;
}

}
