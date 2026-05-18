#include "usb_task.h"
#include "cmsis_os.h"

void usb_update();
void usb_loop();
void usb_send();
void usb_task(){
	osDelay(USB_INIT_TIME);
	//已经初始化到main.c里面了
	while(1){
		//更新信息
		usb_update();
		//计算
		usb_loop();
		//发送消息
		usb_send();
		
	}
	
	
}
