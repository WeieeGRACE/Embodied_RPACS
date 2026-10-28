#include "asr.h"
extern "C"{ void * __dso_handle = 0 ;}
#include "setup.h"
#include "HardwareSerial.h"

uint32_t snid;
void ASR_CODE();
void yaoyao_uart_rx();
void yaoyao_audio();

//{speak:小蝶-清新女声,vol:10,speed:10,platform:haohaodada}
//{playid:10001,voice:幺幺准备好了，请说幺幺耍起唤醒我。}
//{playid:10002,voice:幺幺休息了，请说幺幺耍起唤醒我。}

// Embedded in asrpro.hd; regenerate through tools/build_project.py.
#include "HardwareSerial.h"
#include "queue.h"
#include "semphr.h"
extern "C" uint32_t yy_prompt_play(uint16_t, void (*)(uint32_t,int32_t), uint32_t);
struct YaoyaoAudio { uint16_t id; uint8_t urgent; uint32_t epoch,cookie; };
struct YaoyaoDone { uint32_t cookie; int32_t reason; };
static QueueHandle_t yy_audio_queue=NULL,yy_done_queue=NULL;
static SemaphoreHandle_t yy_serial_mutex=NULL;
static volatile bool yy_ready=false;
static volatile uint32_t yy_epoch=0,yy_active_cookie=0;
static uint32_t yy_cookie_counter=0,yy_active_since=0;
static uint8_t yy_token=0;
static bool yy_avoid=false,yy_suppress_accept=false;
static uint8_t yy_frame[6],yy_used=0,yy_last_status=0xFF;
static uint32_t yy_started=0,yy_dropped_audio=0;
static uint16_t yy_last_status_audio=0;
static void yy_cancel_start(){yy_active_cookie=0;}
static void yy_write_frame(uint8_t type,uint8_t code){
 if(!yy_serial_mutex || xSemaphoreTake(yy_serial_mutex,portMAX_DELAY)!=pdTRUE)return;
 const uint8_t f[6]={0xAA,0x55,type,code,(uint8_t)(type+code),0x0D};
 for(uint8_t i=0;i<6;++i)Serial1.write(f[i]);
 xSemaphoreGive(yy_serial_mutex);
}
static bool yy_queue_audio(uint16_t id,bool urgent,uint32_t cookie=0){
 if(!yy_audio_queue)return false;
 if(urgent){++yy_epoch;xQueueReset(yy_audio_queue);}
 YaoyaoAudio item={id,(uint8_t)urgent,yy_epoch,cookie};
 if(xQueueSend(yy_audio_queue,&item,0)==pdTRUE)return true;
 ++yy_dropped_audio;return false;
}
static void yy_send_command(uint8_t code){
 if(!yy_ready)return;
 if(code==2 || code==0x17 || (code>=0x11 && code<=0x13))yy_cancel_start();
 yy_suppress_accept=code==0x16;
 yy_write_frame(1,code);
 if(code!=0x16 && code!=0x17)yy_queue_audio(9201,code==2);
}
static void yy_dispatch(uint8_t type,uint8_t code){
 if(type==2 && code<=2){
  bool suppress=yy_suppress_accept && code==0;yy_suppress_accept=false;
  if(!suppress)yy_queue_audio(9000+code,false);
 }else if(type==4){
  yy_cancel_start();yy_token=code;yy_active_cookie=++yy_cookie_counter;
  if(!yy_active_cookie)yy_active_cookie=++yy_cookie_counter;
  yy_active_since=millis();
  if(!yy_queue_audio(9300,true,yy_active_cookie))yy_cancel_start();
 }else if(type==6 && code<=1){
  yy_avoid=code!=0;
  if(yy_avoid)set_state_enter_wakeup(8589934);else yy_cancel_start();
 }else if(type==7 && code>=1 && code<=4){
  if(code==3){yy_cancel_start();yy_avoid=false;}
  // Button starts are informational only: no completion cookie or TYPE05.
  if(code==4){yy_cancel_start();yy_queue_audio(9300,true);}
  else yy_queue_audio((uint16_t)(9300+code),code==3);
 }else if(type==3 && (code<=6 || code==0x10 || code==0x11 || (code>=0x20 && code<=0x23))){
  if(code==4 || code==5 || code==6 || code==0x10 || code==0x20)yy_cancel_start();
  if(code==yy_last_status)return;
  // Mode states are silent. Physical route events use TYPE07 and are never deduped.
  if(code>=0x20){yy_last_status=code;yy_last_status_audio=0;return;}
  uint16_t id=(code==1 || code==2 || code==0x10)?9101:code<=6?9100+code:9108;
  if(id==yy_last_status_audio){yy_last_status=code;return;}
  if(yy_queue_audio(id,code==4 || code==5 || code==6)){
   yy_last_status=code;yy_last_status_audio=id;
  }
 }
}
static void yy_expire(uint32_t now){if(yy_used && (uint32_t)(now-yy_started)>=20)yy_used=0;}
static void yy_receive_byte(uint8_t byte,uint32_t now){
 yy_expire(now);
 if(!yy_used){if(byte!=0xAA)return;yy_started=now;}
 yy_frame[yy_used++]=byte;
 for(;;){
  if(yy_used<2)return;
  if(yy_frame[0]==0xAA && yy_frame[1]==0x55){
   if(yy_used<6)return;
   uint8_t t=yy_frame[2],c=yy_frame[3];
   bool known=(t==2 && c<=2)||(t==4)||(t==6 && c<=1)||(t==7 && c>=1 && c<=4)||
    (t==3 && (c<=6 || c==0x10 || c==0x11 || (c>=0x20 && c<=0x23)));
   if(known && yy_frame[4]==(uint8_t)(t+c) && yy_frame[5]==0x0D){yy_used=0;yy_dispatch(t,c);return;}
  }
  for(uint8_t i=1;i<yy_used;++i)yy_frame[i-1]=yy_frame[i];
  --yy_used;if(yy_used && yy_frame[0]!=0xAA)continue;if(yy_used<2)return;
 }
}
static void yy_play_done(uint32_t cookie,int32_t reason){
 YaoyaoDone d={cookie,reason};
 if(yy_done_queue)xQueueSend(yy_done_queue,&d,0);
}
static void yy_init(){
 yy_audio_queue=xQueueCreate(24,sizeof(YaoyaoAudio));
 yy_done_queue=xQueueCreate(8,sizeof(YaoyaoDone));
 yy_serial_mutex=xSemaphoreCreateMutex();yy_ready=true;
}
static void yy_rx_poll(){
 if(!yy_ready){delay(1);return;}
 for(uint16_t i=0;i<128 && Serial1.available()>0;++i){int b=Serial1.read();if(b>=0)yy_receive_byte(b,millis());}
 yy_expire(millis());
 if(yy_active_cookie && (uint32_t)(millis()-yy_active_since)>=10000)yy_cancel_start();
 YaoyaoDone d;
 while(yy_done_queue && xQueueReceive(yy_done_queue,&d,0)==pdTRUE){
  if(d.cookie==yy_active_cookie && yy_active_cookie){
   if(d.reason==0)yy_write_frame(5,yy_token);
   yy_cancel_start();
  }
 }
 delay(1);
}
static void yy_audio_poll(){
 if(!yy_ready || !yy_audio_queue){delay(1);return;}
 YaoyaoAudio item;
 if(xQueueReceive(yy_audio_queue,&item,0)!=pdTRUE){delay(2);return;}
 while(item.epoch==yy_epoch){
  if(item.cookie){
   if(item.cookie!=yy_active_cookie)break;
   if(yy_prompt_play(item.id,yy_play_done,item.cookie)==0)break;
  }else if(prompt_play_by_cmd_id(item.id,-1,NULL,item.urgent!=0)==0)break;
  delay(2);
 }
}


/*描述该功能...
*/
void ASR_CODE(){
  set_state_enter_wakeup(8589934); // 8589.934秒，SDK安全上限
  switch (snid) {
   case 1:
    yy_send_command(0x01); // 寻找红色水瓶
    break;
   case 2:
    yy_send_command(0x02); // 停止任务
    break;
   case 3:
    yy_send_command(0x02); // 停车
    break;
   case 4:
    yy_send_command(0x02); // 停止
    break;
   case 5:
    yy_send_command(0x03); // 查询状态
    break;
   case 6:
    yy_send_command(0x03); // 现在什么状态
    break;
   case 10:
    yy_send_command(0x10); // 前进
    break;
   case 11:
    yy_send_command(0x10); // 向前走
    break;
   case 12:
    yy_send_command(0x11); // 后退
    break;
   case 13:
    yy_send_command(0x11); // 向后走
    break;
   case 14:
    yy_send_command(0x12); // 左转
    break;
   case 15:
    yy_send_command(0x12); // 向左转
    break;
   case 16:
    yy_send_command(0x12); // 向左原地旋转
    break;
   case 17:
    yy_send_command(0x13); // 右转
    break;
   case 18:
    yy_send_command(0x13); // 向右转
    break;
   case 19:
    yy_send_command(0x13); // 向右原地旋转
    break;
   case 20:
    yy_send_command(0x14); // 加速
    break;
   case 21:
    yy_send_command(0x14); // 快一点
    break;
   case 22:
    yy_send_command(0x15); // 减速
    break;
   case 23:
    yy_send_command(0x15); // 慢一点
    break;
   case 24:
    yy_send_command(0x16); // 避障模式
    break;
   case 25:
    yy_send_command(0x17); // 退出避障模式
    break;
  }

}

void yaoyao_uart_rx(){
  while (1) {
    yy_rx_poll();
  }
  vTaskDelete(NULL);
}

void yaoyao_audio(){
  while (1) {
    yy_audio_poll();
  }
  vTaskDelete(NULL);
}

void hardware_init(){
  setPinFun(2,FORTH_FUNCTION);
  setPinFun(3,FORTH_FUNCTION);
  Serial1.begin(115200);
  vol_set(7);
  yy_init();
  xTaskCreate(yaoyao_uart_rx,"yaoyao_uart_rx",512,NULL,5,NULL);
  xTaskCreate(yaoyao_audio,"yaoyao_audio",512,NULL,4,NULL);
  vTaskDelete(NULL);
}

void setup()
{
  //{ID:0,keyword:"唤醒词",ASR:"幺幺耍起",ASRTO:"幺幺在，请说指令。"}
  //{ID:1,keyword:"命令词",ASR:"寻找红色水瓶",ASRTO:""}
  //{ID:2,keyword:"唤醒词",ASR:"停止任务",ASRTO:""}
  //{ID:3,keyword:"唤醒词",ASR:"停车",ASRTO:""}
  //{ID:4,keyword:"唤醒词",ASR:"停止",ASRTO:""}
  //{ID:5,keyword:"命令词",ASR:"查询状态",ASRTO:""}
  //{ID:6,keyword:"命令词",ASR:"现在什么状态",ASRTO:""}
  //{ID:10,keyword:"命令词",ASR:"前进",ASRTO:""}
  //{ID:11,keyword:"命令词",ASR:"向前走",ASRTO:""}
  //{ID:12,keyword:"命令词",ASR:"后退",ASRTO:""}
  //{ID:13,keyword:"命令词",ASR:"向后走",ASRTO:""}
  //{ID:14,keyword:"命令词",ASR:"左转",ASRTO:""}
  //{ID:15,keyword:"命令词",ASR:"向左转",ASRTO:""}
  //{ID:16,keyword:"命令词",ASR:"向左原地旋转",ASRTO:""}
  //{ID:17,keyword:"命令词",ASR:"右转",ASRTO:""}
  //{ID:18,keyword:"命令词",ASR:"向右转",ASRTO:""}
  //{ID:19,keyword:"命令词",ASR:"向右原地旋转",ASRTO:""}
  //{ID:20,keyword:"命令词",ASR:"加速",ASRTO:""}
  //{ID:21,keyword:"命令词",ASR:"快一点",ASRTO:""}
  //{ID:22,keyword:"命令词",ASR:"减速",ASRTO:""}
  //{ID:23,keyword:"命令词",ASR:"慢一点",ASRTO:""}
  //{ID:24,keyword:"命令词",ASR:"避障模式",ASRTO:""}
  //{ID:25,keyword:"唤醒词",ASR:"退出避障模式",ASRTO:""}
  //{playid:9000,voice:已接受}
  //{playid:9001,voice:忙碌中}
  //{playid:9002,voice:无法执行}
  //{playid:9100,voice:空闲}
  //{playid:9101,voice:执行中}
  //{playid:9103,voice:已到达}
  //{playid:9104,voice:已停止}
  //{playid:9105,voice:失败}
  //{playid:9106,voice:故障}
  //{playid:9108,voice:已完成}
  //{playid:9201,voice:收到}
  //{playid:9300,voice:开启避障模式}
  //{playid:9301,voice:遇到障碍开始绕障}
  //{playid:9302,voice:已绕过障碍物}
  //{playid:9303,voice:已退出避障模式}
}
