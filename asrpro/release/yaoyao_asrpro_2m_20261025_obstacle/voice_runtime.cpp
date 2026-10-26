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
 }else if(type==7 && code>=1 && code<=3){
  if(code==3){yy_cancel_start();yy_avoid=false;}
  yy_queue_audio((uint16_t)(9300+code),code==3);
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
   bool known=(t==2 && c<=2)||(t==4)||(t==6 && c<=1)||(t==7 && c>=1 && c<=3)||
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
