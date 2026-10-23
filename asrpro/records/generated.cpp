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

// Embedded in asrpro.hd: edit here, then run tools/build_project.py.
// No motor control, automatic command retries, or heartbeat generation here.
#include "HardwareSerial.h"
#include "queue.h"

struct YaoyaoAudio { uint16_t id; uint8_t urgent; uint32_t epoch; };
static QueueHandle_t yy_audio_queue = NULL;
static volatile bool yy_ready = false;
static volatile uint32_t yy_epoch = 0;
static uint8_t yy_frame[6];
static uint8_t yy_used = 0;
static uint32_t yy_started = 0;
static uint8_t yy_last_status = 0xFF;
static uint16_t yy_last_status_audio = 0;
static uint32_t yy_dropped_audio = 0;

static bool yy_queue_audio(uint16_t id, bool urgent) {
  if (!yy_audio_queue) return false;
  if (urgent) { ++yy_epoch; xQueueReset(yy_audio_queue); }
  YaoyaoAudio item = {id, (uint8_t)urgent, yy_epoch};
  if (xQueueSend(yy_audio_queue, &item, 0) == pdTRUE) return true;
  ++yy_dropped_audio;
  return false;
}

static void yy_send_command(uint8_t code) {
  if (!yy_ready) return;
  const uint8_t frame[6] = {0xAA,0x55,0x01,code,(uint8_t)(0x01+code),0x0D};
  for (uint8_t i=0; i<6; ++i) Serial1.write(frame[i]);
  // Receipt confirms recognition only. Always transmit before queuing speech.
  // Stop cancels stale queued speech and preempts current playback.
  yy_queue_audio(9201, code == 0x02);
}

static void yy_dispatch(uint8_t type, uint8_t code) {
  if (type == 0x02 && code <= 0x02) {
    yy_queue_audio((uint16_t)(9000+code), false);
  } else if (type == 0x03 && (code <= 0x06 || code == 0x10 || code == 0x11)) {
    if (code == yy_last_status) return;
    const uint16_t id = (code == 1 || code == 2 || code == 0x10) ? 9101
      : code <= 6 ? (uint16_t)(9100+code) : 9108;
    // Searching, approaching, and manual movement share one spoken status.
    if (id == yy_last_status_audio) { yy_last_status = code; return; }
    if (yy_queue_audio(id, code == 4 || code == 5 || code == 6)) {
      yy_last_status = code;
      yy_last_status_audio = id;
    }
  }
}

static void yy_expire(uint32_t now) {
  if (yy_used && (uint32_t)(now-yy_started) >= 20) yy_used = 0;
}

static void yy_receive_byte(uint8_t byte, uint32_t now) {
  yy_expire(now);
  if (!yy_used) {
    if (byte != 0xAA) return;
    yy_started = now;
  }
  yy_frame[yy_used++] = byte;
  // Sliding prefix search preserves overlapping AA AA 55 and embedded headers.
  for (;;) {
    if (yy_used < 2) return;
    if (yy_frame[0] == 0xAA && yy_frame[1] == 0x55) {
      if (yy_used < 6) return;
      const uint8_t type = yy_frame[2], code = yy_frame[3];
      const bool known = (type == 2 && code <= 2) ||
        (type == 3 && (code <= 6 || code == 0x10 || code == 0x11));
      if (known && yy_frame[4] == (uint8_t)(type+code) && yy_frame[5] == 0x0D) {
        yy_used = 0;
        yy_dispatch(type,code);
        return;
      }
    }
    for (uint8_t i=1; i<yy_used; ++i) yy_frame[i-1] = yy_frame[i];
    --yy_used;
    if (yy_used && yy_frame[0] != 0xAA) continue;
    // Retained prefixes keep the original deadline (conservative timeout).
    if (yy_used < 2) return;
  }
}

static void yy_init() {
  yy_audio_queue = xQueueCreate(24, sizeof(YaoyaoAudio));
  // Audio allocation failure must never disable the stop command UART.
  yy_ready = true;
}

static void yy_rx_poll() {
  if (!yy_ready) { delay(1); return; }
  // Budget bounds CPU use under continuous noise; no String/readString timeout.
  for (uint16_t i=0; i<128 && Serial1.available()>0; ++i) {
    const int byte = Serial1.read();
    if (byte >= 0) yy_receive_byte((uint8_t)byte, (uint32_t)millis());
  }
  yy_expire((uint32_t)millis());
  delay(1);
}

static void yy_audio_poll() {
  if (!yy_ready || !yy_audio_queue) { delay(1); return; }
  YaoyaoAudio item;
  if (xQueueReceive(yy_audio_queue, &item, 0) != pdTRUE) { delay(2); return; }
  // An urgent stopped/failure/fault state cancels queued, outdated feedback.
  while (item.epoch == yy_epoch) {
    if (prompt_play_by_cmd_id(item.id, -1, NULL, item.urgent != 0) == 0) break;
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
}
