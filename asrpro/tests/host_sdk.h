#pragma once
#include <cstdint>
#include <cstddef>
#include <deque>
#include <vector>
#include <cstring>
#include <cassert>
#include <iostream>
#define pdTRUE 1
#define FORTH_FUNCTION 4
struct HostQueue { size_t size, capacity; std::deque<std::vector<uint8_t>> items; };
typedef HostQueue* QueueHandle_t;
static bool fail_allocate = false;
static uint32_t host_time = 0;
static unsigned wake_ms = 0;
static bool player_busy = false;
static unsigned busy_retries = 0;
static std::vector<uint16_t> spoken;
static std::vector<char> host_events;
static QueueHandle_t xQueueCreate(size_t n,size_t s) {
  return fail_allocate ? nullptr : new HostQueue{s,n,{}};
}
static void xQueueReset(QueueHandle_t q) { q->items.clear(); }
static int xQueueSend(QueueHandle_t q,const void* p,int) {
  host_events.push_back('Q');
  if(q->items.size()==q->capacity) return 0;
  const uint8_t* b=(const uint8_t*)p; q->items.emplace_back(b,b+q->size); return 1;
}
static int xQueueReceive(QueueHandle_t q,void* p,int) {
  if(q->items.empty())return 0;
  std::memcpy(p,q->items.front().data(),q->size);q->items.pop_front();return 1;
}
static uint32_t millis(){return host_time;}
static void delay(unsigned n){host_time+=n;if(busy_retries) --busy_retries;}
static int prompt_play_by_cmd_id(uint16_t id,int,void*,bool preempt) {
  if((player_busy||busy_retries)&&!preempt)return 1;
  spoken.push_back(id);return 0;
}
static void set_state_enter_wakeup(unsigned n){wake_ms=n;}
static void setPinFun(int,int){}
static void vol_set(int){}
static void vTaskDelete(void*){}
static int xTaskCreate(void (*)(),const char*,int,void*,int,void*){return 1;}
struct HostSerial {
  unsigned baud=0;std::vector<uint8_t> tx;std::deque<uint8_t> rx;
  void begin(unsigned n){baud=n;}
  void write(uint8_t b){tx.push_back(b);host_events.push_back('W');}
  int available(){return (int)rx.size();}
  int read(){if(rx.empty())return -1;int b=rx.front();rx.pop_front();return b;}
};
static HostSerial Serial1;
