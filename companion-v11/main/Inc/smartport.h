#pragma once

typedef uint8_t smartportFrame_t;
typedef struct smartportHandle_s {
  smartportFrame_t frame;
  uint32_t bytesReceived;
  void (*onByte)(struct smartportHandle_s *handle, uint8_t byte);
  void (*onFrame)(struct smartportHandle_s *handle);
} smartportHandle_t;

#define SMARTPORT_FRAME_SIZE sizeof(smartportFrame_t)

void _onByte(smartportHandle_t *handle, uint8_t byte) {
  handle->bytesReceived++;

  handle->frame = byte;
  handle->onFrame(handle);
}

uint8_t smartportInit(smartportHandle_t *handle) {
  handle->onByte = &_onByte;

  return 1;
}