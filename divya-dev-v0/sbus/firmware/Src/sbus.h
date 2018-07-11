#pragma once

typedef struct sbusChannels_s {
  // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
  unsigned int chan0 : 11;
  unsigned int chan1 : 11;
  unsigned int chan2 : 11;
  unsigned int chan3 : 11;
  unsigned int chan4 : 11;
  unsigned int chan5 : 11;
  unsigned int chan6 : 11;
  unsigned int chan7 : 11;
  unsigned int chan8 : 11;
  unsigned int chan9 : 11;
  unsigned int chan10 : 11;
  unsigned int chan11 : 11;
  unsigned int chan12 : 11;
  unsigned int chan13 : 11;
  unsigned int chan14 : 11;
  unsigned int chan15 : 11;
  uint8_t flags;
} __attribute__((__packed__)) sbusChannels_t;

#define SBUS_CHANNEL_DATA_LENGTH sizeof(sbusChannels_t)
#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)
#define SBUS_FRAME_BEGIN_BYTE 0x0F

struct sbusFrame_s {
  uint8_t syncByte;
  sbusChannels_t channels;
  uint8_t endByte;
} __attribute__ ((__packed__));

typedef union sbusFrame_u {
  uint8_t bytes[SBUS_FRAME_SIZE];
  struct sbusFrame_s frame;
} sbusFrame_t;

typedef struct sbusHandle_s {
  sbusFrame_t frame;
  uint32_t bytesReceived;
  uint16_t position;
  void (*onByte)(struct sbusHandle_s *handle, uint8_t byte);
  void (*onFrame)(struct sbusHandle_s *handle);
} sbusHandle_t;

void _onByte(sbusHandle_t *handle, uint8_t byte) {
  handle->bytesReceived++;

  if (handle->position == 0 && byte != SBUS_FRAME_BEGIN_BYTE) {
    return;
  }

  handle->frame.bytes[handle->position++] = byte;

  if (handle->position != SBUS_FRAME_SIZE - 1) {
    return;
  }

  handle->position = 0;
  handle->onFrame(handle);
}

uint8_t sbusInit(sbusHandle_t *handle) {
  handle->onByte = &_onByte;
  handle->position = 0;

  return 1;
}