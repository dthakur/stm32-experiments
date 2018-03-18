#ifndef _LOG_H__
#define _LOG_H__

#include <stdio.h>
#include <stdarg.h>
#include "retarget.h"

void log_info(const char *format, ...) {
  va_list args;
  va_start(args, format);

  vprintf(format, args);
  printf("\r\n");
  va_end(args);
}

#endif 