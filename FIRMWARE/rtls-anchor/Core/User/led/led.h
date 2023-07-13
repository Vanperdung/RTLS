#ifndef _LED_H_
#define _LED_H_

#include "main.h"
#include "gpio.h"
#include <stdbool.h>
#define N_LEDS 3

typedef enum {
  ledRanging = 0,
  ledSync,
  ledMode
} led_e;

typedef struct {
  uint32_t pin;
  GPIO_TypeDef * port;
} led_t;

void ledInit(void);

void ledOn(led_e led);

void ledOff(led_e led);

void ledBlink(led_e led, bool oneshot);

void ledTick();

#endif