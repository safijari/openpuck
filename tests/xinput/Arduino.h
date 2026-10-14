#pragma once
#include <stdint.h>

unsigned long millis();
uint32_t __get_PRIMASK();
void __disable_irq();
void __set_PRIMASK(uint32_t mask);