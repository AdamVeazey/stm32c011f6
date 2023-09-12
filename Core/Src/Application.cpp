/*
 * Copyright (c) 2023, Adam Veazey
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Application.hpp"
#include "EDF/MCU/ST/STM32C011F6/GPIO.hpp"

#include "main.h"

GPIOFast led( GPIOB, 6 );
GPIOFast button( GPIOF, 2 );
GPIOFast in( GPIOA, 1 );
GPIOFast out( GPIOA, 0 );

extern "C"
void application_init() {
    // __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    led.configureAsOutput(
        GPIO::Level::HIGH,
        GPIO::OutputMode::OUTPUT_PP,
        GPIO::Pull::NOPULL,
        GPIO::Speed::FREQ_VERY_HIGH
    );
    button.configureAsInput();
    in.configureAsInput();
    out.configureAsOutput( GPIOFast::Level::HIGH );
}

extern "C"
void application_run() {
    led.toggle();
    HAL_Delay( 499 );
}