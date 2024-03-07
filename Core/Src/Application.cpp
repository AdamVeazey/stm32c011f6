/*
 * Copyright (c) 2023, Adam Veazey
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Application.hpp"
#include "EDF/MCU/ST/STM32C011F6/GPIO.hpp"
#include "EDF/MCU/ST/STM32C011F6/SPIController.hpp"
#include "EDF/MCU/ST/STM32C011F6/I2CController.hpp"
#include "EDF/MCU/ST/STM32C011F6/PWM.hpp"
#include "EDF/Math.hpp"
#include "EDF/String.hpp"

#include "main.h"
#include "spi.h"
#include "i2c.h"
#include "tim.h"

static GPIOFast button( GPIOF, 2 );     // CN5_23
static GPIOFast in( GPIOA, 1 );         // CN5_21
static GPIOFast out( GPIOA, 0 );        // CN5_23
static GPIOFast cs( GPIOA, 4 );         // CN5_13
static SPIControllerFast spi( &hspi1, &cs ); // CN5_11, CN5_16, CN5_26, (CN5_13)
static I2CControllerFast i2c( &hi2c1, 0x36 );   // CN5_14, CN5_4
static PWMFast led( &htim1, PWM::Channel::CH_2 );  // CN5_33

extern "C"
void application_init() {
    // __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    led.init();
    button.configureAsInput();
    in.configureAsInput();
    out.configureAsOutput( GPIOFast::Level::HIGH );

    cs.configureAsOutput( GPIO::Level::HIGH );
    i2c.setTimeout( 5'000'000 );
    led.setPeriod_Hz( 2 );
    led.setDutyCyclePercent( 50 ); // blink LED
}

extern "C"
void application_run() {
    EDF::String<32> test = "Hello, world!";
    spi.select();
    spi.transfer( test.asByteData(), test.length() );
    spi.deselect();
}