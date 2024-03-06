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
#include "EDF/Math.hpp"
#include "EDF/String.hpp"

#include "main.h"
#include "spi.h"
#include "i2c.h"

static GPIOFast led( GPIOB, 6 );        // CN5_33
static GPIOFast button( GPIOF, 2 );     // CN5_23
static GPIOFast in( GPIOA, 1 );         // CN5_21
static GPIOFast out( GPIOA, 0 );        // CN5_23
// static GPIOFast cs( GPIOA, 4 );         // CN5_13
static SPIControllerFast spi( &hspi1 );
static I2CControllerFast i2c( &hi2c1, 0x36 );   // CN5_4, CN5_33

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

    // cs.configureAsOutput( GPIO::Level::HIGH );
    i2c.setTimeout( 5'000'000 );
}

extern "C"
void application_run() {
    EDF::String<32> test = "Hello, world!";
    spi.select();
    spi.transfer( test.asByteData(), test.length() );
    spi.deselect();
}