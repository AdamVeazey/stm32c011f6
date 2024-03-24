/*
 * Copyright (c) 2023, Adam Veazey
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Application.hpp"
#include "EDF/MCU/ST/STM32C011F6/ADC.hpp"
#include "EDF/MCU/ST/STM32C011F6/GPIO.hpp"
#include "EDF/MCU/ST/STM32C011F6/SPIController.hpp"
#include "EDF/MCU/ST/STM32C011F6/I2CController.hpp"
#include "EDF/MCU/ST/STM32C011F6/PWM.hpp"
#include "EDF/Math.hpp"
#include "EDF/String.hpp"

#include "main.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "tim.h"

static GPIOFast button( GPIOF, 2 );     // CN5_23
static GPIOFast in( GPIOA, 1 );         // CN5_21
static GPIOFast out( GPIOA, 0 );        // CN5_23
static GPIOFast cs( GPIOA, 4 );         // CN5_13
static SPIControllerFast spi( &hspi1, &cs ); // CN5_11, CN5_16, CN5_26, (CN5_13)
static I2CControllerFast i2c( &hi2c1 );   // CN5_14, CN5_4
static PWMFast led( &htim1, PWM::Channel::CH_2 );  // CN5_33
static ADC adc( &hadc1 );
static ADCChannel joystick( adc, ADC::Pin{ GPIOA, 8 } ); // CN5_15_Joystick
static ADCChannel CN5_23( adc, ADC::Pin{ GPIOA, 0 } );
static ADCScanGroup scanGroup( adc, {
    ADC::Pin{ GPIOA, 8 },
    ADC::Pin{ GPIOA, 0 }
});
// static ADCScanGroup scanGroup( adc, {
//     ADC::Channel::IN0,
//     ADC::Channel::IN2,
// });

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
    // led.setPeriod_Hz( 2 );
    // led.setDutyCyclePercent( 50 ); // blink LED

    led.setPeriod_ticks( adc.getMaxValue() );

    adc.calibrate();
}

void showADCErrorWithLED( ADC::Response r ) {
    switch( r ) {
    case ADC::Response::Error:          led.setDutyCyclePercent( 20 );  break;
    case ADC::Response::ErrorBusy:      led.setDutyCyclePercent( 40 );  break;
    case ADC::Response::ErrorOverrun:   led.setDutyCyclePercent( 60 );  break;
    case ADC::Response::ErrorTimeout:   led.setDutyCyclePercent( 80 );  break;
    default:                            led.setDutyCyclePercent( 100 ); break;
    }
}

extern "C"
void application_run() {
    EDF::String<32> test = "Hello, world!";
    spi.select();
    spi.transfer( test.asByteData(), test.length() );
    spi.deselect();

    auto value = joystick.getSingleConversion();
    if( value == ADC::Response::Ok ){
        // uint32_t percent_x100 = (value.data() * 1000) / adc.getMaxValue();
        led.setDutyCycleTicks( value.data() );
    }
    else {
        showADCErrorWithLED( value );
    }

    auto r = scanGroup.getSamples();
    if( r == ADC::Response::Ok ) {
        for( const auto& value : r ) {
            ADC::to_mV( value );
        }
    }
    else {
        showADCErrorWithLED( r );
    }
}