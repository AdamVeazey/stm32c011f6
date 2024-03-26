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
#include "usart.h"

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

template<std::size_t N>
void print( const EDF::String<N>& str ) {
    HAL_UART_Transmit( &huart1, str.asByteData(), str.length(), HAL_MAX_DELAY );
}

template<std::size_t N>
void print( const char (&s)[N] ) {
    HAL_UART_Transmit( &huart1, (const uint8_t*)s, N - 1, HAL_MAX_DELAY );
}

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
    adc.setTimeout( 5000 );
}

extern "C" {
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit( &huart1, (const uint8_t*)ptr, len, HAL_MAX_DELAY );
    return len;
}
}

void app_update_adc_values() {
    scanGroup.start( [](const ADCScanGroup::ResponseData& values) {
        switch( values ) {
        case ADC::Response::Ok:             print( "response: Ok           ");    break;
        case ADC::Response::ErrorBusy:      print( "response: ErrorBusy    ");    break;
        case ADC::Response::ErrorOverrun:   print( "response: ErrorOverrun ");    break;
        case ADC::Response::Error:          print( "response: Error        ");    break;
        case ADC::Response::ErrorTimeout:   print( "response: ErrorTimeout ");    break;
        }
        for( std::size_t k = 0; k < values.length(); ++k ) {
            print( "Value[" + EDF::String<32>((uint32_t)k) + "]: " +
                    EDF::String<32>(adc.to_mV(values.data()[k])) + " "
            );
        }
        print( "\r\n" );
    });
}

extern "C"
void application_run() {
    // EDF::String<32> test = "Hello, world!";
    // spi.select();
    // spi.transfer( test.asByteData(), test.length() );
    // spi.deselect();

    // auto value = joystick.getSingleConversion();
    // if( value == ADC::Response::Ok ){
    //     // uint32_t percent_x100 = (value.data() * 1000) / adc.getMaxValue();
    //     led.setDutyCycleTicks( value.data() );
    //     print( "joystick: " + EDF::String<32>(value.data()) + "\r\n" );
    // }
    app_update_adc_values();
    adc.handleIRQs();
}

extern "C"{
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    adc.ConvCpltCallback( hadc );
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    adc.ConvHalfCpltCallback( hadc );
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc) {
    adc.LevelOutOfWindowCallback( hadc );
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    adc.ErrorCallback( hadc );
}
} /* extern "C" */