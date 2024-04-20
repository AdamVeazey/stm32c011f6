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
#include "EDF/Drivers/RTC/DS3231.hpp"

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
static I2CController i2c( &hi2c1 );     // CN5_19, CN5_4
static PWMFast led( &htim1, PWM::Channel::CH_2 );  // CN5_33
static ADC adc( &hadc1 );
static ADCChannel joystick( adc, ADC::Pin{ GPIOA, 8 } ); // CN5_15_Joystick
static ADCChannel CN5_23( adc, ADC::Pin{ GPIOA, 0 } );
static ADCScanGroup scanGroup( adc, {
    ADC::Pin{ GPIOA, 8 },
    ADC::Pin{ GPIOA, 0 }
});
static EDF::DS3231 rtc( i2c );
// static ADCScanGroup scanGroup( adc, {
//     ADC::Channel::IN0,
//     ADC::Channel::IN2,
// });

template<std::size_t N>
void print( const EDF::String<N>& str ) {
    HAL_UART_Transmit( &huart2, str.asByteData(), str.length(), HAL_MAX_DELAY );
}

template<std::size_t N>
void print( const char (&s)[N] ) {
    HAL_UART_Transmit( &huart2, (const uint8_t*)s, N - 1, HAL_MAX_DELAY );
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
    led.setPeriod_Hz( 2 );
    led.setDutyCyclePercent( 50 ); // blink LED

    rtc.init();
    // led.setPeriod_ticks( adc.getMaxValue() );

    // adc.calibrate();
    // adc.setTimeout( 5000 );
    print("application_init()\r\n");
}

extern "C" {
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit( &huart2, (const uint8_t*)ptr, len, HAL_MAX_DELAY );
    return len;
}
}

void app_update_adc_values() {
    static bool readSingle = true;
    if( readSingle ) {
        joystick.start([](const ADCChannel::ResponseData& value ) {
            switch( value ) {
            case ADC::Response::ErrorBusy:      print("ErrorBusy    "); break;
            case ADC::Response::ErrorOverrun:   print("ErrorOverrun "); break;
            case ADC::Response::Error:          print("Error        "); break;
            case ADC::Response::ErrorTimeout:   print("ErrorTimeout "); break;
            case ADC::Response::ErrorInternal:  print("ErrorInternal"); break;
            case ADC::Response::ErrorDMA:       print("ErrorDMA     "); break;
            case ADC::Response::Ok:
                readSingle = !readSingle;
                print(EDF::String<32>("Value: ")
                    .append(value.data())
                    .append("\r\n")
                );
                break;
            }
        });
    }
    else {
        scanGroup.start( [](const ADCScanGroup::ResponseData& values) {
            if( values == ADC::Response::Ok ) {
                for( std::size_t k = 0; k < values.length(); ++k ) {
                    print(EDF::String<32>("Value[")
                        .append((uint32_t)k)
                        .append("]: ")
                        .append(adc.to_mV(values.data()[k]))
                        .append(" mV ")
                    );
                }
                print("\r\n");
                readSingle = !readSingle;
            }
        });
    }
}

static void i2c_probe() {
    print("     0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08 0x09 0x0A 0x0B 0x0C 0x0D 0x0E 0x0F\r\n");
    for( uint8_t row = 0x00; row <= 0x70; row += 0x10 ) {
        EDF::String<4> s(row, 16);
        if( s.length() == 1 ) s.insert( 0_uz, '0' );
        print("0x" + s);
        for( uint8_t column = 0x00; column <= 0x0F; column += 0x1 ) {
            uint8_t address_7bit = (row + column);

            using Response = EDF::I2CController::Response;
            auto r = i2c.transfer( address_7bit );

            if( r == Response::ACK ) {
                print(" 0x" + EDF::String<4>(address_7bit, 16) );
            }
            else {
                print(" ----");
            }
        }
        print("\r\n");
    }
}

void test_ds3231() {
    char s[100];
    auto time = rtc.getCurrentTime();
    if( std::strftime( s, EDF::nElements(s), "%a %b %d %T %Y", &time ) ) {
        print( EDF::String<100>(s) + ' ' );
    }

    auto temp = rtc.getTemperature_C_x100();
    print( "Temp: " + EDF::String<32>(temp / 100) + '.' + EDF::String<32>(temp % 100) + "\r\n" );
}

extern "C"
void application_run() {
    print("\r\n");
    app_update_adc_values();
    i2c_probe();
    test_ds3231();

    adc.handleIRQs();
    HAL_Delay( 3000 );
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