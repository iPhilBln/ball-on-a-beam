#include "Arduino.h"
#include "SPI.h"
#include "MCP331x1.h"

//volatile bool CONVERSATION_RUNNING = false;

Mcp331x1::Mcp331x1() {
    setBitResolution(16);
    setVoltageRef(5.0);
    setChipSelectPin(10);
    setSpiClock(2);
    setTimer2Configuration();
} 

Mcp331x1::Mcp331x1(uint8_t bit_resolution) {
    setBitResolution(bit_resolution);
    setVoltageRef(5.0);
    setChipSelectPin(10);
    setSpiClock(2);
    setTimer2Configuration();
} 

Mcp331x1::Mcp331x1(uint8_t bit_resolution, float v_ref) {
    setBitResolution(bit_resolution);
    setVoltageRef(v_ref);
    setChipSelectPin(10);
    setSpiClock(2);
    setTimer2Configuration();
} 

Mcp331x1::Mcp331x1(uint8_t bit_resolution, float v_ref, uint8_t clock_divider) {
    setBitResolution(bit_resolution);
    setVoltageRef(v_ref);
    setChipSelectPin(10);
    setSpiClock(clock_divider);
    setTimer2Configuration();
} 

Mcp331x1::Mcp331x1(uint8_t bit_resolution, float v_ref, uint8_t clock_divider, uint8_t cs_pin_number) {
    setBitResolution(bit_resolution);
    setVoltageRef(v_ref);
    setChipSelectPin(cs_pin_number);
    setSpiClock(clock_divider);
    setTimer2Configuration();
}

void     Mcp331x1::setSpiConfiguration(void) {
    SPISettings mySetting(_spi_clock, MSBFIRST, SPI_MODE1);
    SPI.beginTransaction(mySetting);
}

uint8_t* Mcp331x1::spiTransferTimeout(uint8_t data) {
    static  byte received_data[]  = {0x00, 0x00};
    const   unsigned long s_micro = micros();       

    SPDR = data;
    /*
     * The following NOP introduces a small delay that can prevent the wait
     * loop form iterating when running at the maximum speed. This gives
     * about 10% more speed, even if it seems counter-intuitive. At lower
     * speeds it is unnoticed.
     */
    asm volatile("nop");
    while (micros() - s_micro < adc_read_value_time) {
        if (SPSR & _BV(SPIF)) {
            received_data = {0x01, SPDR};
            break;
        }
    }
    return received_data;
}

void     Mcp331x1::setTimer2Configuration(void) {
    TCCR2A = 0;
    TCCR2B = 0;

    // enable fast PWM mode
    //TCCR2A = _BV(WGM21);

    //TIMSK2 |= _BV(OCIE2A) | _BV(OCIE2B);
    TIMSK2 |= _BV(OCIE2A);
    TCCR2A |= _BV(COM2A1);
/*
    #ifdef __AVR_ATmega2560__
        switch (_cs) {
            case 10: TCCR2A |= _BV(COM2A0); break; // toggle OC2A on compare match
            case  9: TCCR2A |= _BV(COM2B0); break; // toogle OC2B on compare match
        }   
    #else
        digitalWrite(_cs, HIGH);
        pinMode(_cs, OUTPUT);
    #endif
*/
    double  freq_cycle = 1 / (1.0 * _adc_bit_resolution / _spi_clock + 0.000000720);
    int     prescaler  = F_CPU / (freq_cycle * 255.0) - 1;

    if (prescaler < 1)         prescaler = 1;
    else if (prescaler < 8)    prescaler = 8;
    else if (prescaler < 32)   prescaler = 32;
    else if (prescaler < 64)   prescaler = 64;
    else if (prescaler < 128)  prescaler = 128;
    else if (prescaler < 256)  prescaler = 256;
    else if (prescaler < 1024) prescaler = 1024;
    else Serial.println("Please use faster SPI bus speed."); 

    _adc_cycle_time = F_CPU / (freq_cycle * prescaler * 1.0) - 1;    
    _adc_conversion_end = (0.000000720 * freq_cycle) * (_adc_cycle_time - 1);

    OCR2A = _adc_conversion_end;
    //OCR2B = _adc_cycle_time;

    Serial.println("Freq: " + String(freq_cycle) + " PSC: " + String(prescaler) + " OCRA: " + String(_adc_conversion_end) + " OCRB: " + String(_adc_cycle_time));
    Serial.flush();
}

static void Mcp331x1::isrA(void) {
    if (_conversion_running == false) {

    }
    _conversion_running = false;
}

ISR(TIMER2_COMPB_vect) {
  //PORTB &= ~(_BV(PB4));
  _adc_value = SPI.transfer16(0xFFFF);
  //CONVERSATION_RUNNING = false;
}

void     Mcp331x1::startTimer2(void) {
    switch (_prescaler) {
        case 1    :  TCCR2B |= _BV(CS20);                         break;  // SET CS22 & CS21 & CS20 bit for prescaler 1
        case 8    :  TCCR2B |= _BV(CS21);                         break;  // SET CS22 & CS21 & CS20 bit for prescaler 8
        case 32   :  TCCR2B |= _BV(CS21) | _BV(CS20);             break;  // SET CS22 & CS21 & CS20 bit for prescaler 32
        case 64   :  TCCR2B |= _BV(CS22);                         break;  // SET CS22 & CS21 & CS20 bit for prescaler 64
        case 128  :  TCCR2B |= _BV(CS22) | _BV(CS20);             break;  // SET CS22 & CS21 & CS20 bit for prescaler 128
        case 256  :  TCCR2B |= _BV(CS22) | _BV(CS21);             break;  // SET CS22 & CS21 & CS20 bit for prescaler 256
        case 1024 :  TCCR2B |= _BV(CS22) | _BV(CS21) | _BV(CS20); break;  // SET CS22 & CS21 & CS20 bit for prescaler 1024
        default   :  TCCR2B |= _BV(CS20);                         break;  // SET CS22 & CS21 & CS20 bit for prescaler 1
    } 
}

void     Mcp331x1::stopTimer2(void) {
    TCCR2B &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20));
}

void     Mcp331x1::begin(void) {
    startTimer2();
}

void     Mcp331x1::stop(void) {
    stopTimer2();
}

void     Mcp331x1::setBitResolution(uint8_t bit_resolution) {
    switch (bit_resolution) {
        case 12: _adc_bit_resolution = 12; break;
        case 14: _adc_bit_resolution = 14; break;
        case 16: _adc_bit_resolution = 16; break;
        default: _adc_bit_resolution = 16; break;
    }
}

uint8_t  Mcp331x1::getBitResolution(void) {
    return _adc_bit_resolution;
}

void     Mcp331x1::setVoltageRef(float v_ref) {
    _v_ref = v_ref;
}

float    Mcp331x1::getVoltageRef(void) {
    return _v_ref;
}

void     Mcp331x1::setSpiClock(uint8_t clock_divider) {
    switch (clock_divider) {
        case   2: _spi_clock = F_CPU /   2; break; // SPI Bus speed 8MHz
        case   4: _spi_clock = F_CPU /   4; break; // SPI Bus speed 4MHz
        case   8: _spi_clock = F_CPU /   8; break; // SPI Bus speed 2MHz
        case  16: _spi_clock = F_CPU /  16; break; // SPI Bus speed 1MHz
        case  32: _spi_clock = F_CPU /  32; break; // SPI Bus speed 500kHz
        case  64: _spi_clock = F_CPU /  64; break; // SPI Bus speed 250kHz
        case 128: _spi_clock = F_CPU / 128; break; // SPI Bus speed 125kHz
        default : _spi_clock = F_CPU /   2; break; // SPI Bus speed 8MHz
    }

    setSpiConfiguration();
}

uint32_t  Mcp331x1::getSpiClock(void) {
    return _spi_clock;
}

void     Mcp331x1::setChipSelectPin(uint8_t pin_number) {
    _cs = pin_number;

    #ifdef __AVR_ATmega2560__
        switch (_cs) {
            case 10: PORTB |= _BV(PB4); DDRB |= _BV(PB4); break; // OC2A
            case  9: PORTH |= _BV(PH6); DDRH |= _BV(PH6); break; // OC2B
            default: digitalWrite(_cs, HIGH); pinMode(_cs, OUTPUT);
        }   
    #else
        digitalWrite(_cs, HIGH);
        pinMode(_cs, OUTPUT);
    #endif
}

uint8_t  Mcp331x1::getChipSelectPin(void) {
    return _cs;
}

void     Mcp331x1::startCalibartion(void) {
    for(uint8_t i = 0; i < 1024/16; i++)
        SPI.transfer16(0xFFFF);
}
uint16_t Mcp331x1::getValue(void) {
    if (_adc_in_use) return 0;
    _adc_in_use = true;

    double value;

    _adc_in_use = false;
    return value;
}

uint16_t Mcp331x1::getValueExponentialSmoothing(uint32_t value_counter, float smoothing_factor) {
    if (_adc_in_use) return 0;
    _adc_in_use = true;
    
    uint16_t value_arr[value_counter];
    uint16_t value;

    startTimer2();
    Serial.println("start"); Serial.flush();
    delay(5000);

    for (uint8_t i = 0; i < value_counter; i++) {
        PORTB |=   _BV(PB4);
        TCNT2  = 0;
        _conversion_running = true;
        while (_conversion_running == false) {} 


        //while (TCNT2 < _adc_cycle_time) {}
        //PORTB &= ~(_BV(PB4));

        //value_arr[i] = _adc_value;
        //value_arr[i] = SPI.transfer16(0xFFFF);

        while (TCNT2 < _adc_cycle_time) {} // sehr wahrscheinlich nicht mehr nötig
    }
    Serial.println("stop"); Serial.flush();
    stopTimer2();

    for (uint8_t i = 0; i < value_counter; i++) {
        switch (_adc_bit_resolution) {
            case 12: value_arr[i] >> 4;
            case 14: value_arr[i] >> 2;
            case 16: value_arr[i] >> 0;
        }
        if (i > 0) value = smoothing_factor * value_arr[i] + (1 - smoothing_factor) * value_arr[i - 1];
        else       value = value_arr[i];
    }

    _adc_in_use = false;
    return value;
}
double   Mcp331x1::getVoltage(void) {
    if (_adc_in_use) return 0;
    _adc_in_use = true;

    double voltage;

    _adc_in_use = false;
    return voltage;
}
double   Mcp331x1::getVoltageExponentialSmoothing(uint32_t value_counter, float smoothing_factor) {
    if (_adc_in_use) return 0;
    _adc_in_use = true;

    double voltage;
    double value_arr[value_counter];
    
    _adc_in_use = false;
    return voltage;
}