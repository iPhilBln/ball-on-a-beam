#ifndef MCP331x1_H_
#define MCP331x1_H_
    #include "common.h"

    class Mcp331x1 {
        private:
                    uint8_t  _msb;
                    uint8_t  _lsb;
                    uint8_t  _cs;
                    uint32_t _spi_clock;
                    uint8_t  _adc_bit_resolution;
                    float    _v_ref;

            static  uint16_t _adc_value;
            static  bool     _adc_in_use;
            static  bool     _conversion_running;
                    int      _prescaler;
                    uint8_t  _adc_conversion_end;
                    uint8_t  _adc_cycle_time;


                    void        setSpiConfiguration(void);
                    uint8_t*    spiTransferTimeout(uint8_t data);
                    void        setTimer2Configuration(void);
                    void        startTimer2(void);
                    void        stopTimer2(void);
            static  void        isrA() __attribute__((signal, used, externally_visible));

        public:
            // Konstruktor
            Mcp331x1();
            Mcp331x1(uint8_t bit_resolution);
            Mcp331x1(uint8_t bit_resolution, float v_ref);
            Mcp331x1(uint8_t bit_resolution, float v_ref, uint8_t clock_divider);
            Mcp331x1(uint8_t bit_resolution, float v_ref, uint8_t clock_divider, uint8_t cs_pin_number);  

            // SETTER & GETTER
            void        setBitResolution(uint8_t bit_resolution);
            uint8_t     getBitResolution(void);

            void        setVoltageRef(float v_ref);
            float       getVoltageRef(void);

            void        setSpiClock(uint8_t clock_divider);
            uint32_t    getSpiClock(void);

            void        setChipSelectPin(uint8_t pin_number);
            uint8_t     getChipSelectPin(void);

            // ADC functions
            void        begin(void);
            void        stop(void);
            void        startCalibartion(void);
            uint16_t    getValue(void);
            uint16_t    getValueExponentialSmoothing(uint32_t value_counter, float smoothing_factor);
            double      getVoltage(void);
            double      getVoltageExponentialSmoothing(uint32_t value_counter, float smoothing_factor);
    };
#endif
