#ifndef RP2040PIO_I2C_H
#define RP2040PIO_I2C_H

#include <Arduino.h>
#include <Wire.h>
#include "hardware/pio.h"
#include "i2c.pio.h"

class RP2040PIO_I2C : public TwoWire
{
public:
    RP2040PIO_I2C(PIO pio, uint pin_sda, uint pin_scl, uint sm = 0);
    virtual ~RP2040PIO_I2C();

    void begin() override;
    void begin(uint8_t address) override; // Slave mode not supported
    void end() override;

    void setClock(uint32_t freq) override;

    void beginTransmission(uint8_t address) override;
    uint8_t endTransmission(bool sendStop) override;
    uint8_t endTransmission(void) override { return endTransmission(true); }

    size_t write(uint8_t data) override;
    size_t write(const uint8_t *data, size_t quantity) override;

    size_t requestFrom(uint8_t address, size_t quantity, bool sendStop) override;
    size_t requestFrom(uint8_t address, size_t quantity) override { return requestFrom(address, quantity, true); }

    int available() override;
    int read() override;
    int peek() override;
    void flush() override;

    // Additional methods for PIO management
    bool isReady() const { return _initialized; }

private:
    PIO _pio;
    uint _sm;
    uint _pin_sda;
    uint _pin_scl;
    uint _offset;
    uint32_t _clock_freq;
    bool _initialized;
    bool _in_transaction;

    uint8_t _address;
    uint8_t _tx_buffer[256];
    size_t _tx_buffer_len;

    uint8_t _rx_buffer[256];
    size_t _rx_buffer_len;
    size_t _rx_buffer_index;

    // Internal low-level functions
    void pio_i2c_start();
    void pio_i2c_stop();
    void pio_i2c_repstart();
    bool pio_i2c_check_error();
    void pio_i2c_resume_after_error();
    void pio_i2c_rx_enable(bool en);
    void pio_i2c_put_or_err(uint16_t data);
    void pio_i2c_put16(uint16_t data);
    uint8_t pio_i2c_get();
    void pio_i2c_wait_idle();
};

#endif // RP2040PIO_I2C_H
