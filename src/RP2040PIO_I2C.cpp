#include "RP2040PIO_I2C.h"
#include "hardware/clocks.h"

static const int PIO_I2C_ICOUNT_LSB = 10;
static const int PIO_I2C_FINAL_LSB = 9;
static const int PIO_I2C_DATA_LSB = 1;
static const int PIO_I2C_NAK_LSB = 0;

RP2040PIO_I2C::RP2040PIO_I2C(PIO pio, uint pin_sda, uint pin_scl, uint sm)
    : TwoWire(i2c0, pin_sda, pin_scl), 
      _pio(pio), _sm(sm), _pin_sda(pin_sda), _pin_scl(pin_scl), 
      _offset(0), _clock_freq(100000), _initialized(false), _in_transaction(false),
      _tx_buffer_len(0), _rx_buffer_len(0), _rx_buffer_index(0) {
}

RP2040PIO_I2C::~RP2040PIO_I2C() {
    end();
}

void RP2040PIO_I2C::begin() {
    if (_initialized) return;

    _offset = pio_add_program(_pio, &i2c_program);
    i2c_program_init(_pio, _sm, _offset, _pin_sda, _pin_scl);
    
    // Default clock 100kHz
    setClock(_clock_freq);
    
    _initialized = true;
    _in_transaction = false;
}

void RP2040PIO_I2C::begin(uint8_t address) {
    // Slave mode not supported in this PIO implementation
    (void)address;
    begin();
}

void RP2040PIO_I2C::end() {
    if (!_initialized) return;
    pio_sm_set_enabled(_pio, _sm, false);
    pio_remove_program(_pio, &i2c_program, _offset);
    _initialized = false;
    _in_transaction = false;
}

void RP2040PIO_I2C::setClock(uint32_t freq) {
    _clock_freq = freq;
    if (!_initialized) return;

    float div = (float)clock_get_hz(clk_sys) / (32 * freq);
    pio_sm_set_clkdiv(_pio, _sm, div);
}

void RP2040PIO_I2C::beginTransmission(uint8_t address) {
    _address = address;
    _tx_buffer_len = 0;
}

uint8_t RP2040PIO_I2C::endTransmission(bool sendStop) {
    int err = 0;
    
    if (_in_transaction) {
        pio_i2c_repstart();
    } else {
        pio_i2c_start();
    }
    _in_transaction = true;

    pio_i2c_rx_enable(false);
    
    // Address with Write bit (0)
    pio_i2c_put16((_address << 2) | 1u);

    for (size_t i = 0; i < _tx_buffer_len; i++) {
        if (pio_i2c_check_error()) break;
        
        bool last = (i == _tx_buffer_len - 1);
        pio_i2c_put_or_err((_tx_buffer[i] << PIO_I2C_DATA_LSB) | (last << PIO_I2C_FINAL_LSB) | 1u);
    }

    if (sendStop) {
        pio_i2c_stop();
        _in_transaction = false;
    }
    
    pio_i2c_wait_idle();

    if (pio_i2c_check_error()) {
        err = 2; // NACK on address
        pio_i2c_resume_after_error();
        if (sendStop) {
            pio_i2c_stop();
            _in_transaction = false;
        }
    }

    _tx_buffer_len = 0;
    return err;
}

size_t RP2040PIO_I2C::write(uint8_t data) {
    if (_tx_buffer_len < sizeof(_tx_buffer)) {
        _tx_buffer[_tx_buffer_len++] = data;
        return 1;
    }
    return 0;
}

size_t RP2040PIO_I2C::write(const uint8_t *data, size_t quantity) {
    size_t written = 0;
    for (size_t i = 0; i < quantity; i++) {
        if (write(data[i])) written++;
        else break;
    }
    return written;
}

size_t RP2040PIO_I2C::requestFrom(uint8_t address, size_t quantity, bool sendStop) {
    _rx_buffer_len = 0;
    _rx_buffer_index = 0;

    if (_in_transaction) {
        pio_i2c_repstart();
    } else {
        pio_i2c_start();
    }
    _in_transaction = true;

    pio_i2c_rx_enable(true);
    
    // Drain RX FIFO
    while (!pio_sm_is_rx_fifo_empty(_pio, _sm))
        (void)pio_i2c_get();

    // Address with Read bit (1)
    pio_i2c_put16((address << 2) | 3u);

    uint32_t tx_remain = (uint32_t)quantity;
    uint32_t rx_remain = (uint32_t)quantity;
    bool first = true;

    while ((tx_remain || rx_remain) && !pio_i2c_check_error()) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(_pio, _sm)) {
            --tx_remain;
            pio_i2c_put16((0xffu << 1) | (tx_remain ? 0 : (1u << PIO_I2C_FINAL_LSB) | (1u << PIO_I2C_NAK_LSB)));
        }
        if (!pio_sm_is_rx_fifo_empty(_pio, _sm)) {
            if (first) {
                (void)pio_i2c_get();
                first = false;
            } else {
                if (_rx_buffer_len < sizeof(_rx_buffer)) {
                    _rx_buffer[_rx_buffer_len++] = pio_i2c_get();
                } else {
                    (void)pio_i2c_get();
                }
                --rx_remain;
            }
        }
    }

    if (sendStop) {
        pio_i2c_stop();
        _in_transaction = false;
    }
    pio_i2c_wait_idle();

    if (pio_i2c_check_error()) {
        pio_i2c_resume_after_error();
        if (sendStop) {
            pio_i2c_stop();
            _in_transaction = false;
        }
    }

    return _rx_buffer_len;
}

int RP2040PIO_I2C::available() {
    return _rx_buffer_len - _rx_buffer_index;
}

int RP2040PIO_I2C::read() {
    if (_rx_buffer_index < _rx_buffer_len) {
        return _rx_buffer[_rx_buffer_index++];
    }
    return -1;
}

int RP2040PIO_I2C::peek() {
    if (_rx_buffer_index < _rx_buffer_len) {
        return _rx_buffer[_rx_buffer_index];
    }
    return -1;
}

void RP2040PIO_I2C::flush() {
}

// Internal low-level functions

void RP2040PIO_I2C::pio_i2c_start() {
    pio_i2c_put_or_err(2u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);
    pio_i2c_put_or_err(pio_encode_mov(pio_isr, pio_null));
}

void RP2040PIO_I2C::pio_i2c_stop() {
    pio_i2c_put_or_err(2u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD1]);
}

void RP2040PIO_I2C::pio_i2c_repstart() {
    pio_i2c_put_or_err(4u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD1]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD1]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);
    pio_i2c_put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);
    pio_i2c_put_or_err(pio_encode_mov(pio_isr, pio_null));
}

bool RP2040PIO_I2C::pio_i2c_check_error() {
    return pio_interrupt_get(_pio, _sm);
}

void RP2040PIO_I2C::pio_i2c_resume_after_error() {
    pio_sm_drain_tx_fifo(_pio, _sm);
    pio_sm_exec(_pio, _sm, (_pio->sm[_sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    pio_interrupt_clear(_pio, _sm);
}

void RP2040PIO_I2C::pio_i2c_rx_enable(bool en) {
    if (en)
        hw_set_bits(&_pio->sm[_sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    else
        hw_clear_bits(&_pio->sm[_sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
}

void RP2040PIO_I2C::pio_i2c_put_or_err(uint16_t data) {
    while (pio_sm_is_tx_fifo_full(_pio, _sm))
        if (pio_i2c_check_error())
            return;
    if (pio_i2c_check_error())
        return;
    *(io_rw_16 *)&_pio->txf[_sm] = data;
}

void RP2040PIO_I2C::pio_i2c_put16(uint16_t data) {
    while (pio_sm_is_tx_fifo_full(_pio, _sm));
    *(io_rw_16 *)&_pio->txf[_sm] = data;
}

uint8_t RP2040PIO_I2C::pio_i2c_get() {
    return (uint8_t)pio_sm_get(_pio, _sm);
}

void RP2040PIO_I2C::pio_i2c_wait_idle() {
    _pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + _sm);
    while (!(_pio->fdebug & 1u << (PIO_FDEBUG_TXSTALL_LSB + _sm) || pio_i2c_check_error()))
        tight_loop_contents();
}
