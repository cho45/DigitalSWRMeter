/**
 *
 * I2C (TWI) library for AVR
 *
 * ## Supports
 *
 *   * 7bit addressed access
 *   * Master synchronous actions
 *   * Slave automated asynchronous actions
 *      * Just specify data memory array.
 *
 * ## SYNOPSYS
 *
 * Master Usage:
 *
 *     uint8_t data[7];
 *     uint8_t ret;
 *
 *     i2c_set_bitrate(100);
 *     // Set target slave address
 *     i2c_master_init(0x60);
 *     ret = i2c_master_write((uint8_t*)"\x04", 1);
 *     if (ret) goto error;
 *     ret = i2c_master_read(data, 8);
 *     if (ret) goto error;
 *     i2c_master_stop();
 *
 *     error:
 *         i2c_master_stop();
 *
 * Slave Usage:
 *     // Slave memory map (must be smaller than 254 (0xfe) bytes)
 *     uint8_t data[9];
 *     // Enter to slave receive mode with data and size.
 *     // After this operation, data will be changed automatically by TWI interrupt.
 *     i2c_slave_init(0x65, data, 10);
 *
 *     // Access (set or get) to I2C data block
 *     data[0] = 0x10;
 *
 *     // Or more readable code with struct
 *     struct {
 *         uint8_t foo_flag1;
 *         uint8_t foo_flag2;
 *         uint16_t bar_value1;
 *         uint16_t bar_value2;
 *         uint16_t bar_value3;
 *         uint16_t bar_value4;
 *     } data;
 * 
 *     i2c_slave_init(0x65, &data, 10);
 *
 * # Note
 *
 *  * CPU frequency must be must be at least 16 times higher than the SCL frequency
 *    * SCL 100kHz -> F_CPU 1.6MHz
 *    * SCL 400kHz -> F_CPU 6.4MHz
 */

#define I2C_START                       0x08
#define I2C_REPEATED_START              0x10
#define I2C_NO_STATE_INFORMATION        0xF8
#define I2C_BUS_ERROR                   0x00

#define I2C_MASTER_ARB_LOST             0x38
#define I2C_MASTER_TX_SLA_ACK           0x18
#define I2C_MASTER_TX_SLA_NACK          0x20
#define I2C_MASTER_TX_DATA_ACK          0x28
#define I2C_MASTER_TX_DATA_NACK         0x30

#define I2C_MASTER_RX_SLA_ACK           0x40
#define I2C_MASTER_RX_SLA_NACK          0x48
#define I2C_MASTER_RX_DATA_ACK          0x50
#define I2C_MASTER_RX_DATA_NACK         0x58

#define I2C_SLAVE_TX_SLA_ACK            0xA8
#define I2C_SLAVE_TX_ARB_LOST_SLA_ACK   0xB0
#define I2C_SLAVE_TX_DATA_ACK           0xB8
#define I2C_SLAVE_TX_DATA_NACK          0xC0
#define I2C_SLAVE_TX_LAST_DATA          0xC8

#define I2C_SLAVE_RX_SLA_ACK            0x60
#define I2C_SLAVE_RX_ARB_LOST_SLA_ACK   0x68
#define I2C_SLAVE_RX_GCALL_ACK          0x70
#define I2C_SLAVE_RX_ARB_LOST_GCALL_ACK 0x78
#define I2C_SLAVE_RX_DATA_ACK           0x80
#define I2C_SLAVE_RX_DATA_NACK          0x88
#define I2C_SLAVE_RX_GCALL_DATA_ACK     0x90
#define I2C_SLAVE_RX_GCALL_DATA_NACK    0x98
#define I2C_SLAVE_RX_STOP               0xA0

static inline void i2c_set_bitrate (uint16_t bitrate);
static inline void i2c_slave_init (uint8_t address, void* data, uint8_t size);
static inline void i2c_master_init (uint8_t address);
static inline uint8_t i2c_master_read (uint8_t* data, uint8_t size);
static inline uint8_t i2c_master_write (uint8_t* data, uint8_t size);
static inline void i2c_master_stop ();

volatile enum {
	I2C_STATE_IDLE,
	I2C_STATE_MASTER_TX,
	I2C_STATE_MASTER_RX,
	I2C_STATE_MASTER_TX_DONE,
	I2C_STATE_MASTER_RX_DONE,

	I2C_STATE_SLAVE_TX,
	I2C_STATE_SLAVE_RX,

	I2C_STATE_ERROR = 250, 
	I2C_STATE_BUS_ERROR,
} i2c_state;

static uint8_t* _i2c_slave_data;
static uint8_t  _i2c_slave_data_size;

static uint8_t* _i2c_master_data;
static uint8_t  _i2c_master_data_size;
static uint8_t  _i2c_master_target_address;

static inline void _i2c_start ();
static inline void _i2c_stop ();
static inline void _i2c_transmit_byte (uint8_t byte);
static inline void _i2c_receive_continue ();
static inline void _i2c_receive_finish ();


// Typically, bitrate should be 100 (Standard-mode) or 400 (Fast-mode) (kbit/s)
static inline void i2c_set_bitrate (uint16_t bitrate) {
	// Pre-Scaler = 1
	TWSR = 0;
	// Bit Rate
	TWBR = ( (F_CPU / 1000l) - (16 * bitrate) ) / (2 * bitrate);
}

static inline void i2c_slave_init (uint8_t address, void* data, uint8_t size) {
	i2c_state = I2C_STATE_IDLE;
	_i2c_slave_data = (uint8_t*)data;
	_i2c_slave_data_size = size;

	TWAR = address << 1;
	// Slave Mode with interrupt enabled
	TWCR = (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
}

static inline void i2c_master_init (uint8_t address) {
	i2c_state = I2C_STATE_IDLE;
	_i2c_master_target_address = address << 1;
	TWCR = (1<<TWEN) | (1<<TWIE);
}

static inline uint8_t i2c_master_read (uint8_t* data, uint8_t size) {
	i2c_state = I2C_STATE_MASTER_RX;

	_i2c_master_data = data;
	_i2c_master_data_size = size;

	_i2c_master_target_address |= 1;
	_i2c_start();

	while (i2c_state == I2C_STATE_MASTER_RX);

	if (i2c_state == I2C_STATE_MASTER_RX_DONE) {
		return 0;
	} else {
		return i2c_state;
	}
}

static inline uint8_t i2c_master_write (uint8_t* data, uint8_t size) {
	i2c_state = I2C_STATE_MASTER_TX;

	_i2c_master_data = data;
	_i2c_master_data_size = size;

	_i2c_master_target_address &= ~1;
	_i2c_start();

	while (i2c_state == I2C_STATE_MASTER_TX);

	if (i2c_state == I2C_STATE_MASTER_TX_DONE) {
		return 0;
	} else {
		return i2c_state;
	}
}

static inline void i2c_master_stop () {
	_i2c_master_target_address = 0;
	_i2c_stop();
	i2c_state = I2C_STATE_IDLE;
}

// Private functions

static inline void _i2c_start () {
	TWCR = (TWCR & 0b00001111) | (1<<TWINT) | (1<<TWSTA);
}

static inline void _i2c_stop () {
	TWCR = (TWCR & 0b00001111) | (1<<TWINT) | (1<<TWSTO);
}

static inline void _i2c_transmit_byte (uint8_t byte) {
	TWDR = byte;
	TWCR = (TWCR & 0b00001111) | (1<<TWINT);
}

static inline void _i2c_receive_continue () {
	TWCR = (TWCR & 0b00001111) | (1<<TWINT) | (1<<TWEA);
}

static inline void _i2c_receive_finish () {
	TWCR = (TWCR & 0b00001111) | (1<<TWINT);
}

ISR(TWI_vect) {
	uint8_t status = TWSR & 0b11111000;
	static uint8_t data_index;

//	printf("TWI:%x\r\n", status);
//	_delay_ms(100);

	switch (status) {
		case I2C_BUS_ERROR                   : // 0x00
			i2c_state = I2C_STATE_BUS_ERROR;
			TWCR = (TWCR & 0b00001111) | (1<<TWSTO) |  (1<<TWINT);
			break;
		case I2C_NO_STATE_INFORMATION        : // 0xF8
			i2c_state = I2C_STATE_ERROR;
			break;

		/*********************************************************************************************************
		 * Master
		 */
		case I2C_START                       : // 0x08 A START condition has been transmitted
		case I2C_REPEATED_START              : // 0x10 A repeated START condition has been transmitted
			data_index = 0;
			_i2c_transmit_byte(_i2c_master_target_address);
			break;

		case I2C_MASTER_ARB_LOST             : // 0x38 Arbitration lost in SLA+R or NOT ACK bit
			TWCR = (TWCR & 0b00001111) | (1<<TWINT);
			i2c_state = I2C_STATE_IDLE;
			break;

		/**
		 * Master Transmitter Mode
		 */
		case I2C_MASTER_TX_SLA_ACK           : // 0x18 SLA+W has been transmitted; ACK has been received
		case I2C_MASTER_TX_DATA_ACK          : // 0x28 Data byte has been transmitted; ACK has been received
			if (data_index < _i2c_master_data_size) {
				_i2c_transmit_byte(_i2c_master_data[data_index++]);
			} else {
				_i2c_stop();
				i2c_state = I2C_STATE_MASTER_TX_DONE;
			}
			break;

		case I2C_MASTER_TX_SLA_NACK          : // 0x20 SLA+W has been transmitted; NOT ACK has been received
		case I2C_MASTER_TX_DATA_NACK         : // 0x30 Data byte has been transmitted; NOT ACK has been received
			_i2c_stop();
			i2c_state = I2C_STATE_MASTER_TX_DONE;
			break;

		/**
		 * Master Receiver Mode
		 */
		case I2C_MASTER_RX_SLA_ACK           : // 0x40 SLA+R has been transmitted; ACK has been received
			_i2c_receive_continue();
			break;
		case I2C_MASTER_RX_DATA_ACK          : // 0x50 Data byte has been received; ACK has been returned
			if (data_index < _i2c_master_data_size) _i2c_master_data[data_index++] = TWDR;
			if (data_index < _i2c_master_data_size - 1) {
				_i2c_receive_continue();
			} else {
				_i2c_receive_continue();
				i2c_state = I2C_STATE_MASTER_RX_DONE;
			}
			break;
		case I2C_MASTER_RX_SLA_NACK          : // 0x48 SLA+R has been transmitted; NOT ACK has been received
			_i2c_stop();
			i2c_state = I2C_STATE_MASTER_RX_DONE;
			break;
		case I2C_MASTER_RX_DATA_NACK         : // 0x58 Data byte has been received; NOT ACK has been returned
			if (data_index < _i2c_master_data_size) _i2c_master_data[data_index++] = TWDR;
			_i2c_stop();
			i2c_state = I2C_STATE_MASTER_RX_DONE;
			break;



		/*********************************************************************************************************
		 * Slave
		 */

		/**
		 * Slave Transmitter Mode
		 */
		case I2C_SLAVE_TX_SLA_ACK            : // 0xA8 Own SLA+R has been received; ACK has been returned
		case I2C_SLAVE_TX_ARB_LOST_SLA_ACK   : // 0xB0 Own SLA+R has been received; ACK has been returned
			i2c_state = I2C_STATE_SLAVE_TX;
		case I2C_SLAVE_TX_DATA_ACK           : // 0xB8 Data byte in TWDR has been transmitted; ACK has been received
			if (_i2c_slave_data) {
				TWDR = _i2c_slave_data[data_index++]; 
				// XXX : TWDR is broken (msb always set) with -Os below line fix it...
				_delay_us(10);
				if (data_index < _i2c_slave_data_size) {
					_i2c_receive_continue();
				 } else {
					_i2c_receive_finish();
				}
			} else {
				_i2c_receive_continue();
			}
			break;

		case I2C_SLAVE_TX_DATA_NACK          : // 0xC0 Data byte in TWDR has been transmitted; NOT ACK has been received
		case I2C_SLAVE_TX_LAST_DATA          : // 0xC8 Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
			// Back to SLAVE MODE
			_i2c_receive_continue();
			i2c_state = I2C_STATE_IDLE;
			break;

		/**
		 * Slave Receiver Mode
		 */
		case I2C_SLAVE_RX_SLA_ACK            : // 0x60 Own SLA+W has been received; ACK has been returned
		case I2C_SLAVE_RX_ARB_LOST_SLA_ACK   : // 0x68 Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
		case I2C_SLAVE_RX_GCALL_ACK          : // 0x70 General call address has been received; ACK has been returned
		case I2C_SLAVE_RX_ARB_LOST_GCALL_ACK : // 0x78 Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
			i2c_state = I2C_STATE_SLAVE_RX;
			data_index = 0xff;
			_i2c_receive_continue();
			break;

		case I2C_SLAVE_RX_DATA_ACK           : // 0x80 Previously addressed with own SLA+W; data has been received; ACK has been returned
		case I2C_SLAVE_RX_GCALL_DATA_ACK     : // 0x90 Previously addressed with general call; data has been re- ceived; ACK has been returned
			if (_i2c_slave_data) {
				if (data_index != 0xff) {
					if (data_index < _i2c_slave_data_size) _i2c_slave_data[data_index++] = TWDR;
					if (data_index < _i2c_slave_data_size - 1) {
						_i2c_receive_continue();
					 } else {
						_i2c_receive_finish();
					 }
				} else {
					data_index = TWDR;
					_i2c_receive_continue();
				}
			} else {
				_i2c_receive_continue();
			}
			break;

		case I2C_SLAVE_RX_DATA_NACK          : // 0x88 Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
		case I2C_SLAVE_RX_GCALL_DATA_NACK    : // 0x98 Previously addressed with general call; data has been received; NOT ACK has been returned
			if (data_index < _i2c_slave_data_size) _i2c_slave_data[data_index++] = TWDR;
			// Back to SLAVE MODE
			_i2c_receive_continue();
			break;

		case I2C_SLAVE_RX_STOP               : // 0xA0 A STOP condition or repeated START condition has been received while still addressed as Slave
			// Back to SLAVE MODE
			_i2c_receive_continue();
			i2c_state = I2C_STATE_IDLE;
			break;
	}
}
