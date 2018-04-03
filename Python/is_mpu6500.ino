// Accelerometer Sensor Driver - MPU6500
// Tim Trippel & Ofir Weisse
// SPQR Lab
// University of Michigan
// 4/13/16

#include <Wire.h>

// ----------------------------------------------------
// Arduino Uno R3 - Pinout                            |
// ----------------------------------------------------
// SPI Interface:
// SCK  = Pin 13
// MISO = Pin 12
// MOSI = Pin 11
// SS   = Pin 10

#define ACCEL_X_HIGH        0x3B // Upper Byte
#define ACCEL_X_LOW         0x3C // Low Byte
#define ACCEL_Y_HIGH        0x3D // Upper Byte
#define ACCEL_Y_LOW         0x3E // Low Byte
#define ACCEL_Z_HIGH        0x3F // Upper Byte
#define ACCEL_Z_LOW         0x40 // Low Byte
#define PWR_MGMT_1 		    0x6B // R/W
#define PWR_MGMT_2          0x6C // R/W
#define SIGNAL_PATH_RESET   0x68
#define USER_CTL            0x6A
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG_2 		0x1D 
#define WHO_AM_I			0x70
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define INT_STATUS          0x3A

#define NUM_DATA_POINTS     256
#define SERIAL_BAUDRATE     115200
#define SLAVE_SELECT        10
#define INT1                9
#define READ_DATA_FLAG      0x80
#define DEVICE_ADDR		    0x68

// Reset values
#define RESET_DEVICE        0x80
#define RESET_SIGNALS       0x07
#define RESET_DATA_REGS     0x01

// ACCEL_CONFIG
#define RANGE_2G            0x00
#define RANGE_4G            0x04
#define RANGE_8G            0x08
#define RANGE_16G           0x0C

// INT_PIN_CFG
#define INT_HIGH_UNTIL_CLEARED   0x20
#define INT_CLEARED_BY_DATA_READ 0x10

// INT_ENABLE
#define DATA_READY_INT_ENABLE    0x01
#define DATA_READY_INT_DISABLE   0x00

// INT_STATUS
#define DATA_READY_INT_OCCURED   0x01

uint16_t raw_data[NUM_DATA_POINTS];
uint16_t total_time_x, total_time_y, total_time_z;
unsigned long start_time, end_time;

void swap( uint16_t* value )
{
    uint8_t temp;
    temp = *value & 0x00FF;
    *value >>= 8;
    *value |= ( temp << 8 );
}

void initialize_sensor()
{
	// uint8_t data = 0xFF;
	// read_i2c_bytes( WHO_AM_I, &data, 1 );

    uint8_t value = 0;
    // Reset Device
    // value = RESET_DEVICE;
    // write_i2c_bytes( PWR_MGMT_1, &value, 1 );
    // delay(200);
    // value = 0;
    // write_i2c_bytes( PWR_MGMT_1, &value, 1 );

    value = INT_HIGH_UNTIL_CLEARED;
    write_i2c_bytes( INT_PIN_CFG, &value, 1 );
    
    value = DATA_READY_INT_ENABLE;
    write_i2c_bytes( INT_ENABLE, &value, 1 );

    value = 0;
    write_i2c_bytes( PWR_MGMT_1, &value, 1 );

    value = 0;
    write_i2c_bytes( PWR_MGMT_2, &value, 1 );
}

void setup(){
    Serial.begin( SERIAL_BAUDRATE ); // start serial connection to computer/python script
    Wire.begin();
    // TWBR = 12;              // set to 400kHz
    delay(100);

    pinMode( INT1, INPUT );
    delay(5);

    initialize_sensor();    // initialize sensor device
    delay(300);             // pause for initialization
    
    return;
}

void loop(){
    run_experiment();
}

void run_experiment()
{
    wait_for_que_from_pc();
    
    // get x data
    start_time = millis();
    get_sensor_data('x');
    end_time = millis();
    total_time_x = (end_time - start_time);
    send_x_data_to_pc();
    
    // get y data
    start_time = millis();
    get_sensor_data('y');
    end_time = millis();
    total_time_y = (end_time - start_time);
    send_y_data_to_pc();
    
    // get z data
    start_time = millis();
    get_sensor_data('z');
    end_time = millis();
    total_time_z = (end_time - start_time);
    send_z_data_to_pc();
}

void get_sensor_data( char axis )
{   
    int i = 0;
    // update sensor data 
    while ( i < NUM_DATA_POINTS )
    {
        get_sensor_data( axis, i );
        i++; 
    }
}

// Get sensor data
void get_sensor_data( char axis, int sample_num )
{
    uint8_t value = 0x00;

    while( digitalRead( INT1 ) == 0 ){};
    
    if ( axis == 'x' )
    {
        read_i2c_bytes( ACCEL_X_HIGH, (uint8_t*)&raw_data[sample_num], 2 );
    }
    else if ( axis == 'y' )
    {
        read_i2c_bytes( ACCEL_Y_HIGH, (uint8_t*)&raw_data[sample_num], 2 );
    }
    else
    {
        read_i2c_bytes( ACCEL_Z_HIGH, (uint8_t*)&raw_data[sample_num], 2 );
    }

    swap( &raw_data[ sample_num ] );
    read_i2c_bytes( INT_STATUS, &value, 1 );
}

// Wait for PC to begin experiment
void wait_for_que_from_pc()
{
    int status = Serial.available();
    while (Serial.available() <= 0){}
    Serial.read();
}

// Send x data to Python logger --> ALWAYS send 16-bit data
void send_x_data_to_pc()
{
    Serial.print( "X-Data: ");
    Serial.println( sizeof( total_time_x ) + sizeof( raw_data ));
    Serial.write( (uint8_t*) &total_time_x, sizeof( total_time_x ));
    Serial.write( (uint8_t*) &raw_data, sizeof( raw_data ));
    Serial.println( "" ); 
}

// Send y data to Python logger --> ALWAYS send 16-bit data
void send_y_data_to_pc()
{
    Serial.print( "Y-Data: ");
    Serial.println( sizeof( total_time_y ) + sizeof( raw_data ));
    Serial.write( (uint8_t*) &total_time_y, sizeof( total_time_y ));
    Serial.write( (uint8_t*) &raw_data, sizeof( raw_data ));
    Serial.println( "" ); 
}

// Send z data to Python logger --> ALWAYS send 16-bit data
void send_z_data_to_pc()
{
    Serial.print( "Z-Data: ");
    Serial.println( sizeof( total_time_z ) + sizeof( raw_data ));
    Serial.write( (uint8_t*) &total_time_z, sizeof( total_time_z ));
    Serial.write( (uint8_t*) &raw_data, sizeof( raw_data ));
    Serial.println( "" ); 
}

// I2C - RX Bytes Function
void read_i2c_bytes( uint8_t reg_addr, uint8_t *buffer, uint8_t num_bytes )
{
    Wire.beginTransmission( DEVICE_ADDR );      // start WRITE transmission
    Wire.write( reg_addr );                     // transmit register address to read
    Wire.endTransmission(false);                // end WRITE transmission and send RESTART
    Wire.requestFrom( DEVICE_ADDR, num_bytes ); // start READ request
    while ( !Wire.available() ) {}              // wait for response
    Wire.readBytes( buffer, num_bytes );        // read bytes
    return;
}

// I2C - TX Bytes Function
void write_i2c_bytes( uint8_t reg_addr, uint8_t *buffer, uint8_t num_bytes )
{
    uint8_t i = 0;
    Wire.beginTransmission( DEVICE_ADDR );  // start WRITE transmission
    Wire.write( reg_addr );                 // transmit register address to write
    while ( i < num_bytes )
    {
        Wire.write( buffer[i] );            // transmit values to write
        i++;        
    }
    Wire.endTransmission();                 // end WRITE transmission
    return;
}
