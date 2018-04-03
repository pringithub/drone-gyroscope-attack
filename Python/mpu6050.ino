// Accelerometer Sensor Driver Template
// Tim Trippel & Ofir Weisse
// SPQR Lab
// University of Michigan
// 3/10/16

#include <Wire.h>
#include <SPI.h>

// ----------------------------------------------------
// Arduino Uno R3 - Pinout                            |
// ----------------------------------------------------
// I2C Interface:
// SDA = Pin A4
// SCL = Pin A5

// SPI Interface:
// SCK  = Pin 13
// MISO = Pin 12
// MOSI = Pin 11
// SS   = Pin 10

// ----------------------------------------------------
// TODO: Define all device REGISTER OFFSETS here:     |
// ----------------------------------------------------

// sensor data size (bits) --> 16 is DEFAULT
#define DATA_SIZE     16

// 16-Bit/12-Bit sensor data:
#define ACCEL_X_HIGH        0x3B // Upper Byte
#define ACCEL_X_LOW         0x3C // Low Byte
#define ACCEL_Y_HIGH        0x3D // Upper Byte
#define ACCEL_Y_LOW         0x3E // Low Byte
#define ACCEL_Z_HIGH        0x3F // Upper Byte
#define ACCEL_Z_LOW         0x40 // Low Byte
#define GYRO_X_HIGH         0x3B // Upper Byte
#define GYRO_X_LOW          0x3C // Low Byte
#define GYRO_Y_HIGH         0x3D // Upper Byte
#define GYRO_Y_LOW          0x3E // Low Byte
#define GYRO_Z_HIGH         0x3F // Upper Byte
#define GYRO_Z_LOW          0x40 // Low Byte
#define MPU6050_PWR_MGMT_1  0x6B // R/W
// 8-Bit sensor data:
#define ACCEL_X       0xFF
#define ACCEL_Y       0xFF
#define ACCEL_Z       0xFF

// ----------------------------------------------------
// TODO: configure experiment settings here:          |
// ----------------------------------------------------
// Settings include:
// 1. number of data points to collect per frequency  - (NUM_DATA_POINTS)
// 2. baud rate of serial connection to python script - (SERIAL_BAUDRATE)

#define NUM_DATA_POINTS 512
#define SERIAL_BAUDRATE 115200

// ----------------------------------------------------
// TODO: configure driver communication interface     |
// ----------------------------------------------------
// If I2C: define DEVICE_ADDR 
// IF SPI: define SLAVE_SELECT
//
// Configure communication interface type:
// SPI --> com = SPI;
// I2C --> com = I2C; --> DEFAULT

#define DEVICE_ADDR  0x68
#define SLAVE_SELECT 10

#define swap(val) ( ( val & 0x00ff ) << 8) | ( val >> 8 )

enum com_type {
  spi,
  i2c,
};

// ----------------------------------------------------
// TODO: define experimental setup global variables   |
// ----------------------------------------------------
// 1. com_type = communication interface with sensor
// 2. x_data   = x-axis data from sensor
// 3. y_data   = y-axis data from sensor
// 4. z_data   = z-axis data from sensor

com_type com = i2c;

uint16_t raw_data[NUM_DATA_POINTS];
uint16_t total_time_x, total_time_y, total_time_z;
unsigned long start_time, end_time;

// ----------------------------------------------------
// TODO: initialize sensor device                     |
// ----------------------------------------------------
void initialize_sensor()
{
    write_i2c_byte( MPU6050_PWR_MGMT_1, 0x00 );
}

void setup(){
    Serial.begin( SERIAL_BAUDRATE ); // start serial connection to computer/python script

    // initialize Arduino/sensor interface
    switch (com)
    {
        // I2C interface to sensor - DEFAULT
        Wire.begin();
    }

    initialize_sensor();    // initialize sensor device
    delay(100);             // pause for initialization
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

// ----------------------------------------------------
// TODO: Retrieve sensor data over com interface,     |
//       confirm sign extension if 8/12-bit           |
// ----------------------------------------------------
void get_sensor_data( char axis )
{   
    int i = 0;
    // update sensor data 
    while ( i < NUM_DATA_POINTS )
    {
        get_i2c_sensor_data( axis, i );
        i++; 
    }
}

// Get I2C sensor Data
void get_i2c_sensor_data( char axis, int sample_num )
{
    if ( axis == 'x' )
    {
        // read_i2c_bytes( ACCEL_X_HIGH, (uint8_t*)&raw_data[sample_num], 2 );
        raw_data[sample_num] = read_i2c_byte( ACCEL_X_HIGH );
        raw_data[sample_num] = (raw_data[sample_num] << 8) | read_i2c_byte( ACCEL_X_LOW );
    }
    else if ( axis == 'y' )
    {
        // read_i2c_bytes( ACCEL_Y_HIGH, (uint8_t*)&raw_data[sample_num], 2 );
        raw_data[sample_num] = read_i2c_byte( ACCEL_Y_HIGH );
        raw_data[sample_num] = (raw_data[sample_num] << 8) | read_i2c_byte( ACCEL_Y_LOW );
    }
    else if ( axis == 'z' )
    {
        // read_i2c_bytes( ACCEL_Z_HIGH, (uint8_t*)&raw_data[sample_num], 2 );
        raw_data[sample_num] = read_i2c_byte( ACCEL_Z_HIGH );
        raw_data[sample_num] = (raw_data[sample_num] << 8) | read_i2c_byte( ACCEL_Z_LOW );  
    }

    // if ( axis == 'x' )
    // {
    //     // read_i2c_bytes( ACCEL_X_HIGH, (uint8_t*)&raw_data[sample_num], 2 );
    //     raw_data[sample_num] = read_i2c_byte( GYRO_X_HIGH );
    //     raw_data[sample_num] = (raw_data[sample_num] << 8) | read_i2c_byte( GYRO_X_LOW );
    // }
    // else if ( axis == 'y' )
    // {
    //     // read_i2c_bytes( ACCEL_Y_HIGH, (uint8_t*)&raw_data[sample_num], 2 );
    //     raw_data[sample_num] = read_i2c_byte( GYRO_Y_HIGH );
    //     raw_data[sample_num] = (raw_data[sample_num] << 8) | read_i2c_byte( GYRO_Y_LOW );
    // }
    // else if ( axis == 'z' )
    // {
    //     // read_i2c_bytes( ACCEL_Z_HIGH, (uint8_t*)&raw_data[sample_num], 2 );
    //     raw_data[sample_num] = read_i2c_byte( GYRO_Z_HIGH );
    //     raw_data[sample_num] = (raw_data[sample_num] << 8) | read_i2c_byte( GYRO_Z_LOW );  
    // }

    swap( raw_data[ sample_num ] );
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

// I2C - RX Byte Function
uint8_t read_i2c_byte( uint8_t reg_addr )
{
    Wire.beginTransmission( DEVICE_ADDR ); // transmit device address
    Wire.write( reg_addr );                // transmit register address to read
    Wire.endTransmission();                // end transmission
    Wire.requestFrom( DEVICE_ADDR, 1 );    // read byte response
    while( !Wire.available() );            // wait for response
    return( Wire.read() );                 // return result
}

// I2C - TX Byte Function
void write_i2c_byte( uint8_t reg_addr, uint8_t value )
{
    Wire.beginTransmission( DEVICE_ADDR );  // transmit device address
    Wire.write( reg_addr );                 // transmit register address to write
    Wire.write( value );                    // transmit value to write
    Wire.endTransmission();                 // end transmission
    return;
}

// I2C - RX Bytes Function
void read_i2c_bytes( uint8_t reg_addr, uint8_t *buffer, uint8_t num_bytes )
{
    Wire.beginTransmission( DEVICE_ADDR );      // start WRITE transmission
    Wire.write( reg_addr );                     // transmit register address to read
    Wire.endTransmission( false );                // end WRITE transmission and send RESTART
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
