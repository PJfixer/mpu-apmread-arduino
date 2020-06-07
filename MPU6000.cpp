
#include "MPU6000.h"

mpu6000_spi::mpu6000_spi()
{
  
SPI.begin();
SPI.setClockDivider(SPI_CLOCK_DIV2);
SPI.setDataMode(SPI_MODE3);
SPI.setBitOrder(MSBFIRST) ;
pinMode(CS_PIN,OUTPUT);

}


/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
low pass filter value; suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_188HZ
BITS_DLPF_CFG_98HZ
BITS_DLPF_CFG_42HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ 
BITS_DLPF_CFG_5HZ 
BITS_DLPF_CFG_2100HZ_NOLPF
returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/
bool mpu6000_spi::init(int sample_rate_div,int low_pass_filter){
   unsigned int response;
    select();
    response= SPI.transfer(MPUREG_USER_CTRL);
    response=SPI.transfer(BIT_I2C_IF_DIS);
    deselect();
    //RESET CHIP
    select();
    response=SPI.transfer(MPUREG_PWR_MGMT_1);
    response=SPI.transfer(BIT_H_RESET); 
    deselect();
    delay(170);
    //WAKE UP AND SET GYROZ CLOCK
    select();
    response=SPI.transfer(MPUREG_PWR_MGMT_1);
    response=SPI.transfer(MPU_CLK_SEL_PLLGYROZ); 
    deselect();
    //DISABLE I2C
    select();
    response=SPI.transfer(MPUREG_USER_CTRL);
    response=SPI.transfer(BIT_I2C_IF_DIS);
    deselect();
    //WHO AM I?
    select();
    response=SPI.transfer(MPUREG_WHOAMI|READ_FLAG);
    response=SPI.transfer(0x00);
    deselect();
    if(response<100){return 0;}//COULDN'T RECEIVE WHOAMI
    //SET SAMPLE RATE
    select();
    response=SPI.transfer(MPUREG_SMPLRT_DIV);
    response=SPI.transfer(sample_rate_div); 
    deselect();
    // FS & DLPF
    select();
    response=SPI.transfer(MPUREG_CONFIG);
    response=SPI.transfer(low_pass_filter);
    deselect();
    //DISABLE INTERRUPTS
    select();
    response=SPI.transfer(MPUREG_INT_ENABLE);
    response=SPI.transfer(0x00);
    deselect();

    return 0;
}

/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right range for the
accelerometers. Suitable ranges are:
BITS_FS_2G
BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu6000_spi::set_acc_scale(int scale){
    unsigned int temp_scale;
    select();
    SPI.transfer(MPUREG_ACCEL_CONFIG);
    SPI.transfer(scale);  
    deselect();    
    switch (scale){
        case BITS_FS_2G:
            acc_divider=16384;
        break;
        case BITS_FS_4G:
            acc_divider=8192;
        break;
        case BITS_FS_8G:
            acc_divider=4096;
        break;
        case BITS_FS_16G:
            acc_divider=2048;
        break;   
    }
    delay(11);
    select();
    temp_scale=SPI.transfer(MPUREG_ACCEL_CONFIG|READ_FLAG);
    temp_scale=SPI.transfer(0x00);  
    deselect();
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;   
    }
    return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right range for the
gyroscopes. Suitable ranges are:
BITS_FS_250DPS
BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
returns the range set (250,500,1000 or 2000)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu6000_spi::set_gyro_scale(int scale){
    unsigned int temp_scale;
    select();
    SPI.transfer(MPUREG_GYRO_CONFIG);
    SPI.transfer(scale);  
    deselect();    
    switch (scale){
        case BITS_FS_250DPS:
            gyro_divider=131;
        break;
        case BITS_FS_500DPS:
            gyro_divider=65.5;
        break;
        case BITS_FS_1000DPS:
            gyro_divider=32.8;
        break;
        case BITS_FS_2000DPS:
            gyro_divider=16.4;
        break;   
    }
   delay(11);;
    select();
    temp_scale=SPI.transfer(MPUREG_GYRO_CONFIG|READ_FLAG);
    temp_scale=SPI.transfer(0x00);  
    deselect();
    switch (temp_scale){
        case BITS_FS_250DPS:
            temp_scale=250;
        break;
        case BITS_FS_500DPS:
            temp_scale=500;
        break;
        case BITS_FS_1000DPS:
            temp_scale=1000;
        break;
        case BITS_FS_2000DPS:
            temp_scale=2000;
        break;   
    }
    return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                WHO AM I?
usage: call this function to know if SPI is working correctly. It checks the I2C address of the
mpu6000 which should be 104 when in SPI mode.
returns the I2C address (104)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu6000_spi::whoami(){
    unsigned int response = 0;
   
    select();
    response=SPI.transfer(MPUREG_WHOAMI|READ_FLAG);
    response=SPI.transfer(0x00);
    deselect();
 
    return response;
}


/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns the value in Gs
-----------------------------------------------------------------------------------------------*/
float mpu6000_spi::read_acc(int axis){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    switch (axis){
        case 0:
        responseH=SPI.transfer(MPUREG_ACCEL_XOUT_H | READ_FLAG);
        break;
        case 1:
        responseH=SPI.transfer(MPUREG_ACCEL_YOUT_H | READ_FLAG);
        break;
        case 2:
        responseH=SPI.transfer(MPUREG_ACCEL_ZOUT_H | READ_FLAG);
        break;
    }
    responseH=SPI.transfer(0x00);
    responseL=SPI.transfer(0x00);
    bit_data=((int16_t)responseH<<8)|responseL;
    data=(float)bit_data;
    data=data/acc_divider;
    deselect();
    return data;
}

/*-----------------------------------------------------------------------------------------------
                                READ GYROSCOPE
usage: call this function to read gyroscope data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns the value in Degrees per second
-----------------------------------------------------------------------------------------------*/
float mpu6000_spi::read_rot(int axis){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    switch (axis){
        case 0:
        responseH=SPI.transfer(MPUREG_GYRO_XOUT_H | READ_FLAG);
        break;
        case 1:
        responseH=SPI.transfer(MPUREG_GYRO_YOUT_H | READ_FLAG);
        break;
        case 2:
        responseH=SPI.transfer(MPUREG_GYRO_ZOUT_H | READ_FLAG);
        break;
    }
    responseH=SPI.transfer(0x00);
    responseL=SPI.transfer(0x00);
    bit_data=((int16_t)responseH<<8)|responseL;
    data=(float)bit_data;
    data=data/gyro_divider;
    deselect();
    return data;
}

/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE
usage: call this function to read temperature data. 
returns the value in °C
-----------------------------------------------------------------------------------------------*/
float mpu6000_spi::read_temp(){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    responseH=SPI.transfer(MPUREG_TEMP_OUT_H | READ_FLAG);
    responseH=SPI.transfer(0x00);
    responseL=SPI.transfer(0x00);
    bit_data=((int16_t)responseH<<8)|responseL;
    data=(float)bit_data;
    data=(data/340)+36.53;
    deselect();
    return data;
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/
int mpu6000_spi::calib_acc(int axis){
    uint8_t responseH,responseL,calib_data;
    int temp_scale;
    //READ CURRENT ACC SCALE
    select();
    responseH=SPI.transfer(MPUREG_ACCEL_CONFIG|READ_FLAG);
    temp_scale=SPI.transfer(0x00);  
    deselect();
    delay(11);
    set_acc_scale(BITS_FS_8G);
     delay(11);
    //ENABLE SELF TEST
    select();
    responseH=SPI.transfer(MPUREG_ACCEL_CONFIG);
    temp_scale=SPI.transfer(0x80>>axis);  
    deselect();
    delay(11);
    select();
    responseH=SPI.transfer(MPUREG_SELF_TEST_X|READ_FLAG);
    switch(axis){
        case 0:
            responseH=SPI.transfer(0x00);
            responseL=SPI.transfer(0x00);
            responseL=SPI.transfer(0x00);
            responseL=SPI.transfer(0x00);
            calib_data=((responseH&11100000)>>3)|((responseL&00110000)>>4);
        break;
        case 1:
            responseH=SPI.transfer(0x00);
            responseH=SPI.transfer(0x00);
            responseL=SPI.transfer(0x00);
            responseL=SPI.transfer(0x00);
            calib_data=((responseH&11100000)>>3)|((responseL&00001100)>>2);
        break;
        case 2:
            responseH=SPI.transfer(0x00);
            responseH=SPI.transfer(0x00);
            responseH=SPI.transfer(0x00);
            responseL=SPI.transfer(0x00);
            calib_data=((responseH&11100000)>>3)|((responseL&00000011));
        break;
    }
    deselect();
    delay(11);
    set_acc_scale(temp_scale);
    return calib_data;
} 

/*-----------------------------------------------------------------------------------------------
                                SPI SELECT AND DESELECT
usage: enable and disable mpu6000 communication bus
-----------------------------------------------------------------------------------------------*/
void mpu6000_spi::select() {
    //Set CS low to start transmission (interrupts conversion)
    digitalWrite(CS_PIN, LOW);
}
void mpu6000_spi::deselect() {
    //Set CS high to stop transmission (restarts conversion)
     digitalWrite(CS_PIN, HIGH);
}
