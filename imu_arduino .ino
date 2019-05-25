//         File: imu_arduino.ino
//         Name: Peter Griger
//        Email: p.griger@gmail.com
//         Date: 2015
//    Libraries: Wire.h
// Dependencies: Arduino UNO board connected to USB interface
//        Desc.: Program prints to serial:
//					- acceleration in three axis
//			 	 	- turn rate in three axis
//				   	- magnetic induction in three axis
//					- altitude
//					- temperature
//					- pressure


#include <Wire.h>


// I2C ADDRESS OF COMPONENTS
int acc = 0x53;      // ADXL345
int gyro = 105;      // L3G4200D
int compas = 0x1E;   // HMC5883L
int bar = 0x77;      // BMP085


// INIT CALIBRATION VALUES
int gyroX0; //gyroX zero-rate lvl
int gyroY0; //gyroY zero-rate lvl
int gyroZ0; //gyroZ zero-rate lvl

const float p0 = 101325; // avg atm pressure at current alt
int oss = 2; // high res oversampling
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;
int b5; // calculated in getTemp(int)


// OUPUT DATA
int accdata[3];
int gyrodata[3];
int compasdata[3];
double temp;
long pressure;
float alt;
double output[12];
// output merges data into one array to be send to serial
// output = [     accX,     accY,   accZ,
//			     gyroX,    gyroY,  gyroZ,
//			      magX,     magY,   magZ,
//			  altitude, pressure,   temp ]


// INIT OF COMUNICATION *************************************************************
void setup(){

  Wire.begin();
  Serial.begin(9600);

  setAcc(4);      // config acc range to +-4g; can be 4|8|16
  
  setGyro(500);   // config gyro range to +-500deg/sec; can be 250|500|2000
  calGyro();
  
  setCompas(13);  // config compas to +-1.3Gs; can be 1.3|4.0|8.1 Gs
  
  barCal();
  
  char s;    // wait till char 's' is send to serial
  do{
    s = Serial.read();
  }while(s!='s');
}


// MAIN LOOP *********************************************************
void loop()
{
  getAcc(&accdata[0], &accdata[1], &accdata[2]);
  getGyro(&gyrodata[0], &gyrodata[1], &gyrodata[2]);
  getCompas(&compasdata[0], &compasdata[1], &compasdata[2]);
  temp = getTemp(getUT());
  pressure = getPressure(getUP());
  alt = (((pow(p0/(float)pressure,0.190223)-1))*44330);
  
  output[0] = -0.00782*accdata[0];
  output[1] = -0.00782*accdata[1];
  output[2] = -0.00782*accdata[2];
  output[3] = 0.00763*(gyrodata[0]-gyroX0);
  output[4] = 0.00763*(gyrodata[1]-gyroY0);
  output[5] = 0.00763*(gyrodata[2]-gyroZ0);
  output[6] = 0.000635*compasdata[0];
  output[7] = 0.000635*compasdata[1];
  output[8] = 0.000635*compasdata[2];
  output[9] = alt;
  output[10] = pressure;
  output[11] = temp/10;
  
  for(int i=0;i<=11;i++){
    Serial.print(output[i]);
    Serial.print(",");
  } 
  Serial.println();
}


// GET 3D ACCELEROMETER VALUES
void getAcc(int *x, int *y, int *z){

  byte xMSB = readRegister(acc, 0x33);
  byte xLSB = readRegister(acc, 0x32);
  *x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(acc, 0x35);
  byte yLSB = readRegister(acc, 0x34);
  *y = ((yMSB << 8) | yLSB);
  
  byte zMSB = readRegister(acc, 0x37);
  byte zLSB = readRegister(acc, 0x36);
  *z = ((zMSB << 8) | zLSB);
}


// GET 3D GYROSCOPE VALUES
void getGyro(int *x, int *y, int *z){

  byte xMSB = readRegister(gyro, 0x29);
  byte xLSB = readRegister(gyro, 0x28);
  *x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(gyro, 0x2B);
  byte yLSB = readRegister(gyro, 0x2A);
  *y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(gyro, 0x2D);
  byte zLSB = readRegister(gyro, 0x2C);
  *z = ((zMSB << 8) | zLSB);
}


// BAROMETER FUNCTIONS
// get uncompensated temp
unsigned int getUT(){
  writeRegister(bar,0xf4,0x2e);
  delay(5);
  return read16Bar(0xF6, 0xF7);
}

// get uncompensated pressure
unsigned long getUP(){
  unsigned int msb,lsb,xlsb;
  
  writeRegister(bar,0xF4,(0x34+(oss<<6)));
  delay(2+(3<<oss));
  
  msb  = readRegister(bar,0xF6);
  lsb  = readRegister(bar,0xF7);
  xlsb = readRegister(bar,0xF8);
  
  return (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-oss);
}

// calculate temerature
short getTemp(int ut){
  long x1,x2;
  x1 = ((long)ut - (long)ac6)*(long)ac5 >>15 ;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;
  return ((b5 + 8)>>4);
}

// calculate pressure
long getPressure(unsigned long up){
  long x1,x2,x3,b3,b6,p;
  unsigned long b4,b7;
  
  b6=b5-4000;
  
  x1 = (b2 * (b6 * b6)>>12)>>11; //calc b3
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<oss) + 2)>>2;
  
  x1 = (ac3 * b6)>>13;           //calc b4
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>oss));
  
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
    
  return p;
}


// GET 3D GYROSCOPE VALUES
void getCompas(int *x, int *y, int *z){

  byte xMSB = readRegister(compas, 0x03);
  byte xLSB = readRegister(compas, 0x04);
  *x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(compas, 0x07);
  byte yLSB = readRegister(compas, 0x08);
  *y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(compas, 0x05);
  byte zLSB = readRegister(compas, 0x06);
  *z = ((zMSB << 8) | zLSB);
}


// COMPONENT SETUP FUNC
// accelerometer setup
int setAcc(int scale){

  writeRegister(acc, 0x2D, 0x08);        // power on

  writeRegister(acc, 0x2C, 0b00001010);  // 100Hz output rate

  if(scale == 2){   // range setup
    writeRegister(acc, 0x31, 0b00000000);
  }else if(scale == 4){
    writeRegister(acc, 0x31, 0b00000001);
  }else{
    writeRegister(acc, 0x31, 0x01);
  }
}

// compas setup
int setCompas(int scale){

  writeRegister(compas, 0x00, 0b01010000); // output rate 15Hz, 4avrg samples

  writeRegister(compas, 0x02, 0b00000000); // continuous meas mode

  if(scale == 13){   //range setup
    writeRegister(compas, 0x01, 0b00100000);
  }else if(scale == 40){
    writeRegister(compas, 0x01, 0b10000000);
  }else{
    writeRegister(compas, 0x01, 0b11100000);
  }
}

// gyroscope setup
int setGyro(int scale){

  writeRegister(gyro, 0x20, 0b10101111); // power on; xyz enable; 400Hz; 20cutoff

  writeRegister(gyro, 0x21, 0b00000000); // HP Filter 30hz; 

  writeRegister(gyro, 0x22, 0b00000000); // interrupts & drdy off

  if(scale == 250){   //range setup
    writeRegister(gyro, 0x23, 0b00000000);
  }else if(scale == 500){
    writeRegister(gyro, 0x23, 0b00010000);
  }else{
    writeRegister(gyro, 0x23, 0b00100000);
  }

   writeRegister(gyro, 0x24, 0b00000000); // interrupt from hpf disable
}


// CALIBRATION FUNC
// gyroscope calibration
void calGyro(){ 
  for(int n=0;n<500;n++){
  getGyro(&gyrodata[0], &gyrodata[1], &gyrodata[2]);
   gyroX0+=gyrodata[0];
   gyroY0+=gyrodata[1];
   gyroZ0+=gyrodata[2]; 
  }
  gyroX0/=500; //gyroX zero-rate lvl
  gyroY0/=500; //gyroY zero-rate lvl
  gyroZ0/=500; //gyroZ zero-rate lvl
}

// barometer calibration
void barCal()
{
  ac1 = read16Bar(0xAA,0xAB);
  ac2 = read16Bar(0xAC,0xAD);
  ac3 = read16Bar(0xAE,0xAF);
  ac4 = read16Bar(0xB0,0xB1);
  ac5 = read16Bar(0xB2,0xB3);
  ac6 = read16Bar(0xB4,0xB5);
  b1 =  read16Bar(0xB6,0xB7);
  b2 =  read16Bar(0xB8,0xB9);
  mb =  read16Bar(0xBA,0xBB);
  mc =  read16Bar(0xBC,0xBD);
  md =  read16Bar(0xBE,0xBF);
}

// COMMUNICATION FUNC
void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);      // send register address
    Wire.write(val);          // send value to write
    Wire.endTransmission();   // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);   // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1);  // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}

int read16Bar(int msb, int lsb){ //read 16bits from msb & lsb

  byte xMSB = readRegister(bar, msb);
  byte xLSB = readRegister(bar, lsb);
  return ((xMSB << 8) | xLSB);
}
