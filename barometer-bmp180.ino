/*  
    library for accessing barometric pressure data 
    on the BMP180 sensor through the I2C/Wire interface
    @author mika.metsaniemi - gmail.com
    by Omen Solutions 2014
*/

#include "math.h"

#define BMP180_ADDR 0b1110111                // I2C address for BMP180: 0b1110111
#define NAN 999999
#define oss 0                               // oversampling, 0 = single TODO: add oss functionality

class BMP {
    private:
        int16_t AC1, AC2, AC3;              // calibration data
        uint16_t AC4, AC5, AC6;
        int16_t B1, B2, MB, MC, MD;
        long B3, B5, B6;
        unsigned long B4, B7;        

        long UT;                            // uncompensated temperature
        long UP;                            // uncompensated pressure;
        bool read(void);
        unsigned long _lastreadtime;
        bool calibrated;
        void readUTemperature(void);
        void readUPressure(void);
    
    public:
        BMP();
        void readCalibrationData(void);
        long getTemperature(void);
        long getPressure(void);
        int16_t readWord(char reg);
        byte readByte(char reg);
};

BMP::BMP() {
    calibrated = false;
}


int16_t BMP::readWord(char reg) {
    byte data[2];

    Wire.beginTransmission(BMP180_ADDR);    // signal start of transmission
    Wire.write(reg);                        // select registry to read           
    Wire.endTransmission(false);            // signal end of transmission with restart
    Wire.requestFrom(BMP180_ADDR, 2);
    
    data[0] = Wire.read();                  // MSB
    data[1] = Wire.read();                  // LSB

    return((data[0] << 8) | data[1]);       // (MSB << 8) | LSB
}

byte BMP::readByte(char reg) {
    Wire.beginTransmission(BMP180_ADDR);    // signal start of transmission
    Wire.write(reg);                        // select registry to read           
    Wire.endTransmission(false);            // signal end of transmission with restart
    Wire.requestFrom(BMP180_ADDR, 2);
    return(Wire.read()); 
}

void BMP::readCalibrationData(void) {
    AC1 = readWord(0xAA);              // read the calibration values
    AC2 = readWord(0xAC);              // from the on-chip EEPROM
    AC3 = readWord(0xAE);
    AC4 = (uint16_t) readWord(0xB0);
    AC5 = (uint16_t) readWord(0xB2);
    AC6 = readWord(0xB4);
    B1 = readWord(0xB6);
    B2 = readWord(0xB8);
    MB = readWord(0xBA);
    MC = readWord(0xBC);
    MD = readWord(0xBE);
        
    Serial.print("This one should be 0x55: ");
    Serial.println(readByte(0xD0), HEX);

    calibrated = true;
}

void BMP::readUTemperature() { 
    Wire.beginTransmission(BMP180_ADDR);    // signal start of transmission
    Wire.write(0xF4);                       // select registry address to write to (0xF4)           
    Wire.write(0x2E);                       // data to write into registry (0x2E = temperature)        
    Wire.endTransmission(true);             // signal end of transmission with stop
    
    delay(5);                               // wait for 5 ms for measurement

    UT = (long) readWord(0xF6);             // store the temperature value for further calculations

}

void BMP::readUPressure() {                 
    byte MSB, LSB, XLSB;
    
    Wire.beginTransmission(BMP180_ADDR);    // signal start of transmission
    Wire.write(0xF4);                       // select registry address to write to (0xF4)           
    Wire.write(0x34 | (oss << 6));          // data to write into registry (0x34 = pressure)
                                            // TODO: IMPLEMENT OSS
    Wire.endTransmission(true);             // signal end of transmission with stop
    
    delay(5);                               // wait for 5 ms for measurement

    
    MSB = readByte(0xF6);
    LSB = readByte(0xF7);
    XLSB = readByte(0xF8);
    UP = (long) (MSB << 16 | LSB << 8 | XLSB) >> (8-oss);
        
}

long BMP::getTemperature() {
    long X1, X2, T;
    
    readUTemperature();

    X1 = (UT - AC6) * AC5 / pow(2, 15);     // calculating true temperature
    X2 = MC * pow(2, 11) / (X1 + MD);       // equations from datasheet
    B5 = X1 + X2;
    
    T = (B5 + 8) / pow(2, 4);
    
    return T;
}

long BMP::getPressure() {
    long X1, X2, X3, p;
    
    readUPressure();
    
    B6 = B5 - 4000;                         // calculating true pressure as per datasheet
    X1 = (B2 * (B6 * B6 / pow(2, 12))) / pow(2, 11);
    X2 = AC2 * B6 / pow(2, 11);
    X3 = X1 + X2;
    B3 = (((AC1*4+X3) << oss) + 2) / 4;
    X1 = AC3 * B6 / pow(2, 13);
    X2 = (B1 * (B6 * B6 / pow(2, 12))) / pow(2, 16);
    X3 = ((X1 + X2) + 2) / pow(2, 2);
    B4 = AC4 * (unsigned long) (X3 + 32768) / pow(2, 15);
    B7 = ((unsigned long) UP - B3) * (50000 >> oss);
    
    if(B7 < 0x80000000) { p = (B7 * 2) / B4; }
        else { p = (B7 / B4) * 2; }
    X1 = (p / pow(2, 8)) * (p / pow(2, 8));
    X1 = (X1 * 3038) / pow(2, 16);
    X2 = (-7357 * p) / pow(2, 16);
    
    p = p + (X1 + X2 + 3791) / pow(2, 4);
    
    return p;
}


/////////////////////////////////////////////////////////////
// end of library part

BMP bmp;
double tempC = 0;
double pressure = 0;

void setup() {

    
    Spark.variable("tempC", &tempC, DOUBLE);            // register spark variables first to the cloud
    Spark.variable("pressure", &pressure, DOUBLE);


    Serial.begin(9600);     // setup serial for outputting data to user
    // On Windows it will be necessary to implement the following line:
    // Make sure your Serial Terminal app is closed before powering your Core
    // Now open your Serial Terminal, and hit any key to continue!
    //while(!Serial.available()) SPARK_WLAN_Loop();

    Serial.println("Barometric Pressure Library - BMP180 - Omen Solutions 2014");
    


    Wire.begin();           // initialize I2C and join the wire as master 
    bmp.readCalibrationData();  // TODO: move this to bmp.begin();
}

void loop() {

    Serial.print("Temperature (C): ");
    Serial.println((tempC = (double) bmp.getTemperature() / 10));    

    Serial.print("Pressure (hPa): ");
    Serial.println((pressure = (double) bmp.getPressure() / 100));    

    delay(2000);
    
    Serial.println();
}




