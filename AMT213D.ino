/*
 * Improved AMT213D Arduino Code by Kiran Sreekumar
 * Enhanced version with proper checksum verification and error handling
 * Compatible with AMT21xD (14-bit) and AMT21xB (12-bit) encoders connected to and Arduino Mega 2560 through serial1 and RS485 
 * 
 * Improvements
 * - Proper checksum verification
 * - Optimized RS485 timing  
 * - Better error handling
 * - Configurable resolution
 * - Cleaner code structure
 * 
 * Functions-
 * 1. Position (Raw data / angles)  (sending 0x54 to AMT213D makes it return the postion data)
 * 2. Number of turns               (sending 0x55 returns turns)
 * 3. Reset                         (sending 0x56 followed by 0x5E sets zero position)
 * 4. Set zero postion              (sending 0x56 followed by 0x75 resets the encoder)
 * 
 * Note:
 * We are using a mega to use the dual serial ports to easily interface it with the computer and the encoder at the same time. 
 * This can be run on an Uno as well using software serial
 * Arduino Mega Serial1 Pins:
 * Pin 18 = TX1 (transmit)
 * Pin 19 = RX1 (receive)

 */

#include <Arduino.h>

class AMT21Encoder {
private:
    uint8_t nodeAddress;
    uint8_t resolution;
    uint8_t dePin;
    uint8_t rePin;
    
    static const uint16_t TURNAROUND_DELAY_US = 10;
    static const uint16_t RESPONSE_TIMEOUT_MS = 5;
    static const uint16_t RESET_DELAY_MS = 250;
    static const uint16_t ERROR_TIMEOUT = 0xFFFF;
    static const int16_t ERROR_TURNS = -32768;
    
    uint32_t lastValidPosition;
    int16_t lastValidTurns;
    bool validDataFlag;
    unsigned long lastReadTime;
    
    void setMode(bool transmit) {
        digitalWrite(dePin, transmit ? HIGH : LOW);
        digitalWrite(rePin,  transmit ? LOW  : HIGH);
        if (transmit) delayMicroseconds(1);
    }
    
    bool verifyChecksum(uint16_t response) {
        uint16_t pos = response & 0x3FFF;
        uint8_t odd = ((pos>>13)&1)^((pos>>11)&1)^((pos>>9)&1)^((pos>>7)&1)^((pos>>5)&1)^((pos>>3)&1)^((pos>>1)&1);
        uint8_t even= ((pos>>12)&1)^((pos>>10)&1)^((pos>>8)&1)^((pos>>6)&1)^((pos>>4)&1)^((pos>>2)&1)^(pos&1);
        uint8_t k1 = odd^1, k0 = even^1;
        return (((response>>15)&1)==k1) && (((response>>14)&1)==k0);
    }
    
    uint16_t sendCommand(uint8_t cmd) {     //This is for position and turns requests
        while(Serial1.available()) Serial1.read();
        setMode(true);
        Serial1.write(cmd); Serial1.flush();
        delayMicroseconds(TURNAROUND_DELAY_US);
        setMode(false);
        
        unsigned long start = millis();
        while(Serial1.available() < 2) {
            if (millis()-start > RESPONSE_TIMEOUT_MS) return ERROR_TIMEOUT;
        }
        // Read 2-byte response (low byte first)
        uint8_t lo = Serial1.read(), hi = Serial1.read(); 
        return (hi<<8)|lo;
    }
    
public:
    //constructor to allow for different configurations
    AMT21Encoder(uint8_t addr=0x54, uint8_t res=14, uint8_t de=3, uint8_t re=5) {
        nodeAddress = addr;
        resolution  = res;
        dePin       = de;
        rePin       = re;
        lastValidPosition = 0;
        lastValidTurns    = 0;
        validDataFlag     = false;
        lastReadTime      = 0;
    }
    
    bool begin() {
        pinMode(dePin, OUTPUT);
        pinMode(rePin, OUTPUT);
        setMode(false);
        Serial1.begin(2000000);
        delay(100);
        while(Serial1.available()) Serial1.read();
        return true;
    }
    
    float readPosition() {
        uint16_t resp = sendCommand(nodeAddress);
        // Check for flagged errors
        if (resp==ERROR_TIMEOUT) { validDataFlag=false; return -1.0; }
        if (!verifyChecksum(resp)) { validDataFlag=false; return -2.0; }
        // Masking off the checksum bits to get the corrected position
        uint16_t pos = resp & 0x3FFF; 
        //shift two bits for 12 bit response
        if (resolution==12) pos >>=2; 
        validDataFlag=true;
        lastValidPosition=pos;
        lastReadTime=millis();
        return pos;
    }
    
    float readAngleDegrees() {
        float pos = readPosition();
        if (pos<0) return pos;
        float maxC = (resolution==12?4095.0:16383.0); //for the 12 bit and 14 bit variants
        return pos*360.0/maxC; //basic scaling
    }
    
    int16_t readTurnsCounter() {
        uint16_t resp = sendCommand(nodeAddress+1);
        if (resp==ERROR_TIMEOUT) return ERROR_TURNS;
        if (!verifyChecksum(resp)) return ERROR_TURNS;
        int16_t t = resp & 0x3FFF;
        if (t & 0x2000) t |= 0xC000;
        lastValidTurns=t;
        return t;
    }
    
    bool setZeroPosition() {
        setMode(true);
        Serial1.write(nodeAddress+2); //from datasheet, this commands 
        Serial1.write(0x5E);
        Serial1.flush();
        setMode(false);
        delay(RESET_DELAY_MS);
        return true;
    }
    
    bool resetEncoder() {
        setMode(true);
        Serial1.write(nodeAddress+2);
        Serial1.write(0x75);
        Serial1.flush();
        setMode(false);
        delay(RESET_DELAY_MS);
        return true;
    }
    
    bool isDataValid() const { return validDataFlag; }
    uint32_t getLastValidPosition() const { return lastValidPosition; }
    int16_t getLastValidTurns() const { return lastValidTurns; }
};







// Example usage to read angle
AMT21Encoder encoder;

void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("AMT21 Encoder on Mega (Serial1) Demo");
    encoder.begin();
}

void loop() {
    float angle = encoder.readAngleDegrees();
    if (angle >= 0) {
        Serial.print("Angle: ");
        Serial.print(angle,2);
        Serial.println("°");
    }
    delay(100);
}
