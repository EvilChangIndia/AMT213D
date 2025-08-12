/*
 * Improved AMT21 RS485 Arduino Code by Kiran Sreekumar
 * Enhanced version with proper checksum verification and error handling
 * Compatible with AMT21xD (14-bit) and AMT21xB (12-bit) encoders
 * 
 * - Proper checksum verification
 * - Optimized RS485 timing  
 * - Better error handling
 * - Configurable resolution
 * 
 * Functions-
 * 1. Position (Raw data / angles)  (sending 0x54 to AMT213D makes it return the postion data)
 * 2. Number of turns               (sending 0x55 returns turns)
 * 3. Reset                         (sending 0x56 followed by 0x5E sets zero position)
 * 4. Set zero postion              (sending 0x56 followed by 0x75 resets the encoder)
 * 
 * Note:
 * Arduino Mega Serial1 Pins:
 * Pin 18 = TX1 (transmit)
 * Pin 19 = RX1 (receive)
 */

#include <Arduino.h>

// Pin definitions for Arduino Mega
#define DE_PIN    3     // Driver Enable
#define RE_PIN    5     // Receiver Enable

// AMT21 Configuration  
#define AMT21_BAUDRATE    2000000     // 2 Mbps
#define AMT21_NODE_ADDR   0x54        // Default node address. Double check yours
#define AMT21_RESOLUTION  14          // 14-bit for AMT21xD 

// Control modes
#define RS485_TX_MODE  HIGH
#define RS485_RX_MODE  LOW

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    // Configure RS485 control pins
    pinMode(DE_PIN, OUTPUT);
    pinMode(RE_PIN, OUTPUT);
    RS485Receive();
    
    // Initialize Serial1
    Serial1.begin(AMT21_BAUDRATE);
    
    Serial.println("AMT21 Encoder Interface Ready");
    Serial.println("Commands: 'p'=position, 't'=turns, 'z'=set zero, 'r'=reset");
    Serial.println("==========================================");
}

void RS485Transmit() {
    digitalWrite(DE_PIN, HIGH);
    digitalWrite(RE_PIN, LOW);
    delayMicroseconds(1);
}

void RS485Receive() {
    digitalWrite(DE_PIN, LOW);
    digitalWrite(RE_PIN, HIGH);
}

bool verifyChecksum(uint16_t response) {
    // Extract position data (lower 14 bits)
    uint16_t position = response & 0x3FFF;
    
    // Calculate checksum according to datasheet
    uint8_t odd_xor = 0;
    uint8_t even_xor = 0;
    
    // Odd bit positions: 13,11,9,7,5,3,1
    odd_xor = ((position >> 13) & 1) ^ ((position >> 11) & 1) ^ 
              ((position >> 9) & 1) ^ ((position >> 7) & 1) ^ 
              ((position >> 5) & 1) ^ ((position >> 3) & 1) ^ 
              ((position >> 1) & 1);
    
    // Even bit positions: 12,10,8,6,4,2,0  
    even_xor = ((position >> 12) & 1) ^ ((position >> 10) & 1) ^ 
               ((position >> 8) & 1) ^ ((position >> 6) & 1) ^ 
               ((position >> 4) & 1) ^ ((position >> 2) & 1) ^ 
               (position & 1);
    
    // Calculate expected checksum (inverted)
    uint8_t expected_k1 = odd_xor ^ 1;   // K1 = !odd_xor
    uint8_t expected_k0 = even_xor ^ 1;  // K0 = !even_xor
    
    // Extract received checksum
    uint8_t received_k1 = (response >> 15) & 1;
    uint8_t received_k0 = (response >> 14) & 1;
    
    return (received_k1 == expected_k1) && (received_k0 == expected_k0);
}

float readPosition() {
    // Clear any pending data
    while(Serial1.available()) Serial1.read();
    
    // Send position request
    RS485Transmit();
    Serial1.write(AMT21_NODE_ADDR);  // 0x54
    Serial1.flush();
    
    // Switch to receive with minimal delay
    delayMicroseconds(10);
    RS485Receive();
    
    // Wait for 2-byte response with timeout
    unsigned long startTime = millis();
    while(Serial1.available() < 2) {
        if(millis() - startTime > 5) {
            return -1.0; // Timeout error
        }
    }
    
    // Read 2-byte response (low byte first)
    uint8_t lowByte = Serial1.read();
    uint8_t highByte = Serial1.read();
    uint16_t response = (highByte << 8) | lowByte;
    
    // Verify checksum
    if(!verifyChecksum(response)) {
        return -2.0; // Checksum error
    }
    
    // Extract position (remove checksum bits)
    uint16_t position = response & 0x3FFF;
    
    // Convert to degrees
    return position * (360.0 / 16383.0);
}

int16_t readTurnsCounter() {
    // Clear buffer
    while(Serial1.available()) Serial1.read();
    
    // Send turns request
    RS485Transmit();
    Serial1.write(AMT21_NODE_ADDR + 0x01);  // 0x55
    Serial1.flush();
    
    delayMicroseconds(10);
    RS485Receive();
    
    // Wait for response
    unsigned long startTime = millis();
    while(Serial1.available() < 2) {
        if(millis() - startTime > 5) return -32768;
    }
    
    // Read response
    uint8_t lowByte = Serial1.read();
    uint8_t highByte = Serial1.read();
    uint16_t response = (highByte << 8) | lowByte;
    
    // Verify checksum
    if(!verifyChecksum(response)) return -32768;
    
    // Extract 14-bit signed value
    int16_t turns = response & 0x3FFF;
    
    // Sign extend if negative (bit 13 set)
    if(turns & 0x2000) {
        turns |= 0xC000;
    }
    
    return turns;
}

bool setZeroPosition() {
    RS485Transmit();
    Serial1.write(AMT21_NODE_ADDR + 0x02);  // 0x56
    Serial1.write(0x5E);                    // Set zero command
    Serial1.flush();
    RS485Receive();
    delay(250);  // Wait for reset
    return true;
}

bool resetEncoder() {
    RS485Transmit();
    Serial1.write(AMT21_NODE_ADDR + 0x02);  // 0x56  
    Serial1.write(0x75);                    // Reset command
    Serial1.flush();
    RS485Receive();
    delay(250);  // Wait for reset
    return true;
}

void loop() {
    static unsigned long lastRead = 0;
    static float lastAngle = -999;
    
    // Handle serial commands
    if(Serial.available()) {
        char cmd = tolower(Serial.read());
        
        switch(cmd) {
            case 'p': {
                float angle = readPosition();
                if(angle >= 0) {
                    Serial.print("Position: ");
                    Serial.print(angle, 2);
                    Serial.println("°");
                } else if(angle == -1) {
                    Serial.println("Timeout error");
                } else {
                    Serial.println("Checksum error");
                }
                break;
            }
            
            case 't': {
                int16_t turns = readTurnsCounter();
                if(turns != -32768) {
                    Serial.print("Turns: ");
                    Serial.println(turns);
                } else {
                    Serial.println("Error reading turns");
                }
                break;
            }
            
            case 'z':
                Serial.println("Setting zero position...");
                setZeroPosition();
                Serial.println("Done");
                break;
                
            case 'r':
                Serial.println("Resetting encoder...");
                resetEncoder(); 
                Serial.println("Done");
                break;
        }
    }
    
    // Continuous monitoring every 50ms
    if(millis() - lastRead > 50) {
        lastRead = millis();
        float angle = readPosition();
        
        if(angle >= 0 && abs(angle - lastAngle) > 0.1) {
            Serial.print("Angle: ");
            Serial.print(angle, 2);
            Serial.println("°");
            lastAngle = angle;
        }
    }
}
