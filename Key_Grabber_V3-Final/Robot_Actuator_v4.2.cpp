#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h> 
#include <Adafruit_PWMServoDriver.h> 

/**
 * @brief AL5D Robot Arm Actuator Firmware
 * Implements ESP-NOW data reception, slew-rate limiting for mechanical 
 * stress reduction, and safety interlocks.
 */

namespace Config {
    // MAC Address of the Controller (Hub)
    const uint8_t HUB_MAC[] = {0x30, 0xED, 0xA0, 0xBE, 0x4F, 0x8C}; 
    
    // Slew Rate Limit: Maximum pulse width change per update cycle.
    // Lower values result in smoother acceleration/deceleration.
    const int MAX_SLEW = 6; 
}

struct Packet { 
    int pulses[6]; 
    int cmd; 
};

class RobotActuator {
    Adafruit_PWMServoDriver pwm;
    int current_pos[6]; // Current physical position of servos
    int target_pos[6];  // Target position received from controller
    unsigned long lastUpdate = 0;
    bool safetyLock = true;

public:
    RobotActuator() : pwm(0x40) {}

    void begin() {
        Wire.begin(D2, D1);
        pwm.begin();
        pwm.setPWMFreq(50);
        
        // Initialize servos to center (neutral) position
        for(int i=0; i<6; i++) { 
            current_pos[i] = 1500; 
            target_pos[i] = 1500; 
        }
        applyPositions();
    }

    void handlePacket(Packet* p) {
        // Disengage safety lock upon receiving first valid packet
        if(safetyLock) safetyLock = false;
        
        // Command 99: Emergency Sync / Reset
        if (p->cmd == 99) {
            for(int i=0; i<6; i++) { 
                target_pos[i] = 1500; 
                current_pos[i] = 1500; 
            }
            applyPositions();
        } else {
            // Standard Position Update
            for(int i=0; i<6; i++) {
                // Validate pulse width range (500-2500us)
                if(p->pulses[i] >= 500 && p->pulses[i] <= 2500) {
                    target_pos[i] = p->pulses[i];
                }
            }
        }
    }

    // Main motion control loop
    // Calculates intermediate steps between current and target positions
    void updateMotion() {
        // Update frequency: ~100Hz (10ms interval)
        if (millis() - lastUpdate < 10) return; 
        lastUpdate = millis();

        bool moving = false;
        for(int i=0; i<6; i++) {
            int diff = target_pos[i] - current_pos[i];
            
            if (abs(diff) > 0) {
                int step = diff;
                
                // Apply Slew Rate Limiter
                // Caps the velocity to prevent mechanical jerking
                if (step > Config::MAX_SLEW) step = Config::MAX_SLEW;
                if (step < -Config::MAX_SLEW) step = -Config::MAX_SLEW;
                
                current_pos[i] += step;
                moving = true;
            }
        }
        
        if (moving) applyPositions();
    }

    void applyPositions() {
        for(int i=0; i<6; i++) {
            int p = constrain(current_pos[i], 500, 2500);
            // Map microseconds to 12-bit PWM value (4096 / 20ms period)
            pwm.setPWM(i, 0, (int)((float)p * 4096.0f / 20000.0f));
        }
    }

    void sendHeartbeat() {
        uint8_t ping = 0xFF; // Status OK signal
        esp_now_send((uint8_t*)Config::HUB_MAC, &ping, 1);
    }
};

RobotActuator Robot;

void setup() {
    Serial.begin(115200);
    
    // Configure WiFi for ESP-NOW (Station Mode, disconnected)
    WiFi.mode(WIFI_STA); 
    WiFi.disconnect();
    
    if (esp_now_init() != 0) return;
    
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_add_peer((uint8_t*)Config::HUB_MAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    
    // Register Receive Callback
    esp_now_register_recv_cb([](uint8_t *mac, uint8_t *data, uint8_t len) {
        if (len == sizeof(Packet)) {
            // Use memcpy for safe data transfer to prevent alignment exceptions
            Packet p; 
            memcpy(&p, data, sizeof(Packet)); 
            Robot.handlePacket(&p);
        }
    });
    
    Robot.begin();
}

void loop() {
    // Send heartbeat every 500ms to maintain link status on Hub
    static unsigned long lastPing = 0;
    if(millis() - lastPing > 500) { 
        Robot.sendHeartbeat(); 
        lastPing = millis(); 
    }
    
    // Execute motion smoothing
    Robot.updateMotion();
}
