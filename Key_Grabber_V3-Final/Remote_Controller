#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <vector>

/**
 * @brief AL5D Robot Arm Wireless Controller
 * Implements inverse kinematics for wrist leveling, center-of-gravity 
 * compensation, and safety interlocks for system stability.
 */

namespace Config {
    const uint8_t RobotMac[] = {0xEC, 0xFA, 0xBC, 0xD6, 0x7E, 0x0D}; 
    const int J_PINS[4]  = {1, 2, 6, 7}; // J1X, J1Y, J2X, J2Y
    const int BTN_SEL_REC = 11;
    const int BTN_PLAY    = 12; 
    const int BTN_DIAG    = 10; 
    const int SW_LEFT_CLICK  = 3; 
    const int SW_RIGHT_CLICK = 8; 
}

// Data packet structure for ESP-NOW communication
struct __attribute__((packed)) Packet { 
    int pulses[6]; 
    int cmd; 
};

struct Waypoint { 
    int pos[6]; 
};

enum class SystemState { MENU, SAFETY_CHECK, DRIVE, DIAG_OVERLAY };

class RobotController {
    LiquidCrystal_I2C lcd;
    SystemState currentState = SystemState::MENU;
    SystemState lastState = SystemState::MENU;
    float filtered_adc[4];
    int tare_offsets[4]; 
    
    // Joint Indices: 0:Base, 1:Shoulder, 2:Elbow, 3:Wrist, 4:HeadRot, 5:Gripper
    int current_pos[6] = {1500, 1500, 1500, 1500, 1500, 1500};
    
    std::vector<Waypoint> sequence;
    bool isReplaying = false;
    unsigned long lastReplayStep = 0;
    int replayIndex = 0;
    int menuIdx = 0;
    bool robotLinked = false;
    unsigned long lastPacketTime = 0;
    Packet pkt;

public:
    RobotController() : lcd(0x27, 16, 2) {}
    static RobotController* instance;

    void begin() {
        instance = this;
        Wire.begin(4, 5);
        lcd.init(); lcd.backlight();
        
        pinMode(Config::BTN_SEL_REC, INPUT_PULLUP);
        pinMode(Config::BTN_PLAY, INPUT_PULLUP);
        pinMode(Config::BTN_DIAG, INPUT_PULLUP);
        pinMode(Config::SW_LEFT_CLICK, INPUT_PULLUP);
        pinMode(Config::SW_RIGHT_CLICK, INPUT_PULLUP);

        WiFi.mode(WIFI_STA);
        esp_now_init();
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, Config::RobotMac, 6);
        peer.channel = 0; peer.encrypt = false;
        esp_now_add_peer(&peer);
        esp_now_register_recv_cb(OnDataRecv);

        lcd.print("SYSTEM READY"); 
        delay(1000); 
        showMenu();
    }

    static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
        // Robot sends 0xFF as a heartbeat signal
        if (len > 0 && incomingData[0] == 0xFF) {
            instance->robotLinked = true;
            instance->lastPacketTime = millis();
        }
    }

    // Calculates wrist angle required to maintain parallel orientation relative to ground
    int calculateWristParallel(int pulseShoulder, int pulseElbow) {
        double theta2 = (pulseShoulder - 1500) / 11.1; 
        double theta3 = (pulseElbow - 1500) / 11.1;    
        double theta4 = -(theta2 + theta3); // Inverted to counteract arm dip
        int pulseWristRotate = 1500 + (theta4 * 11.1);
        return constrain(pulseWristRotate, 500, 2500);
    }

    void run() {
        readJoysticks();
        
        // Global Diagnostics Toggle
        if (digitalRead(Config::BTN_DIAG) == LOW) {
            delay(250); // Debounce
            if (currentState != SystemState::DIAG_OVERLAY) {
                lastState = currentState; 
                currentState = SystemState::DIAG_OVERLAY; 
                lcd.clear();
            } else {
                currentState = lastState; 
                lcd.clear(); 
                if(currentState == SystemState::MENU) showMenu();
            }
        }

        switch(currentState) {
            case SystemState::MENU: updateMenu(); break;
            case SystemState::SAFETY_CHECK: updateSafetyCheck(); break;
            case SystemState::DRIVE: updateDrive(); break;
            case SystemState::DIAG_OVERLAY: updateDiag(); break;
        }
        
        // Connection timeout check (2 seconds)
        if (millis() - lastPacketTime > 2000) robotLinked = false;
    }

    void updateMenu() {
        lcd.setCursor(0,1); 
        lcd.printf("LNK:%s Mem:%d   ", robotLinked ? "OK" : "NO", sequence.size());
        
        if(digitalRead(Config::SW_RIGHT_CLICK) == LOW) { menuIdx = 1; showMenu(); delay(150); }
        if(digitalRead(Config::SW_LEFT_CLICK) == LOW) { menuIdx = 0; showMenu(); delay(150); }
        
        if(digitalRead(Config::BTN_SEL_REC) == LOW) {
            if(menuIdx == 0) startCalibration();
            if(menuIdx == 1) { 
                sequence.clear(); 
                lcd.clear(); lcd.print("CLEARED"); 
                delay(1000); showMenu(); 
            }
        }
    }

    void showMenu() {
        lcd.setCursor(0,0);
        lcd.print(menuIdx==0 ? "> 1. ENGAGE DRIVE" : "> 2. CLEAR MEMORY");
    }

    void startCalibration() {
        lcd.clear(); lcd.print("HANDS OFF...");
        delay(500);
        
        // Reset local positions
        for(int i=0; i<6; i++) current_pos[i] = 1500;
        
        // Average 50 samples for zero-reference
        float sums[4] = {0};
        for(int s=0; s<50; s++) {
            readJoysticks();
            for(int k=0; k<4; k++) sums[k] += filtered_adc[k];
            delay(5);
        }
        for(int i=0; i<4; i++) tare_offsets[i] = (int)(sums[i]/50.0);
        
        currentState = SystemState::SAFETY_CHECK; 
        lcd.clear();
    }

    // Ensures joystick drift is within safe limits before enabling motors
    void updateSafetyCheck() {
        int max_drift = 0;
        for(int i=0; i<4; i++) {
            int d = abs(filtered_adc[i] - tare_offsets[i]);
            if(d > max_drift) max_drift = d;
        }
        
        lcd.setCursor(0,0); lcd.printf("DRIFT: %d   ", max_drift);
        
        if(max_drift < 200) {
            lcd.setCursor(0,1); lcd.print("B1: CONFIRM GO ");
            if(digitalRead(Config::BTN_SEL_REC) == LOW) {
                // Send reset command to robot to sync states
                pkt.cmd = 99; 
                memcpy(pkt.pulses, current_pos, 24);
                for(int i=0; i<5; i++) { 
                    esp_now_send(Config::RobotMac, (uint8_t*)&pkt, 28); 
                    delay(20); 
                }
                currentState = SystemState::DRIVE; 
                lcd.clear(); lcd.print("DRIVE ENGAGED"); 
                delay(500);
            }
        } else {
            lcd.setCursor(0,1); lcd.print("RETRY TARE?    ");
            if(digitalRead(Config::BTN_SEL_REC) == LOW) startCalibration();
        }
    }

    void updateDrive() {
        // Buffer to hold final motor targets (separates physics calc from state)
        int target_pulses[6];
        for(int i=0; i<6; i++) target_pulses[i] = current_pos[i];

        if(!isReplaying) {
            float L_Y = filtered_adc[0] - tare_offsets[0]; 
            float L_X = filtered_adc[1] - tare_offsets[1]; 
            float R_Y = filtered_adc[2] - tare_offsets[2]; 
            float R_X = filtered_adc[3] - tare_offsets[3]; 

            // Update State based on Inputs
            if(abs(L_Y) > 250) current_pos[4] = constrain(current_pos[4] + map(abs(L_Y), 250, 2048, 1, 8) * (L_Y>0?-1:1), 500, 2500);
            if(abs(L_X) > 250) current_pos[2] = constrain(current_pos[2] + map(abs(L_X), 250, 2048, 1, 8) * (L_X>0?1:-1), 500, 2500);
            if(abs(R_Y) > 250) current_pos[1] = constrain(current_pos[1] + map(abs(R_Y), 250, 2048, 1, 8) * (R_Y>0?1:-1), 500, 2500);
            if(abs(R_X) > 250) current_pos[0] = constrain(current_pos[0] + map(abs(R_X), 250, 2048, 1, 8) * (R_X>0?-1:1), 500, 2500);

            // --- Kinematic Compensation Layer ---
            int compensated_shoulder = current_pos[1];
            
            // Center of Gravity Compensation (Anti-Topple)
            // Adjust shoulder backwards as elbow extends forward
            if (current_pos[2] > 1600) {
                int extension = current_pos[2] - 1600;
                compensated_shoulder += (extension * 0.15); 
            }

            // Wrist Parallelism (Inverse Kinematics)
            int compensated_wrist = calculateWristParallel(compensated_shoulder, current_pos[2]);

            // Populate transmission buffer with compensated values
            target_pulses[0] = current_pos[0];
            target_pulses[1] = constrain(compensated_shoulder, 500, 2500);
            target_pulses[2] = current_pos[2];
            target_pulses[3] = compensated_wrist;
            target_pulses[4] = current_pos[4];

            // Gripper Logic
            if(digitalRead(Config::SW_LEFT_CLICK) == LOW) current_pos[5] = constrain(current_pos[5]-25, 1000, 2500);
            if(digitalRead(Config::SW_RIGHT_CLICK) == LOW) current_pos[5] = constrain(current_pos[5]+25, 1000, 2500);
            target_pulses[5] = current_pos[5];
            
        } else {
            handleReplay();
            // During replay, recorded positions are already compensated
            for(int i=0; i<6; i++) target_pulses[i] = current_pos[i];
        }

        // Transmit Data
        pkt.cmd = isReplaying ? 2 : 0; 
        memcpy(pkt.pulses, target_pulses, 24); 
        esp_now_send(Config::RobotMac, (uint8_t*)&pkt, 28);

        // UI Updates
        static unsigned long lastUI = 0;
        if(millis()-lastUI > 250) { 
            lcd.setCursor(0,1); 
            lcd.printf("P:%d L:%s %04d", sequence.size(), robotLinked?"OK":"NO", current_pos[1]); 
            lastUI = millis(); 
        }
        
        // Recording Logic
        if(digitalRead(Config::BTN_SEL_REC)==LOW && !isReplaying) {
            Waypoint wp; 
            memcpy(wp.pos, target_pulses, 24); // Store the compensated position
            sequence.push_back(wp);
            lcd.setCursor(0,0); lcd.print("SAVED!         "); 
            delay(300);
        }
        
        if(digitalRead(Config::BTN_PLAY)==LOW && sequence.size()>0) {
            isReplaying = true; 
            replayIndex = 0; 
            lcd.setCursor(0,0); lcd.print("REPLAY...      ");
        }
    }

    void handleReplay() {
        if(replayIndex >= sequence.size()) { 
            isReplaying = false; 
            lcd.setCursor(0,0); lcd.print("DONE"); 
            return; 
        }
        if(millis() - lastReplayStep > 1500) {
            memcpy(current_pos, sequence[replayIndex].pos, 24); 
            replayIndex++; 
            lastReplayStep = millis();
        }
    }

    void updateDiag() {
        lcd.setCursor(0,0); lcd.printf("L-X:%3.0f Y:%3.0f", filtered_adc[1], filtered_adc[0]);
        lcd.setCursor(0,1); lcd.printf("R-X:%3.0f Y:%3.0f", filtered_adc[3], filtered_adc[2]);
    }

    void readJoysticks() {
        // Exponential smoothing filter for noise reduction
        for(int i=0; i<4; i++) {
            filtered_adc[i] = (analogRead(Config::J_PINS[i]) * 0.2) + (filtered_adc[i] * 0.8);
        }
    }
};

RobotController* RobotController::instance = nullptr;
RobotController Hub;

void setup() { Hub.begin(); }
void loop() { Hub.run(); delay(10); }
