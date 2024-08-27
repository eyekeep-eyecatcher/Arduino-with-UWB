#include <DW1000Jang.hpp>
#include <DW1000JangUtils.hpp>
#include <DW1000JangTime.hpp>
#include <DW1000JangConstants.hpp>
#include <DW1000JangRanging.hpp>
#include <DW1000JangRTLS.hpp>
#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(2, 3); // RX, TX

#define Delay 1000
// connection pins
#if defined(ESP8266)
const uint8_t PIN_SS = 15;
#else
const uint8_t PIN_SS = 10; // spi select pin
const uint8_t PIN_RST = 7;
#endif

#define SWITCH_PIN 4 // 스위치가 연결된 핀 번호

double range_A;
double range_B;
int cnt = 0;
bool flag = false;
bool btnflag = true;
bool bluetoothConnected = false;

typedef struct Position {
    double x;
    double y;
} Position;

Position position_A = {0,0};
Position position_B = {1.2,0};
Position position_C = {1.2,1.2};

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

frame_filtering_configuration_t TAG_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false
};

void setup() {
    // DEBUG monitoring
    Serial.begin(9600);
    Bluetooth.begin(9600);
    Serial.println(F("### DW1000Jang-arduino-ranging-tag ###"));
    // initialize the driver
    #if defined(ESP8266)
    DW1000Jang::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Jang::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println("DW1000Jang initialized ...");
    // general configuration
    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
    DW1000Jang::enableFrameFiltering(TAG_FRAME_FILTER_CONFIG);

    DW1000Jang::setDeviceAddress(4); // 1조 Device Address : 4
                                     // 2조 Device Address : 5
                                     // ...
                                     // 10조 Device Address : 13
    DW1000Jang::setNetworkId(100);

    DW1000Jang::setAntennaDelay(16436);

    DW1000Jang::setPreambleDetectionTimeout(64);
    DW1000Jang::setSfdDetectionTimeout(273);
    DW1000Jang::setReceiveFrameWaitTimeoutPeriod(5000);
    
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Jang::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Jang::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Jang::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Jang::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);    

    pinMode(SWITCH_PIN, INPUT); // 스위치 핀을 입력 모드로 설정
}

void loop() {
    if (cnt > 100) cnt = 0;
    int rand;
    byte transmit_data[1];

    // 블루투스 연결 상태 확인
    if (Bluetooth.available()) {
        bluetoothConnected = true; // 데이터가 수신되면 블루투스 연결 상태를 true로 설정
    } else {
        bluetoothConnected = false; // 데이터가 없으면 블루투스 연결 상태를 false로 설정
    }

    if (bluetoothConnected) {
        int switchState = digitalRead(SWITCH_PIN); // 스위치 상태 읽기

        if (switchState == HIGH) {
            flag = true;
            if(btnflag == false){
               Bluetooth.write("Q");
               btnflag = true;
            }
        } else {
            flag = false;
            if(btnflag == true){
               Bluetooth.write("W");
               btnflag = false;
            }
            transmit_data[0] = 'S';
            Serial.println((char)transmit_data[0]);
            for (int i = 0; i < 5; i++) { // 'S' 신호를 여러 번 전송
                DW1000Jang::setTransmitData(transmit_data);
                DW1000Jang::startTransmit(TransmitMode::IMMEDIATE);
                while (!DW1000Jang::isTransmitDone()) {
                    #if defined(ESP8266)
                    yield();
                    #endif
                }
                DW1000Jang::clearTransmitStatus();
                delay(10); // 0.01초 대기
            }
        }

        if (flag) {
            New_structure result_A_Anchor = DW1000JangRTLS::Tag_Distance_Request(1, 1500);
            if (result_A_Anchor.success) {
                range_A = result_A_Anchor.distance;
                Serial.print("A의 거리:");
                Serial.println(range_A);
                randomSeed(analogRead(0));
                rand = random(1, 10);
                delay(10 + 10 * rand);
            }
            New_structure result_B_Anchor = DW1000JangRTLS::Tag_Distance_Request(2, 1500);
            if (result_B_Anchor.success) {
                range_B = result_B_Anchor.distance;
                Serial.print("B의 거리:");
                Serial.println(range_B);
                randomSeed(analogRead(0));
                rand = random(1, 10);
                delay(10 + 10 * rand);
            }

            if (range_A > range_B) transmit_data[0] = 'H'; // B스피커가 사용자와 더 가까우면 'H'
            else if (range_A < range_B) transmit_data[0] = 'L'; // A스피커가 사용자와 더 가까우면 'L'
            Serial.println((char)transmit_data[0]);
            DW1000Jang::setTransmitData(transmit_data);
            DW1000Jang::startTransmit(TransmitMode::IMMEDIATE);

            while (!DW1000Jang::isTransmitDone()) {
                #if defined(ESP8266)
                yield();
                #endif
            }
            DW1000Jang::clearTransmitStatus();
            delay(10); // 0.05초 대기
        }
    } else {
        // 블루투스 연결이 끊어졌을 때 모든 동작 중지
        Serial.println("Bluetooth disconnected. Stopping all actions.");
    }

    cnt++;
}
