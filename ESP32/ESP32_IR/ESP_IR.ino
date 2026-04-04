#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

const int ledIndicator = 2;
const int pin = 23;
const int freq = 38000;  // 38 kHz
const int resolution = 8; // 8-bit resolution
const int duty = 84;      // 33% of 255 (max 8-bit)
uint32_t currentCode = 0x00000000;
bool repeatEnabled = false;
unsigned long lastSendTime = 0;

enum NECMode {
    NEC_STANDARD,
    NEC_EXTENDED
};

NECMode currentMode = NEC_EXTENDED; // default

// Parse new code
uint32_t parseCode(String value) {
    value.trim();
    value.replace("0x", "");
    value.replace("0X", "");

    return (uint32_t) strtoul(value.c_str(), NULL, 16);
}

// Turn carrier ON (mark) for duration in microseconds
void mark(unsigned int duration) {
  ledcWrite(pin, duty);  // Enable PWM with 33% duty cycle
  delayMicroseconds(duration);  // Wait for the mark duration
}

// Turn carrier OFF (space) for duration in microseconds
void space(unsigned int duration) {
  ledcWrite(pin, 0);     // Disable PWM (LED off)
  delayMicroseconds(duration);  // Wait for the space duration
}

// Send the Code
void sendNEC(uint32_t data, NECMode mode = NEC_EXTENDED) {
    digitalWrite(ledIndicator, HIGH);

    // Header
    mark(9000);
    space(4500);

    if (mode == NEC_STANDARD) {
        // NEC standard: 2 bytes (addr + cmd)
        for (int byteIndex = 1; byteIndex >= 0; byteIndex--) {
            uint8_t currentByte = (data >> (byteIndex * 8)) & 0xFF;

            for (int bit = 0; bit < 8; bit++) {
                mark(560);
                if (currentByte & (1 << bit)) space(1600);
                else space(560);
            }
        }
    } else {  
        // EXT_NEC: 4 bytes (16+16 bits, current)
        for (int byteIndex = 3; byteIndex >= 0; byteIndex--) {
            uint8_t currentByte = (data >> (byteIndex * 8)) & 0xFF;

            for (int bit = 0; bit < 8; bit++) {
                mark(560);
                if (currentByte & (1 << bit)) space(1600);
                else space(560);
            }
        }
    }

    mark(560);
    ledcWrite(pin, 0);
    digitalWrite(ledIndicator, LOW);
}

BLECharacteristic *pCharacteristicTX;
bool deviceConnected = false;
String inputBuffer = "";

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        BLEDevice::startAdvertising();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue();

        for (int i = 0; i < rxValue.length(); i++) {
            char c = rxValue.charAt(i);

            if (c == ';') {
                inputBuffer.trim();
                if (inputBuffer.length() > 0) {
                    processCommand(inputBuffer);
                }
                inputBuffer = "";
            } 
            else {
                inputBuffer += c;
            }
        }
    }

    void processCommand(const String &cmdLine) {
        String line = cmdLine;
        line.trim();
        if (line.length() == 0) return;

        int spaceIndex = line.indexOf(' ');
        String command = (spaceIndex == -1) ? line : line.substring(0, spaceIndex);
        String args = (spaceIndex == -1) ? "" : line.substring(spaceIndex + 1);

        command.toLowerCase();
        args.trim();

        if (command == "send") {
            if (repeatEnabled) {
                notifyClient("Cannot send: Repeat mode is ON");
            } else {
                sendNEC(currentCode, currentMode);
                lastSendTime = millis();
                notifyClient("Sent");
            }

        } else if (command == "repeat") {
            if (!repeatEnabled) {
                repeatEnabled = true;
                notifyClient("Repeat ON");
            } else {
                repeatEnabled = false;
                notifyClient("Repeat OFF");
            }

        } else if (command == "set") {
            if (args.startsWith("NEC")) {
                String signal = args.substring(3);
                signal.trim();
                currentCode = parseCode(signal);   // corrected name
                currentMode = NEC_STANDARD;
                notifyClient("NEC_STANDARD set to 0x" + String(currentCode, HEX));

            } else if (args.startsWith("EXT_NEC")) {
                String signal = args.substring(7);
                signal.trim();
                currentCode = parseCode(signal);   // corrected name
                currentMode = NEC_EXTENDED;
                notifyClient("EXT_NEC set to 0x" + String(currentCode, HEX));

            } else {
                notifyClient("Unknown set protocol");
            }

        } else {
            notifyClient("Unknown command");
        }
    }

    void notifyClient(String message) {
        if (deviceConnected) {
            pCharacteristicTX->setValue(message.c_str());
            pCharacteristicTX->notify();
        }
    }
};

void setupBLE() {
    BLEDevice::init("ESP32_IR");

    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    BLECharacteristic *pCharacteristicRX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE
    );

    pCharacteristicTX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristicTX->addDescriptor(new BLE2902()); // idk maybe remove if not needed

    pCharacteristicRX->setCallbacks(new MyCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);

    BLEDevice::startAdvertising();
}

void setup() {
    // Setup pwm pin
    ledcAttach(pin, freq, resolution);

    // Setup LED indicator pin
    pinMode(ledIndicator, OUTPUT);

    setupBLE();
}

void loop() {
    if (repeatEnabled && deviceConnected) {

        if (millis() - lastSendTime > 110) {  // NEC repeat timing
            sendNEC(currentCode, currentMode);
            lastSendTime = millis();
        }
    }
}