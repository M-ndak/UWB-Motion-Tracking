/*
 * UWB Motion Tracking — Tag Firmware
 * Based on ESP32-DWM3000-UWB-Indoor-RTLS-Tracker by Circuit Digest
 * Original: https://github.com/Circuit-Digest/ESP32-DWM3000-UWB-Indoor-RTLS-Tracker
 * Original licensed under GNU GPL v3
 *
 * Modified by Mehrak Singh Sachdev, 2026
 * Changes:
 *   - Scalable anchor system via NUM_ANCHORS define
 *   - FILTER_SIZE reduced from 30 to 5 for faster valid readings
 *   - anyAnchorHasValidData() replaces allAnchorsHaveValidData()
 *     so WiFi data transmits as soon as one anchor has a reading
 *   - MAX_DISTANCE increased to 1500cm for larger rooms
 *   - WiFi connection non-blocking — ranging continues offline
 *   - JSON packet includes raw distance, RSSI, first-path RSSI,
 *     round time, reply time, clock offset per anchor
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 */



#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>

// ============================================================
// CONFIGURATION — edit these before flashing
// ============================================================
const char *ssid     = "parallax";
const char *password = "parallax2006";
const char *host     = "192.168.1.102";  // Your PC's IP
const int   port     = 7007;

// Scalable Anchor Configuration
#define NUM_ANCHORS    3     // Change to scale
#define TAG_ID         10
#define FIRST_ANCHOR_ID 1   // Anchors will be ID 1, 2, 3

// FIX: Filter size reduced from 30 to 5 — gets valid data in ~5 ranging cycles
#define FILTER_SIZE    5
#define MIN_DISTANCE   0
#define MAX_DISTANCE   1500.0   // cm — increased for larger rooms

// ============================================================
// UWB register/constant definitions (unchanged from original)
// ============================================================
#define LEN_RX_CAL_CONF 4
#define LEN_TX_FCTRL_CONF 6
#define LEN_AON_DIG_CFG_CONF 3
#define PMSC_STATE_IDLE 0x3
#define FCS_LEN 2
#define STDRD_SYS_CONFIG 0x188
#define DTUNE0_CONFIG 0x0F
#define SYS_STATUS_FRAME_RX_SUCC 0x2000
#define SYS_STATUS_RX_ERR 0x4279000
#define SYS_STATUS_FRAME_TX_SUCC 0x80
#define PREAMBLE_32 4
#define PREAMBLE_64 8
#define PREAMBLE_128 5
#define PREAMBLE_256 9
#define PREAMBLE_512 11
#define PREAMBLE_1024 2
#define PREAMBLE_2048 10
#define PREAMBLE_4096 3
#define PREAMBLE_1536 6
#define CHANNEL_5 0x0
#define CHANNEL_9 0x1
#define PAC4 0x03
#define PAC8 0x00
#define PAC16 0x01
#define PAC32 0x02
#define DATARATE_6_8MB 0x1
#define DATARATE_850KB 0x0
#define PHR_MODE_STANDARD 0x0
#define PHR_MODE_LONG 0x1
#define PHR_RATE_6_8MB 0x1
#define PHR_RATE_850KB 0x0
#define SPIRDY_MASK 0x80
#define RCINIT_MASK 0x100
#define BIAS_CTRL_BIAS_MASK 0x1F
#define GEN_CFG_AES_LOW_REG 0x00
#define GEN_CFG_AES_HIGH_REG 0x01
#define STS_CFG_REG 0x2
#define RX_TUNE_REG 0x3
#define EXT_SYNC_REG 0x4
#define GPIO_CTRL_REG 0x5
#define DRX_REG 0x6
#define RF_CONF_REG 0x7
#define RF_CAL_REG 0x8
#define FS_CTRL_REG 0x9
#define AON_REG 0xA
#define OTP_IF_REG 0xB
#define CIA_REG1 0xC
#define CIA_REG2 0xD
#define CIA_REG3 0xE
#define DIG_DIAG_REG 0xF
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define RX_BUFFER_1_REG 0x13
#define TX_BUFFER_REG 0x14
#define ACC_MEM_REG 0x15
#define SCRATCH_RAM_REG 0x16
#define AES_RAM_REG 0x17
#define SET_1_2_REG 0x18
#define INDIRECT_PTR_A_REG 0x1D
#define INDIRECT_PTR_B_REG 0x1E
#define IN_PTR_CFG_REG 0x1F
#define TRANSMIT_DELAY 0x3B9ACA00
#define TRANSMIT_DIFF 0x1FF
#define NS_UNIT 4.0064102564102564
#define PS_UNIT 15.6500400641025641
#define SPEED_OF_LIGHT 0.029979245800
#define CLOCK_OFFSET_CHAN_5_CONSTANT -0.5731e-3f
#define CLOCK_OFFSET_CHAN_9_CONSTANT -0.1252e-3f
#define NO_OFFSET 0x0
#define DEBUG_OUTPUT 0

// SPI pins
#define RST_PIN 27
#define CHIP_SELECT_PIN 5

static int ANTENNA_DELAY = 16350;
int led_status = 0;
int destination = 0x0;
int sender = 0x0;

// Radio configuration
int config[] = {
    CHANNEL_5, PREAMBLE_128, 9, PAC8, DATARATE_6_8MB, PHR_MODE_STANDARD, PHR_RATE_850KB
};

// State machine
static int rx_status;
static int current_anchor_index = 0;
static int curr_stage = 0;

// WiFi state
WiFiClient client;
bool wifiConnected = false;

// ============================================================
// Anchor data structure
// ============================================================
struct AnchorData {
    int anchor_id;
    long long t_roundA = 0;
    long long t_replyA = 0;
    long long rx = 0;
    long long tx = 0;
    int clock_offset = 0;
    float distance = 0;
    float distance_history[FILTER_SIZE];
    int history_index = 0;
    float filtered_distance = 0;
    float signal_strength = 0;
    float fp_signal_strength = 0;

    AnchorData() {
        memset(distance_history, 0, sizeof(distance_history));
    }
};

AnchorData anchors[NUM_ANCHORS];

void initializeAnchors() {
    for (int i = 0; i < NUM_ANCHORS; i++) {
        anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
    }
}

AnchorData *getCurrentAnchor() { return &anchors[current_anchor_index]; }
int getCurrentAnchorId()       { return anchors[current_anchor_index].anchor_id; }

void switchToNextAnchor() {
    current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
}

// FIX: Replaced allAnchorsHaveValidData() with anyAnchorHasValidData()
// This allows WiFi data to be sent as soon as at least one anchor has a reading,
// instead of waiting for all FILTER_SIZE * NUM_ANCHORS exchanges to succeed.
bool anyAnchorHasValidData() {
    for (int i = 0; i < NUM_ANCHORS; i++) {
        if (anchors[i].filtered_distance > 0) return true;
    }
    return false;
}

// ============================================================
// DWM3000 Class
// ============================================================
class DWM3000Class {
public:
    static int config_arr[9];  // renamed to avoid collision with global config[]
    static void spiSelect(uint8_t cs);
    static void begin();
    static void init();
    static void writeSysConfig();
    static void configureAsTX();
    static void setupGPIO();
    static void ds_sendFrame(int stage);
    static void ds_sendRTInfo(int t_roundB, int t_replyB);
    static int  ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clock_offset);
    static int  ds_getStage();
    static bool ds_isErrorFrame();
    static void ds_sendErrorFrame();
    static void setChannel(uint8_t data);
    static void setPreambleLength(uint8_t data);
    static void setPreambleCode(uint8_t data);
    static void setPACSize(uint8_t data);
    static void setDatarate(uint8_t data);
    static void setPHRMode(uint8_t data);
    static void setPHRRate(uint8_t data);
    static void setMode(int mode);
    static void setTXFrame(unsigned long long frame_data);
    static void setFrameLength(int frame_len);
    static void setTXAntennaDelay(int delay);
    static void setSenderID(int senderID);
    static void setDestinationID(int destID);
    static int  receivedFrameSucc();
    static int  sentFrameSucc();
    static int  getSenderID();
    static int  getDestinationID();
    static bool checkForIDLE();
    static bool checkSPI();
    static double      getSignalStrength();
    static double      getFirstPathSignalStrength();
    static int         getTXAntennaDelay();
    static long double getClockOffset();
    static long double getClockOffset(int32_t ext_clock_offset);
    static int         getRawClockOffset();
    static float       getTempInC();
    static unsigned long long readRXTimestamp();
    static unsigned long long readTXTimestamp();
    static uint32_t write(int base, int sub, uint32_t data, int data_len);
    static uint32_t write(int base, int sub, uint32_t data);
    static uint32_t read(int base, int sub);
    static uint8_t  read8bit(int base, int sub);
    static uint32_t readOTP(uint8_t addr);
    static void writeTXDelay(uint32_t delay);
    static void prepareDelayedTX();
    static void delayedTXThenRX();
    static void delayedTX();
    static void standardTX();
    static void standardRX();
    static void TXInstantRX();
    static void softReset();
    static void hardReset();
    static void clearSystemStatus();
    static void pullLEDHigh(int led);
    static void pullLEDLow(int led);
    static double convertToCM(int DWM3000_ps_units);
    static void calculateTXRXdiff();
    static void printRoundTripInformation();
    static void printDouble(double val, unsigned int precision, bool linebreak);

private:
    static void setBit(int reg_addr, int sub_addr, int shift, bool b);
    static void setBitLow(int reg_addr, int sub_addr, int shift);
    static void setBitHigh(int reg_addr, int sub_addr, int shift);
    static void writeFastCommand(int cmd);
    static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit);
    static uint32_t sendBytes(int b[], int lenB, int recLen);
    static void clearAONConfig();
    static unsigned int countBits(unsigned int number);
    static int checkForDevID();
};

DWM3000Class DWM3000;
// Use the global config[] array directly in method bodies below
int DWM3000Class::config_arr[] = {
    CHANNEL_5, PREAMBLE_128, 9, PAC8, DATARATE_6_8MB, PHR_MODE_STANDARD, PHR_RATE_850KB
};

// ============================================================
// WiFi helpers
// ============================================================
void connectToWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println("\nWiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed — ranging will continue offline");
    }
}

void sendDataOverWiFi() {
    if (!wifiConnected) {
        connectToWiFi();
        if (!wifiConnected) return;
    }
    if (!client.connected()) {
        if (!client.connect(host, port)) {
            Serial.println("TCP connection to host failed");
            wifiConnected = false;
            return;
        }
    }

    // Build JSON — include anchors that have a valid filtered distance;
    // use 0.0 for anchors not yet measured so Python can handle it gracefully.
    String data = "{\"tag_id\":" + String(TAG_ID) + ",\"anchors\":{";
    for (int i = 0; i < NUM_ANCHORS; i++) {
        data += "\"A" + String(anchors[i].anchor_id) + "\":{";
        data += "\"distance\":"   + String(anchors[i].filtered_distance, 2) + ",";
        data += "\"raw\":"        + String(anchors[i].distance, 2)          + ",";
        data += "\"rssi\":"       + String(anchors[i].signal_strength, 2)   + ",";
        data += "\"fp_rssi\":"    + String(anchors[i].fp_signal_strength, 2) + ",";
        data += "\"round_time\":" + String((long)anchors[i].t_roundA)       + ",";
        data += "\"reply_time\":" + String((long)anchors[i].t_replyA)       + ",";
        data += "\"clock_offset\":" + String((double)DWM3000.getClockOffset(anchors[i].clock_offset), 6);
        data += "}";
        if (i < NUM_ANCHORS - 1) data += ",";
    }
    data += "}}\n";

    client.print(data);
    Serial.println("Sent: " + data);
}

// ============================================================
// Distance filtering helpers
// ============================================================
bool isValidDistance(float d) {
    return (d > MIN_DISTANCE && d <= MAX_DISTANCE);
}

float calculateMedian(float arr[], int size) {
    float temp[FILTER_SIZE];
    memcpy(temp, arr, size * sizeof(float));
    // Simple insertion sort (fine for FILTER_SIZE = 5)
    for (int i = 1; i < size; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) { temp[j + 1] = temp[j]; j--; }
        temp[j + 1] = key;
    }
    return size % 2 == 0 ? (temp[size/2 - 1] + temp[size/2]) / 2.0f : temp[size/2];
}

void updateFilteredDistance(AnchorData &data) {
    data.distance_history[data.history_index] = data.distance;
    data.history_index = (data.history_index + 1) % FILTER_SIZE;

    float valid[FILTER_SIZE];
    int cnt = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        if (isValidDistance(data.distance_history[i]))
            valid[cnt++] = data.distance_history[i];
    }
    data.filtered_distance = (cnt > 0) ? calculateMedian(valid, cnt) : 0;
}

void printAllDistances() {
    Serial.print("Distances - ");
    for (int i = 0; i < NUM_ANCHORS; i++) {
        Serial.print("A"); Serial.print(anchors[i].anchor_id); Serial.print(": ");
        if (anchors[i].filtered_distance > 0) {
            DWM3000.printDouble(anchors[i].filtered_distance, 100, false);
            Serial.print(" cm");
        } else {
            Serial.print("INVALID");
        }
        if (i < NUM_ANCHORS - 1) Serial.print(" | ");
    }
    Serial.println();
}

// ============================================================
// DWM3000Class method implementations
// ============================================================
void DWM3000Class::spiSelect(uint8_t cs) {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    delay(5);
}

void DWM3000Class::begin() {
    delay(5);
    pinMode(CHIP_SELECT_PIN, OUTPUT);
    SPI.begin();
    delay(5);
    spiSelect(CHIP_SELECT_PIN);
    Serial.println("[INFO] SPI ready");
}

void DWM3000Class::init() {
    if (!checkForDevID()) {
        Serial.println("[ERROR] Dev ID is wrong! Aborting!");
        return;
    }
    setBitHigh(GEN_CFG_AES_LOW_REG, 0x10, 4);
    while (!checkForIDLE()) { Serial.println("[WARNING] IDLE FAILED (stage 1)"); delay(100); }
    softReset();
    delay(200);
    while (!checkForIDLE()) { Serial.println("[WARNING] IDLE FAILED (stage 2)"); delay(100); }

    uint32_t ldo_low  = readOTP(0x04);
    uint32_t ldo_high = readOTP(0x05);
    uint32_t bias_tune = readOTP(0xA);
    bias_tune = (bias_tune >> 16) & BIAS_CTRL_BIAS_MASK;
    if (ldo_low != 0 && ldo_high != 0 && bias_tune != 0) {
        write(0x11, 0x1F, bias_tune);
        write(0x0B, 0x08, 0x0100);
    }

    int xtrim_value = readOTP(0x1E);
    xtrim_value = xtrim_value == 0 ? 0x2E : xtrim_value;
    write(FS_CTRL_REG, 0x14, xtrim_value);

    writeSysConfig();
    write(0x00, 0x3C, 0xFFFFFFFF);
    write(0x00, 0x40, 0xFFFF);
    write(0x0A, 0x00, 0x000900, 3);
    write(0x3, 0x1C, 0x10000240);
    write(0x3, 0x20, 0x1B6DA489);
    write(0x3, 0x38, 0x0001C0FD);
    write(0x3, 0x3C, 0x0001C43E);
    write(0x3, 0x40, 0x0001C6BE);
    write(0x3, 0x44, 0x0001C77E);
    write(0x3, 0x48, 0x0001CF36);
    write(0x3, 0x4C, 0x0001CFB5);
    write(0x3, 0x50, 0x0001CFF5);
    write(0x3, 0x18, 0xE5E5);
    write(0x6, 0x0, 0x81101C);
    write(0x07, 0x34, 0x4);
    write(0x07, 0x48, 0x14);
    write(0x07, 0x1A, 0x0E);
    write(0x07, 0x1C, 0x1C071134);
    write(0x09, 0x00, 0x1F3C);
    write(0x09, 0x80, 0x81);
    write(0x11, 0x04, 0xB40200);
    write(0x11, 0x08, 0x80030738);
    Serial.println("[INFO] Initialization finished.\n");
}

void DWM3000Class::writeSysConfig() {
    int usr_cfg = (STDRD_SYS_CONFIG & 0xFFF) | (config[5] << 3) | (config[6] << 4);
    write(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg);
    int otp_write = 0x1400;
    if (config[1] >= 256) otp_write |= 0x04;
    write(OTP_IF_REG, 0x08, otp_write);
    write(DRX_REG, 0x00, 0x00, 1);
    write(DRX_REG, 0x0, config[3]);
    write(STS_CFG_REG, 0x0, 64 / 8 - 1);
    write(GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1);
    write(DRX_REG, 0x0C, 0xAF5F584C);

    int chan_ctrl_val = read(GEN_CFG_AES_HIGH_REG, 0x14);
    chan_ctrl_val &= (~0x1FFF);
    chan_ctrl_val |= config[0];
    chan_ctrl_val |= 0x1F00 & (config[2] << 8);
    chan_ctrl_val |= 0xF8 & (config[2] << 3);
    chan_ctrl_val |= 0x06 & (0x01 << 1);
    write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_val);

    int tx_fctrl_val = read(GEN_CFG_AES_LOW_REG, 0x24);
    tx_fctrl_val |= (config[1] << 12);
    tx_fctrl_val |= (config[4] << 10);
    write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_val);
    write(DRX_REG, 0x02, 0x81);

    int rf_tx_ctrl_2 = 0x1C071134;
    int pll_conf = 0x0F3C;
    if (config[0]) {
        rf_tx_ctrl_2 &= ~0x00FFFF;
        rf_tx_ctrl_2 |= 0x000001;
        pll_conf &= 0x00FF;
        pll_conf |= 0x001F;
    }
    write(RF_CONF_REG, 0x1C, rf_tx_ctrl_2);
    write(FS_CTRL_REG, 0x00, pll_conf);
    write(RF_CONF_REG, 0x51, 0x14);
    write(RF_CONF_REG, 0x1A, 0x0E);
    write(FS_CTRL_REG, 0x08, 0x81);
    write(GEN_CFG_AES_LOW_REG, 0x44, 0x02);
    write(PMSC_REG, 0x04, 0x300200);
    write(PMSC_REG, 0x08, 0x0138);

    int success = 0;
    for (int i = 0; i < 100; i++) {
        if (read(GEN_CFG_AES_LOW_REG, 0x0) & 0x2) { success = 1; break; }
    }
    if (!success) Serial.println("[ERROR] Couldn't lock PLL Clock!");
    else          Serial.println("[INFO] PLL is now locked.");

    int otp_val = read(OTP_IF_REG, 0x08);
    otp_val |= 0x40;
    if (config[0]) otp_val |= 0x2000;
    write(OTP_IF_REG, 0x08, otp_val);
    write(RX_TUNE_REG, 0x19, 0xF0);

    int ldo_ctrl_val = read(RF_CONF_REG, 0x48);
    int tmp_ldo = (0x105 | 0x100 | 0x4 | 0x1);
    write(RF_CONF_REG, 0x48, tmp_ldo);
    write(EXT_SYNC_REG, 0x0C, 0x020000);
    delay(20);
    write(EXT_SYNC_REG, 0x0C, 0x11);

    int succ = 0;
    for (int i = 0; i < 100; i++) {
        if (read(EXT_SYNC_REG, 0x20)) { succ = 1; break; }
        delay(10);
    }
    if (succ) Serial.println("[INFO] PGF calibration complete.");
    else      Serial.println("[ERROR] PGF calibration failed!");

    write(EXT_SYNC_REG, 0x0C, 0x00);
    write(EXT_SYNC_REG, 0x20, 0x01);
    int rx_cal_res = read(EXT_SYNC_REG, 0x14);
    if (rx_cal_res == 0x1fffffff) Serial.println("[ERROR] PGF_CAL failed in stage I!");
    rx_cal_res = read(EXT_SYNC_REG, 0x1C);
    if (rx_cal_res == 0x1fffffff) Serial.println("[ERROR] PGF_CAL failed in stage Q!");

    write(RF_CONF_REG, 0x48, ldo_ctrl_val);
    write(0x0E, 0x02, 0x01);
    setTXAntennaDelay(ANTENNA_DELAY);
}

void DWM3000Class::configureAsTX() {
    write(RF_CONF_REG, 0x1C, 0x34);
    write(GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD);
}

void DWM3000Class::setupGPIO() { write(0x05, 0x08, 0xF0); }

void DWM3000Class::ds_sendFrame(int stage) {
    setMode(1);
    write(0x14, 0x01, sender & 0xFF);
    write(0x14, 0x02, destination & 0xFF);
    write(0x14, 0x03, stage & 0x7);
    setFrameLength(4);
    TXInstantRX();
    bool error = true;
    for (int i = 0; i < 50; i++) { if (sentFrameSucc()) { error = false; break; } }
    if (error) Serial.println("[ERROR] Could not send frame successfully!");
}

void DWM3000Class::ds_sendRTInfo(int t_roundB, int t_replyB) {
    setMode(1);
    write(0x14, 0x01, destination & 0xFF);
    write(0x14, 0x02, sender & 0xFF);
    write(0x14, 0x03, 4);
    write(0x14, 0x04, t_roundB);
    write(0x14, 0x08, t_replyB);
    setFrameLength(12);
    TXInstantRX();
}

int DWM3000Class::ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clk_offset) {
    int reply_diff = t_replyA - t_replyB;
    long double clock_offset = t_replyA > t_replyB ? 1.0 + getClockOffset(clk_offset) : 1.0 - getClockOffset(clk_offset);
    int first_rt  = t_roundA - t_replyB;
    int second_rt = t_roundB - t_replyA;
    int combined_rt = (first_rt + second_rt - (reply_diff - (reply_diff * clock_offset))) / 2;
    return combined_rt / 2;
}

int  DWM3000Class::ds_getStage()      { return read(0x12, 0x03) & 0b111; }
bool DWM3000Class::ds_isErrorFrame()  { return ((read(0x12, 0x00) & 0x7) == 7); }

void DWM3000Class::ds_sendErrorFrame() {
    Serial.println("[WARNING] Error Frame sent. Reverting back to stage 0.");
    setMode(7);
    setFrameLength(3);
    standardTX();
}

void DWM3000Class::setChannel(uint8_t data)       { if (data == CHANNEL_5 || data == CHANNEL_9) config[0] = data; }
void DWM3000Class::setPreambleLength(uint8_t data) { config[1] = data; }
void DWM3000Class::setPreambleCode(uint8_t data)   { if (data <= 12 && data >= 9) config[2] = data; }
void DWM3000Class::setPACSize(uint8_t data)        { config[3] = data; }
void DWM3000Class::setDatarate(uint8_t data)       { config[4] = data; }
void DWM3000Class::setPHRMode(uint8_t data)        { config[5] = data; }
void DWM3000Class::setPHRRate(uint8_t data)        { config[6] = data; }
void DWM3000Class::setMode(int mode)               { write(0x14, 0x00, mode & 0x7); }

void DWM3000Class::setTXFrame(unsigned long long frame_data) {
    write(TX_BUFFER_REG, 0x00, frame_data);
}

void DWM3000Class::setFrameLength(int frameLen) {
    frameLen = frameLen + FCS_LEN;
    int curr_cfg = read(0x00, 0x24);
    if (frameLen > 1023) { Serial.println("[ERROR] Frame too long!"); return; }
    write(GEN_CFG_AES_LOW_REG, 0x24, (curr_cfg & 0xFFFFFC00) | frameLen);
}

void DWM3000Class::setTXAntennaDelay(int delay) { ANTENNA_DELAY = delay; write(0x01, 0x04, delay); }
void DWM3000Class::setSenderID(int senderID)    { sender = senderID; }
void DWM3000Class::setDestinationID(int destID) { destination = destID; }

int DWM3000Class::receivedFrameSucc() {
    int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
    if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC) > 0) return 1;
    if ((sys_stat & SYS_STATUS_RX_ERR) > 0) return 2;
    return 0;
}

int DWM3000Class::sentFrameSucc() {
    int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
    return (sys_stat & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC ? 1 : 0;
}

int  DWM3000Class::getSenderID()      { return read(0x12, 0x01) & 0xFF; }
int  DWM3000Class::getDestinationID() { return read(0x12, 0x02) & 0xFF; }

bool DWM3000Class::checkForIDLE() {
    return (read(0x0F, 0x30) >> 16 & PMSC_STATE_IDLE) == PMSC_STATE_IDLE ||
           (read(0x00, 0x44) >> 16 & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK) ? 1 : 0;
}
bool DWM3000Class::checkSPI() { return checkForDevID(); }

double DWM3000Class::getSignalStrength() {
    int CIRpower = read(0x0C, 0x2C) & 0x1FF;
    int PAC_val  = read(0x0C, 0x58) & 0xFFF;
    unsigned int DGC = (read(0x03, 0x60) >> 28) & 0x7;
    return 10 * log10((CIRpower * (1 << 21)) / pow(PAC_val, 2)) + (6 * DGC) - 121.7;
}

double DWM3000Class::getFirstPathSignalStrength() {
    float f1 = (read(0x0C, 0x30) & 0x3FFFFF) >> 2;
    float f2 = (read(0x0C, 0x34) & 0x3FFFFF) >> 2;
    float f3 = (read(0x0C, 0x38) & 0x3FFFFF) >> 2;
    int PAC_val = read(0x0C, 0x58) & 0xFFF;
    unsigned int DGC = (read(0x03, 0x60) >> 28) & 0x7;
    return 10 * log10((pow(f1,2) + pow(f2,2) + pow(f3,2)) / pow(PAC_val,2)) + (6 * DGC) - 121.7;
}

int DWM3000Class::getTXAntennaDelay() { return read(0x01, 0x04) & 0xFFFF; }

long double DWM3000Class::getClockOffset() {
    return getRawClockOffset() * (config[0] == CHANNEL_5 ? CLOCK_OFFSET_CHAN_5_CONSTANT : CLOCK_OFFSET_CHAN_9_CONSTANT) / 1000000;
}
long double DWM3000Class::getClockOffset(int32_t sec) {
    return sec * (config[0] == CHANNEL_5 ? CLOCK_OFFSET_CHAN_5_CONSTANT : CLOCK_OFFSET_CHAN_9_CONSTANT) / 1000000;
}

int DWM3000Class::getRawClockOffset() {
    int raw = read(0x06, 0x29) & 0x1FFFFF;
    if (raw & (1 << 20)) raw |= ~((1 << 21) - 1);
    return raw;
}

float DWM3000Class::getTempInC() {
    write(0x07, 0x34, 0x04);
    write(0x08, 0x00, 0x01);
    while (!(read(0x08, 0x04) & 0x01)) {}
    int res = (read(0x08, 0x08) & 0xFF00) >> 8;
    int otp_temp = readOTP(0x09) & 0xFF;
    write(0x08, 0x00, 0x00, 1);
    return (float)((res - otp_temp) * 1.05f) + 22.0f;
}

unsigned long long DWM3000Class::readRXTimestamp() {
    uint32_t ts_low = read(0x0C, 0x00);
    unsigned long long ts_high = read(0x0C, 0x04) & 0xFF;
    return (ts_high << 32) | ts_low;
}
unsigned long long DWM3000Class::readTXTimestamp() {
    unsigned long long ts_low  = read(0x00, 0x74);
    unsigned long long ts_high = read(0x00, 0x78) & 0xFF;
    return (ts_high << 32) + ts_low;
}

uint32_t DWM3000Class::write(int base, int sub, uint32_t data, int dataLen) { return readOrWriteFullAddress(base, sub, data, dataLen, 1); }
uint32_t DWM3000Class::write(int base, int sub, uint32_t data)              { return readOrWriteFullAddress(base, sub, data, 0, 1); }
uint32_t DWM3000Class::read(int base, int sub)                              { return readOrWriteFullAddress(base, sub, 0, 0, 0); }
uint8_t  DWM3000Class::read8bit(int base, int sub)                         { return (uint8_t)(read(base, sub) >> 24); }
uint32_t DWM3000Class::readOTP(uint8_t addr) {
    write(OTP_IF_REG, 0x04, addr);
    write(OTP_IF_REG, 0x08, 0x02);
    return read(OTP_IF_REG, 0x10);
}

void DWM3000Class::writeTXDelay(uint32_t delay) { write(0x00, 0x2C, delay); }
void DWM3000Class::prepareDelayedTX() {
    long long rx_ts = readRXTimestamp();
    uint32_t exact_tx = (long long)(rx_ts + TRANSMIT_DELAY) >> 8;
    long long calc_tx = ((rx_ts + TRANSMIT_DELAY) & ~TRANSMIT_DIFF) + ANTENNA_DELAY;
    uint32_t reply_delay = calc_tx - rx_ts;
    write(0x14, 0x01, sender & 0xFF);
    write(0x14, 0x02, destination & 0xFF);
    write(0x14, 0x03, reply_delay);
    setFrameLength(7);
    writeTXDelay(exact_tx);
}

void DWM3000Class::delayedTXThenRX() { writeFastCommand(0x0F); }
void DWM3000Class::delayedTX()       { writeFastCommand(0x3); }
void DWM3000Class::standardTX()      { writeFastCommand(0x01); }
void DWM3000Class::standardRX()      { writeFastCommand(0x02); }
void DWM3000Class::TXInstantRX()     { writeFastCommand(0x0C); }

void DWM3000Class::softReset() {
    clearAONConfig();
    write(PMSC_REG, 0x04, 0x1);
    write(PMSC_REG, 0x00, 0x00, 2);
    delay(100);
    write(PMSC_REG, 0x00, 0xFFFF);
    write(PMSC_REG, 0x04, 0x00, 1);
}
void DWM3000Class::hardReset() {
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW);
    delay(10);
    pinMode(RST_PIN, INPUT);
}
void DWM3000Class::clearSystemStatus() { write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF); }
void DWM3000Class::pullLEDHigh(int led) { if (led > 2) return; led_status |= (1 << led); write(0x05, 0x0C, led_status); }
void DWM3000Class::pullLEDLow(int led)  { if (led > 2) return; led_status &= ~((int)1 << led); write(0x05, 0x0C, led_status); }
double DWM3000Class::convertToCM(int u) { return (double)u * PS_UNIT * SPEED_OF_LIGHT; }

void DWM3000Class::calculateTXRXdiff() {
    unsigned long long ping_tx = readTXTimestamp();
    unsigned long long ping_rx = readRXTimestamp();
    long double clk_offset = getClockOffset();
    long double clock_offset = 1.0 + clk_offset;
    long long t_reply = read(RX_BUFFER_0_REG, 0x03);
    if (t_reply == 0) return;
    long long t_round = ping_rx - ping_tx;
    long long t_prop = lround((t_round - lround(t_reply * clock_offset)) / 2);
    long double cm = t_prop * PS_UNIT * SPEED_OF_LIGHT;
    if (cm >= 0) { printDouble(cm, 100, false); Serial.println("cm"); }
}

void DWM3000Class::printRoundTripInformation() {
    Serial.println("\nRound Trip Information:");
    Serial.print("TX: "); Serial.println(readTXTimestamp());
    Serial.print("RX: "); Serial.println(readRXTimestamp());
}

void DWM3000Class::printDouble(double val, unsigned int precision, bool linebreak) {
    Serial.print(int(val)); Serial.print(".");
    unsigned int frac = val >= 0 ? (val - int(val)) * precision : (int(val) - val) * precision;
    if (linebreak) Serial.println(frac, DEC);
    else           Serial.print(frac, DEC);
}

void DWM3000Class::setBit(int r, int s, int shift, bool b) {
    uint8_t tmp = read8bit(r, s);
    if (b) bitSet(tmp, shift); else bitClear(tmp, shift);
    write(r, s, tmp);
}
void DWM3000Class::setBitLow(int r, int s, int shift)  { setBit(r, s, shift, 0); }
void DWM3000Class::setBitHigh(int r, int s, int shift) { setBit(r, s, shift, 1); }

void DWM3000Class::writeFastCommand(int cmd) {
    int header = 0x1 | ((cmd & 0x1F) << 1) | 0x80;
    int arr[] = {header};
    sendBytes(arr, 1, 0);
}

uint32_t DWM3000Class::readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t dataLen, uint32_t rw) {
    uint32_t header = 0;
    if (rw) header |= 0x80;
    header |= (base & 0x1F) << 1;
    if (sub > 0) { header |= 0x40; header <<= 8; header |= (sub & 0x7F) << 2; }
    uint32_t hsz = header > 0xFF ? 2 : 1;
    if (!rw) {
        int ha[hsz];
        if (hsz == 1) ha[0] = header;
        else { ha[0] = (header & 0xFF00) >> 8; ha[1] = header & 0xFF; }
        return sendBytes(ha, hsz, 4);
    } else {
        uint32_t pb = dataLen == 0 ? (data > 0 ? ((countBits(data) + 7) / 8) : 1) : dataLen;
        int payload[hsz + pb];
        if (hsz == 1) payload[0] = header;
        else { payload[0] = (header & 0xFF00) >> 8; payload[1] = header & 0xFF; }
        for (int i = 0; i < pb; i++) payload[hsz + i] = (data >> i * 8) & 0xFF;
        return sendBytes(payload, 2 + pb, 0);
    }
}

uint32_t DWM3000Class::sendBytes(int b[], int lenB, int recLen) {
    digitalWrite(CHIP_SELECT_PIN, LOW);
    for (int i = 0; i < lenB; i++) SPI.transfer(b[i]);
    uint32_t val = 0, tmp;
    for (int i = 0; i < recLen; i++) {
        tmp = SPI.transfer(0x00);
        if (i == 0) val = tmp; else val |= (uint32_t)tmp << 8 * i;
    }
    digitalWrite(CHIP_SELECT_PIN, HIGH);
    return val;
}

void DWM3000Class::clearAONConfig() {
    write(AON_REG, NO_OFFSET, 0x00, 2);
    write(AON_REG, 0x14, 0x00, 1);
    write(AON_REG, 0x04, 0x00, 1);
    write(AON_REG, 0x04, 0x02);
    delay(1);
}

unsigned int DWM3000Class::countBits(unsigned int n) { return (int)log2(n) + 1; }

int DWM3000Class::checkForDevID() {
    int res = read(GEN_CFG_AES_LOW_REG, NO_OFFSET);
    if (res != 0xDECA0302 && res != 0xDECA0312) {
        Serial.println("[ERROR] DEV_ID IS WRONG!");
        return 0;
    }
    return 1;
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(2000);  // Give DWM3000 time to boot

    initializeAnchors();
    Serial.print("System: "); Serial.print(NUM_ANCHORS); Serial.println(" anchors configured");
    for (int i = 0; i < NUM_ANCHORS; i++) {
        Serial.print("  Anchor ID: "); Serial.println(anchors[i].anchor_id);
    }

    connectToWiFi();  // Non-blocking — ranging works even without WiFi

    DWM3000.begin();
    DWM3000.hardReset();
    delay(200);

    if (!DWM3000.checkSPI()) {
        Serial.println("[ERROR] Could not establish SPI Connection to DWM3000!");
        while (1);
    }
    while (!DWM3000.checkForIDLE()) { Serial.println("[ERROR] IDLE1 FAILED"); delay(1000); }
    DWM3000.softReset();
    delay(200);
    if (!DWM3000.checkForIDLE()) { Serial.println("[ERROR] IDLE2 FAILED"); while (1); }

    DWM3000.init();
    DWM3000.setupGPIO();
    DWM3000.setTXAntennaDelay(ANTENNA_DELAY);
    DWM3000.setSenderID(TAG_ID);

    Serial.println("> TAG - Three Anchor Ranging System <");
    Serial.println("> With WiFi Communication <");
    Serial.println("[INFO] Setup is finished.");
    Serial.print("Antenna delay set to: "); Serial.println(DWM3000.getTXAntennaDelay());

    DWM3000.configureAsTX();
    DWM3000.clearSystemStatus();
}

void loop() {
    AnchorData *a   = getCurrentAnchor();
    int         aid = getCurrentAnchorId();

    switch (curr_stage) {

    // ---- STAGE 0: Poll anchor ----
    case 0:
        a->t_roundA = 0;
        a->t_replyA = 0;
        DWM3000.setDestinationID(aid);
        DWM3000.ds_sendFrame(1);           // Poll (stage 1)
        a->tx = DWM3000.readTXTimestamp(); // T1
        curr_stage = 1;
        break;

    // ---- STAGE 1: Await Response from anchor ----
    case 1:
        rx_status = DWM3000.receivedFrameSucc();
        if (rx_status == 1) {
            DWM3000.clearSystemStatus();
            if (DWM3000.ds_isErrorFrame()) {
                Serial.print("[WARNING] Error frame from A"); Serial.println(aid);
                curr_stage = 0;
            } else if (DWM3000.ds_getStage() != 2) {
                Serial.print("[WARNING] Unexpected stage from A"); Serial.print(aid);
                Serial.print(": "); Serial.println(DWM3000.ds_getStage());
                DWM3000.ds_sendErrorFrame();
                curr_stage = 0;
            } else {
                curr_stage = 2;
            }
        } else if (rx_status == 2) {
            DWM3000.clearSystemStatus();
            curr_stage = 0;  // Retry
        }
        break;

    // ---- STAGE 2: Response received — send Final ----
    case 2:
        a->rx = DWM3000.readRXTimestamp();              // T4
        DWM3000.ds_sendFrame(3);                        // Final (stage 3)
        a->t_roundA = a->rx - a->tx;                    // T4 - T1
        a->tx = DWM3000.readTXTimestamp();              // T5
        a->t_replyA = a->tx - a->rx;                    // T5 - T4
        curr_stage = 3;
        break;

    // ---- STAGE 3: Await RT Info from anchor ----
    case 3:
        rx_status = DWM3000.receivedFrameSucc();
        if (rx_status == 1) {
            DWM3000.clearSystemStatus();
            if (DWM3000.ds_isErrorFrame()) {
                Serial.print("[WARNING] Error frame (RT Info) from A"); Serial.println(aid);
                curr_stage = 0;
            } else {
                a->clock_offset = DWM3000.getRawClockOffset();
                curr_stage = 4;
            }
        } else if (rx_status == 2) {
            DWM3000.clearSystemStatus();
            curr_stage = 0;
        }
        break;

    // ---- STAGE 4: Calculate distance ----
    case 4: {
        int ranging_time = DWM3000.ds_processRTInfo(
            (int)a->t_roundA,
            (int)a->t_replyA,
            DWM3000.read(0x12, 0x04),   // t_roundB from anchor packet
            DWM3000.read(0x12, 0x08),   // t_replyB from anchor packet
            a->clock_offset);

        a->distance           = DWM3000.convertToCM(ranging_time);
        a->signal_strength    = DWM3000.getSignalStrength();
        a->fp_signal_strength = DWM3000.getFirstPathSignalStrength();
        updateFilteredDistance(*a);

        printAllDistances();

        // FIX: Send data as soon as ANY anchor has valid filtered data (not all)
        if (anyAnchorHasValidData()) {
            sendDataOverWiFi();
        }

        switchToNextAnchor();
        curr_stage = 0;
        break;
    }

    default:
        Serial.print("Unknown stage ("); Serial.print(curr_stage); Serial.println(") — reset to 0");
        curr_stage = 0;
        break;
    }
}
