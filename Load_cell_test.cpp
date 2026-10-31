// Load-cell bring-up test - NUCLEO-F411RE + SparkFun Qwiic Scale (NAU7802) + TAL220B 5 kg
//
// Wiring (Qwiic cable -> Nucleo Arduino header):
//   black  GND -> GND
//   red    3V3 -> 3V3          (NOT 5V)
//   blue   SDA -> D14 (PB9)
//   yellow SCL -> D15 (PB8)
//
// Serial monitor: 115200 baud (PlatformIO: "Monitor"). Type a command and press Enter:
//   t          tare (zero) - nothing on the plate
//   c <grams>  calibrate with a known mass on the plate, e.g.  c 100
//   s          toggle streaming of every sample as CSV (ms,raw,grams)
//   r <sps>    set sample rate: 10, 20, 40, 80 or 320
//   i          show settings and measured sample rate
//   h          help

// Single-file version: drop into src/main.cpp of a PlatformIO project with
//   board = nucleo_f411re, framework = arduino, monitor_speed = 115200

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =================== USER SETTINGS ===================
// ---------------- User settings ----------------

// Counts per gram. Run "c <grams>" with a known mass and paste the printed value here.
// Default is the theory value for a 1.0 mV/V, 5 kg cell at gain 128:
//   1e-3 / 5000 g * 2^24 * 128 = 429.5 counts/g  (the TAL220B datasheet gives 1.0 +/- 0.1 mV/V,
//   and the sign depends on which way the cell is loaded, so calibrate it).
#define CAL_COUNTS_PER_GRAM   1068.041f

// Sample rate in samples/s: 10, 20, 40, 80 or 320. Use 80 for quiet weighing tests;
// the airspeed code will need 320 to stream at 200 Hz. Can be changed live with "r <sps>".
#define NAU_SAMPLE_RATE       80

// How often the normal display prints (ms). Each line is the mean of the samples in that window.
#define PRINT_PERIOD_MS       200

// Samples averaged when taring / calibrating.
#define TARE_SAMPLES          80

// =================== NAU7802 DRIVER ===================
// Minimal NAU7802 24-bit ADC driver (SparkFun Qwiic Scale), written from the
// Nuvoton NAU7802 datasheet rev 1.7. Uses the Arduino Wire API.

class NAU7802 {
public:
    static const uint8_t ADDR = 0x2A;          // 7-bit address (0101010)

    // Register addresses (datasheet section 10)
    enum Reg : uint8_t {
        PU_CTRL = 0x00, CTRL1 = 0x01, CTRL2 = 0x02,
        OCAL1_B2 = 0x03, GCAL1_B3 = 0x06,
        I2C_CTRL = 0x11, ADCO_B2 = 0x12, ADC_REG = 0x15,
        PGA_REG = 0x1B, PWR_CTRL = 0x1C, REV_ID = 0x1F
    };

    // PU_CTRL bits
    enum : uint8_t { RR = 0x01, PUD = 0x02, PUA = 0x04, PUR = 0x08, CS = 0x10, CR = 0x20, AVDDS = 0x80 };

    enum Gain : uint8_t { GAIN_1 = 0, GAIN_2, GAIN_4, GAIN_8, GAIN_16, GAIN_32, GAIN_64, GAIN_128 };
    enum Ldo  : uint8_t { LDO_4V5 = 0, LDO_4V2, LDO_3V9, LDO_3V6, LDO_3V3, LDO_3V0, LDO_2V7, LDO_2V4 };

    explicit NAU7802(TwoWire &w = Wire) : wire_(w) {}

    // Full power-up and configuration (datasheet 9.1). Returns false if the chip
    // does not answer, does not power up, or the offset calibration fails.
    bool begin(uint16_t sps = 80, Gain gain = GAIN_128, Ldo ldo = LDO_3V0);

    bool isConnected();
    uint8_t revision();                        // low nibble of REG 0x1F

    bool setSampleRate(uint16_t sps);          // 10, 20, 40, 80, 320; re-calibrates
    uint16_t sampleRate() const { return sps_; }
    bool setGain(Gain g);
    bool setLdo(Ldo l);

    // Internal offset calibration (CALMOD = 00). Blocks up to timeout_ms.
    bool calibrateInternal(uint32_t timeout_ms = 3000);

    bool available();                          // PU_CTRL.CR - a new conversion is ready
    int32_t readRaw();                         // signed 24-bit result, sign-extended
    // Wait for and return the next sample. Returns false on timeout.
    bool waitRaw(int32_t &out, uint32_t timeout_ms = 200);
    // Throw away n conversions (needed after power-up / rate change, datasheet 8.8).
    void discard(uint8_t n);

    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t &val);
    bool setBits(uint8_t reg, uint8_t mask, uint8_t val);   // read-modify-write

private:
    TwoWire &wire_;
    uint16_t sps_ = 10;
};

bool NAU7802::writeReg(uint8_t reg, uint8_t val) {
    wire_.beginTransmission(ADDR);
    wire_.write(reg);
    wire_.write(val);
    return wire_.endTransmission() == 0;
}

bool NAU7802::readReg(uint8_t reg, uint8_t &val) {
    wire_.beginTransmission(ADDR);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;     // repeated start
    if (wire_.requestFrom((uint8_t)ADDR, (uint8_t)1) != 1) return false;
    val = wire_.read();
    return true;
}

bool NAU7802::setBits(uint8_t reg, uint8_t mask, uint8_t val) {
    uint8_t v;
    if (!readReg(reg, v)) return false;
    v = (v & ~mask) | (val & mask);
    return writeReg(reg, v);
}

bool NAU7802::isConnected() {
    wire_.beginTransmission(ADDR);
    return wire_.endTransmission() == 0;
}

uint8_t NAU7802::revision() {
    uint8_t v = 0;
    readReg(REV_ID, v);
    return v & 0x0F;
}

bool NAU7802::begin(uint16_t sps, Gain gain, Ldo ldo) {
    if (!isConnected()) return false;

    // 1-2. Register reset, then leave reset and power up digital + analog
    if (!writeReg(PU_CTRL, RR)) return false;
    delay(1);
    if (!writeReg(PU_CTRL, PUD | PUA)) return false;

    // 3. Wait for PUR (typ. 200 us)
    uint32_t t0 = millis();
    uint8_t pu = 0;
    do {
        if (!readReg(PU_CTRL, pu)) return false;
        if (millis() - t0 > 200) return false;
    } while (!(pu & PUR));

    // 4. Configuration
    //    LDO voltage + gain, then select the internal LDO as AVDD source
    if (!writeReg(CTRL1, (uint8_t)((ldo & 0x07) << 3) | (gain & 0x07))) return false;
    if (!writeReg(PU_CTRL, PUD | PUA | AVDDS)) return false;
    //    REG 0x15 = 0x30: CLK_CHP off (datasheet 9.1 example)
    if (!writeReg(ADC_REG, 0x30)) return false;
    //    PGA_CAP_EN: filter cap across VIN2P/VIN2N (fitted on the Qwiic Scale) - better ENOB on CH1
    if (!setBits(PWR_CTRL, 0x80, 0x80)) return false;

    // 5. Start conversions (CS), then set rate (which also calibrates and flushes)
    if (!setBits(PU_CTRL, CS, CS)) return false;
    delay(10);   // let the LDO settle
    return setSampleRate(sps);
}

bool NAU7802::setGain(Gain g) {
    return setBits(CTRL1, 0x07, g) && calibrateInternal();
}

bool NAU7802::setLdo(Ldo l) {
    return setBits(CTRL1, 0x38, (uint8_t)(l << 3)) && calibrateInternal();
}

bool NAU7802::setSampleRate(uint16_t sps) {
    uint8_t crs;
    switch (sps) {
        case 10:  crs = 0b000; break;
        case 20:  crs = 0b001; break;
        case 40:  crs = 0b010; break;
        case 80:  crs = 0b011; break;
        case 320: crs = 0b111; break;
        default:  return false;
    }
    if (!setBits(CTRL2, 0x70, (uint8_t)(crs << 4))) return false;
    sps_ = sps;
    // Datasheet 8.6: recalibrate after a rate change
    return calibrateInternal();
}

bool NAU7802::calibrateInternal(uint32_t timeout_ms) {
    // CALMOD = 00 (internal offset), then set CALS
    if (!setBits(CTRL2, 0x03, 0x00)) return false;
    if (!setBits(CTRL2, 0x04, 0x04)) return false;
    uint32_t t0 = millis();
    uint8_t c2 = 0x04;
    while (c2 & 0x04) {                      // CALS reads 1 while busy
        if (millis() - t0 > timeout_ms) return false;
        delay(1);
        if (!readReg(CTRL2, c2)) return false;
    }
    if (c2 & 0x08) return false;             // CAL_ERR
    discard(6);                              // flush old data (datasheet 8.8)
    return true;
}

bool NAU7802::available() {
    uint8_t pu = 0;
    return readReg(PU_CTRL, pu) && (pu & CR);
}

int32_t NAU7802::readRaw() {
    // Burst read 3 bytes from 0x12 (datasheet 11.9, option 1)
    wire_.beginTransmission(ADDR);
    wire_.write(ADCO_B2);
    if (wire_.endTransmission(false) != 0) return 0;
    if (wire_.requestFrom((uint8_t)ADDR, (uint8_t)3) != 3) return 0;
    uint32_t v = ((uint32_t)wire_.read() << 16);
    v |= ((uint32_t)wire_.read() << 8);
    v |= (uint32_t)wire_.read();
    // sign-extend 24 -> 32 bits
    if (v & 0x800000UL) v |= 0xFF000000UL;
    return (int32_t)v;
}

bool NAU7802::waitRaw(int32_t &out, uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (!available()) {
        if (millis() - t0 > timeout_ms) return false;
    }
    out = readRaw();
    return true;
}

void NAU7802::discard(uint8_t n) {
    int32_t dummy;
    // the slowest rate is 10 SPS -> allow 150 ms per sample
    for (uint8_t i = 0; i < n; i++) waitRaw(dummy, 150);
}

// =================== TEST PROGRAM ===================
static NAU7802 adc(Wire);

static float   countsPerGram = CAL_COUNTS_PER_GRAM;
static int32_t offsetCounts  = 0;
static bool    streamMode    = false;

// statistics over one print window
static double   winSum = 0, winSumSq = 0;
static uint32_t winN = 0;
static uint32_t winStart = 0;
static bool     winSaturated = false;

// measured sample rate
static uint32_t rateCount = 0, rateStart = 0;
static float    measuredSps = 0;

static const int32_t ADC_MAX = 0x7FFFFF, ADC_MIN = -0x800000;

static float countsToGrams(double counts) { return (float)((counts - offsetCounts) / countsPerGram); }

static void printHelp() {
    Serial.println(F("Commands (type then Enter):"));
    Serial.println(F("  t          tare (zero) - nothing on the plate"));
    Serial.println(F("  c <grams>  calibrate with a known mass, e.g.  c 100"));
    Serial.println(F("  s          toggle streaming every sample as CSV: ms,raw,grams"));
    Serial.println(F("  r <sps>    sample rate 10 | 20 | 40 | 80 | 320"));
    Serial.println(F("  i          info"));
    Serial.println(F("  h          help"));
}

static void printInfo() {
    Serial.println(F("---- info ----"));
    Serial.print(F("NAU7802 revision ID : 0x")); Serial.println(adc.revision(), HEX);
    Serial.print(F("Sample rate set     : ")); Serial.print(adc.sampleRate()); Serial.println(F(" SPS"));
    Serial.print(F("Sample rate measured: ")); Serial.print(measuredSps, 1); Serial.println(F(" SPS"));
    Serial.print(F("Offset (tare)       : ")); Serial.print(offsetCounts); Serial.println(F(" counts"));
    Serial.print(F("Calibration         : ")); Serial.print(countsPerGram, 3); Serial.println(F(" counts/g"));
    Serial.println(F("--------------"));
}

// Average n fresh samples. Returns false if the ADC stops responding.
static bool averageRaw(uint16_t n, double &mean) {
    double sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        int32_t v;
        if (!adc.waitRaw(v, 500)) return false;
        sum += v;
    }
    mean = sum / n;
    return true;
}

static void resetWindow() {
    winSum = winSumSq = 0; winN = 0; winSaturated = false; winStart = millis();
}

static void doTare() {
    Serial.print(F("Taring - keep the plate still... "));
    double m;
    if (!averageRaw(TARE_SAMPLES, m)) { Serial.println(F("FAILED (no data from NAU7802)")); return; }
    offsetCounts = (int32_t)lround(m);
    Serial.print(F("done. offset = ")); Serial.println(offsetCounts);
    resetWindow();
}

static void doCalibrate(float grams) {
    if (grams <= 0) { Serial.println(F("Usage: c <grams>   e.g.  c 100   (tare first with nothing on)")); return; }
    Serial.print(F("Calibrating with ")); Serial.print(grams, 2); Serial.print(F(" g... "));
    double m;
    if (!averageRaw(TARE_SAMPLES, m)) { Serial.println(F("FAILED (no data from NAU7802)")); return; }
    double delta = m - offsetCounts;
    if (fabs(delta) < 50) {
        Serial.println(F("FAILED: almost no change from the tare value."));
        Serial.println(F("  Is the mass on the plate, and is the load acting along the cell's sensing axis?"));
        return;
    }
    countsPerGram = (float)(delta / grams);
    Serial.println(F("done."));
    Serial.print(F("  counts/g = ")); Serial.println(countsPerGram, 3);
    Serial.println(F("  To keep it, replace the CAL_COUNTS_PER_GRAM line at the top of main.cpp with:"));
    Serial.print(F("  #define CAL_COUNTS_PER_GRAM   ")); Serial.print(countsPerGram, 3); Serial.println('f');
    float theory = 2147.5f;
    float ratio = fabs(countsPerGram) / theory;
    if (ratio < 0.8f || ratio > 1.2f) {
        Serial.print(F("  WARNING: ")); Serial.print(ratio * 100, 0);
        Serial.println(F("% of the theoretical 429.5 counts/g - check for rubbing/friction or a wrong mass."));
    }
    resetWindow();
}

static void doRate(int sps) {
    Serial.print(F("Setting ")); Serial.print(sps); Serial.print(F(" SPS... "));
    if (adc.setSampleRate((uint16_t)sps)) {
        Serial.println(F("ok (internal offset recalibrated). Re-tare recommended: t"));
    } else {
        Serial.println(F("FAILED - allowed values: 10 20 40 80 320"));
    }
    rateCount = 0; rateStart = millis();
    resetWindow();
}

static void handleCommand(char *line) {
    while (*line == ' ') line++;
    if (!*line) return;
    char cmd = tolower(line[0]);
    float arg = atof(line + 1);
    switch (cmd) {
        case 't': doTare(); break;
        case 'c': doCalibrate(arg); break;
        case 's':
            streamMode = !streamMode;
            Serial.println(streamMode ? F("Streaming every sample: ms,raw,grams") : F("Back to averaged display"));
            resetWindow();
            break;
        case 'r': doRate((int)arg); break;
        case 'i': printInfo(); break;
        case 'h': case '?': printHelp(); break;
        default: Serial.println(F("Unknown command - type h for help"));
    }
}

static void pollSerial() {
    static char buf[32];
    static uint8_t len = 0;
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\r' || ch == '\n') {
            if (len) { buf[len] = 0; handleCommand(buf); len = 0; }
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = ch;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println(F("=== Load-cell test: NAU7802 + TAL220B (5 kg) ==="));

    Wire.begin();            // D14 = SDA (PB9), D15 = SCL (PB8)
    Wire.setClock(400000);

    while (!adc.begin(NAU_SAMPLE_RATE, NAU7802::GAIN_128, NAU7802::LDO_3V0)) {
        Serial.println(F("NAU7802 not found / failed to start. Check:"));
        Serial.println(F("  - Qwiic Scale powered from 3V3, GND connected"));
        Serial.println(F("  - SDA -> D14, SCL -> D15 (not swapped)"));
        Serial.println(F("  Retrying in 2 s..."));
        delay(2000);
    }
    Serial.print(F("NAU7802 ok, rev 0x")); Serial.print(adc.revision(), HEX);
    Serial.print(F(", gain 128, AVDD 3.0 V, ")); Serial.print(adc.sampleRate()); Serial.println(F(" SPS"));

    doTare();
    printHelp();
    Serial.println(F("Output:  weight [g] | force [N] | raw mean | noise sd [g] | samples"));
    rateStart = millis();
    resetWindow();
}

void loop() {
    pollSerial();

    if (adc.available()) {
        int32_t raw = adc.readRaw();
        rateCount++;
        if (raw >= ADC_MAX || raw <= ADC_MIN) winSaturated = true;

        if (streamMode) {
            Serial.print(millis()); Serial.print(',');
            Serial.print(raw); Serial.print(',');
            Serial.println(countsToGrams(raw), 3);
        } else {
            winSum += raw;
            winSumSq += (double)raw * raw;
            winN++;
        }
    }

    uint32_t now = millis();
    if (now - rateStart >= 2000) {
        measuredSps = rateCount * 1000.0f / (now - rateStart);
        rateCount = 0; rateStart = now;
    }

    if (!streamMode && now - winStart >= PRINT_PERIOD_MS) {
        if (winN == 0) {
            Serial.println(F("No samples - NAU7802 stopped responding?"));
        } else {
            double mean = winSum / winN;
            double var = winSumSq / winN - mean * mean;
            float sdGrams = (float)(sqrt(var > 0 ? var : 0) / fabs(countsPerGram));
            float g = countsToGrams(mean);
            Serial.print(g, 2);                       Serial.print(F(" g   "));
            Serial.print(g * 9.80665e-3f, 4);         Serial.print(F(" N   raw "));
            Serial.print((int32_t)lround(mean));      Serial.print(F("   sd "));
            Serial.print(sdGrams, 2);                 Serial.print(F(" g   n="));
            Serial.print(winN);
            if (winSaturated) Serial.print(F("   ** ADC SATURATED **"));
            Serial.println();
        }
        resetWindow();
    }
}
