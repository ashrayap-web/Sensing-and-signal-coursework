// Encoder test: Broadcom AS22-M530-5B12 on NUCLEO-F411RE
//
// AS22-M530-5B12 = 2-channel differential (RS422) output, no index,
// 1000 CPR, 6 mm shaft, 5 V supply.
//
// Wiring (encoder JST-SH 8-pin connector -> Nucleo):
//   Pin 1 GND  -> GND
//   Pin 4 Vcc  -> 5V
//   Pin 2 A+   -> PA6 (Arduino D12)  TIM3_CH1
//   Pin 5 B+   -> PA7 (Arduino D11)  TIM3_CH2
//   Pin 3 A-, Pin 6 B- -> leave unconnected (or feed an RS422 receiver)
//
// TIM3 runs in hardware quadrature (x4) mode, so 1000 CPR = 4000 counts/rev.
// Serial monitor commands: 'z' = zero position.

#include <Arduino.h>

static constexpr int32_t CPR = 1000;
static constexpr int32_t COUNTS_PER_REV = CPR * 4;
static constexpr uint32_t PRINT_INTERVAL_MS = 100;

static int32_t position = 0;     // extended 32-bit count (TIM3 is only 16-bit)
static uint16_t lastCnt = 0;

static void encoderInit() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  (void)RCC->APB1ENR;

  // PA6, PA7 -> alternate function mode, AF2 (TIM3), no pull-ups
  GPIOA->MODER = (GPIOA->MODER & ~((3U << 12) | (3U << 14))) | (2U << 12) | (2U << 14);
  GPIOA->PUPDR &= ~((3U << 12) | (3U << 14));
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFU << 24) | (0xFU << 28))) | (2U << 24) | (2U << 28);

  TIM3->CR1 = 0;
  TIM3->SMCR = 3;                         // SMS = 011: encoder mode 3 (count both edges of A and B)
  TIM3->CCMR1 = (1U << 0) | (3U << 4)     // CC1S = TI1, IC1F = 8-sample filter
              | (1U << 8) | (3U << 12);   // CC2S = TI2, IC2F = 8-sample filter
  TIM3->CCER = 0;                         // non-inverted; set TIM_CCER_CC1P to reverse direction
  TIM3->ARR = 0xFFFF;
  TIM3->CNT = 0;
  TIM3->CR1 = TIM_CR1_CEN;
}

static void encoderUpdate() {
  uint16_t cnt = TIM3->CNT;
  position += (int16_t)(cnt - lastCnt);   // handles 16-bit wraparound
  lastCnt = cnt;
}

void setup() {
  Serial.begin(115200);
  encoderInit();
  Serial.println("AS22-M530-5B12 encoder test (1000 CPR, x4 = 4000 counts/rev)");
  Serial.println("Send 'z' to zero.");
}

void loop() {
  static uint32_t lastPrint = 0;
  static int32_t lastPos = 0;

  if (Serial.available() && Serial.read() == 'z') {
    encoderUpdate();
    position = 0;
    lastPos = 0;
    Serial.println("-- zeroed --");
  }

  uint32_t now = millis();
  if (now - lastPrint < PRINT_INTERVAL_MS) return;
  float dt = (now - lastPrint) / 1000.0f;
  lastPrint = now;

  encoderUpdate();
  int32_t delta = position - lastPos;
  lastPos = position;

  float revs = (float)position / COUNTS_PER_REV;
  int32_t angleCounts = position % COUNTS_PER_REV;
  if (angleCounts < 0) angleCounts += COUNTS_PER_REV;
  float angleDeg = angleCounts * 360.0f / COUNTS_PER_REV;
  float rpm = (delta / (float)COUNTS_PER_REV) / dt * 60.0f;

  // Raw A/B pin states (useful for checking wiring by turning slowly)
  uint32_t idr = GPIOA->IDR;
  int a = (idr >> 6) & 1;
  int b = (idr >> 7) & 1;

  Serial.printf("count=%ld  revs=%.3f  angle=%.2f deg  rpm=%.1f  A=%d B=%d\r\n",
                (long)position, revs, angleDeg, rpm, a, b);
}
