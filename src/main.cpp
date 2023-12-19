#include <Arduino.h>
#include <Wire.h>
#include <U8x8lib.h>

#define IN            PA8 // Tim1 chan1
#define FILTER_VAL    5000 // Hz max
#define BUFF_LEN      5 // for median filtering
#define OVERFLOW_VAL  0x10000 // 16 bit overflow val

//#define DBG // disable display to reduce flash using

#ifndef DBG 
U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(U8X8_PIN_NONE); // SCL=PB6, SDA=PB7
char str[8];
#endif

uint32_t freq_buff[BUFF_LEN]; // for median filtering

HardwareTimer *Tim;
uint32_t channel; // will be find automatically

volatile uint32_t FrequencyMeasured, LastPeriodCapture = 0, CurrentCapture;
uint32_t input_freq = 0; // timer freq
volatile uint32_t rolloverCompareCount = 0;

void push_freq(uint32_t freq) {
  static uint8_t i = 0;

  freq_buff[i] = freq;
  if(++i >= BUFF_LEN) {
    i = 0;
  }
}

int compare(const void * num1, const void * num2) { // for qsort()
   if(*(int*)num1 > *(int*)num2)
    return 1;
   else
    return -1;
}

uint32_t pop_freq(void) { // median
  qsort(freq_buff, BUFF_LEN, sizeof(*freq_buff), compare); 
  return freq_buff[BUFF_LEN / 2];
}

void led(bool state) { // true is ON
  digitalWrite(LED_BUILTIN, state?LOW:HIGH);
}

void TIMINPUT_Capture_Rising_IT_callback(void) {
  static uint32_t prev_freq = 0; // prev val of FrequencyMeasured

  CurrentCapture = Tim->getCaptureCompare(channel); // get timestamp

  if (CurrentCapture > LastPeriodCapture) {
    FrequencyMeasured = input_freq / (CurrentCapture - LastPeriodCapture);
  }
  else if (CurrentCapture <= LastPeriodCapture) { // was wrapped
    FrequencyMeasured = input_freq / (OVERFLOW_VAL + CurrentCapture - LastPeriodCapture);
  }

  if (FrequencyMeasured > FILTER_VAL) { // 432000
    return;
  }

  // "Freq changed too fast" filter
  if (FrequencyMeasured > prev_freq) {
    if((FrequencyMeasured - prev_freq) > (FrequencyMeasured / 2)) {
      prev_freq += (prev_freq / 2) + 1;
      FrequencyMeasured = prev_freq;
      //return;
    }
  } else if (FrequencyMeasured < prev_freq) {
      if((prev_freq - FrequencyMeasured) > (FrequencyMeasured / 2)) {
      prev_freq -= (prev_freq / 2) + 1;
      FrequencyMeasured = prev_freq;;
      //return;
    }
  }
  prev_freq = FrequencyMeasured;

  LastPeriodCapture = CurrentCapture; // accept new value
  rolloverCompareCount = 0;
  led(0);

  push_freq(FrequencyMeasured);
}

void Rollover_IT_callback(void) {
  rolloverCompareCount++;
  if (rolloverCompareCount > 1) {
    led(1); // indicate rollover
  }
}



void setup() {
#ifndef DBG 
  u8x8.begin();
  u8x8.setFont(u8x8_font_courB24_3x4_n);
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // led off

  // Timer configure 
  // https://github.com/stm32duino/STM32Examples/blob/main/examples/Peripherals/HardwareTimer/Frequency_Dutycycle_measurement/Frequency_Dutycycle_measurement.ino
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IN), PinMap_PWM);
  channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN), PinMap_PWM));
  Tim = new HardwareTimer(Instance);
  Tim->setMode(channel, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, IN);
  uint32_t PrescalerFactor = 10000;
  Tim->setPrescaleFactor(PrescalerFactor);
  Tim->setOverflow(OVERFLOW_VAL);
  Tim->attachInterrupt(channel, TIMINPUT_Capture_Rising_IT_callback);
  Tim->attachInterrupt(Rollover_IT_callback);
  Tim->resume();
  input_freq = Tim->getTimerClkFreq() / Tim->getPrescaleFactor(); // 7200 for PrescalerFactor = 10000;
  input_freq *= 60; // rev/min instead of rev/sec
}

void loop(void) {
#ifndef DBG 
  sprintf(str, "%d", pop_freq());
  u8x8.clearDisplay();
  //u8x8.drawString(0,0, "     "); // same as clearDisplay()
  u8x8.drawString(0,0, str);
#endif
  delay(500); // display update period
}
