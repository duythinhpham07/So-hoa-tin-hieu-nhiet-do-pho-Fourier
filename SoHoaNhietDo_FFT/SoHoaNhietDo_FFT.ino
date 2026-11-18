/*
 * BTL Tín hiệu và Hệ thống
 * Đề tài: Số hóa tín hiệu nhiệt độ analog qua ADC của ESP32
 *         và phân tích phổ Fourier hiển thị trên OLED
 * Nhóm thực hiện:
 *   - Trần Xuân Duy      - B25DCTV013
 *   - Ngô Trọng Cường    - B25DCTV011
 *   - Phạm Duy Thịnh     - B25DCTV072
 *   - Nguyễn Trọng Khánh - B25DCTV037
 *
 * Phần cứng
 *   - ESP32 DevKit
 *   - Cảm biến LM35 (10 mV/°C)   -> GPIO32 (ADC1_CH4)
 *   - Dây nối DAC -> ADC          : GPIO25 (DAC1) -> GPIO33 (ADC1_CH5)
 *   - OLED SSD1306 128x64 I2C     : SDA = GPIO21, SCL = GPIO22
 *   - LED đỏ GPIO2, LED xanh GPIO4
 *   - Nút BOOT (GPIO0): nhấn ngắn = đổi chế độ, nhấn giữ = đổi cửa sổ Hann/Chữ nhật
 *
 * Các chế độ
 *   0. LM35      : đo nhiệt độ thật, fs = 2 Hz
 *   1. SIN 2.5 Hz: DAC phát sin, fs = 20 Hz, f0 trùng đúng bin FFT (k = 8)
 *   2. SIN 4.0 Hz: DAC phát sin, fs = 20 Hz, f0 lệch bin (k = 12.8) -> rò rỉ phổ
 *   3. ALIAS     : DAC phát sin 8 Hz, fs = 10 Hz < 2 f0 -> chồng phổ về 2 Hz
 *
 * Kiến trúc
 *   - Core 0: task thu thập chạy mỗi 1 ms (vTaskDelayUntil), cập nhật DAC và lấy mẫu
 *             ADC đúng chu kỳ Ts, nên khoảng cách giữa các mẫu luôn đều nhau.
 *   - Core 1: loop() nhận mẫu qua hàng đợi, tính FFT 64 điểm, vẽ OLED, gửi dữ liệu Serial.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ======================= Cấu hình phần cứng =======================
constexpr int PIN_LM35      = 32;
constexpr int PIN_TEST_IN   = 33;
constexpr int PIN_DAC       = 25;
constexpr int PIN_LED_RED   = 2;
constexpr int PIN_LED_GREEN = 4;
constexpr int PIN_BUTTON    = 0;
constexpr int PIN_SDA       = 21;
constexpr int PIN_SCL       = 22;
constexpr uint8_t OLED_ADDR = 0x3C;

constexpr float TEMP_ALARM_C = 30.0f;  // ngưỡng bật LED đỏ ở chế độ LM35
constexpr float DAC_MID      = 128.0f; // mức giữa của DAC 8 bit (~1.65 V)
constexpr float DAC_AMP      = 90.0f;  // biên độ sin (mã DAC) ~ 1.16 V

static_assert(configTICK_RATE_HZ == 1000, "Can tick FreeRTOS = 1 ms");

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// ======================= Cấu hình chế độ đo =======================
struct Config {
  bool     useDac;    // true: đo sóng sin từ DAC; false: đo LM35
  uint16_t periodMs;  // chu kỳ lấy mẫu Ts (ms), fs = 1000 / Ts
  float    f0;        // tần số sin DAC phát ra (Hz)
};

const Config PRESETS[] = {
  {false, 500, 0.0f},  // 0: LM35, fs = 2 Hz
  {true,   50, 2.5f},  // 1: sin trùng bin
  {true,   50, 4.0f},  // 2: sin lệch bin
  {true,  100, 8.0f},  // 3: aliasing
};
constexpr int NUM_PRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

// Cấu hình dùng chung giữa 2 core, bảo vệ bằng spinlock
portMUX_TYPE cfgMux = portMUX_INITIALIZER_UNLOCKED;
Config  sharedCfg   = PRESETS[0];
uint32_t sharedGen  = 0;  // tăng mỗi lần đổi cấu hình để bỏ mẫu cũ

struct Sample {
  float    value;  // °C (LM35) hoặc mV (DAC)
  uint32_t gen;
};
QueueHandle_t sampleQueue;

// ======================= Task thu thập (core 0) =======================
float readMilliVolts(int pin, int count) {
  uint32_t sum = 0;
  for (int i = 0; i < count; i++) sum += analogReadMilliVolts(pin);
  return sum / (float)count;
}

// Lọc trung bình trong đúng 20 ms = 1 chu kỳ điện lưới 50 Hz:
// bộ lọc trung bình độ dài T có điểm không tại f = 1/T, 2/T, ... nên triệt nhiễu 50 Hz và các hài.
float readLm35MilliVolts() {
  const int COUNT = 40;
  uint32_t sum = 0;
  for (int i = 0; i < COUNT; i++) {
    sum += analogReadMilliVolts(PIN_LM35);
    delayMicroseconds(500);  // 40 x 0.5 ms = 20 ms
  }
  return sum / (float)COUNT;
}

void acquisitionTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t myGen = UINT32_MAX;
  Config cfg = PRESETS[0];
  uint32_t msInScene = 0;
  float phase = 0.0f;

  for (;;) {
    vTaskDelayUntil(&lastWake, 1);  // nhịp đều 1 ms

    portENTER_CRITICAL(&cfgMux);
    bool changed = (sharedGen != myGen);
    if (changed) { myGen = sharedGen; cfg = sharedCfg; }
    portEXIT_CRITICAL(&cfgMux);

    if (changed) {
      msInScene = 0;
      phase = 0.0f;
      if (!cfg.useDac) dacWrite(PIN_DAC, 0);
    }

    // 1) Phát sin bằng bộ cộng dồn pha (DAC cập nhật 1000 lần/giây)
    if (cfg.useDac) {
      float code = DAC_MID + DAC_AMP * sinf(phase);
      dacWrite(PIN_DAC, (uint8_t)lroundf(code));
      phase += 2.0f * PI * cfg.f0 * 0.001f;
      if (phase >= 2.0f * PI) phase -= 2.0f * PI;
    }

    // 2) Lấy mẫu đúng chu kỳ Ts
    if (msInScene % cfg.periodMs == 0) {
      Sample s;
      s.gen = myGen;
      if (cfg.useDac) {
        s.value = readMilliVolts(PIN_TEST_IN, 4);
      } else {
        s.value = readLm35MilliVolts() / 10.0f;  // LM35: 10 mV = 1 °C
      }
      xQueueSend(sampleQueue, &s, 0);
    }
    msInScene++;
  }
}

// ======================= FFT radix-2 (tự cài đặt) =======================
constexpr int N     = 64;
constexpr int LOG2N = 6;
constexpr int HALF  = N / 2;

uint8_t bitRev[N];
float   twCos[HALF], twSin[HALF];  // W_N^k = cos(2πk/N) - j sin(2πk/N)
float   hann[N];

void initFFT() {
  for (int i = 0; i < N; i++) {
    int r = 0;
    for (int b = 0; b < LOG2N; b++) if (i & (1 << b)) r |= 1 << (LOG2N - 1 - b);
    bitRev[i] = r;
    hann[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / N));  // Hann tuần hoàn
  }
  for (int k = 0; k < HALF; k++) {
    twCos[k] = cosf(2.0f * PI * k / N);
    twSin[k] = -sinf(2.0f * PI * k / N);
  }
}

void fft(float *re, float *im) {
  for (int i = 0; i < N; i++) {
    int j = bitRev[i];
    if (i < j) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (int size = 2; size <= N; size <<= 1) {
    int half = size >> 1;
    int step = N / size;
    for (int start = 0; start < N; start += size) {
      for (int k = 0; k < half; k++) {
        float wr = twCos[k * step], wi = twSin[k * step];
        int a = start + k, b = a + half;
        float tr = re[b] * wr - im[b] * wi;
        float ti = re[b] * wi + im[b] * wr;
        re[b] = re[a] - tr;  im[b] = im[a] - ti;
        re[a] += tr;         im[a] += ti;
      }
    }
  }
}

// ======================= Trạng thái xử lý (core 1) =======================
int      presetIdx = 0;        // -1 = cấu hình tùy chỉnh qua Serial
Config   cfg       = PRESETS[0];
uint32_t gen       = 0;
bool     useHann   = true;

float    ring[N];
int      ringHead  = 0;
int      filled    = 0;
uint32_t sampleNo  = 0;
float    lastValue = 0;

float mag[HALF + 1];  // phổ biên độ một phía, mag[0] đã khử DC
float dcValue  = 0;
float fPeak    = 0;   // tần số đỉnh đã nội suy
float aPeak    = 0;   // biên độ tại bin đỉnh
int   kPeak    = 0;
float fExpect  = 0;   // tần số lý thuyết (đã tính aliasing)
float errPct   = 0;
float yScale   = 1;
bool  hasPeak  = false;

float fsHz()   { return 1000.0f / cfg.periodMs; }
float binHz()  { return fsHz() / N; }
const char *unit() { return cfg.useDac ? "mV" : "C"; }

// Tần số biểu kiến sau lấy mẫu: |f0 - fs * round(f0 / fs)|
float aliasFreq(float f0, float fs) { return fabsf(f0 - fs * roundf(f0 / fs)); }

void announceConfig();
void analyze();

// Đổi cửa sổ: giữ nguyên các mẫu đã có, chỉ tính lại phổ
void setWindow(bool hannOn) {
  useHann = hannOn;
  announceConfig();
  if (filled >= N) analyze();
}

void applyConfig(const Config &c, int idx) {
  cfg = c;
  presetIdx = idx;
  filled = 0; ringHead = 0; sampleNo = 0;
  hasPeak = false;
  yScale = cfg.useDac ? 100.0f : 0.5f;
  fExpect = cfg.useDac ? aliasFreq(cfg.f0, fsHz()) : 0.0f;

  portENTER_CRITICAL(&cfgMux);
  sharedCfg = cfg;
  gen = ++sharedGen;
  portEXIT_CRITICAL(&cfgMux);

  announceConfig();
}

void announceConfig() {
  Serial.printf("{\"t\":\"cfg\",\"sc\":%d,\"src\":\"%s\",\"fs\":%.4g,\"f0\":%.4g,\"fe\":%.4g,"
                "\"N\":%d,\"u\":\"%s\",\"w\":\"%s\"}\n",
                presetIdx, cfg.useDac ? "dac" : "lm35", fsHz(), cfg.f0, fExpect,
                N, unit(), useHann ? "hann" : "rect");
}

void analyze() {
  static float re[N], im[N];

  // Sắp xếp mẫu cũ -> mới, tính thành phần DC (trung bình)
  float x[N], sum = 0;
  for (int i = 0; i < N; i++) { x[i] = ring[(ringHead + i) % N]; sum += x[i]; }
  dcValue = sum / N;

  // Khử DC rồi nhân cửa sổ
  float wSum = 0;
  for (int i = 0; i < N; i++) {
    float w = useHann ? hann[i] : 1.0f;
    re[i] = (x[i] - dcValue) * w;
    im[i] = 0;
    wSum += w;
  }
  fft(re, im);

  // Phổ biên độ một phía, chuẩn hóa về đơn vị gốc (°C hoặc mV)
  float raw[HALF + 1];
  for (int k = 0; k <= HALF; k++) {
    raw[k] = sqrtf(re[k] * re[k] + im[k] * im[k]);
    mag[k] = raw[k] * ((k == 0 || k == HALF) ? 1.0f : 2.0f) / wSum;
  }
  mag[0] = 0;

  // Tìm bin đỉnh (bỏ qua k = 0)
  kPeak = 1;
  for (int k = 2; k <= HALF; k++) if (mag[k] > mag[kPeak]) kPeak = k;
  aPeak = mag[kPeak];

  // Nội suy parabol trên thang log để ước lượng tần số nằm giữa 2 bin
  float delta = 0;
  if (kPeak >= 1 && kPeak <= HALF - 1) {
    const float eps = 1e-9f;
    float a = logf(raw[kPeak - 1] + eps), b = logf(raw[kPeak] + eps), c = logf(raw[kPeak + 1] + eps);
    float den = a - 2 * b + c;
    if (fabsf(den) > 1e-12f) delta = constrain(0.5f * (a - c) / den, -0.5f, 0.5f);
  }
  fPeak = (kPeak + delta) * binHz();

  // Ngưỡng nhiễu: LM35 ~ 0.15 °C (gần cỡ lượng tử ADC sau lấy trung bình), DAC ~ 20 mV
  hasPeak = aPeak > (cfg.useDac ? 20.0f : 0.15f);
  errPct = (cfg.useDac && fExpect > 0) ? fabsf(fPeak - fExpect) / fExpect * 100.0f : 0;

  // Thang trục Y tự co giãn (tăng ngay, giảm từ từ)
  float target = max(aPeak * 1.25f, cfg.useDac ? 100.0f : 0.5f);
  yScale = (target > yScale) ? target : yScale * 0.95f + target * 0.05f;
}

// ======================= Hiển thị OLED =======================
void fmtCompact(char *buf, size_t n, float v) {
  if (v >= 1000) snprintf(buf, n, "%.1fk", v / 1000);
  else if (v >= 10) snprintf(buf, n, "%.0f", v);
  else snprintf(buf, n, "%.1f", v);
}

void drawOled() {
  char line[32];
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // Dòng 1: chế độ + cửa sổ
  if (!cfg.useDac) snprintf(line, sizeof(line), "LM35 fs=%.4gHz", fsHz());
  else if (cfg.f0 > fsHz() / 2) snprintf(line, sizeof(line), "ALIAS %.4gHz fs%.4g", cfg.f0, fsHz());
  else snprintf(line, sizeof(line), "SIN %.4gHz fs%.4g", cfg.f0, fsHz());
  oled.setCursor(0, 0);
  oled.print(line);
  oled.setCursor(110, 0);
  oled.print(useHann ? "HN" : "CN");

  // Dòng 2: giá trị đo + tần số đỉnh
  oled.setCursor(0, 10);
  if (!cfg.useDac) {
    snprintf(line, sizeof(line), "T=%.1fC", lastValue);
    oled.print(line);
    oled.setCursor(52, 10);
    if (filled < N) oled.print("fp=...");
    else if (!hasPeak) oled.print("fp=-- (nhieu)");
    else { snprintf(line, sizeof(line), "fp=%.3fHz", fPeak); oled.print(line); }
  } else {
    if (filled < N) snprintf(line, sizeof(line), "v=%.0fmV", lastValue);
    else snprintf(line, sizeof(line), "fp=%.2fHz e=%.1f%%", fPeak, errPct);
    oled.print(line);
  }
  oled.drawFastHLine(0, 19, 128, SSD1306_WHITE);

  // Đang gom đủ N mẫu
  if (filled < N) {
    snprintf(line, sizeof(line), "Lay mau %d/%d", filled, N);
    oled.setCursor(22, 30);
    oled.print(line);
    oled.drawRect(14, 42, 100, 8, SSD1306_WHITE);
    oled.fillRect(16, 44, 96 * filled / N, 4, SSD1306_WHITE);
    oled.display();
    return;
  }

  // Khung đồ thị phổ: bin k = 1..32, mỗi bin rộng 3 px
  const int X0 = 22, YT = 22, YB = 54, H = YB - YT;
  oled.drawFastVLine(X0 - 1, YT, H + 1, SSD1306_WHITE);
  oled.drawFastHLine(X0 - 1, YB, 128 - X0 + 1, SSD1306_WHITE);

  char lab[12];
  fmtCompact(lab, sizeof(lab), yScale);
  oled.setCursor(0, YT);
  oled.print(lab);
  oled.setCursor(0, YB - 7);
  oled.print(unit());

  for (int k = 1; k <= HALF; k++) {
    int x = X0 + (k - 1) * 3;
    int h = (int)(mag[k] / yScale * H);
    if (h > H) h = H;
    if (h > 0) oled.fillRect(x, YB - h, 2, h, SSD1306_WHITE);
    if (k == kPeak && hasPeak) oled.fillTriangle(x - 1, YB - h - 5, x + 2, YB - h - 5, x + 0, YB - h - 2, SSD1306_WHITE);
  }

  // Vạch chấm: tần số lý thuyết (chế độ DAC)
  if (cfg.useDac && fExpect > 0) {
    int x = X0 + (int)lroundf((fExpect / binHz() - 1) * 3);
    for (int y = YT; y < YB; y += 3) oled.drawPixel(x, y, SSD1306_WHITE);
  }

  // Nhãn trục tần số: 0, fs/4, fs/2
  oled.setCursor(X0 - 6, 57);
  oled.print("0");
  snprintf(lab, sizeof(lab), "%.3g", fsHz() / 4);
  oled.setCursor(X0 + 45 - 3 * strlen(lab), 57);
  oled.print(lab);
  snprintf(lab, sizeof(lab), "%.3gHz", fsHz() / 2);
  oled.setCursor(128 - 6 * strlen(lab), 57);
  oled.print(lab);

  oled.display();
}

// ======================= LED, nút nhấn, Serial =======================
void updateLeds() {
  bool red = false, green = false;
  if (!cfg.useDac) {
    red = lastValue > TEMP_ALARM_C;
    green = !red;
  } else if (filled >= N) {
    green = hasPeak && errPct < 2.0f;  // kiểm chứng đạt
    red = !green;
  }
  digitalWrite(PIN_LED_RED, red);
  digitalWrite(PIN_LED_GREEN, green);
}

void handleButton() {
  static bool wasDown = false;
  static uint32_t downAt = 0;
  static bool longDone = false;
  bool down = digitalRead(PIN_BUTTON) == LOW;
  uint32_t now = millis();

  if (down && !wasDown) { downAt = now; longDone = false; }
  if (down && !longDone && now - downAt > 700) {   // nhấn giữ: đổi cửa sổ
    setWindow(!useHann);
    longDone = true;
  }
  if (!down && wasDown && !longDone && now - downAt > 30) {  // nhấn ngắn: đổi chế độ
    int next = (presetIdx < 0) ? 0 : (presetIdx + 1) % NUM_PRESETS;
    applyConfig(PRESETS[next], next);
  }
  wasDown = down;
}

// Lệnh Serial: S<n> chọn chế độ | W đổi cửa sổ | WH / WR chọn cửa sổ | C<Ts_ms>,<f0> tùy chỉnh | ? xem cấu hình
void handleCommand(const char *cmd) {
  if (cmd[0] == 'S') {
    int idx = atoi(cmd + 1);
    if (idx >= 0 && idx < NUM_PRESETS) applyConfig(PRESETS[idx], idx);
  } else if (cmd[0] == 'W') {
    if (cmd[1] == 'H') setWindow(true);
    else if (cmd[1] == 'R') setWindow(false);
    else setWindow(!useHann);
  } else if (cmd[0] == 'C') {
    int ts = 0; float f0 = 0;
    if (sscanf(cmd + 1, "%d,%f", &ts, &f0) == 2 && ts >= 10 && ts <= 1000 && f0 > 0 && f0 <= 50) {
      applyConfig(Config{true, (uint16_t)ts, f0}, -1);
    } else {
      Serial.println("{\"t\":\"err\",\"msg\":\"C<Ts_ms 10..1000>,<f0 0..50>\"}");
    }
  } else if (cmd[0] == '?') {
    announceConfig();
  }
}

void handleSerial() {
  static char buf[32];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; handleCommand(buf); len = 0; }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void sendSpectrum() {
  Serial.printf("{\"t\":\"f\",\"dc\":%.4g,\"fp\":%.4g,\"a\":%.4g,\"ok\":%d", dcValue, fPeak, aPeak, hasPeak);
  if (cfg.useDac) Serial.printf(",\"fe\":%.4g,\"e\":%.3g", fExpect, errPct);
  Serial.print(",\"m\":[");
  for (int k = 1; k <= HALF; k++) {
    Serial.printf(k == 1 ? "%.4g" : ",%.4g", mag[k]);
  }
  Serial.println("]}");
}

// ======================= setup / loop =======================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // dải đo ~0..3.1 V

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("{\"t\":\"err\",\"msg\":\"Khong tim thay OLED\"}");
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("So hoa tin hieu nhiet");
  oled.println("ADC ESP32 + pho FFT");
  oled.println();
  oled.println("Tran Xuan Duy");
  oled.println("Ngo Trong Cuong");
  oled.println("Pham Duy Thinh");
  oled.println("Nguyen Trong Khanh");
  oled.display();

  initFFT();
  sampleQueue = xQueueCreate(32, sizeof(Sample));
  xTaskCreatePinnedToCore(acquisitionTask, "acq", 4096, nullptr, 3, nullptr, 0);
  delay(1500);
  applyConfig(PRESETS[0], 0);
}

void loop() {
  handleButton();
  handleSerial();

  bool fresh = false;
  Sample s;
  while (xQueueReceive(sampleQueue, &s, 0) == pdTRUE) {
    if (s.gen != gen) continue;  // mẫu của cấu hình cũ
    ring[ringHead] = s.value;
    ringHead = (ringHead + 1) % N;
    if (filled < N) filled++;
    lastValue = s.value;
    sampleNo++;
    fresh = true;
    Serial.printf("{\"t\":\"x\",\"n\":%lu,\"v\":%.4g}\n", (unsigned long)sampleNo, s.value);
  }

  if (fresh) {
    if (filled >= N) analyze();
    updateLeds();
  }

  static uint32_t lastDraw = 0, lastSpec = 0;
  uint32_t now = millis();
  if (fresh && now - lastDraw >= 100) {   // tối đa 10 khung hình/giây
    lastDraw = now;
    drawOled();
  }
  if (fresh && filled >= N && now - lastSpec >= 250) {
    lastSpec = now;
    sendSpectrum();
  }
}
