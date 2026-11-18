# Số hóa tín hiệu nhiệt độ qua ADC ESP32 và phân tích phổ Fourier

Bài tập lớn môn Tín hiệu và Hệ thống. Cảm biến LM35 cho ra điện áp analog (10 mV/°C). ESP32 lấy mẫu điện áp này bằng ADC 12 bit rồi tự tính FFT. Phổ Fourier được vẽ trên màn hình OLED và trên trang web. Ngoài ra ESP32 dùng DAC phát sóng sin đã biết tần số và đưa ngược vào ADC, để kiểm tra xem phổ tính ra có đúng không.

### 1. Phần cứng & nối dây
| Linh kiện | Chân ESP32 |
|---|---|
| Cảm biến LM35 (nguồn 5 V từ chân VIN) | OUT → GPIO32 (ADC1_CH4) |
| Dây kiểm chứng DAC → ADC | GPIO25 (DAC1) → GPIO33 (ADC1_CH5) |
| OLED SSD1306 128×64 I2C | SDA → GPIO21, SCL → GPIO22 |
| LED đỏ / LED xanh | GPIO2 / GPIO4 |
| Nút BOOT có sẵn trên board | GPIO0 |

---

### 2. Thu thập dữ liệu (firmware `SoHoaNhietDo_FFT.ino`)
* **Lấy mẫu đều nhau**: lõi 0 của ESP32 chạy một task 1 ms (`vTaskDelayUntil`), vừa cập nhật DAC vừa đọc ADC đúng chu kỳ Ts. Màn hình có đang vẽ thì khoảng cách giữa các mẫu vẫn không đổi.
* **Lọc nhiễu điện lưới 50 Hz**: mỗi mẫu LM35 là trung bình của 40 lần đọc trải đều trong 20 ms, tức đúng một chu kỳ điện lưới. Độ lệch chuẩn giảm từ 3.52 °C xuống còn 0.30 °C.
* **Truyền dữ liệu giữa hai lõi**: mẫu được gửi qua hàng đợi FreeRTOS sang lõi 1. Lõi 1 tính FFT, vẽ OLED và gửi dữ liệu qua Serial (115200 baud, dạng JSON).

---

### 3. Xử lý tín hiệu số: biến đổi Fourier
* **FFT radix-2 tự cài đặt**, N = 64 điểm, không dùng thư viện. Các bước: đảo bit, rồi 6 tầng cánh bướm, hệ số xoay W_N^k được tính sẵn.
* **Khử thành phần DC** (trừ giá trị trung bình) trước khi tính FFT.
* **Cửa sổ Hann hoặc cửa sổ chữ nhật** để so sánh hiện tượng rò rỉ phổ.
* **Phổ biên độ một phía** |X(f)|, chuẩn hóa về đơn vị gốc (°C hoặc mV).
* **Nội suy parabol** trên log biên độ của 3 bin quanh đỉnh, để ước lượng tần số nằm giữa hai bin.
* Tính sẵn **tần số lý thuyết sau khi bị aliasing**: |f0 − fs·round(f0/fs)|, rồi so với tần số đo được để ra % sai số.

---

### 4. Các chế độ đo
| Chế độ | Tín hiệu | fs | Ý nghĩa |
|---|---|---|---|
| 0. LM35 | nhiệt độ thật | 2 Hz | đo nhiệt độ phòng |
| 1. SIN 2.5 Hz | DAC phát sin | 20 Hz | f0 trùng đúng bin k = 8 |
| 2. SIN 4.0 Hz | DAC phát sin | 20 Hz | f0 lệch bin → thấy rò rỉ phổ |
| 3. ALIAS | DAC phát sin 8 Hz | 10 Hz | fs < 2·f0 → chồng phổ về 2 Hz |

* **Nút BOOT**: nhấn ngắn để đổi chế độ, nhấn giữ để đổi cửa sổ Hann / chữ nhật.
* **Lệnh Serial**: `S0`–`S3` chọn chế độ · `WH` / `WR` chọn cửa sổ · `C<Ts_ms>,<f0>` tạo thí nghiệm DAC tùy chỉnh · `?` xem cấu hình hiện tại.
* **LED**: ở chế độ LM35, đèn đỏ sáng khi nhiệt độ trên 30 °C, đèn xanh sáng khi từ 30 °C trở xuống. Ở các chế độ DAC, đèn xanh nghĩa là kiểm chứng đạt (sai số dưới 2 %).

---

### 5. Kết quả đo thật trên ESP32
| Thí nghiệm | Hann: f đo / sai số | Chữ nhật: f đo / sai số |
|---|---|---|
| sin 1.0 Hz | 1.004 Hz / 0.44 % | 0.966 Hz / 5.23 % |
| sin 2.5 Hz | 2.500 Hz / 0.03 % | 2.489 Hz / 0.44 % |
| sin 4.0 Hz | 3.996 Hz / 0.11 % | 4.038 Hz / 0.96 % |
| 8 Hz @ fs 10 Hz (lý thuyết 2 Hz) | 1.998 Hz / 0.11 % | 2.022 Hz / 1.10 % |

---

### 6. Trang web (`index.html`)
* **Web Serial API**: Chrome hoặc Edge kết nối thẳng với cổng COM của ESP32 qua cáp USB.
* **Chế độ mô phỏng**: chạy đúng thuật toán trên, không cần cắm phần cứng.
* **Hai đồ thị**: tín hiệu đã số hóa x[n] theo thời gian và phổ biên độ |X(f)|. Có đánh dấu bin đỉnh và vạch tần số lý thuyết.
* **Thông số lấy mẫu**: fs, Nyquist fs/2, N, độ phân giải Δf = fs/N, độ dài cửa sổ N/fs.
* **Bảng kết quả thí nghiệm**: ghi từng lần đo, xuất ra file CSV.

---

### 7. Cách chạy
1. Mở `SoHoaNhietDo_FFT.ino` bằng Arduino IDE, cài lõi **esp32** và thư viện **Adafruit SSD1306** + **Adafruit GFX**, rồi nạp vào board **ESP32 Dev Module**.
2. Bấm đúp `chay_web.bat` (cần có Python). Trình duyệt sẽ mở `http://localhost:8000`.
3. Bấm **Kết nối ESP32** và chọn cổng COM của board. Nếu không có phần cứng thì bấm **Chạy mô phỏng**.
