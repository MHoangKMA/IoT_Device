#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <HTTPClient.h>

//----------------
TaskHandle_t Task1;
TaskHandle_t Task2;

// Địa chỉ I2C và kích thước LCD 20x4
LiquidCrystal_I2C lcd(0x27, 20, 4);

// WiFi Configuration
const char *ssid = "P5B";
const char *password = "Nambkav123";

// Google Apps Script Configuration
String GOOGLE_SCRIPT_ID = "AKfycbysCOU2GAlgQdi1_ZNWO4hjvfmYsgpMdEpZjq0hm9tiN0mj4vZ1F6xhMaM0Ze-OmNPeDA";

// Interval Configuration
const int sendInterval = 10000;  // Send data every 5 seconds
unsigned long previousMillis = 0;

// Cấu hình chân cho nút bấm và cảm biến
const int button1Pin = 35;  // Nút 1 - Đo nhiệt độ DHT22
const int button2Pin = 32;  // Nút 2 - Đo nhiệt độ LM35
const int button3Pin = 25;  // Nút 3 - Đo dữ liệu từ MQ2
const int button4Pin = 26;  // Nút 3 - Đo dữ liệu từ MQ2
const int button5Pin = 27;  // Nút 3 - Đo dữ liệu từ MQ2

const int dhtPin = 15;   // Chân cho DHT22
const int lm35Pin = 33;  // Chân cho LM35
const int mq2Pin = 34;   // Chân cho MQ2

//-----------------------
int mq2Value;
float temperatureDHT11;
float temperatureLM35;
float humidityDHT11;

// Lưu trạng thái cũ của dữ liệu
float lastTemperatureDHT11 = 0.0;
float lastHumidityDHT11 = 0.0;
float lastTemperatureLM35 = 0.0;
int lastMQ2Value = 0;

//-----------------------
bool isConnectedWifi = false;

// Cấu hình DHT
#define DHTTYPE DHT22
DHT dht(dhtPin, DHTTYPE);

// Biến lưu trạng thái
int lastButton1State = HIGH;
int lastButton2State = HIGH;
int lastButton3State = HIGH;
int lastButton4State = HIGH;
int lastButton5State = HIGH;

int button1State = 0;
int button2State = 0;
int button3State = 0;
int button4State = 0;
int button5State = 0;

// Chân kết nối với 3 đèn LED
const int greenLedPin = 13;   // Đèn xanh
const int yellowLedPin = 12;  // Đèn vàng
const int redLedPin = 14;     // Đèn đỏ

const int buzzerPin = 2;  // Chân cho buzzer

bool isDHTActive = false;   // Trạng thái của cảm biến DHT
bool isLM35Active = false;  // Trạng thái của cảm biến LM35
bool isMQ2Active = false;   // Trạng thái của cảm biến MQ2

void setup() {
  // Khởi tạo LCD
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
    delay(500);
    Serial.print(".");
    retryCount++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    lcd.print("WiFi connected!");
    isConnectedWifi = true;
  } else {
    Serial.println("WiFi connection failed.");
    lcd.print("WiFi failed.");
    isConnectedWifi = false;
  }

  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("Choice channel:");

  // Khởi tạo nút bấm
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);  // Khởi tạo nút cho MQ2
  pinMode(button4Pin, INPUT_PULLUP);  // Khởi tạo nút cho MQ2
  pinMode(button5Pin, INPUT_PULLUP);  // Khởi tạo nút cho MQ2

  // Khởi tạo đèn LED
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  pinMode(buzzerPin, OUTPUT);  // Khởi tạo buzzer

  // Khởi tạo DHT22
  dht.begin();


  // Chạy Task1 trên Core 0
  xTaskCreatePinnedToCore(
    Task1code,  // Hàm xử lý task
    "Task1",    // Tên task
    8192,       // Kích thước stack
    NULL,       // Tham số truyền vào task
    1,          // Mức ưu tiên task
    &Task1,     // Handle của task (không dùng)
    0           // Core chạy task (Core 0)
  );

  // Chạy Task2 trên Core 1
  xTaskCreatePinnedToCore(
    Task2code,  // Hàm xử lý task
    "Task2",    // Tên task
    8192,       // Kích thước stack
    NULL,       // Tham số truyền vào task
    1,          // Mức ưu tiên task
    &Task2,     // Handle của task (không dùng)
    1           // Core chạy task (Core 1)
  );
}

// Hàm chạy trên Core 0
void Task1code(void *pvParameters) {
  for (;;) {
    // Đọc trạng thái của nút bấm
    button1State = digitalRead(button1Pin);
    button2State = digitalRead(button2Pin);
    button3State = digitalRead(button3Pin);  // Đọc nút cho MQ2
    button4State = digitalRead(button4Pin);  // Đọc nút cho MQ2

    if (button4State == LOW && lastButton4State == HIGH) {
      isDHTActive = true;  // Kích hoạt cảm biến DHT
      isMQ2Active = true;  // Tắt cảm biến MQ2
      isLM35Active = true;
      lcd.clear();
    }

    // Kiểm tra nếu trạng thái nút 1 thay đổi từ HIGH sang LOW (nhấn nút)
    if (button1State == LOW && lastButton1State == HIGH) {
      isDHTActive = true;    // Kích hoạt cảm biến DHT
      isLM35Active = false;  // Tắt cảm biến LM35
      isMQ2Active = true;    // Tắt cảm biến MQ2
      lcd.clear();
      lcd.print("Using DHT22..");
    }

    // Kiểm tra nếu trạng thái nút 2 thay đổi từ HIGH sang LOW (nhấn nút)
    if (button2State == LOW && lastButton2State == HIGH) {
      isDHTActive = false;  // Tắt cảm biến DHT
      isLM35Active = true;  // Kích hoạt cảm biến LM35
      isMQ2Active = true;   // Tắt cảm biến MQ2
      lcd.clear();
      // lcd.print("Using LM35...");
    }

    // Kiểm tra nếu trạng thái nút 3 thay đổi từ HIGH sang LOW (nhấn nút)
    if (button3State == LOW && lastButton3State == HIGH) {
      isDHTActive = false;   // Tắt cảm biến DHT
      isLM35Active = false;  // Tắt cảm biến LM35
      isMQ2Active = true;    // Kích hoạt cảm biến MQ2
      lcd.clear();
      lcd.print("Using MQ2...");
    }

    // Cập nhật hiển thị dựa trên trạng thái cảm biến
    if (isDHTActive == true) {
      temperatureDHT11 = measureAndDisplayDHT11();
      updateLeds(temperatureDHT11);  // Cập nhật đèn LED theo nhiệt độ
    }

    if (isLM35Active == true) {
      temperatureLM35 = measureAndDisplayLM35();
      updateLeds(temperatureLM35);  // Cập nhật đèn LED theo nhiệt độ
    }


    if (isMQ2Active == true) {
      measureAndDisplayMQ2();
    }

    // Cập nhật trạng thái nút bấm lần trước
    lastButton1State = button1State;
    lastButton2State = button2State;
    lastButton3State = button3State;  // Cập nhật trạng thái nút cho MQ2
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


// Hàm chạy trên Core 1
void Task2code(void *pvParameters) {
  for (;;) {
    unsigned long currentMillis = millis();

    // Chỉ gửi dữ liệu nếu đủ thời gian đã trôi qua
    if (currentMillis - previousMillis >= sendInterval) {
      previousMillis = currentMillis;

      // Kiểm tra sự thay đổi của các giá trị
      bool isTemperatureDHT11Changed = abs(temperatureDHT11 - lastTemperatureDHT11) >= 0.5;
      bool isHumidityDHT11Changed = abs(humidityDHT11 - lastHumidityDHT11) >= 0.5;
      bool isTemperatureLM35Changed = abs(temperatureLM35 - lastTemperatureLM35) >= 0.5;
      bool isMQ2ValueChanged = (mq2Value != lastMQ2Value);

      // Chỉ gửi dữ liệu nếu có sự thay đổi
      if (isConnectedWifi == true && (isTemperatureDHT11Changed || isHumidityDHT11Changed || isTemperatureLM35Changed || isMQ2ValueChanged)) {
        String params = "sts=write&tempDHT11=" + String(temperatureDHT11) + "&humdDHT11=" + String(humidityDHT11) + "&tempLM35=" + String(temperatureLM35) + "&mq2Value=" + String(mq2Value);

        write_to_google_sheet(params);

        // Cập nhật giá trị cuối cùng
        lastTemperatureDHT11 = temperatureDHT11;
        lastHumidityDHT11 = humidityDHT11;
        lastTemperatureLM35 = temperatureLM35;
        lastMQ2Value = mq2Value;
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Nghỉ giữa các vòng lặp để giảm tải CPU
  }
}


void write_to_google_sheet(String params) {
  HTTPClient http;
  String url = "https://script.google.com/macros/s/" + GOOGLE_SCRIPT_ID + "/exec?" + params;

  Serial.println("Posting data to Google Sheet...");
  Serial.print("URL: ");
  Serial.println(url);

  http.begin(url.c_str());
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int httpCode = http.GET();

  Serial.print("HTTP Status Code: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("Response: " + payload);
  } else {
    Serial.println("Error: Unable to connect to server.");
  }

  http.end();
}


void loop() {
}

// Hàm đo nhiệt độ từ DHT22 và hiển thị lên LCD
float measureAndDisplayDHT11() {
  temperatureDHT11 = dht.readTemperature();  // Đo nhiệt độ từ DHT22
  humidityDHT11 = dht.readHumidity();
  if (isnan(temperatureDHT11) || isnan(humidityDHT11)) {
    Serial.println("Error reading DHT22!");
    lcd.setCursor(0, 1);
    lcd.print("Error reading DHT!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return -1;
  } else {
    lcd.setCursor(0, 1);
    lcd.print("DHT11 Temp: ");
    lcd.print(temperatureDHT11);
    lcd.print(" C  ");
    lcd.setCursor(0, 2);
    lcd.print("Humidity: ");
    lcd.print(humidityDHT11);
    lcd.print("%  ");
    checkTemperLevel(temperatureDHT11);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return temperatureDHT11;  // Trả về nhiệt độ
  }
}


float measureAndDisplayLM352() {
  int adcValue2 = analogRead(lm35Pin);

  if (adcValue2 <= 0) {
    Serial.println("Error reading LM35!");
    lcd.setCursor(0, 1);
    lcd.print("Error reading LM35!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return -1;
  }

  float voltage2 = adcValue2 * (3.3 / 4095.0);  // Chuyển đổi giá trị ADC sang điện áp (3.3V là mức tham chiếu)
  temperatureLM35 = voltage2 * 100;
  checkTemperLevel(temperatureLM35);
  vTaskDelay(100 / portTICK_PERIOD_MS);  // LM35: 10 mV = 1°C, nên nhân với 100
  return temperatureLM35;                // Trả về nhiệt độ
}

// Hàm đo nhiệt độ từ LM35 và hiển thị lên LCD
float measureAndDisplayLM35() {
  int adcValue = analogRead(lm35Pin);

  if (adcValue <= 0) {
    Serial.println("Error reading LM35!");
    lcd.setCursor(0, 1);
    lcd.print("Error reading LM35!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return -1;
  }

  float voltage = adcValue * (3.3 / 4095.0);  // Chuyển đổi giá trị ADC sang điện áp (3.3V là mức tham chiếu)
  temperatureLM35 = voltage * 100;            // LM35: 10 mV = 1°C, nên nhân với 100

  lcd.setCursor(0, 0);
  lcd.print("LM35 Temp: ");
  lcd.print(temperatureLM35);
  lcd.print(" C  ");
  checkTemperLevel(temperatureLM35);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  return temperatureLM35;  // Trả về nhiệt độ
}

// Hàm đo dữ liệu từ MQ2 và hiển thị lên LCD
void measureAndDisplayMQ2() {
  mq2Value = analogRead(mq2Pin);  // Đọc giá trị ADC từ MQ2
  lcd.setCursor(0, 3);
  lcd.print("MQ2 Value: ");
  if (mq2Value >= 0 && mq2Value <= 100) mq2Value = 0;
  lcd.print(mq2Value);  // Hiển thị giá trị đọc được từ MQ2
  lcd.print("     ");   // Xóa dấu hiệu còn lại
  checkGasLevel(mq2Value);
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

// Hàm điều khiển LED dựa trên nhiệt độ
void updateLeds(float temperature) {
  // Nếu nhiệt độ trong khoảng 0 - 50, sáng đèn xanh
  if (temperature >= 0 && temperature <= 50) {
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  }
  // Nếu nhiệt độ trong khoảng 50 - 100, sáng đèn vàng
  else if (temperature > 50 && temperature <= 100) {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
  }
  // Nếu nhiệt độ trong khoảng 100 - 150, sáng đèn đỏ
  else if (temperature > 100 && temperature <= 150) {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  }
  // Nếu nhiệt độ ngoài khoảng cho phép, tắt hết đèn
  else {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  }
}

void checkTemperLevel(float temperature) {
  if (temperature > 50 && temperature <= 100) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temperature is hot");
    buzzerOn();
  }
  // Nếu nhiệt độ trong khoảng 100 - 150, sáng đèn đỏ
  else if (temperature > 100 && temperature <= 150) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temperature is very hot");
    buzzerOn();
  }
  // Nếu nhiệt độ ngoài khoảng cho phép, tắt hết đèn
  else {
    buzzerOff();
  }
}

// Hàm kiểm tra giá trị MQ2 và điều khiển buzzer
void checkGasLevel(int mq2Value) {
  if (mq2Value > 100) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gas Over Threshold");
    lcd.setCursor(0, 1);
    lcd.print("Warning ON");

    buzzerOn();  // Kích hoạt buzzer nếu giá trị gas vượt ngưỡng
  } else {
    buzzerOff();  // Tắt buzzer nếu giá trị gas an toàn
  }
}

// Hàm kích hoạt buzzer
void buzzerOn() {
  digitalWrite(buzzerPin, HIGH);  // Bật buzzer
  delay(800);
  digitalWrite(buzzerPin, LOW);
  delay(250);
}

// Hàm tắt buzzer
void buzzerOff() {
  digitalWrite(buzzerPin, LOW);
}
