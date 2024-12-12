#include <Wire.h>              /* Library for I2C communication */
#include <LiquidCrystal_I2C.h> /* Library for controlling LCD with I2C */
#include <DHT.h>               /* Library for interfacing with DHT sensors */
#include <WiFi.h>              /* Library for WiFi connectivity */
#include <HTTPClient.h>        /* Library for handling HTTP requests */

/* Define gas threshold for detecting dangerous levels */
#define GAS_THRESHOLD 3500

/* Define timer interval for periodic updates (in milliseconds) */
#define TIMER_UPDATE 10000

/* Define minimum and maximum temperature thresholds */
#define THRESHOLD_TEMP_MIN 0
#define THRESHOLD_TEMP_AVG1 50  /* Threshold for average temperature level 1 */
#define THRESHOLD_TEMP_AVG2 100 /* Threshold for average temperature level 2 */
#define THRESHOLD_TEMP_MAX 150  /* Maximum temperature threshold */

/* Define the update value for adjusting readings */
#define VALUE_UPDATE 0.5

/* Define maximum attempts to connect to WiFi */
#define WIFI_CONNECT_COUNT 20

/* Task handles for managing FreeRTOS tasks */
TaskHandle_t Task1; /* Handle for Task 1 */
TaskHandle_t Task2; /* Handle for Task 2 */

/* Define I2C address and dimensions for 20x4 LCD */
LiquidCrystal_I2C lcd(0x27, 20, 4); /* LCD object with I2C address 0x27 and size 20x4 */

/*WiFi Configuration*/
const char *ssid = "P5B";            /* WiFi SSID for network connection */
const char *password = "Nambkav123"; /* WiFi password for network connection */

/* Google Apps Script Configuration */
/* Script ID for connecting to Google Apps Script */
String GOOGLE_SCRIPT_ID = "AKfycbysCOU2GAlgQdi1_ZNWO4hjvfmYsgpMdEpZjq0hm9tiN0mj4vZ1F6xhMaM0Ze-OmNPeDA";

const int sendInterval = TIMER_UPDATE; /* Interval for sending data (10 seconds defined by TIMER_UPDATE) */
unsigned long previousMillis = 0U;     /* Variable to track the last recorded time in milliseconds */

/* Pin configuration for buttons and sensors */
const int button1Pin = 35; /* Button 1 - Measure temperature using DHT22 */
const int button2Pin = 32; /* Button 2 - Measure temperature using LM35 */
const int button3Pin = 25; /* Button 3 - Read data from MQ2 */
const int button4Pin = 26; /* Button 4 - Read data from MQ2 */
const int button5Pin = 27; /* Button 5 - Read data from MQ2 */

const int dhtPin = 15;  /* Pin for DHT22 sensor */
const int lm35Pin = 33; /* Pin for LM35 sensor */
const int mq2Pin = 34;  /* Pin for MQ2 sensor */


/* Pins connected to three LEDs */
const int greenLedPin = 13;  /* Pin for green LED */
const int yellowLedPin = 12; /* Pin for yellow LED */
const int redLedPin = 14;    /* Pin for red LED */

const int buzzerPin = 2; /* Pin for the buzzer */

/* Variables to store sensor data */
int mq2Value;           /* Value read from MQ2 sensor */
float temperatureDHT11; /* Temperature value from DHT11 sensor */
float temperatureLM35;  /* Temperature value from LM35 sensor */
float humidityDHT11;    /* Humidity value from DHT11 sensor */

/* Variables to store previous state of data */
float lastTemperatureDHT11 = 0.0; /* Previous temperature value from DHT11 */
float lastHumidityDHT11 = 0.0;    /* Previous humidity value from DHT11 */
float lastTemperatureLM35 = 0.0;  /* Previous temperature value from LM35 */
int lastMQ2Value = 0;             /* Previous value read from MQ2 sensor */

/* Flags to track system and sensor states */
bool isConnectedWifi = false; /* WiFi connection status */
bool isDHTActive = false;     /* Status of the DHT sensor */
bool isLM35Active = false;    /* Status of the LM35 sensor */
bool isMQ2Active = false;     /* Status of the MQ2 sensor */

/* DHT Sensor configuration */
#define DHTTYPE DHT22     /* Define the type of DHT sensor (DHT22) */
DHT dht(dhtPin, DHTTYPE); /* Initialize DHT sensor with pin and type */

/* Variables to store button states */
int lastButton1State = HIGH; /* Last state of Button 1 */
int lastButton2State = HIGH; /* Last state of Button 2 */
int lastButton3State = HIGH; /* Last state of Button 3 */
int lastButton4State = HIGH; /* Last state of Button 4 */
int lastButton5State = HIGH; /* Last state of Button 5 */

int button1State = 0; /* Current state of Button 1 */
int button2State = 0; /* Current state of Button 2 */
int button3State = 0; /* Current state of Button 3 */
int button4State = 0; /* Current state of Button 4 */
int button5State = 0; /* Current state of Button 5 */



void setup() {
  Serial.begin(115200); /* Begin serial communication at 115200 baud rate */
  lcd.init();           /* Initialize the LCD */
  lcd.backlight();      /* Turn on the LCD backlight */
  lcd.setCursor(0, 0);  /* Set the cursor to the top left corner of the LCD */

  WiFi.mode(WIFI_STA);        /* Set WiFi mode to Station (STA) */
  WiFi.begin(ssid, password); /* Begin WiFi connection using predefined SSID and password */
  int retryCount = 0;         /* Initialize retry counter */

  /* Attempt to connect to WiFi until successful or retry count exceeds the limit */
  while (WiFi.status() != WL_CONNECTED && retryCount < WIFI_CONNECT_COUNT) {
    delay(500);        /* Wait for 500 milliseconds before retrying */
    Serial.print("."); /* Print dot for each retry attempt */
    retryCount++;      /* Increment retry counter */
  }

  /* Check if WiFi is connected */
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!"); /* Print success message */
    lcd.print("WiFi connected!");      /* Display success message on LCD */
    isConnectedWifi = true;            /* Set WiFi connection status to true */
  } else {
    Serial.println("WiFi connection failed."); /* Print failure message */
    lcd.print("WiFi failed.");                 /* Display failure message on LCD */
    isConnectedWifi = false;                   /* Set WiFi connection status to false */
  }

  lcd.clear();         /* Clear the LCD screen */
  lcd.setCursor(0, 0); /* Set the cursor to the top-left corner of the LCD */

  lcd.print("Choice channel:"); /* Display "Choice channel:" on the LCD */

  pinMode(button1Pin, INPUT_PULLUP); /* Set Button 1 pin as input with internal pull-up resistor */
  pinMode(button2Pin, INPUT_PULLUP); /* Set Button 2 pin as input with internal pull-up resistor */
  pinMode(button3Pin, INPUT_PULLUP); /* Set Button 3 pin as input with internal pull-up resistor (for MQ2) */
  pinMode(button4Pin, INPUT_PULLUP); /* Set Button 4 pin as input with internal pull-up resistor (for MQ2) */
  pinMode(button5Pin, INPUT_PULLUP); /* Set Button 5 pin as input with internal pull-up resistor (for MQ2) */

  pinMode(greenLedPin, OUTPUT);  /* Set Green LED pin as output */
  pinMode(yellowLedPin, OUTPUT); /* Set Yellow LED pin as output */
  pinMode(redLedPin, OUTPUT);    /* Set Red LED pin as output */
  pinMode(buzzerPin, OUTPUT);    /* Set Buzzer pin as output */

  dht.begin(); /* Begin the DHT sensor (DHT22) */


  xTaskCreatePinnedToCore(
    Task1code, /* Task function to be executed */
    "Task1",   /* Name of the task */
    8192,      /* Stack size for the task */
    NULL,      /* Parameter passed to the task (not used) */
    1,         /* Task priority level */
    &Task1,    /* Task handle (not used) */
    0          /* Core on which to run the task (Core 0) */
  );


  xTaskCreatePinnedToCore(
    Task2code, /* Pointer to the function that implements the task logic */
    "Task2",   /* Descriptive name of the task for debugging purposes */
    8192,      /* Size of the stack allocated for the task (in bytes) */
    NULL,      /* Parameter passed to the task; NULL means no parameters */
    1,         /* Task priority; higher numbers mean higher priority */
    &Task2,    /* Task handle, used for managing or referencing the task */
    1          /* Core on which the task should run; Core 1 is specified */
  );
}

void Task1code(void *pvParameters) {
  /* Infinite loop to continuously check button states and sensor activation */
  for (;;) {
    /* Read the state of each button */
    button1State = digitalRead(button1Pin);
    button2State = digitalRead(button2Pin);
    button3State = digitalRead(button3Pin); /* Read button for MQ2 */
    button4State = digitalRead(button4Pin); /* Read button for activating all sensors */

    /* If button 4 is pressed (change from HIGH to LOW), activate all sensors */
    if (button4State == LOW && lastButton4State == HIGH) {
      isDHTActive = true; /* Activate the DHT sensor */
      isMQ2Active = true; /* Deactivate MQ2 sensor */
      isLM35Active = true;
      lcd.clear(); /* Clear the display */
    }

    /* If button 1 is pressed, activate the DHT sensor and deactivate the LM35 and MQ2 sensors */
    if (button1State == LOW && lastButton1State == HIGH) {
      isDHTActive = true;         /* Activate the DHT sensor */
      isLM35Active = false;       /* Deactivate the LM35 sensor */
      isMQ2Active = true;         /* Deactivate the MQ2 sensor */
      lcd.clear();                /* Clear the display */
      lcd.print("Using DHT22.."); /* Display DHT sensor is in use */
    }

    /* If button 2 is pressed, activate the LM35 sensor and deactivate the DHT and MQ2 sensors */
    if (button2State == LOW && lastButton2State == HIGH) {
      isDHTActive = false; /* Deactivate the DHT sensor */
      isLM35Active = true; /* Activate the LM35 sensor */
      isMQ2Active = true;  /* Deactivate the MQ2 sensor */
      lcd.clear();         /* Clear the display */
      /* lcd.print("Using LM35...");  /* Display LM35 sensor is in use */
    }

    /* If button 3 is pressed, activate the MQ2 sensor and deactivate the DHT and LM35 sensors */
    if (button3State == LOW && lastButton3State == HIGH) {
      isDHTActive = false;       /* Deactivate the DHT sensor */
      isLM35Active = false;      /* Deactivate the LM35 sensor */
      isMQ2Active = true;        /* Activate the MQ2 sensor */
      lcd.clear();               /* Clear the display */
      lcd.print("Using MQ2..."); /* Display MQ2 sensor is in use */
    }

    /* Update the display and LED indicators based on the active sensor */
    if (isDHTActive == true) {
      temperatureDHT11 = measureAndDisplayDHT11(); /* Measure and display DHT temperature */
      updateLeds(temperatureDHT11);                /* Update LED indicators based on temperature */
    }

    if (isLM35Active == true) {
      temperatureLM35 = measureAndDisplayLM35(); /* Measure and display LM35 temperature */
      updateLeds(temperatureLM35);               /* Update LED indicators based on temperature */
    }

    if (isMQ2Active == true) {
      measureAndDisplayMQ2(); /* Measure and display MQ2 gas level */
    }

    /* Update the last button states to detect the next press */
    lastButton1State = button1State;
    lastButton2State = button2State;
    lastButton3State = button3State;      /* Update button state for MQ2 */
    vTaskDelay(100 / portTICK_PERIOD_MS); /* Delay to reduce CPU load and allow other tasks to run */
  }
}



void Task2code(void *pvParameters) {
  /* Infinite loop to continuously check and send data */
  for (;;) {
    /* Get the current time in milliseconds */
    unsigned long currentMillis = millis();

    /* Only send data if the specified interval has passed */
    if (currentMillis - previousMillis >= sendInterval) {
      previousMillis = currentMillis;

      /* Check if there has been a significant change in the values */
      bool isTemperatureDHT11Changed = abs(temperatureDHT11 - lastTemperatureDHT11) >= VALUE_UPDATE;
      bool isHumidityDHT11Changed = abs(humidityDHT11 - lastHumidityDHT11) >= VALUE_UPDATE;
      bool isTemperatureLM35Changed = abs(temperatureLM35 - lastTemperatureLM35) >= VALUE_UPDATE;
      bool isMQ2ValueChanged = (mq2Value != lastMQ2Value);

      /* Only send data if there is a change and the WiFi is connected */
      if (isConnectedWifi == true && (isTemperatureDHT11Changed || isHumidityDHT11Changed || isTemperatureLM35Changed || isMQ2ValueChanged)) {
        /* Construct the query parameters to send to Google Sheets */
        String params = "sts=write&tempDHT11=" + String(temperatureDHT11) + "&humdDHT11=" + String(humidityDHT11) + "&tempLM35=" + String(temperatureLM35) + "&mq2Value=" + String(mq2Value);

        /* Call the function to send data to Google Sheets */
        write_to_google_sheet(params);

        /* Update the last known values */
        lastTemperatureDHT11 = temperatureDHT11;
        lastHumidityDHT11 = humidityDHT11;
        lastTemperatureLM35 = temperatureLM35;
        lastMQ2Value = mq2Value;
      }
    }

    /* Delay between iterations to reduce CPU load */
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}



void write_to_google_sheet(String params) {
  /* Create an HTTPClient object to handle the request */
  HTTPClient http;

  /* Construct the URL to send data to the Google Sheets script using the given parameters */
  String url = "https://script.google.com/macros/s/" + GOOGLE_SCRIPT_ID + "/exec?" + params;

  /* Print a message to indicate that the data is being posted to Google Sheets */
  Serial.println("Posting data to Google Sheet...");
  Serial.print("URL: ");
  Serial.println(url);

  /* Initialize the HTTP request with the constructed URL */
  http.begin(url.c_str());

  /* Set HTTP client to strictly follow redirects */
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  /* Send a GET request to the URL and store the response code */
  int httpCode = http.GET();

  /* Print the HTTP status code returned from the server */
  Serial.print("HTTP Status Code: ");
  Serial.println(httpCode);

  /* If the HTTP request was successful (code > 0), read and print the response */
  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("Response: " + payload);
  } else {
    /* If the HTTP request failed, print an error message */
    Serial.println("Error: Unable to connect to server.");
  }

  /* End the HTTP request */
  http.end();
}



void loop() {
}

float measureAndDisplayDHT11() {
  /* Measure temperature from the DHT22 sensor */
  temperatureDHT11 = dht.readTemperature();
  /* Measure humidity from the DHT22 sensor */
  humidityDHT11 = dht.readHumidity();

  /* Check if the readings are invalid (NaN values) */
  if (isnan(temperatureDHT11) || isnan(humidityDHT11)) {
    Serial.println("Error reading DHT22!");
    /* Display an error message on the LCD */
    lcd.setCursor(0, 1);
    lcd.print("Error reading DHT!");
    /* Delay to allow the error message to remain visible */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return -1; /* Return an error indicator */
  } else {
    /* Display the temperature on the LCD */
    lcd.setCursor(0, 1);
    lcd.print("DHT11 Temp: ");
    lcd.print(temperatureDHT11);
    lcd.print(" C  ");

    /* Display the humidity on the LCD */
    lcd.setCursor(0, 2);
    lcd.print("Humidity: ");
    lcd.print(humidityDHT11);
    lcd.print("%  ");

    /* Check the temperature level and take appropriate action */
    checkTemperLevel(temperatureDHT11);

    /* Delay the task for 100 milliseconds, considering the FreeRTOS tick period */
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /* Return the measured temperature value */
    return temperatureDHT11;
  }
}



float measureAndDisplayLM352() {
  /* Read the ADC value from the second LM35 sensor */
  int adcValue2 = analogRead(lm35Pin);

  /* If the ADC value is invalid (less than or equal to zero), display an error message */
  if (adcValue2 <= 0) {
    Serial.println("Error reading LM35!");
    lcd.setCursor(0, 1);
    lcd.print("Error reading LM35!");
    vTaskDelay(1000 / portTICK_PERIOD_MS); /* Delay to allow the error message to be visible */
    return -1;                             /* Return an error indicator */
  }

  /* Convert the ADC value to voltage (3.3V is the reference voltage) */
  float voltage2 = adcValue2 * (3.3 / 4095.0);

  /* Convert the voltage to temperature in Celsius using LM35 characteristics 
     (10 mV corresponds to 1°C, so multiply by 100) */
  temperatureLM35 = voltage2 * 100;

  /* Check the temperature level and trigger actions based on the defined thresholds */
  checkTemperLevel(temperatureLM35);

  /* Delay the task for 100 milliseconds, considering the FreeRTOS tick period */
  vTaskDelay(100 / portTICK_PERIOD_MS);

  /* Return the measured temperature value */
  return temperatureLM35;
}


float measureAndDisplayLM35() {
  /* Read the ADC value from the LM35 sensor */
  int adcValue = analogRead(lm35Pin);

  /* If the ADC value is invalid (less than or equal to zero), display an error message */
  if (adcValue <= 0) {
    Serial.println("Error reading LM35!");
    lcd.setCursor(0, 1);
    lcd.print("Error reading LM35!");
    vTaskDelay(1000 / portTICK_PERIOD_MS); /* Delay to allow the message to be visible */
    return -1;                             /* Return an error indicator */
  }

  /* Convert the ADC value to voltage (3.3V is the reference voltage) */
  float voltage = adcValue * (3.3 / 4095.0);

  /* Convert the voltage to temperature in Celsius using LM35 characteristics 
     (10 mV corresponds to 1°C, so multiply by 100) */
  temperatureLM35 = voltage * 100;

  /* Display the temperature on the LCD */
  lcd.setCursor(0, 0);
  lcd.print("LM35 Temp: ");
  lcd.print(temperatureLM35);
  lcd.print(" C  ");

  /* Check the temperature level and take appropriate action */
  checkTemperLevel(temperatureLM35);

  /* Delay the task for 100 milliseconds, considering the FreeRTOS tick period */
  vTaskDelay(100 / portTICK_PERIOD_MS);

  /* Return the measured temperature value */
  return temperatureLM35;
}


void measureAndDisplayMQ2() {
  /* Read the ADC value from the MQ2 sensor */
  mq2Value = analogRead(mq2Pin);

  /* Set the LCD cursor to the fourth row and the first column, then print the label */
  lcd.setCursor(0, 3);
  lcd.print("Gas Value: ");

  /* Check if the MQ2 value is within the safe range */
  if (mq2Value >= 0 && mq2Value <= GAS_THRESHOLD) {
    /* Clear any previous value at the cursor position and reset the gas value to zero */
    lcd.setCursor(11, 3);
    lcd.print(" ");
    mq2Value = 0;
    lcd.print(mq2Value);
  } else {
    /* If the gas value exceeds the safe range, display it directly */
    lcd.print(mq2Value);
  }

  /* Call the function to check the gas level and take appropriate action */
  checkGasLevel(mq2Value);

  /* Delay the task for 500 milliseconds, considering the FreeRTOS tick period */
  vTaskDelay(500 / portTICK_PERIOD_MS);
}



void updateLeds(float temperature) {
  /* If the temperature is between the minimum threshold and the first average threshold,
     turn on the green LED and turn off the yellow and red LEDs */
  if (temperature >= THRESHOLD_TEMP_MIN && temperature <= THRESHOLD_TEMP_AVG1) {
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  }
  /* If the temperature is between the first and second average thresholds,
     turn on the yellow LED and turn off the green and red LEDs */
  else if (temperature > THRESHOLD_TEMP_AVG1 && temperature <= THRESHOLD_TEMP_AVG2) {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
  }
  /* If the temperature is between the second average threshold and the maximum threshold,
     turn on the red LED and turn off the green and yellow LEDs */
  else if (temperature > THRESHOLD_TEMP_AVG2 && temperature <= THRESHOLD_TEMP_MAX) {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  }
  /* If the temperature is outside the allowed range, turn off all LEDs */
  else {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  }
}


void checkTemperLevel(float temperature) {
  /* If the temperature is greater than the first threshold and less than or equal to the second threshold */
  if (temperature > THRESHOLD_TEMP_AVG1 && temperature <= THRESHOLD_TEMP_AVG2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temperature is hot");
    buzzerOn();
  }
  /* If the temperature is between the second threshold and the maximum threshold */
  else if (temperature > THRESHOLD_TEMP_AVG2 && temperature <= THRESHOLD_TEMP_MAX) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temperature is very hot");
    buzzerOn();
  }
  /* If the temperature is outside the allowed range, deactivate the buzzer */
  else {
    buzzerOff();
  }
}


void checkGasLevel(int mq2Value) {
  /* Check if the MQ2 sensor value exceeds the gas threshold */
  if (mq2Value > GAS_THRESHOLD) {
    /* Clear the LCD display */
    lcd.clear();
    /* Set the cursor to the first row and print the warning message */
    lcd.setCursor(0, 0);
    lcd.print("Gas Over Threshold");
    /* Set the cursor to the second row and print an additional warning */
    lcd.setCursor(0, 1);
    lcd.print("Warning ON");

    /* Activate the buzzer if the gas value is above the threshold */
    buzzerOn();
  } else {
    /* Deactivate the buzzer if the gas value is within a safe range */
    buzzerOff();
  }
}


void buzzerOn() {
  /* Set the buzzer pin to HIGH to activate the buzzer */
  digitalWrite(buzzerPin, HIGH);
  /* Wait for 800 milliseconds while the buzzer remains on */
  delay(800);
  /* Set the buzzer pin to LOW to deactivate the buzzer */
  digitalWrite(buzzerPin, LOW);
  /* Wait for 250 milliseconds before the next action */
  delay(250);
}

void buzzerOff() {
  /* Ensure the buzzer pin is set to LOW, turning the buzzer off */
  digitalWrite(buzzerPin, LOW);
}
