#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include <HTTPClient.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <SPI.h>
//#include "heltec.h"
//#include <DallasTemperature.h>
// GPIO where the DS18B20 is connected to
//const int oneWireBus = 6;     
// Setup a oneWire instance to communicate with any OneWire devices
//OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
//DallasTemperature sensors(&oneWire);

#define PIN_MQ2 4
#define PIN_LM35 7
#define LoRa_nss 8
#define LoRa_SCK 9
#define LoRa_MOSI 10
#define LoRa_MISO 11
#define LoRa_nrst 12
#define LoRa_busy 13
#define LoRa_dio1 14

#define oled_sda 17
#define oled_scl 18
#define SW1 19
#define SW2 20
#define oled_rst 21
#define GREENLED 33
#define REDLED 34
#define LED 35

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/oled_scl, /* data=*/oled_sda, /* reset=*/oled_rst);
SX1262 radio = new Module(LoRa_nss, LoRa_dio1, LoRa_nrst, LoRa_busy);
// Frecuencia LoRa
#define BAND 915E6

#define ADC_VREF_mV    3300.0 // in millivolt, el sensor era para Arduino
#define ADC_RESOLUTION 4096.0
 
//const char *SSID = "your_wifi-ssid";
//const char *PWD = "your_wifi_password";
//const char *SSID = "sc-199d 2.4";
//const char *PWD = "0142439647";
const char *SSID = "WiFi_Fibertel_bgt_2.4GHz";
const char *PWD = "Punto2024";

// Variables para FreeRTOS
TaskHandle_t TaskLoRaHandle;
TaskHandle_t TaskBlinkHandle;

// Web server running on port 80
WebServer server(80);
 
// JSON data buffer
StaticJsonDocument<250> jsonDocument;
char buffer[250];
 
// env variable
int temperature;
int temperature1;
int temp_ant;
float tempC;
int gas;
float pressure;
const char* serverUrl = "http://192.168.1.14:6000/add_number";
const char* apiKey = "tu_api_key";

void enviarJSONaAPI(int temp) {
  // Iniciar la conexión HTTP
  HTTPClient http;
  http.begin(serverUrl);
  // Crear un objeto JSON
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["valor"] = temp; // Aquí puedes poner los datos que quieras enviar en formato JSON
  // Convertir el objeto JSON en una cadena
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  // Configurar las cabeceras HTTP
  http.addHeader("Content-Type", "application/json");
  //http.addHeader("X-API-Key", apiKey);
  // Enviar la solicitud POST con el JSON
  int httpResponseCode = http.POST(jsonString);
  // Verificar la respuesta
  if (httpResponseCode > 0) {
    Serial.print("Respuesta del servidor: ");
    Serial.println(http.getString());
  } else {
    Serial.print("Error en la solicitud HTTP: ");
    Serial.println(httpResponseCode);  }
  // Cerrar la conexión HTTP
  http.end();
}

void connectToWiFi() {
  Serial.print("Connecting to ");
  Serial.println(SSID);
  WiFi.begin(SSID, PWD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500); 
    // we can even make the ESP32 to sleep
  } 
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
}
 
void create_json(char *tag, float value, char *unit) { 
  jsonDocument.clear(); 
  jsonDocument["type"] = tag;
  jsonDocument["value"] = value;
  jsonDocument["unit"] = unit;
  serializeJson(jsonDocument, buffer);
  Serial.println("Buffer:");
  Serial.println(buffer);  
}
 
void add_json_object(char *tag, float value, char *unit) {
  JsonObject obj = jsonDocument.createNestedObject();
  obj["type"] = tag;
  obj["value"] = value;
  obj["unit"] = unit; 
}

void read_sensor_data(void * parameter) {
   for (;;) {
     //temperature = analogRead(2);
     //humidity = 20; // analogRead(37); //bme.readHumidity();
     pressure = 30; //bme.readPressure() / 100;
     Serial.println("Read sensor data");
 
     // delay the task
     vTaskDelay(60000 / portTICK_PERIOD_MS);
   }
}
 
void getTemperature() {
  Serial.println("Get temperature");
  //temperature = analogRead(2);
  int adcVal = analogRead(PIN_LM35); //lee entrada analogica
  //calcula el valor del voltaje en mV
  //OJO en la simulación del LM35 tengo que usar como valor de 
  //voltaje de referencia superior 5V, en lugar de 3.3V (ADC_VREF_mV 50000)
  //cuando lo monte en protoboard usaremos ADC_VREF_mV 33000
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  float temperature = milliVolt / 10; 
  Serial.print(temperature);
  Serial.println(" C");
  
  create_json("temperature", temperature, "°C");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buffer);

}

void getGas() {
  Serial.println("Get Gas");
  //temperature = analogRead(2);
  int adcVal = analogRead(PIN_MQ2); //lee entrada analogica
  //calcula el valor del voltaje en mV
  //OJO en la simulación del LM35 tengo que usar como valor de 
  //voltaje de referencia superior 5V, en lugar de 3.3V (ADC_VREF_mV 50000)
  //cuando lo monte en protoboard usaremos ADC_VREF_mV 33000
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  float temperature = milliVolt / 10; 
  Serial.print(temperature);
  Serial.println(" %");
  create_json("gas", gas, "%");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buffer);
}
 
void getPressure() {
  Serial.println("Get pressure");
  pressure = analogRead(2);
  create_json("pressure", pressure, "mBar");
  server.send(200, "application/json", buffer);
}
 
void getEnv() {
  Serial.println("Get env");
  jsonDocument.clear();
  add_json_object("temperature", temperature, "°C");
  add_json_object("humidity", tempC, "%");
  add_json_object("pressure", pressure, "mBar");
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}

void handleOptions() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(200);
}

void handlePost() {
  if (server.hasArg("plain") == false) {
    //handle error here
  }

  String body = server.arg("plain");
  Serial.println(body);
  deserializeJson(jsonDocument, body);
  
  // Get RGB components
  int red = jsonDocument["red"];
  int green = jsonDocument["green"];
  int blue = jsonDocument["blue"];

  Serial.print("Red: ");
  Serial.print(red);
  if (red==1){
    digitalWrite(REDLED,true);
  }
  else
    digitalWrite(REDLED,false);
  if (green==1){
    digitalWrite(GREENLED,true);
  }
  else
    digitalWrite(GREENLED,false);
  //pixels.fill(pixels.Color(red, green, blue));
  delay(30);
  //pixels.show();

  // Respond to the client
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{}");
}
  
// setup API resources
void setup_routing() {
  
  server.on("/temperature", getTemperature);
  server.on("/pressure", getPressure);
  server.on("/humidity", getGas);
  server.on("/env", getEnv);
  server.on("/led", HTTP_POST, handlePost);
  server.on("/led", HTTP_OPTIONS, handleOptions);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  // start server
  server.begin();
}

void setup_task() {
  xTaskCreate(
    read_sensor_data,    
    "Read sensor data",   // Name of the task (for debugging)
    1000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );
}

// Tarea para transmitir datos LoRa
void TaskLoRaReceive(void *pvParameters) {
  while (true) {
  Serial.print(F("[SX1262] Waiting for incoming transmission ... "));
  // you can receive data as an Arduino String
  // NOTE: receive() is a blocking method!
  //       See example ReceiveInterrupt for details
  //       on non-blocking reception method.
  String str;
  int state = radio.receive(str);
  // you can also receive data as byte array
  /*
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);
  */
  if (state == RADIOLIB_ERR_NONE)  {
    // packet was successfully received
    Serial.println(F("success!"));
    //temperature1=analogRead(2);
    temperature1=+1;
    if (!(temperature1 == temp_ant)){
     enviarJSONaAPI(temperature1);
     temp_ant=temperature1;
    }
    // print the data of the packet
    Serial.print(F("[SX1262] Data:\t\t"));
    Serial.println(str);
    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[SX1262] RSSI:\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));
    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("[SX1262] SNR:\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
  }
  else if (state == RADIOLIB_ERR_RX_TIMEOUT)  {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH)  {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));
  }
  else  {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Espera de 5 segundos
  }
}

// Tarea para parpadear LED
void TaskBlink(void *pvParameters) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Encendido por 500 ms
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Apagado por 500 ms
  }
}

void setup() {
  Serial.begin(9600);
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_nss);
  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE)  {
    Serial.println(F("success!"));
  }
  else  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true)
      ;
  }

  u8g2.begin();
  connectToWiFi();
  setup_task();
  setup_routing();  
	pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);    
  pinMode(REDLED,OUTPUT);
  digitalWrite(REDLED,HIGH);
  pinMode(GREENLED,OUTPUT);
  digitalWrite(GREENLED,HIGH);
  digitalWrite(REDLED,LOW);
  digitalWrite(GREENLED,LOW);
  pinMode(SW1,PULLUP);
  pinMode(SW2,PULLUP);  
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW); 
    Serial.println("[SX1262] Inicializado correctamente.");
    // Crear tareas para FreeRTOS
  xTaskCreatePinnedToCore(
    TaskLoRaReceive,    // Función de la tarea
    "LoRaTask",          // Nombre de la tarea
    10000,               // Tamaño del stack
    NULL,                // Parámetros para la tarea
    1,                   // Prioridad
    &TaskLoRaHandle,     // Identificador de la tarea
    1                    // Núcleo donde se ejecutará (1 = Core 1)
  );

  xTaskCreatePinnedToCore(
    TaskBlink,           // Función de la tarea
    "BlinkTask",         // Nombre de la tarea
    10000,               // Tamaño del stack
    NULL,                // Parámetros para la tarea
    1,                   // Prioridad
    &TaskBlinkHandle,    // Identificador de la tarea
    0                    // Núcleo donde se ejecutará (0 = Core 0)
  );
  delay(1);

}
 
void loop() {
  server.handleClient();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(5, 20, "SAT PUNI 1"); 
    u8g2.setFont(u8g2_font_ncenB10_tr);       
    u8g2.drawStr(4, 40, "EST.TERRENA");
    u8g2.drawStr(20, 60, "Punilla Valley");
  } while (u8g2.nextPage());

  delay(10);
 
}


/*
#include <LoRa.h>

// Pines específicos del Heltec WiFi LoRa ESP32
#define SS 18
#define RST 14
#define DIO0 26

// Variables para FreeRTOS
TaskHandle_t TaskLoRaHandle;
TaskHandle_t TaskBlinkHandle;

// Configuración inicial
void setup() {
  Serial.begin(115200);

  // Inicializa LoRa
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(915E6)) { // Cambiar frecuencia según región
    Serial.println("Fallo al inicializar LoRa");
    while (1);
  }
  Serial.println("LoRa inicializado.");

  // Crear tareas para FreeRTOS
  xTaskCreatePinnedToCore(
    TaskLoRaTransmit,    // Función de la tarea
    "LoRaTask",          // Nombre de la tarea
    10000,               // Tamaño del stack
    NULL,                // Parámetros para la tarea
    1,                   // Prioridad
    &TaskLoRaHandle,     // Identificador de la tarea
    1                    // Núcleo donde se ejecutará (1 = Core 1)
  );

  xTaskCreatePinnedToCore(
    TaskBlink,           // Función de la tarea
    "BlinkTask",         // Nombre de la tarea
    10000,               // Tamaño del stack
    NULL,                // Parámetros para la tarea
    1,                   // Prioridad
    &TaskBlinkHandle,    // Identificador de la tarea
    0                    // Núcleo donde se ejecutará (0 = Core 0)
  );
}

// Tarea para transmitir datos LoRa
void TaskLoRaTransmit(void *pvParameters) {
  while (true) {
    Serial.println("Transmitiendo mensaje LoRa...");
    LoRa.beginPacket();
    LoRa.print("Hola desde ESP32 LoRa");
    LoRa.endPacket();
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Espera de 5 segundos
  }
}

// Tarea para parpadear LED
void TaskBlink(void *pvParameters) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Encendido por 500 ms
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Apagado por 500 ms
  }
}

void loop() {
  // El loop queda vacío, todo está manejado por las tareas.
}
*/