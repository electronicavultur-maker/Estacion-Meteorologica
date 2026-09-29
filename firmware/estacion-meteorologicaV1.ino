#include <esp_system.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <RTClib.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Preferences.h>
#include <time.h>

// ================= CONFIGURACIÓN =================
#define ESTACION_NUMERO 1
#define FIRMWARE_VERSION "1.6" //Implementacion de actualizacion por OTA

#define MQTT_BROKER    "200.44.171.179"
#define MQTT_PORT      4033

// Añade estas tres líneas:
#define MQTT_USER     "" // Déjalo vacío si tu broker no pide usuario
#define MQTT_PASSWORD "" // Déjalo vacío si tu broker no pide contraseña
#define TOPIC_ESTADO  "estacion/estado"

#define TOPIC_OTA_CMD     "estacion/ota/cmd"
#define TOPIC_OTA_ESTADO  "estacion/ota/estado"

#define MQTT_RETRY_MS 10000

String mqtt_client_id = "";

// ─── Seguridad OTA ───────────────────────────────────────────
#define OTA_TOKEN "12345678"


#define pinH 27
#define pinAne 34
#define pinVel 35
#define pluviometro 14
#define pinPan 39

#define Indef 33
#define Ok 25
#define Error 17

// ================= NTP =================
unsigned long ultimaSyncNTP = 0;

// 12 horas
const unsigned long NTP_INTERVAL = 43200000;

// Bandera para primera sincronizacion
bool ntpSincronizado = false;

// ================= OBJETOS =================

BluetoothSerial BT;  // Objeto Bluetooth
Preferences preferences;

DHT dht(pinH, DHT11);
Adafruit_BMP280 bmp;
RTC_DS1307 rtc;

File myFile;

// ================= VARIABLES =================

float V1, Pow, Vin, Vreal;
int contPulso = 0;

String ssid, password, apiEndpoint, confT, serN;

bool SD_True = false;

// ================= CONSTANTES =================

const float Voltaje_max = 1.9;  // El valor se encuentra en unidades de volts (V)
const float angulo_max = 360;   // El valor se encuentra en unidades de grados (°)

const float vol = 4;  //Volumen de agua que hace que la cubeta caiga

const float R1 = 10;  // 10.000 ohm
const float R2 = 13.3;
const float a = 0.0064;  // 64 centimetros cuadrados

const float R_V = (R2 / (R1 + R2));
const float Req = R1 + R2;

// ================= ESTRUCTURA MEDIDAS =================

struct Medidas {
  float T;
  float H;
  float P;
  float anemometro;
  float angulo;
  float rad;
  float precp = 0;
};

// ================= MODOS =================
enum DeviceMode {
  MODE_SIMPLE,
  MODE_SD
};

DeviceMode deviceMode = MODE_SD;
// ================= WIFI =================
unsigned long lastWifiTry = 0;
const unsigned long WIFI_RETRY_MS = 30000;

// ================= MQTT =================
WiFiClient        wifiClientMqtt;
PubSubClient      mqttClient(wifiClientMqtt);
unsigned long lastMqttRetry = 0;

volatile bool otaRequested  = false;
String        otaVersion    = "";
String        otaUrl        = "";

// ================= PROTOTIPOS =================
Medidas leerSensores();
void asegurarWiFi();
void configurarAPI();
bool tMedidas(int conf, int minutos, int segundos);
bool sincronizarRTCconNTP();
bool ajustarRTCManual(String fechaHora);

void SDloop();
void regLoop();
void contar();

int sendPostRequest(DateTime now, Medidas m);
int sendPostRequestFromJson(String paquete);
void connectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publicarOtaEstado(const char* msg);
void ejecutarOTA();

void bluetoothT(void *p);

void setup() {

  Serial.begin(9600);  //Inicializacion del puerto serial
  delay(5000);

  pinMode(Indef, OUTPUT);
  pinMode(Ok, OUTPUT);
  pinMode(Error, OUTPUT);
  digitalWrite(Ok, LOW);
  digitalWrite(Indef, LOW);
  digitalWrite(Error, LOW);


  pinMode(pluviometro, INPUT);                                           //Inicializacion de el pin digital que va a contar las caidas de la cubeta
  attachInterrupt(digitalPinToInterrupt(pluviometro), contar, FALLING);  //Inicializacion de la interrupcion del pluviometro

  dht.begin();  //Inicializacion del sensor DHT11

  // ---- RTC ----
  if (!rtc.begin()) {
    Serial.println(F("No se puede detectar el módulo RTC"));
    digitalWrite(Error, HIGH);
    while (1)
      ;
  }
  if (!rtc.isrunning()) {
    Serial.println(F("RTC no está configurado, ajustando hora..."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // ---- SD ----
  Serial.println(F("Inicializando SD"));
  if (!SD.begin(5)) {
    Serial.println(F("No se pudo inicializar SD"));
    SD_True = false;
    deviceMode = MODE_SIMPLE;
    digitalWrite(Error, HIGH);
  }

  // ---- BMP ----
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    digitalWrite(Error, HIGH);
    while (1)
      ;
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Modo de operación    */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling   */
                  Adafruit_BMP280::SAMPLING_X16,    /* Presion oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtrado.            */
                  Adafruit_BMP280::STANDBY_MS_500); /* Tiempo Standby.      */


  // ---- NVS ----
  preferences.begin("wifi-config", false);
  ssid = preferences.getString("ssid", "Vultur");
  password = preferences.getString("password", "STVultur*.");
  confT = preferences.getString("confT", "1");
  apiEndpoint = preferences.getString("apiURL", "");
  serN = preferences.getString("serN", "");

  //serN = String(ESTACION_NUMERO);          //Despues de subir el codigo la primera vez comentar y volver a subir
  //preferences.putString("serN", serN);     //Despues de subir el codigo la primera vez comentar y volver a subir

  if (serN == "") {
    serN = String(ESTACION_NUMERO);
    preferences.putString("serN", serN);
  }

  mqtt_client_id = "estacion-" + String(serN);

  configurarAPI();

  // ---- WIFI ----
  WiFi.begin(ssid.c_str(), password.c_str());

  asegurarWiFi();

  // ─── MQTT ─────────────────────────────────────────────────────
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512); 
  connectMQTT();

  BT.begin(("Estacion Meteorologica_" + String(serN)).c_str());  //Inicializacion de Bluetooth
  xTaskCreate(bluetoothT, "bluetooth", 4096, NULL, 1, NULL);     //Inicializacion de la tarea para la conexion Bluetooth
  digitalWrite(Indef, HIGH);
  delay(1000);
  digitalWrite(Indef, LOW);
  Serial.printf("====== Estacion Meteorologica firmware v%s ======\n", FIRMWARE_VERSION);
  BT.printf("====== Estacion Meteorologica firmware v%s ======\n", FIRMWARE_VERSION);

  Serial.println(F("inicializacion exitosa"));
  BT.println(F("inicializacion exitosa"));
  digitalWrite(Error, LOW);
}

void loop() {

  if (deviceMode == MODE_SD) {
    SDloop();
  } else {
    regLoop();
  }

  if (!mqttClient.connected()) {
    if (millis() - lastMqttRetry > MQTT_RETRY_MS) {
      lastMqttRetry = millis();
      connectMQTT();
    }
  } else {
    mqttClient.loop(); 
  }

  if (otaRequested) {
    otaRequested = false;
    ejecutarOTA();
  }
}


// ================= LECTURA SENSORES =================
Medidas leerSensores() {
  Medidas m;

  m.T = bmp.readTemperature();
  m.P = bmp.readPressure();
  m.H = (dht.readHumidity() + 10.315808) / 1.0829217;
  m.anemometro = analogRead(pinAne) * 3300.0 / 4095;
  V1 = analogRead(pinVel) * 3.3 / 4095;
  if (V1 > Voltaje_max)
    V1 = Voltaje_max;
  m.angulo = ((V1)*angulo_max) / Voltaje_max;
  Vin = analogRead(pinPan) * 3.3 / 4095;
  Vreal = (R_V)*Vin;
  Pow = (Vreal * Vreal) / Req;
  m.rad = Pow / a;
  m.precp = contPulso * vol;
  contPulso = 0;

  return m;
}

void SDloop() {
  DateTime now = rtc.now();
  if (tMedidas(confT.toInt(), now.minute(), now.second())) {
    BT.println("Loop SD");
    asegurarWiFi();
    Medidas m = leerSensores();
    BT.println("--- Lectura de sensores ---");
    BT.println("Temp: " + String(m.T) + " C");
    BT.println("Hum: " + String(m.H) + " %");
    BT.println("Pres: " + String(m.P) + " Pa");
    BT.println("Velocidad: " + String(m.anemometro) + " mV");
    BT.println("Dirección: " + String(m.angulo) + " °");
    BT.println("Radiancia: " + String(m.rad) + " W");
    BT.println("Radiancia: " + String(m.precp) + " ml");
    BT.println("fecha: " + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()));
    BT.println("hora: " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));
    digitalWrite(Ok, LOW);
    digitalWrite(Error, LOW);

    myFile = SD.open("/log.csv", FILE_APPEND);
    if (myFile) {
      myFile.printf("%d;%d;%d;%d;%d;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\n",
                    now.day(), now.month(), now.year(),
                    now.hour(), now.minute(), now.second(),
                    m.T, m.P, m.H, m.anemometro, m.angulo, m.rad, m.precp);
      myFile.close();
    } else digitalWrite(Error, HIGH);

    if (WiFi.status() == WL_CONNECTED) {
      // ================= SINCRONIZACION NTP =================
      if (!ntpSincronizado || (millis() - ultimaSyncNTP >= NTP_INTERVAL)) {

        bool syncOK = sincronizarRTCconNTP();

        if (syncOK) {
          ultimaSyncNTP = millis();
          ntpSincronizado = true;
        }

        // Leer nuevamente la hora ya corregida
        now = rtc.now();
      }
      int r = sendPostRequest(now, m);
      if (r != 201) {
        BT.print("Error ");
        BT.print(r);
        BT.println(" al intentar enviar la informacion al servidor, guardando en la memoria");
        myFile = SD.open("/resp.csv", FILE_APPEND);
        if (myFile) {
          String ins = "{\"d\":\"" + String(now.day()) + "\", \"m\":\"" + String(now.month()) + "\", \"a\":\"" + String(now.year()) + "\", \"h\":\"" + String(now.hour()) + "\", \"min\":\"" + String(now.minute()) + "\", \"s\":\""
                       + String(now.second()) + "\", \"t\":\"" + String(m.T) + "\", \"p\":\"" + String(m.P) + "\", \"hum\":\"" + String(m.H) + "\", \"v\":\""
                       + String(m.anemometro) + "\", \"dir\":\"" + String(m.angulo) + "\", \"rad\":\""
                       + String(m.rad) + "\", \"pre\":\"" + String(m.precp) + "\"}";
          myFile.println(ins);
          myFile.close();
        }
      } else {
        digitalWrite(Ok, HIGH);
        BT.println("Datos enviados con exito al servidor");
        myFile = SD.open("/resp.csv", FILE_READ);
        if (myFile) {
          int cantL = 0;
          while (myFile.available()) {
            cantL++;
            BT.println("Enviando datos en memoria");
            String linea = myFile.readStringUntil('\n');
            int respReintento = sendPostRequestFromJson(linea);
            if (respReintento != 201) {
              BT.println("Error enviando");
              cantL--;
              break;
            }
          }
          BT.println("Se enviaron " + String(cantL) + " Lineas de datos");
          myFile.close();
          if (SD.remove("/resp.csv")) {
          }
        }
      }
    } else {
      BT.println("Sin conexion a internet, guardando en la memoria");
      myFile = SD.open("/resp.csv", FILE_APPEND);
      if (myFile) {
        String ins = "{\"d\":\"" + String(now.day()) + "\", \"m\":\"" + String(now.month()) + "\", \"a\":\"" + String(now.year()) + "\", \"h\":\"" + String(now.hour()) + "\", \"min\":\"" + String(now.minute()) + "\", \"s\":\"" + String(now.second()) + "\", \"t\":\"" + String(m.T) + "\", \"p\":\"" + String(m.P) + "\", \"hum\":\"" + String(m.H) + "\", \"v\":\""
                     + String(m.anemometro) + "\", \"dir\":\"" + String(m.angulo) + "\", \"rad\":\""
                     + String(m.rad) + "\", \"pre\":\"" + String(m.precp) + "\"}";
        myFile.println(ins);
        myFile.close();
      }
    }
  }
  delay(1000);
}

// ================= LOOP SIMPLE =================
void regLoop() {
  DateTime now = rtc.now();
  if (tMedidas(confT.toInt(), now.minute(), now.second())) {
    BT.println("Loop Sin SD");
    asegurarWiFi();
    Medidas m = leerSensores();
    BT.println("--- Lectura de sensores ---");
    BT.println("Temp: " + String(m.T) + " C");
    BT.println("Hum: " + String(m.H) + " %");
    BT.println("Pres: " + String(m.P) + " Pa");
    BT.println("Velocidad: " + String(m.anemometro) + " mV");
    BT.println("Dirección: " + String(m.angulo) + " °");
    BT.println("Radiancia: " + String(m.rad) + " W");
    BT.println("Radiancia: " + String(m.precp) + " ml");
    BT.println("fecha: " + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()));
    BT.println("hora: " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));
    digitalWrite(Ok, LOW);
    digitalWrite(Error, LOW);


    if (WiFi.status() == WL_CONNECTED) {
      // ================= SINCRONIZACION NTP =================
      if (!ntpSincronizado || (millis() - ultimaSyncNTP >= NTP_INTERVAL)) {

        bool syncOK = sincronizarRTCconNTP();

        if (syncOK) {
          ultimaSyncNTP = millis();
          ntpSincronizado = true;
        }

        // Leer nuevamente la hora ya corregida
        now = rtc.now();
      }
      int r = sendPostRequest(now, m);
      if (r != 201) {
        BT.print("Error ");
        BT.print(r);
        BT.println(" al intentar enviar la informacion al servidor");
      } else {
        digitalWrite(Ok, HIGH);
        BT.println("Datos enviados con exito al servidor");
      }
    } else {
      BT.println("Sin conexion a internet");
    }
  }
  delay(1000);
}

// ================= WIFI =================
void asegurarWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    BT.println("Conectado a WiFi");
    return;
  }

  if (millis() - lastWifiTry > WIFI_RETRY_MS) {
    lastWifiTry = millis();
    WiFi.begin(ssid.c_str(), password.c_str());
    digitalWrite(Indef, HIGH);
    delay(100);
    digitalWrite(Indef, LOW);
    delay(100);
    digitalWrite(Indef, HIGH);
    delay(100);
    digitalWrite(Indef, LOW);
    delay(100);
  }
}

void configurarAPI() {
  if (apiEndpoint.length() > 0) {
    Serial.println(apiEndpoint);
    BT.println(apiEndpoint);
    return;
  }
  apiEndpoint = "http://200.44.171.177:9134/api/medidas2/insert/" + String(serN);
  preferences.putString("apiURL", apiEndpoint);
  Serial.println(apiEndpoint);
}

int sendPostRequest(DateTime now, Medidas m) {
  HTTPClient http;
  digitalWrite(Ok, LOW);
  digitalWrite(Indef, LOW);
  digitalWrite(Error, LOW);

  // Configura la solicitud POST
  http.begin(apiEndpoint);
  BT.println("Endpoint: ");
  BT.println(apiEndpoint);
  http.addHeader("Content-Type", "application/json");

  // Construye el cuerpo JSON de la solicitud
  String jsonBody = "{\"d\":\"" + String(now.day()) + "\", \"m\":\"" + String(now.month()) + "\", \"a\":\""
                    + String(now.year()) + "\", \"h\":\"" + String(now.hour()) + "\", \"min\":\"" + String(now.minute()) + "\", \"s\":\""
                    + String(now.second()) + "\", \"t\":\"" + String(m.T) + "\", \"p\":\"" + String(m.P) + "\", \"hum\":\""
                    + String(m.H) + "\", \"v\":\"" + String(m.anemometro) + "\", \"dir\":\"" + String(m.angulo) + "\", \"rad\":\""
                    + String(m.rad) + "\", \"pre\":\"" + String(m.precp) + "\"}";

  BT.println("JSON enviado: " + jsonBody);

  // Realiza la solicitud POST
  int httpResponseCode = http.POST(jsonBody);

  // Maneja la respuesta del servidor
  if (httpResponseCode > 0) {
    BT.print("Respuesta del servidor: ");
    BT.println(httpResponseCode);
    if (httpResponseCode == 201) {
      digitalWrite(Ok, HIGH);
    } else
      digitalWrite(Indef, HIGH);
  } else {
    BT.print("Error en la solicitud: ");
    BT.println(httpResponseCode);
    digitalWrite(Error, HIGH);
  }

  // Libera los recursos
  http.end();
  return httpResponseCode;
}

int sendPostRequestFromJson(String paquete) {
  HTTPClient http;

  digitalWrite(Ok, LOW);
  digitalWrite(Indef, LOW);
  digitalWrite(Error, LOW);

  // Configura la solicitud POST
  http.begin(apiEndpoint);
  http.addHeader("Content-Type", "application/json");

  // Construye el cuerpo JSON de la solicitud
  // Realiza la solicitud POST
  int httpResponseCode = http.POST(paquete);

  // Maneja la respuesta del servidor
  if (httpResponseCode > 0) {
    Serial.print("Respuesta del servidor: ");
    Serial.println(httpResponseCode);
    if (httpResponseCode == 201) {
      digitalWrite(Ok, HIGH);
    } else
      digitalWrite(Indef, HIGH);
  } else {
    BT.print("Error en la solicitud: ");
    BT.println(httpResponseCode);
    digitalWrite(Error, HIGH);
  }
  // Libera los recursos
  http.end();
  return httpResponseCode;
}

void bluetoothT(void *p) {
  while (1) {
    if (BT.available()) {
      Serial.println("Recibiendo");

      String incoming = BT.readStringUntil('\n');
      incoming.trim();
      incoming.toLowerCase();

      Serial.println(incoming);
      if (incoming == "descargar") {
        Serial.println("descargando informacion");
        myFile = SD.open("/resp.csv", FILE_READ);
        if (!myFile) {
          BT.println("Error al leer el archivo");
          Serial.println("Error al leer el archivo");
          return;
        }
        BT.println("Transmitiendo información...");
        Serial.println("Transmitiendo información...");
        while (myFile.available()) {
          BT.write(myFile.read());
        }
        myFile.close();
        if (SD.remove("/resp.csv")) {
          Serial.println("Archivo resp.csv eliminado.");
        } else {
          Serial.println("Error al eliminar el archivo resp.csv.");
        }
        BT.println("Transmisión completa.");
        Serial.println("Transmisión completa.");
      } else if (incoming == "1") {
        confT = "1";
        preferences.putString("confT", confT);
        BT.println("Tomando medidas cada 5 minutos");
      } else if (incoming == "2") {
        confT = "2";
        preferences.putString("confT", confT);
        BT.println("Tomando medidas cada 10 minutos");
      } else if (incoming == "3") {
        confT = "3";
        preferences.putString("confT", confT);
        BT.println("Tomando medidas cada 30 minutos");
      } else if (incoming == "4") {
        confT = "4";
        preferences.putString("confT", confT);
        BT.println("Tomando medidas cada 60 minutos");
      } else if (incoming == "wifi") {
        BT.println("Ingrese el nombre de la red");
        while (1) {
          if (BT.available()) {
            ssid = BT.readStringUntil('\n');
            ssid.trim();
            Serial.println(ssid);
            preferences.putString("ssid", ssid);

            break;
          }
        }
        BT.println("Ingrese la contraseña");
        while (1) {
          if (BT.available()) {
            password = BT.readStringUntil('\n');
            password.trim();
            Serial.println(password);
            preferences.putString("password", password);
            esp_restart();
          }
        }
      } else if (incoming == "api") {
        BT.println("Ingrese la nueva URL de la API:");
        while (1) {
          if (BT.available()) {
            apiEndpoint = BT.readStringUntil('\n');
            apiEndpoint.trim();
            Serial.println("Nueva API: " + apiEndpoint);
            BT.println("Nueva API: " + apiEndpoint);
            preferences.putString("apiURL", apiEndpoint);
            BT.println("URL guardada. Reiniciando...");
            delay(500);
            esp_restart();
          }
        }
      } else if (incoming == "resetapi") {
        preferences.remove("apiURL");
        BT.println("API restaurada a valores por defecto");
        delay(500);
        esp_restart();
      } else if (incoming == "hora") {

        BT.println("Enviar fecha y hora:");
        BT.println("Formato: YYYY-MM-DD HH:MM:SS");

        unsigned long timeout = millis();

        while (millis() - timeout < 30000) {

          if (BT.available()) {

            String fechaHora = BT.readStringUntil('\n');
            fechaHora.trim();

            if (ajustarRTCManual(fechaHora)) {

              DateTime now = rtc.now();

              BT.println("RTC actualizada correctamente");
              BT.println(now.timestamp());

            } else {

              BT.println("Formato invalido");
            }

            break;
          }

          vTaskDelay(10);
        }
      }
    }
    vTaskDelay(1);
  }
}

void contar() {
  static unsigned long ultimo_tiempo_interrupcion = 0;
  unsigned long tiempo_interrupcion = millis();
  if (tiempo_interrupcion - ultimo_tiempo_interrupcion > 600) {  //condicion para evitar rebotes
    contPulso++;
    Serial.println(contPulso);
  }
  ultimo_tiempo_interrupcion = tiempo_interrupcion;
}

bool tMedidas(int conf, int minutos, int segundos) {
  switch (conf) {
    case 1:
      if (/*((minutos % 5) == 0) &&*/ (segundos == 0)) {
        return true;
      } else {
        return false;
      }
      break;
    case 2:
      if (((minutos % 10) == 0) && (segundos == 0)) {
        return true;
      } else {
        return false;
      }
      break;
    case 3:
      if (((minutos % 30) == 0) && (segundos == 0)) {
        return true;
      } else {
        return false;
      }
      break;
    case 4:
      if ((minutos == 0) && (segundos == 0)) {
        return true;
      } else {
        return false;
      }
      break;
    default:
      return false;  // Retorno por defecto si 'conf' no coincide con ningún caso
  }
}

bool sincronizarRTCconNTP() {

  Serial.println("Sincronizando hora con NTP...");
  BT.println("Sincronizando hora con NTP...");

  // GMT-4 Venezuela
  configTime(-4 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  struct tm timeinfo;

  // Esperar respuesta NTP
  int intentos = 0;
  while (!getLocalTime(&timeinfo) && intentos < 10) {
    delay(500);
    intentos++;
  }

  if (intentos >= 10) {
    Serial.println("No se pudo obtener hora NTP");
    BT.println("No se pudo obtener hora NTP");
    return false;
  }

  // Convertir a DateTime
  DateTime ntpTime(
    timeinfo.tm_year + 1900,
    timeinfo.tm_mon + 1,
    timeinfo.tm_mday,
    timeinfo.tm_hour,
    timeinfo.tm_min,
    timeinfo.tm_sec);

  DateTime rtcTime = rtc.now();

  Serial.println("Hora RTC actual:");
  Serial.println(rtcTime.timestamp());

  Serial.println("Hora NTP:");
  Serial.println(ntpTime.timestamp());

  // Diferencia en segundos
  uint32_t rtcUnix = rtcTime.unixtime();
  uint32_t ntpUnix = ntpTime.unixtime();

  long diferencia = abs((long)(ntpUnix - rtcUnix));

  Serial.print("Diferencia RTC-NTP: ");
  Serial.print(diferencia);
  Serial.println(" segundos");

  // Solo corregir si hay diferencia significativa
  if (diferencia > 5) {

    rtc.adjust(ntpTime);

    Serial.println("RTC actualizada con NTP");
    BT.println("RTC actualizada con NTP");

  } else {

    Serial.println("RTC ya sincronizada");
    BT.println("RTC ya sincronizada");
  }

  return true;
}

bool ajustarRTCManual(String fechaHora) {

  if (fechaHora.length() != 19) {
    return false;
  }

  int year = fechaHora.substring(0, 4).toInt();
  int month = fechaHora.substring(5, 7).toInt();
  int day = fechaHora.substring(8, 10).toInt();

  int hour = fechaHora.substring(11, 13).toInt();
  int minute = fechaHora.substring(14, 16).toInt();
  int second = fechaHora.substring(17, 19).toInt();

  if (year < 2020 || year > 2100) return false;
  if (month < 1 || month > 12) return false;
  if (day < 1 || day > 31) return false;
  if (hour > 23) return false;
  if (minute > 59) return false;
  if (second > 59) return false;

  rtc.adjust(DateTime(
    year,
    month,
    day,
    hour,
    minute,
    second));

  return true;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  msg.replace("\\\"", "\"");

  Serial.printf("[MQTT] <- [%s]: %s\n", topic, msg.c_str());

  if (strcmp(topic, TOPIC_OTA_CMD) == 0) {
    String token   = "";
    String version = "";
    String url     = "";

    int idxT = msg.indexOf("\"token\"");
    if (idxT != -1) {
      int q1 = msg.indexOf('"', idxT + 7);
      int q2 = msg.indexOf('"', q1 + 1);
      if (q1 != -1 && q2 != -1) token = msg.substring(q1 + 1, q2);
    }

    int idxV = msg.indexOf("\"version\"");
    if (idxV != -1) {
      int q1 = msg.indexOf('"', idxV + 9);
      int q2 = msg.indexOf('"', q1 + 1);
      if (q1 != -1 && q2 != -1) version = msg.substring(q1 + 1, q2);
    }

    int idxU = msg.indexOf("\"url\"");
    if (idxU != -1) {
      int q1 = msg.indexOf('"', idxU + 5);
      int q2 = msg.indexOf('"', q1 + 1);
      if (q1 != -1 && q2 != -1) url = msg.substring(q1 + 1, q2);
    }

    if (token.length() < 8 || token != String(OTA_TOKEN)) {
      Serial.println("[SEC] OTA rechazado: token inválido");
      publicarOtaEstado("{\"error\":\"token_invalido\"}");
      return;
    }
    if (version.length() == 0 || url.length() == 0) {
      Serial.println("[SEC] OTA rechazado: faltan campos version o url");
      publicarOtaEstado("{\"error\":\"campos_faltantes\"}");
      return;
    }
    if (!url.startsWith("https://")) {
      Serial.println("[SEC] OTA rechazado: URL no es HTTPS");
      publicarOtaEstado("{\"error\":\"url_no_https\"}");
      return;
    }
    if (version == String(FIRMWARE_VERSION)) {
      Serial.printf("[OTA] Ya tengo la versión %s, ignorando.\n", FIRMWARE_VERSION);
      publicarOtaEstado("{\"info\":\"ya_tengo_esta_version\"}");
      return;
    }

    otaVersion    = version;
    otaUrl        = url;
    otaRequested  = true;
    Serial.printf("[OTA] Actualización encolada: v%s → v%s\n", FIRMWARE_VERSION, version.c_str());
  }
}

// ============================================================
//  MQTT — conexión
// ============================================================

void connectMQTT() {
  if (mqttClient.connected() || WiFi.status() != WL_CONNECTED) return;

  Serial.printf("[MQTT] Conectando a %s:%d...\n", MQTT_BROKER, MQTT_PORT);

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String clientId = mqtt_client_id + "_" + mac.substring(6);

  bool ok;
  if (strlen(MQTT_USER) > 0) {
    ok = mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD, TOPIC_ESTADO, 0, true, "offline");
  } else {
    ok = mqttClient.connect(clientId.c_str(), nullptr, nullptr, TOPIC_ESTADO, 0, true, "offline");
  }

  if (ok) {
    mqttClient.publish(TOPIC_ESTADO, "online", true);
    mqttClient.subscribe(TOPIC_OTA_CMD);
    Serial.printf("[MQTT] Conectado como '%s'\n", clientId.c_str());
  } else {
    Serial.printf("[MQTT] Fallo rc=%d\n", mqttClient.state());
  }
}

void ejecutarOTA() {
  Serial.printf("\n[OTA] ======= INICIANDO OTA =======\n");
  Serial.printf("[OTA] Actual: v%s  →  Nueva: v%s\n", FIRMWARE_VERSION, otaVersion.c_str());
  Serial.printf("[OTA] URL: %s\n", otaUrl.c_str());

  publicarOtaEstado((String("{\"status\":\"descargando\",\"version_actual\":\"") + FIRMWARE_VERSION + "\",\"version_nueva\":\"" + otaVersion + "\"}").c_str());
  mqttClient.loop();
  delay(300);

  // Liberar RAM: el BT consume mucha y TLS necesita un bloque grande
  Serial.printf("[OTA] Heap libre antes: %u  |  bloque max: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  BT.end();
  delay(200);
  Serial.printf("[OTA] Heap libre después de BT.end(): %u  |  bloque max: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  WiFiClientSecure client;
  client.setInsecure();

  httpUpdate.rebootOnUpdate(false);
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  Serial.println("[OTA] Descargando .bin...");
  t_httpUpdate_return result = httpUpdate.update(client, otaUrl);

  switch (result) {
    case HTTP_UPDATE_FAILED: {
      String errMsg = String("{\"error\":\"") + httpUpdate.getLastErrorString() + "\"}";
      Serial.printf("[OTA] ERROR: %s\n", httpUpdate.getLastErrorString().c_str());
      publicarOtaEstado(errMsg.c_str());
      break;
    }
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA] El servidor dice: sin cambios en el .bin");
      publicarOtaEstado("{\"info\":\"sin_cambios_en_servidor\"}");
      break;
    case HTTP_UPDATE_OK: {
      String okMsg = String("{\"status\":\"ok\",\"version\":\"") + otaVersion + "\"}";
      Serial.println("[OTA] Descarga OK. Reiniciando...");
      publicarOtaEstado(okMsg.c_str());
      mqttClient.loop();
      delay(500);
      ESP.restart();
      break;
    }
  }
}

// ============================================================
//  PUBLICACIÓN MQTT
// ============================================================

void publicarOtaEstado(const char* msg) {
  if (mqttClient.connected()) {
    mqttClient.publish(TOPIC_OTA_ESTADO, msg, false);
    Serial.printf("[OTA] Estado: %s\n", msg);
  }
}
