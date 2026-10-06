#include <esp_system.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <RTClib.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Preferences.h>
#include <time.h>

// ================= CONFIGURACIÓN =================
#define ESTACION_NUMERO 122
#define FIRMWARE_VERSION "2.71"  //Comando para forzar la actulizacion de forma manual

// ─── OTA diaria desde GitHub ─────────────────────────────────
// Archivo JSON con el formato: {"version":"2.5","url":"https://.../firmware.bin"}
#define OTA_VERSION_URL "https://raw.githubusercontent.com/electronicavultur-maker/Estacion-Meteorologica/main/firmware_version.json"
#define OTA_HORA_REVISION 3         // Hora (RTC) a partir de la cual se revisa 1 vez al dia
#define OTA_REINTENTO_MS 1800000UL  // Si la revision falla, reintenta cada 30 min

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
volatile int pulsosPl = 0;
int acumuladoPulsos = 0;

String ssid, password, apiEndpoint, confT, serN;

bool SD_True = false;
bool STARTUP_OK = 1;
float ultimaHumedad = NAN;

volatile bool otaManualSolicitada = false;

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

struct Sensores {
  float T;
  float H;
  float Precp;
};

struct Medidas {
  DateTime fecha_hora;
  float precipitacion_mm;
  float temp_aire_prom_c;
  float temp_aire_max_c;
  float temp_aire_min_c;
  float humedad_rel_prom;
  float humedad_rel_max;
  float humedad_rel_min;
  float punto_rocio_c;
  float radiacion_solar_w_m2 = 0;
  float insolacion_horas = 0;
  float viento_vel_prom_ms;
  float viento_vel_max_ms;
  float viento_vel_min_ms;
  int viento_dir_moda_deg;
  float viento_recorrido_km;
  float presion_atmos_hpa;

  void reset() {
    radiacion_solar_w_m2 = 0;
    insolacion_horas = 0;
  }

  void guardarCSV(File &f) const {
    f.printf(
      "%04d-%02d-%02d;%02d:%02d:%02d;"
      "%.2f;%.2f;%.2f;%.2f;"
      "%.2f;%.2f;%.2f;"
      "%.2f;%.2f;%.2f;"
      "%.2f;%.2f;%.2f;"
      "%d;%.2f;%.2f\n",

      fecha_hora.year(), fecha_hora.month(), fecha_hora.day(),
      fecha_hora.hour(), fecha_hora.minute(), fecha_hora.second(),

      precipitacion_mm,
      temp_aire_prom_c,
      temp_aire_max_c,
      temp_aire_min_c,

      humedad_rel_prom,
      humedad_rel_max,
      humedad_rel_min,

      punto_rocio_c,

      radiacion_solar_w_m2,
      insolacion_horas,

      viento_vel_prom_ms,
      viento_vel_max_ms,
      viento_vel_min_ms,

      viento_dir_moda_deg,
      viento_recorrido_km,
      presion_atmos_hpa);
  }
};

Medidas M;

struct AcumuladosSensores {

  float sumaTemp = 0;
  float tempMax = -100;
  float tempMin = 100;

  float sumaHum = 0;
  float humMax = 0;
  float humMin = 100;

  float precipitacion = 0;

  int muestras = 0;
  void reset() {

    sumaTemp = 0;
    tempMax = -100;
    tempMin = 100;

    sumaHum = 0;
    humMax = 0;
    humMin = 100;

    precipitacion = 0;

    muestras = 0;
  }
};

AcumuladosSensores S;

struct AcumuladosAnemometro {

  float sumaVelocidad = 0;
  float velMax = -1000;
  float velMin = 1000;
  int muestras = 0;

  uint16_t histograma[36] = {};

  float recorrido_km = 0;

  void reset() {
    sumaVelocidad = 0;
    velMax = -1000;
    velMin = 1000;
    muestras = 0;
    memset(histograma, 0, sizeof(histograma));
    recorrido_km = 0;
  }
};

AcumuladosAnemometro A;

// ================= MODOS =================
enum DeviceMode {
  MODE_SIMPLE,
  MODE_SD
};
DeviceMode deviceMode = MODE_SD;

// ================= WIFI =================
unsigned long lastWifiTry = 0;
const unsigned long WIFI_RETRY_MS = 30000;

// ================= PROTOTIPOS =================
bool scanI2C();
float leerRadiancia();
void actualizarSensores(AcumuladosSensores &S);
void actualizarAnemometro(AcumuladosAnemometro &A);
bool asegurarWiFi();
void configurarAPI();
bool tMedidas(int conf, int minutos, int segundos);
bool sincronizarRTCconNTP();
bool ajustarRTCManual(String fechaHora);
void guardarRespaldoSD(Medidas M);
float leerHumedadFiltrada();

void SDloop();
void regLoop();
void contar();

int sendPostRequest(DateTime now, Medidas m);
int sendPostRequestFromJson(String paquete);
void gestionarOTADiaria();
bool revisarActualizacionOTA();
bool obtenerInfoOTA(String &version, String &url);
void ejecutarOTA(const String &version, const String &url);
bool versionEsMayor(String nueva, String actual);
String extraerCampoJson(const String &json, const char *campo);

void bluetoothT(void *p);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //Inicializacion del puerto serial
  delay(5000);

  pinMode(Indef, OUTPUT);
  pinMode(Ok, OUTPUT);
  pinMode(Error, OUTPUT);
  digitalWrite(Ok, LOW);
  digitalWrite(Indef, LOW);
  digitalWrite(Error, LOW);

  pinMode(pluviometro, INPUT);
  attachInterrupt(digitalPinToInterrupt(pluviometro), contar, FALLING);

  dht.begin();

  STARTUP_OK = scanI2C();

  if (STARTUP_OK) {
    rtc.begin();
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

    bmp.begin();
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
    if (serN == "") {
      serN = String(ESTACION_NUMERO);
      preferences.putString("serN", serN);
    }


    configurarAPI();

    // ---- WIFI ----
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.println(WiFi.macAddress());

    asegurarWiFi();

    BT.begin(("Estacion Meteorologica_" + String(serN)).c_str());  //Inicializacion de Bluetooth
    xTaskCreate(bluetoothT, "bluetooth", 4096, NULL, 1, NULL);     //Inicializacion de la tarea para la conexion Bluetooth
    digitalWrite(Indef, HIGH);
    delay(1000);
    digitalWrite(Indef, LOW);
    Serial.println(F("inicializacion exitosa"));
    BT.println(F("inicializacion exitosa"));
    digitalWrite(Error, LOW);
  } else {
    while (1) {
      digitalWrite(Error, HIGH);
      delay(1000);
      Serial.println("Error iniciando BMP280 o RTC");
      digitalWrite(Error, LOW);
    }
  }
}

void loop() {
  if (deviceMode == MODE_SD) {
    SDloop();
  } else {
    regLoop();
  }

  asegurarWiFi();
  if (otaManualSolicitada) {
    otaManualSolicitada = false;
    if (WiFi.status() == WL_CONNECTED) {
      revisarActualizacionOTA();
    } else {
      Serial.println(F("[OTA] Sin WiFi, no se puede revisar"));
      BT.println(F("[OTA] Sin WiFi, no se puede revisar"));
    }
  }
  gestionarOTADiaria();
}

void SDloop() {
  DateTime now = rtc.now();
  if (tMedidas(1, now.minute(), now.second())) {
    BT.println("Tomando las medidas por minuto (Radiancia Solar e Insolacion)");
    Serial.println("Tomando las medidas por minuto (Radiancia Solar e Insolacion)");
    leerRadiancia();
  }
  if (tMedidas(2, now.minute(), now.second())) {
    BT.println("Tomando las medidas de 5 minutos (Temperatura, Humedad, Precipitacion)");
    Serial.println("Tomando las medidas de 5 minutos (Temperatura, Humedad, Precipitacion)");
    actualizarSensores(S);
  }
  if (tMedidas(3, now.minute(), now.second())) {
    BT.println("Calculando las medidas al final de la hora y preparando para enviar (Punto rocio y presion)");
    Serial.println("Calculando las medidas al final de la hora y preparando para enviar (Punto rocio y presion)");
    if (S.muestras > 0) {
      M.temp_aire_prom_c = S.sumaTemp / S.muestras;
      M.humedad_rel_prom = S.sumaHum / S.muestras;
    }

    M.temp_aire_max_c = S.tempMax;
    M.temp_aire_min_c = S.tempMin;

    M.humedad_rel_max = S.humMax;
    M.humedad_rel_min = S.humMin;

    M.precipitacion_mm = S.precipitacion;
    M.punto_rocio_c = calcularPuntoRocio(M.temp_aire_prom_c, M.humedad_rel_prom);

    M.insolacion_horas = M.insolacion_horas / 60;

    if (A.muestras > 0) {
      M.viento_vel_prom_ms = A.sumaVelocidad / A.muestras;
    }

    M.viento_vel_max_ms = A.velMax;
    M.viento_vel_min_ms = A.velMin;

    int sectorModa = 0;

    for (int i = 1; i < 36; i++) {
      if (A.histograma[i] > A.histograma[sectorModa]) {
        sectorModa = i;
      }
    }

    M.viento_dir_moda_deg = sectorModa * 10 + 5;

    M.presion_atmos_hpa = bmp.readPressure() / 100;

    asegurarWiFi();
    BT.println("--- Lectura de sensores ---");
    BT.println("Temperatura Promedio del Aire: " + String(M.temp_aire_prom_c) + " C");
    BT.println("Temperatura Maxima del Aire: " + String(M.temp_aire_max_c) + " C");
    BT.println("Temperatura Minima del Aire: " + String(M.temp_aire_min_c) + " C");
    BT.println("Punto de Rocio Calculado: " + String(M.punto_rocio_c) + " C");
    BT.println("Humedad Relativa Promedio: " + String(M.humedad_rel_prom) + " %");
    BT.println("Humedad Relativa Maxima: " + String(M.humedad_rel_max) + " %");
    BT.println("Humedad Relativa Minima: " + String(M.humedad_rel_min) + " %");
    BT.println("Radiancia Solar: " + String(M.radiacion_solar_w_m2) + " W/m2");
    BT.println("Insolación: " + String(M.insolacion_horas) + " h");
    BT.println("Velocidad del Viento Promedio: " + String(M.viento_vel_prom_ms) + " m/s");
    BT.println("Velocidad Máxima del Viento: " + String(M.viento_vel_max_ms) + " m/s");
    BT.println("Velocidad Minima del Viento: " + String(M.viento_vel_min_ms) + " m/s");
    BT.println("Moda Dirección del Viento: " + String(M.viento_dir_moda_deg) + " °");
    BT.println("Presión Atmosférica: " + String(M.presion_atmos_hpa) + " Pa");
    BT.println("Fecha: " + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()));
    BT.println("Hora: " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));
    digitalWrite(Ok, LOW);
    digitalWrite(Error, LOW);

    if (WiFi.status() == WL_CONNECTED) {
      if (!ntpSincronizado || (millis() - ultimaSyncNTP >= NTP_INTERVAL)) {
        bool syncOK = sincronizarRTCconNTP();

        if (syncOK) {
          ultimaSyncNTP = millis();
          ntpSincronizado = true;
        }

        now = rtc.now();
      }

      M.fecha_hora = now;

      File myFile = SD.open("/logN.csv", FILE_APPEND);
      if (myFile) {
        M.guardarCSV(myFile);
        myFile.close();
      } else {
        digitalWrite(Error, HIGH);
      }

      int r = sendPostRequest(now, M);
      if (r != 201) {
        BT.print("Error ");
        BT.print(r);
        BT.println(" al intentar enviar la informacion al servidor, guardando en la memoria");
        guardarRespaldoSD(M);

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
      guardarRespaldoSD(M);
    }

    M.reset();
    S.reset();
    A.reset();
  }
  actualizarAnemometro(A);
  delay(1000);
}

void regLoop() {
  DateTime now = rtc.now();
  if (tMedidas(1, now.minute(), now.second())) {
    BT.println("Tomando las medidas por minuto (Radiancia Solar e Insolacion)");
    Serial.println("Tomando las medidas por minuto (Radiancia Solar e Insolacion)");
    leerRadiancia();
  }
  if (tMedidas(2, now.minute(), now.second())) {
    BT.println("Tomando las medidas de 5 minutos (Temperatura, Humedad, Precipitacion)");
    Serial.println("Tomando las medidas de 5 minutos (Temperatura, Humedad, Precipitacion)");
    actualizarSensores(S);
  }
  if (tMedidas(3, now.minute(), now.second())) {
    BT.println("Calculando las medidas al final de la hora y preparando para enviar (Punto rocio y presion)");
    Serial.println("Calculando las medidas al final de la hora y preparando para enviar (Punto rocio y presion)");
    if (S.muestras > 0) {
      M.temp_aire_prom_c = S.sumaTemp / S.muestras;
      M.humedad_rel_prom = S.sumaHum / S.muestras;
    }
    M.temp_aire_max_c = S.tempMax;
    M.temp_aire_min_c = S.tempMin;

    M.humedad_rel_max = S.humMax;
    M.humedad_rel_min = S.humMin;

    M.precipitacion_mm = S.precipitacion;
    M.punto_rocio_c = calcularPuntoRocio(M.temp_aire_prom_c, M.humedad_rel_prom);

    M.insolacion_horas = M.insolacion_horas / 60;

    if (A.muestras > 0) {
      M.viento_vel_prom_ms = A.sumaVelocidad / A.muestras;
    }

    M.viento_vel_max_ms = A.velMax;
    M.viento_vel_min_ms = A.velMin;

    int sectorModa = 0;

    for (int i = 1; i < 36; i++) {
      if (A.histograma[i] > A.histograma[sectorModa]) {
        sectorModa = i;
      }
    }

    M.viento_dir_moda_deg = sectorModa * 10 + 5;

    M.presion_atmos_hpa = bmp.readPressure() / 100;

    asegurarWiFi();
    BT.println("--- Lectura de sensores ---");
    BT.println("Temperatura Promedio del Aire: " + String(M.temp_aire_prom_c) + " C");
    BT.println("Temperatura Maxima del Aire: " + String(M.temp_aire_max_c) + " C");
    BT.println("Temperatura Minima del Aire: " + String(M.temp_aire_min_c) + " C");
    BT.println("Punto de Rocio Calculado: " + String(M.punto_rocio_c) + " C");
    BT.println("Humedad Relativa Promedio: " + String(M.humedad_rel_prom) + " %");
    BT.println("Humedad Relativa Maxima: " + String(M.humedad_rel_max) + " %");
    BT.println("Humedad Relativa Minima: " + String(M.humedad_rel_min) + " %");
    BT.println("Radiancia Solar: " + String(M.radiacion_solar_w_m2) + " W/m2");
    BT.println("Insolación: " + String(M.insolacion_horas) + " h");
    BT.println("Velocidad del Viento Promedio: " + String(M.viento_vel_prom_ms) + " m/s");
    BT.println("Velocidad Máxima del Viento: " + String(M.viento_vel_max_ms) + " m/s");
    BT.println("Velocidad Minima del Viento: " + String(M.viento_vel_min_ms) + " m/s");
    BT.println("Moda Dirección del Viento: " + String(M.viento_dir_moda_deg) + " °");
    BT.println("Presión Atmosférica: " + String(M.presion_atmos_hpa) + " Pa");
    BT.println("Fecha: " + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()));
    BT.println("Hora: " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));

    digitalWrite(Ok, LOW);
    digitalWrite(Error, LOW);

    if (WiFi.status() == WL_CONNECTED) {
      if (!ntpSincronizado || (millis() - ultimaSyncNTP >= NTP_INTERVAL)) {

        bool syncOK = sincronizarRTCconNTP();

        if (syncOK) {
          ultimaSyncNTP = millis();
          ntpSincronizado = true;
        }

        now = rtc.now();
      }
      M.fecha_hora = now;
      int r = sendPostRequest(now, M);
      if (r != 201) {
        BT.print("Error ");
        BT.print(r);
        BT.println(" al intentar enviar la información al servidor");
        digitalWrite(Error, HIGH);
      } else {
        digitalWrite(Ok, HIGH);
        BT.println("Datos enviados con exito al servidor");
      }
    } else BT.println("Sin conexión a internet");


    M.reset();
    S.reset();
    A.reset();
  }
  actualizarAnemometro(A);
  delay(1000);
}

// ================= WIFI =================
void configurarAPI() {
  if (apiEndpoint.length() > 0) {
    Serial.println(apiEndpoint);
    BT.println(apiEndpoint);
    return;
  }
  apiEndpoint = "http://200.44.171.179:4069/api/stations/" + String(serN) + "/medidas";
  preferences.putString("apiURL", apiEndpoint);
  Serial.println(apiEndpoint);
}

bool asegurarWiFi() {
  // 1. Si ya está conectado, devuelve true inmediatamente
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  // 2. Si no está conectado, verifica si ya pasó el tiempo para reintentar
  if (millis() - lastWifiTry > WIFI_RETRY_MS) {
    lastWifiTry = millis();
    WiFi.begin(ssid.c_str(), password.c_str());

    // Secuencia de parpadeo del indicador
    digitalWrite(Indef, HIGH);
    delay(100);
    digitalWrite(Indef, LOW);
    delay(100);
    digitalWrite(Indef, HIGH);
    delay(100);
    digitalWrite(Indef, LOW);
    delay(100);
  }

  // 3. Si no hay conexión (o el intento acaba de iniciarse en segundo plano), devuelve false
  return false;
}

int sendPostRequest(DateTime now, Medidas M) {
  HTTPClient http;
  digitalWrite(Ok, LOW);
  digitalWrite(Indef, LOW);
  digitalWrite(Error, LOW);

  // Configura la solicitud POST
  http.begin(apiEndpoint);
  BT.println("Endpoint: ");
  BT.println(apiEndpoint);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-KEY", WiFi.macAddress());
  char fecha[25];

  sprintf(fecha,
          "%04d-%02d-%02dT%02d:%02d:%02d",
          M.fecha_hora.year(),
          M.fecha_hora.month(),
          M.fecha_hora.day(),
          M.fecha_hora.hour(),
          M.fecha_hora.minute(),
          M.fecha_hora.second());


  // Construye el cuerpo JSON de la solicitud
  String jsonBody;
  jsonBody.reserve(512);

  jsonBody = "{";
  jsonBody += "\"id\":\"" + String(serN) + "\"";
  jsonBody += ",\"fecha_hora\":\"" + String(fecha) + "\"";
  jsonBody += ",\"precipitacion_mm\":" + String(M.precipitacion_mm, 2);
  jsonBody += ",\"temp_aire_prom_c\":" + String(M.temp_aire_prom_c, 2);
  jsonBody += ",\"temp_aire_max_c\":" + String(M.temp_aire_max_c, 2);
  jsonBody += ",\"temp_aire_min_c\":" + String(M.temp_aire_min_c, 2);
  jsonBody += ",\"humedad_rel_prom\":" + String(M.humedad_rel_prom, 2);
  jsonBody += ",\"humedad_rel_max\":" + String(M.humedad_rel_max, 2);
  jsonBody += ",\"humedad_rel_min\":" + String(M.humedad_rel_min, 2);
  jsonBody += ",\"punto_rocio_c\":" + String(M.punto_rocio_c, 2);
  jsonBody += ",\"radiacion_solar_w_m2\":" + String(M.radiacion_solar_w_m2, 2);
  jsonBody += ",\"insolacion_horas\":" + String(M.insolacion_horas, 2);
  jsonBody += ",\"viento_vel_prom_ms\":" + String(M.viento_vel_prom_ms, 2);
  jsonBody += ",\"viento_vel_max_ms\":" + String(M.viento_vel_max_ms, 2);
  jsonBody += ",\"viento_vel_min_ms\":" + String(M.viento_vel_min_ms, 2);
  jsonBody += ",\"viento_dir_moda_deg\":" + String(M.viento_dir_moda_deg);
  jsonBody += ",\"viento_recorrido_km\":" + String(M.viento_recorrido_km, 2);
  jsonBody += ",\"presion_atmos_hpa\":" + String(M.presion_atmos_hpa, 2);
  jsonBody += "}";
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
  http.addHeader("X-API-KEY", WiFi.macAddress());

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

// ================= MEDIDAS ==============
float leerRadiancia() {
  Vin = analogRead(pinPan) * 3.3 / 4095;
  Vreal = (R_V)*Vin;
  Pow = (Vreal * Vreal) / Req;
  float rad = Pow / a;
  M.radiacion_solar_w_m2 += rad;
  if (rad > 1) {
    M.insolacion_horas++;
  }
}

void actualizarSensores(AcumuladosSensores &S) {
  Sensores s;
  s.T = bmp.readTemperature();
  s.H = (leerHumedadFiltrada() + 10.315808) / 1.0829217;
  s.Precp = pulsosPl * vol;
  pulsosPl = 0;

  BT.println("--- Lectura de sensores ---");
  BT.println("Temp: " + String(s.T) + " C");
  BT.println("Hum: " + String(s.H) + " %");
  BT.println("Precipitacion: " + String(s.Precp) + " mm");

  S.sumaTemp += s.T;
  if (s.T > S.tempMax)
    S.tempMax = s.T;
  if (s.T < S.tempMin)
    S.tempMin = s.T;

  S.sumaHum += s.H;
  if (s.H > S.humMax)
    S.humMax = s.H;
  if (s.H < S.humMin)
    S.humMin = s.H;

  S.precipitacion += s.Precp;

  S.muestras++;
}

void actualizarAnemometro(AcumuladosAnemometro &A) {


  float anemometro = analogRead(pinAne) * 3300.0 / 4095;
  V1 = analogRead(pinVel) * 3.3 / 4095;
  if (V1 > Voltaje_max)
    V1 = Voltaje_max;
  float angulo = ((V1)*angulo_max) / Voltaje_max;

  A.sumaVelocidad += anemometro;
  if (anemometro > A.velMax)
    A.velMax = anemometro;

  if (anemometro < A.velMin)
    A.velMin = anemometro;

  A.muestras++;

  int sector = ((int)angulo) / 10;
  sector %= 36;
  A.histograma[sector]++;
}

float calcularPuntoRocio(float T, float HR) {
  float a = 17.27;
  float b = 237.7;

  float alpha =
    ((a * T) / (b + T))
    + log(HR / 100.0);

  return (b * alpha) / (a - alpha);
}

bool tMedidas(int conf, int minutos, int segundos) {
  switch (conf) {
    case 1:  //Cada minuto
      if ((segundos == 0)) {
        return true;
      } else {
        return false;
      }
      break;
    case 2:  //Cada 5 minutos
      if (((minutos % 5) == 0) && (segundos == 0)) {
        return true;
      } else {
        return false;
      }
      break;
    case 3:  //Cada hora
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

void contar() {
  static unsigned long ultimo_tiempo_interrupcion = 0;
  unsigned long tiempo_interrupcion = millis();
  if (tiempo_interrupcion - ultimo_tiempo_interrupcion > 600) {  //condicion para evitar rebotes
    pulsosPl++;
  }
  ultimo_tiempo_interrupcion = tiempo_interrupcion;
}

void guardarRespaldoSD(Medidas M) {
  myFile = SD.open("/resp.csv", FILE_APPEND);
  if (myFile) {
    char fecha[25];
    sprintf(fecha,
            "%04d-%02d-%02dT%02d:%02d:%02d",
            M.fecha_hora.year(),
            M.fecha_hora.month(),
            M.fecha_hora.day(),
            M.fecha_hora.hour(),
            M.fecha_hora.minute(),
            M.fecha_hora.second());
    String ins;
    ins.reserve(512);

    ins = "{";
    ins += "\"id\":\"" + String(serN) + "\"";
    ins += ",\"fecha_hora\":\"" + String(fecha) + "\"";
    ins += ",\"precipitacion_mm\":" + String(M.precipitacion_mm, 2);
    ins += ",\"temp_aire_prom_c\":" + String(M.temp_aire_prom_c, 2);
    ins += ",\"temp_aire_max_c\":" + String(M.temp_aire_max_c, 2);
    ins += ",\"temp_aire_min_c\":" + String(M.temp_aire_min_c, 2);
    ins += ",\"humedad_rel_prom\":" + String(M.humedad_rel_prom, 2);
    ins += ",\"humedad_rel_max\":" + String(M.humedad_rel_max, 2);
    ins += ",\"humedad_rel_min\":" + String(M.humedad_rel_min, 2);
    ins += ",\"punto_rocio_c\":" + String(M.punto_rocio_c, 2);
    ins += ",\"radiacion_solar_w_m2\":" + String(M.radiacion_solar_w_m2, 2);
    ins += ",\"insolacion_horas\":" + String(M.insolacion_horas, 2);
    ins += ",\"viento_vel_prom_ms\":" + String(M.viento_vel_prom_ms, 2);
    ins += ",\"viento_vel_max_ms\":" + String(M.viento_vel_max_ms, 2);
    ins += ",\"viento_vel_min_ms\":" + String(M.viento_vel_min_ms, 2);
    ins += ",\"viento_dir_moda_deg\":" + String(M.viento_dir_moda_deg);
    ins += ",\"viento_recorrido_km\":" + String(M.viento_recorrido_km, 2);
    ins += ",\"presion_atmos_hpa\":" + String(M.presion_atmos_hpa, 2);
    ins += "}";
    myFile.println(ins);
    myFile.close();
  }
}

float leerHumedadFiltrada() {

  float h;

  // Reintenta hasta 3 veces
  for (int i = 0; i < 3; i++) {
    h = dht.readHumidity();

    if (!isnan(h)) {
      ultimaHumedad = h;
      return h;
    }

    delay(50);
  }

  // Si falló, usa el último valor válido
  return ultimaHumedad;
}

//============= BLUETOOTH ===============
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
      } else if (incoming == "mac") {
        BT.print("Direccion MAC del diaspositivo (Para API key):");
        BT.println(WiFi.macAddress());
      } else if (incoming == "version") {
        BT.println("Revisando actualizacion... el Bluetooth se reiniciara, reconecta en unos segundos");
        otaManualSolicitada = true;  // lo ejecuta el loop()
      }
    }
    vTaskDelay(1);
  }
}

//============= SISTEMA ====================
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

bool scanI2C() {
  uint8_t found = 0;
  Wire.begin();

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);

    if (Wire.endTransmission() == 0) {
      switch (addr) {
        case 0x50: found |= 0x01; break;
        case 0x58: found |= 0x02; break;
        case 0x68: found |= 0x04; break;
        case 0x76: found |= 0x08; break;
      }
    }
  }

  return found == 0x0F;
}
// ============================================================
//  OTA DIARIA DESDE GITHUB
// ============================================================

// Se llama en cada vuelta del loop. Revisa como maximo 1 vez al dia
// (a partir de OTA_HORA_REVISION) y solo si hay WiFi.
// La revision solo arranca 1-2 minutos despues de una marca de 5 min,
// para no pisar las mediciones (que se toman en minutos multiplos de 5).
void gestionarOTADiaria() {
  static int ultimoDiaRevisado = -1;
  static unsigned long ultimoIntento = 0;
  static bool hayIntentoPrevio = false;

  if (WiFi.status() != WL_CONNECTED) return;

  DateTime now = rtc.now();
  int claveDia = now.month() * 100 + now.day();

  if (claveDia == ultimoDiaRevisado) return;   // ya se reviso hoy
  if (now.hour() < OTA_HORA_REVISION) return;  // aun no es la hora

  int m5 = now.minute() % 5;
  if (m5 < 1 || m5 > 2) return;  // evitar chocar con mediciones

  if (hayIntentoPrevio && (millis() - ultimoIntento < OTA_REINTENTO_MS)) return;
  hayIntentoPrevio = true;
  ultimoIntento = millis();

  if (revisarActualizacionOTA()) {
    ultimoDiaRevisado = claveDia;  // consulta exitosa: no volver a revisar hasta manana
  }
}

// Devuelve true si pudo consultar el JSON (haya o no actualizacion).
// Si instala un firmware nuevo, reinicia el equipo y no retorna.
bool revisarActualizacionOTA() {
  Serial.println(F("[OTA] Revisando version_firmware.json..."));

  // Liberar RAM: el BT consume mucha y TLS necesita un bloque grande
  Serial.printf("[OTA] Heap libre antes: %u  |  bloque max: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  BT.end();
  delay(200);
  Serial.printf("[OTA] Heap libre despues de BT.end(): %u  |  bloque max: %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  String version, url;
  bool consultaOK = obtenerInfoOTA(version, url);

  if (consultaOK) {
    if (versionEsMayor(version, FIRMWARE_VERSION)) {
      Serial.printf("[OTA] Nueva version disponible: v%s (actual v%s)\n", version.c_str(), FIRMWARE_VERSION);
      ejecutarOTA(version, url);  // si sale bien, reinicia y no vuelve
    } else {
      Serial.printf("[OTA] Firmware al dia (actual v%s, remota v%s)\n", FIRMWARE_VERSION, version.c_str());
    }
  }

  // Si llegamos aqui no se actualizo: reactivar Bluetooth
  BT.begin(("Estacion Meteorologica_" + String(serN)).c_str());
  return consultaOK;
}

// Descarga y valida el JSON {"version":"x.y","url":"https://..."}
bool obtenerInfoOTA(String &version, String &url) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(10000);

  if (!http.begin(client, OTA_VERSION_URL)) {
    Serial.println(F("[OTA] No se pudo iniciar la conexion con GitHub"));
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[OTA] Error HTTP al leer el JSON: %d\n", code);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  if (payload.length() == 0 || payload.length() > 1024) {
    Serial.println(F("[OTA] JSON vacio o demasiado grande"));
    return false;
  }

  version = extraerCampoJson(payload, "version");
  url = extraerCampoJson(payload, "url");
  version.trim();
  url.trim();

  if (version.length() == 0 || url.length() == 0) {
    Serial.println(F("[OTA] JSON invalido: faltan campos version o url"));
    return false;
  }
  if (!url.startsWith("https://")) {
    Serial.println(F("[OTA] Rechazado: la URL del firmware no es HTTPS"));
    return false;
  }
  return true;
}

// Extrae el valor de un campo, tanto si viene entre comillas ("2.5") como sin ellas (2.5)
String extraerCampoJson(const String &json, const char *campo) {
  String clave = String("\"") + campo + "\"";
  int idx = json.indexOf(clave);
  if (idx < 0) return "";

  int dosPuntos = json.indexOf(':', idx + clave.length());
  if (dosPuntos < 0) return "";

  int i = dosPuntos + 1;
  int len = json.length();
  while (i < len && isspace((unsigned char)json[i])) i++;
  if (i >= len) return "";

  if (json[i] == '"') {
    int fin = json.indexOf('"', i + 1);
    if (fin < 0) return "";
    return json.substring(i + 1, fin);
  }

  int fin = i;
  while (fin < len && json[fin] != ',' && json[fin] != '}' && !isspace((unsigned char)json[fin])) fin++;
  return json.substring(i, fin);
}

// Compara versiones numericas separadas por punto: "2.10" > "2.9", "2.4" > "2.3"
bool versionEsMayor(String nueva, String actual) {
  nueva.trim();
  actual.trim();
  if (nueva.startsWith("v") || nueva.startsWith("V")) nueva.remove(0, 1);
  if (actual.startsWith("v") || actual.startsWith("V")) actual.remove(0, 1);

  unsigned int i = 0, j = 0;
  while (i < nueva.length() || j < actual.length()) {
    long a = 0, b = 0;
    while (i < nueva.length() && nueva[i] != '.') {
      if (isDigit(nueva[i])) a = a * 10 + (nueva[i] - '0');
      i++;
    }
    while (j < actual.length() && actual[j] != '.') {
      if (isDigit(actual[j])) b = b * 10 + (actual[j] - '0');
      j++;
    }
    i++;  // saltar el punto
    j++;
    if (a != b) return a > b;
  }
  return false;  // iguales
}

void ejecutarOTA(const String &version, const String &url) {
  Serial.printf("\n[OTA] ======= INICIANDO OTA =======\n");
  Serial.printf("[OTA] Actual: v%s  ->  Nueva: v%s\n", FIRMWARE_VERSION, version.c_str());
  Serial.printf("[OTA] URL: %s\n", url.c_str());

  WiFiClientSecure client;
  client.setInsecure();

  httpUpdate.rebootOnUpdate(false);
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  Serial.println("[OTA] Descargando .bin...");
  t_httpUpdate_return result = httpUpdate.update(client, url);

  switch (result) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("[OTA] ERROR: %s\n", httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA] El servidor dice: sin cambios en el .bin");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("[OTA] Descarga OK. Reiniciando...");
      delay(500);
      ESP.restart();
      break;
  }
}
