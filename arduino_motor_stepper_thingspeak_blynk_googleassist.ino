//================== CONFIGURACIÓN BLYNK ================== //

#define BLYNK_TEMPLATE_ID "TMPL2_5BVEXz5"
#define BLYNK_TEMPLATE_NAME "CONTROL MOTOR STEPPER"
#define BLYNK_AUTH_TOKEN "XXqfVv9VRrGpBQURDQayk-okOTuprB7H"
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>

//=======LIBRERIAS=====//
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

// ================== CONFIGURACIÓN SINRIC PRO ================== //
#include "SinricPro.h"
#include "SinricProSwitch.h"

// Credenciales SinricPro
#define APP_KEY           "7f133fa7-3bbc-4017-b661-96d5d61763b2"      
#define APP_SECRET        "d7242e6c-1fd5-42b2-88e7-a4a61d268447-0244b9a7-5df4-40ef-b103-0884176628eb"   

// IDs de dispositivos SinricPro
#define SWITCH_LED_ID         "69149165729a4887d7ccf364"
#define SWITCH_GIRO_IZQUIERDA_ID "691498ac6ebb39d664b6aefe"
#define SWITCH_GIRO_DERECHA_ID   "691499a900f870dd77b7f92a"

//CREDENCIALES THINGSPEAK
#define CHANNEL_ID 3163848 
#define WRITE_API_KEY "EZI88MICILGC5P76"

// CAMPOS THINGSPEAK
#define FIELD_MOTOR_STATE 1
#define FIELD_MOTOR_PROGRESS 2
#define FIELD_LED_STATE 3
#define FIELD_MOTOR_DURATION 4
#define FIELD_ACTION_COUNT 5
#define FIELD_SYSTEM_HEALTH 6


// Variables para ThingSpeak
unsigned long lastThingSpeakUpdate = 0;
const unsigned long thingSpeakInterval = 15000; // 15 segundos
unsigned long motorStartTime = 0;
int actionCounter = 0;
int motorDuration = 0;
bool thingSpeakEnabled = true;


//====TOKEN BLYNK====//
const char blynk_auth[] = BLYNK_AUTH_TOKEN;

//================== CONFIGURACIÓN WIFI ==================
const char* ssid = "FMLA ACOSTA_EXT";
const char* password = "Acost@333";
const char* API_HOST = "192.168.1.120";
const uint16_t API_PORT = 8000;
const String API_BASE = String("http://") + API_HOST + ":" + String(API_PORT);

//================== GLOBAL ==================
WebSocketsClient webSocket;
LiquidCrystal_I2C lcd(0x27, 16, 2);

String tokenActual = "";
String userName = "";
bool logeado = false;

//================== TECLADO ==================
const byte rowsCount = 4;
const byte columsCount = 3;
char keys[rowsCount][columsCount] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};
byte rowPins[rowsCount] = { 5, 18, 19, 21 };
byte columnPins[columsCount] = { 3, 1, 22 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, columnPins, rowsCount, columsCount);

//================== PINES ==================
int ledPin1 = 25;
int ledPin3 = 33;
int ledPin4 = 32;
int ledPin5 = 23;

int buttonPin1 = 34;
int buttonPin3_izquierda = 15;
int buttonPin4_derecha = 2;

//================== MOTOR ==================
#define IN1 13
#define IN2 12
#define IN3 14
#define IN4 27

const int pasoSecuencia[8][4] = {
  { 1, 0, 0, 0 },
  { 1, 1, 0, 0 },
  { 0, 1, 0, 0 },
  { 0, 1, 1, 0 },
  { 0, 0, 1, 0 },
  { 0, 0, 1, 1 },
  { 0, 0, 0, 1 },
  { 1, 0, 0, 1 }
};

//================== VARIABLES ==================
int pasoActual = 0;
int pasosPorVuelta = 4096;
int retardoMotor = 1;
bool motorGirando = false;
bool direccionMotor = true;
int pasosRestantes = 0;
unsigned long previousMotorTime = 0;
unsigned long previousDebounceTime = 0;
const unsigned long debounceDelay = 50;
int porcentajeActual = 0;

int ledState1 = 0;
int buttonOld1 = 1;
int buttonOld3 = 1;
int buttonOld4 = 1;

int menuNivel = 0;  // 0 = principal, 1 = LED, 2 = MOTOR
bool loginManualActivo = false;
String inputUsuario = "";
String inputClave = "";
bool ingresandoUsuario = true;

//================== PROTOTIPOS ==================
bool loginESP32(String username, String password);
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
bool verificarConexionBackend();
void enviarAccionBackend(String tipoAccion);
void mostrarMenuPrincipal();
void mostrarMenuLED();
void mostrarMenuMotor();
void procesarTeclado(char key);
void iniciarGiroMotor(bool dir);
void pararMotor();
void ejecutarPaso(bool dir);
void delayWithWebSocket(unsigned long ms);
void procesarLoginManual(char key);
void mostrarMenuLogin();
void toggleLed1();
void mostrarPantallaMotor(bool direccion);
void actualizarPorcentajeEnPantalla();
void reconectarWebSocket();
void diagnosticoMotor();
void setupThingSpeak();
void enviarDatosThingSpeak();
void incrementarContadorAccion();
int calcularHealthScore();

// ================== FUNCIONES SINRIC PRO ================== //
bool onPowerStateLED(const String &deviceId, bool &state);
bool onPowerStateGiroIzquierda(const String &deviceId, bool &state);
bool onPowerStateGiroDerecha(const String &deviceId, bool &state);
void setupSinricPro();
void controlarLEDDesdeSinric(bool state);
void controlarMotorDesdeSinric(bool direccion, bool state);

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Wire.begin(26, 4);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Pines
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(ledPin5, OUTPUT);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin3_izquierda, INPUT_PULLUP);
  pinMode(buttonPin4_derecha, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin5, LOW);
  digitalWrite(ledPin4, HIGH);

  // ================== CONEXIÓN WIFI ==================
  lcd.setCursor(0, 0);
  lcd.print("Conectando WiFi");
  WiFi.begin(ssid, password);
  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    lcd.print(".");
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.print("WIFI NO CONECTADO");
    Serial.println("\n[ERROR] No se pudo conectar al WiFi");
    while (true)
      ;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi OK");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  Serial.println("\n[OK] WiFi conectado con IP: " + WiFi.localIP().toString());
  delay(2000);

  // ================== CONFIGURACIÓN THINGSPEAK ================== //
  setupThingSpeak();

  // ================== INICIACIÓN DE COMUNICACION CON BLYNK ================== //
  Blynk.begin(blynk_auth, ssid, password);
  
  // 🔥 SINCRONIZAR ESTADO INICIAL DE LEDs
  delay(2000); // Esperar conexión Blynk
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, ledState1);  // Switch estado actual
    Blynk.virtualWrite(V9, ledState1);  // LED Status estado actual
    Blynk.virtualWrite(V5, 1);          // LED Stop ON (inicial)
    Serial.println("[SETUP] Estados de LEDs sincronizados con Blynk");
  }

  // ================== CONFIGURACIÓN SINRIC PRO ================== //
  setupSinricPro();

  // ================== VALIDAR BACKEND ==================
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Verificando API...");
  Serial.println("[INFO] Verificando API...");

  // Intentar múltiples veces con delay
  bool backendConectado = false;
  for (int intento = 1; intento <= 5; intento++) {
    lcd.setCursor(0, 1);
    lcd.print("Intento ");
    lcd.print(intento);
    lcd.print("/5");

    if (verificarConexionBackend()) {
      backendConectado = true;
      break;
    }
    delay(2000);  // Esperar 2 segundos entre intentos
  }

  if (backendConectado) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SISTEMA ");
    lcd.setCursor(0, 1);
    lcd.print("CONECTADO CON APP");
    Serial.println("[OK] SISTEMA CONECTADO CON API");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SISTEMA NO");
    lcd.setCursor(0, 1);
    lcd.print("CONECTADO CON API");
    Serial.println("[WARN] SISTEMA NO CONECTADO CON API");
    // Pero continuamos de todos modos, puede que funcione después
  }

  delay(3000);

  // ================== WEBSOCKET MEJORADO ==================
  webSocket.begin(API_HOST, API_PORT, "/ws/device/1");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);       // Reconectar cada 3 segundos
  webSocket.enableHeartbeat(15000, 3000, 2);  // Heartbeat para mantener conexión

  Serial.println("[WS] 🔄 Iniciando WebSocket...");

  mostrarMenuLogin();
}

// ================== LOOP ==================
void loop() {
  
  //==== COMUNICACION CON BLYNK ===//
  Blynk.run();
  
  //==== COMUNICACION CON WEBSOCKET ===//
  webSocket.loop();
  
  //==== COMUNICACION CON SINRIC PRO ===//
  SinricPro.handle();

  //==== ENVÍO DE DATOS A THINGSPEAK ===//
  enviarDatosThingSpeak();

  // Verificar periodicamente si estamos conectados
  static unsigned long lastConnectionCheck = 0;
  if (millis() - lastConnectionCheck > 10000) {  // Cada 10 segundos
    lastConnectionCheck = millis();
    if (!webSocket.isConnected()) {
      Serial.println("[WS] ⚠️ WebSocket desconectado, intentando reconectar...");
    }
  }

  unsigned long currentTime = millis();

  if (!logeado) {
    char key = keypad.getKey();
    if (key) {
      procesarLoginManual(key);
    }
  } else {
    // Usuario logeado - procesar teclado y botones
    char key = keypad.getKey();
    if (key) procesarTeclado(key);

    if (currentTime - previousDebounceTime >= debounceDelay) {
      previousDebounceTime = currentTime;
      int buttonNew1 = digitalRead(buttonPin1);
      int buttonNew3 = digitalRead(buttonPin3_izquierda);
      int buttonNew4 = digitalRead(buttonPin4_derecha);

      if (buttonNew3 == LOW && buttonOld3 == HIGH) {
        if (motorGirando && !direccionMotor) {
          lcd.clear();
          pararMotor();
          enviarAccionBackend("MOTOR_STOP");
          mostrarMenuPrincipal();
        } else {
          lcd.clear();
          iniciarGiroMotor(false);
          mostrarPantallaMotor(false);
          enviarAccionBackend("MOTOR_IZQ");
        }
      }

      if (buttonNew4 == LOW && buttonOld4 == HIGH) {
        if (motorGirando && direccionMotor) {
          lcd.clear();
          pararMotor();
          mostrarMenuPrincipal();
          enviarAccionBackend("MOTOR_STOP");
        } else {
          lcd.clear();
          iniciarGiroMotor(true);
          mostrarPantallaMotor(true);
          enviarAccionBackend("MOTOR_DER");
        }
      }

      if (buttonNew1 == LOW && buttonOld1 == HIGH) {
        lcd.clear();
        toggleLed1();
      }

      buttonOld1 = buttonNew1;
      buttonOld3 = buttonNew3;
      buttonOld4 = buttonNew4;
    }

    if (motorGirando && currentTime - previousMotorTime >= retardoMotor) {
      previousMotorTime = currentTime;
      ejecutarPaso(direccionMotor);
      pasosRestantes--;
      int nuevoPorcentaje = 100 - (pasosRestantes * 100 / pasosPorVuelta);
      if (nuevoPorcentaje >= porcentajeActual + 5 || pasosRestantes <= 0) {
        porcentajeActual = nuevoPorcentaje;
        actualizarPorcentajeEnPantalla();
      }
      if (pasosRestantes <= 0) {
        pararMotor();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Giro Completo!");
        delay(2000);
        mostrarMenuPrincipal();
      }
    }
  }
}

// ================== FUNCIONES SINRIC PRO ================== //

// Control para el LED
bool onPowerStateLED(const String &deviceId, bool &state) {
  Serial.printf("[SINRIC] LED: %s\r\n", state ? "ENCENDIDO" : "APAGADO");
  controlarLEDDesdeSinric(state);
  return true;
}

// Control para giro izquierda
bool onPowerStateGiroIzquierda(const String &deviceId, bool &state) {
  Serial.printf("[SINRIC] Giro Izquierda: %s\r\n", state ? "ACTIVADO" : "DESACTIVADO");
  controlarMotorDesdeSinric(false, state); // false = izquierda
  return true;
}

// Control para giro derecha
bool onPowerStateGiroDerecha(const String &deviceId, bool &state) {
  Serial.printf("[SINRIC] Giro Derecha: %s\r\n", state ? "ACTIVADO" : "DESACTIVADO");
  controlarMotorDesdeSinric(true, state); // true = derecha
  return true;
}

// Configuración de SinricPro
void setupSinricPro() {
  Serial.println("[SINRIC] Configurando SinricPro...");
  
  // Configurar dispositivos
  SinricProSwitch& mySwitchLED = SinricPro[SWITCH_LED_ID];
  mySwitchLED.onPowerState(onPowerStateLED);
  
  SinricProSwitch& mySwitchGiroIzquierda = SinricPro[SWITCH_GIRO_IZQUIERDA_ID];
  mySwitchGiroIzquierda.onPowerState(onPowerStateGiroIzquierda);
  
  SinricProSwitch& mySwitchGiroDerecha = SinricPro[SWITCH_GIRO_DERECHA_ID];
  mySwitchGiroDerecha.onPowerState(onPowerStateGiroDerecha);

  // Configurar callbacks de conexión
  SinricPro.onConnected([](){ 
    Serial.println("[SINRIC] ✅ Conectado a SinricPro"); 
  }); 
  
  SinricPro.onDisconnected([](){ 
    Serial.println("[SINRIC] ❌ Desconectado de SinricPro"); 
  });

  // Iniciar SinricPro
  SinricPro.begin(APP_KEY, APP_SECRET);
  Serial.println("[SINRIC] ✅ SinricPro iniciado correctamente");
}

// Controlar LED desde SinricPro
void controlarLEDDesdeSinric(bool state) {
  digitalWrite(ledPin1, state ? HIGH : LOW);
  ledState1 = state;
  
  // Sincronizar con Blynk
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, state);
    Blynk.virtualWrite(V9, state);
  }
  
  // Enviar acción al backend
  String tipoAccion = state ? "LED_ON" : "LED_OFF";
  enviarAccionBackend(tipoAccion);
  
  // Mostrar en LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SINRIC PRO:");
  lcd.setCursor(0, 1);
  lcd.print(state ? "LED ENCENDIDO" : "LED APAGADO");
  delay(2000);
  mostrarMenuPrincipal();
  
  Serial.printf("[SINRIC] LED controlado: %s\n", state ? "ON" : "OFF");
}

// Controlar motor desde SinricPro
void controlarMotorDesdeSinric(bool direccion, bool state) {
  if (state) {
    // Si se activa el switch
    if ((motorGirando && direccionMotor == direccion)) {
      // Si ya está girando en la misma dirección, detener
      pararMotor();
      enviarAccionBackend("MOTOR_STOP");
      Serial.println("[SINRIC] Motor detenido (misma dirección)");
    } else {
      // Iniciar giro en la dirección solicitada
      pararMotor(); // Detener cualquier movimiento previo
      delay(50);
      iniciarGiroMotor(direccion);
      enviarAccionBackend(direccion ? "MOTOR_DER" : "MOTOR_IZQ");
      Serial.printf("[SINRIC] Motor giro %s iniciado\n", direccion ? "DERECHA" : "IZQUIERDA");
    }
  } else {
    // Si se desactiva el switch y está girando en esa dirección, detener
    if (motorGirando && direccionMotor == direccion) {
      pararMotor();
      enviarAccionBackend("MOTOR_STOP");
      Serial.printf("[SINRIC] Motor %s detenido\n", direccion ? "DERECHA" : "IZQUIERDA");
    }
  }
}

//==== FUNCION PARA LLAMAR TODO EL TIEMPO A BLYNK EN LA PLATAFORMA ===//
BLYNK_WRITE(V0)
{
 int pinValue = param.asInt();
 digitalWrite(ledPin1, pinValue);
 
  // 1. Actualiza el estado local del LED para sincronización
  ledState1 = pinValue; 
  incrementarContadorAccion();//ENVIO A THINGSPEAK

  // 2. Determina la acción a enviar al backend
  String tipoAccion = (pinValue == HIGH) ? "LED_ON" : "LED_OFF";
 
  // 3. Envía la acción al backend para almacenar la petición en BD
  enviarAccionBackend(tipoAccion); 

  // 4. 🔥 ACTUALIZAR LED STATUS EN BLYNK (V9)
  if (Blynk.connected()) {
    Blynk.virtualWrite(V9, pinValue); // Sincronizar LED status
  }

  Serial.print("[BLYNK] LED_PIN1 cambiado a: ");
  Serial.println(pinValue);
  Serial.println("[BLYNK] Acción enviada al Backend: " + tipoAccion);
  Serial.printf("[BLYNK] LED Status (V9) actualizado: %s\n", pinValue ? "ON" : "OFF");
}

// ==== FUNCIONES PARA SWITCH (ON/OFF) ==== //

// V1: SWITCH DERECHA (ON=Girar, OFF=Detener)
BLYNK_WRITE(V1) {
  int pinValue = param.asInt();
  Serial.printf("[BLYNK] V1 - Switch DERECHA: %s\n", pinValue ? "ON" : "OFF");
  
  if (pinValue == 1) { // Switch en ON
  incrementarContadorAccion();//ENVIO A THINGSPEAK
    if (!motorGirando || !direccionMotor) {
      pararMotor(); // Detener cualquier movimiento previo
      delay(50);
      iniciarGiroMotor(true);
      enviarAccionBackend("MOTOR_DER");
      Serial.println("[BLYNK] 🔄 Motor: DERECHA - GIRO CONTINUO");
      
    }
  } else { // Switch en OFF
    if (motorGirando && direccionMotor) {
      pararMotor();
      enviarAccionBackend("MOTOR_STOP");
      Serial.println("[BLYNK] 🛑 Motor detenido (Switch DER OFF)");
    }
  }
}

// V2: SWITCH STOP (ON=Forzar parada)
BLYNK_WRITE(V2) {
  int pinValue = param.asInt();
  Serial.printf("[BLYNK] V2 - Switch STOP: %s\n", pinValue ? "ON" : "OFF");
  
  if (pinValue == 1) {
    incrementarContadorAccion();//ENVIO A THINGSPEAK
    pararMotor();
    enviarAccionBackend("MOTOR_STOP");
    Serial.println("[BLYNK] 🛑 Motor: DETENIDO POR SWITCH STOP");
    
    // Apagar los otros switches
    Blynk.virtualWrite(V1, 0);
    Blynk.virtualWrite(V3, 0);
  }
}

// V3: SWITCH IZQUIERDA (ON=Girar, OFF=Detener)
BLYNK_WRITE(V3) {
  int pinValue = param.asInt();
  Serial.printf("[BLYNK] V3 - Switch IZQUIERDA: %s\n", pinValue ? "ON" : "OFF");
  
  if (pinValue == 1) { // Switch en ON
  incrementarContadorAccion();//ENVIO A THINGSPEAK
    if (!motorGirando || direccionMotor) {
      pararMotor(); // Detener cualquier movimiento previo
      delay(50);
      iniciarGiroMotor(false);
      enviarAccionBackend("MOTOR_IZQ");
      Serial.println("[BLYNK] 🔄 Motor: IZQUIERDA - GIRO CONTINUO");
    }
  } else { // Switch en OFF
    if (motorGirando && !direccionMotor) {
      pararMotor();
      enviarAccionBackend("MOTOR_STOP");
      Serial.println("[BLYNK] 🛑 Motor detenido (Switch IZQ OFF)");
    }
  }
}

// V7: PORCENTAJE DE AVANCE (Solo lectura)
BLYNK_READ(V7) {
  Blynk.virtualWrite(V7, porcentajeActual);
  Serial.printf("[BLYNK] Enviando porcentaje: %d%%\n", porcentajeActual);
}

// V8: DIAGNÓSTICO (opcional)
BLYNK_WRITE(V8) {
  int pinValue = param.asInt();
  if (pinValue == 1) {
    Serial.println("[BLYNK] 🔧 Solicitando diagnóstico...");
    diagnosticoMotor();
    
    // Enviar estado actual a Blynk
    Blynk.virtualWrite(V0, ledState1);  // Estado LED principal
    Blynk.virtualWrite(V9, ledState1);  // LED Status
    Blynk.virtualWrite(V4, motorGirando && direccionMotor ? 1 : 0);
    Blynk.virtualWrite(V5, !motorGirando ? 1 : 0);
    Blynk.virtualWrite(V6, motorGirando && !direccionMotor ? 1 : 0);
    Blynk.virtualWrite(V7, porcentajeActual);
  }
}

// ================== DIAGNÓSTICO DEL MOTOR ==================
void diagnosticoMotor() {
  Serial.println("\n=== DIAGNÓSTICO COMPLETO ===");
  Serial.printf("Motor girando: %s\n", motorGirando ? "SI" : "NO");
  Serial.printf("Dirección: %s\n", direccionMotor ? "DERECHA" : "IZQUIERDA");
  Serial.printf("Pasos restantes: %d\n", pasosRestantes);
  Serial.printf("Porcentaje: %d%%\n", porcentajeActual);
  Serial.printf("LED Principal (V0): %s\n", ledState1 ? "ON" : "OFF");
  Serial.printf("Pines motor: IN1=%d, IN2=%d, IN3=%d, IN4=%d\n", 
                digitalRead(IN1), digitalRead(IN2), digitalRead(IN3), digitalRead(IN4));
  Serial.printf("LEDs físicos: Pin1=%d, Pin3=%d, Pin4=%d, Pin5=%d\n",
                digitalRead(ledPin1), digitalRead(ledPin3), digitalRead(ledPin4), digitalRead(ledPin5));
  Serial.println("=============================\n");
}

// ================== WEBSOCKET MEJORADO ==================
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] 🔌 Desconectado");
      // Intentar reconectar automáticamente
      webSocket.begin(API_HOST, API_PORT, "/ws/device/1");
      webSocket.onEvent(webSocketEvent);
      break;

    case WStype_CONNECTED:
      Serial.println("[WS] ✅ Conectado al servidor");
      // Enviar autenticación si ya tenemos token
      if (tokenActual != "") {
        String authMsg = "{\"type\":\"auth\",\"token\":\"" + tokenActual + "\"}";
        webSocket.sendTXT(authMsg);
        Serial.println("[WS] 🔐 Enviando autenticación: " + authMsg);
      }
      break;

    case WStype_TEXT:
      {
        Serial.printf("[WS] 📩 Mensaje recibido: %s\n", payload);

        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, payload, length);

        if (error) {
          Serial.println("[WS] ❌ Error parse JSON");
          return;
        }

        String tipo = doc["type"] | "";
        String event = doc["event"] | "";

        // 🔥 CAPTURAR LOGIN DESDE LA APP - MÚLTIPLES FORMATOS
        if (tipo == "login" || tipo == "auth_success") {
          bool success = doc["success"] | false;
          String message = doc["message"] | "";

          Serial.printf("[WS] 🔑 Login event: success=%d, message=%s\n", success, message.c_str());

          if (success) {
            // Obtener token si viene en el mensaje
            if (doc.containsKey("token")) {
              tokenActual = doc["token"].as<String>();
              Serial.println("[WS] 🔑 Token actualizado: " + tokenActual);
            }

            // Obtener información del usuario
            if (doc.containsKey("user")) {
              JsonObject user = doc["user"];
              if (user.containsKey("name")) {
                userName = user["name"].as<String>();
              } else if (user.containsKey("username")) {
                userName = user["username"].as<String>();
              }
            } else if (doc.containsKey("name")) {
              userName = doc["name"].as<String>();
            }

            logeado = true;

            // Mostrar en pantalla
            lcd.clear();
            lcd.print("LOGGIN EXITOSO");
            delayWithWebSocket(1000);
            lcd.clear();
            lcd.print("Bienvenido:");
            lcd.setCursor(0, 1);
            lcd.print(userName);
            delayWithWebSocket(2000);
            mostrarMenuPrincipal();

            Serial.println("[WS] Login exitoso desde app - Usuario: " + userName);
          }
        }
        // Manejar notificación de nuevo login
        else if (event == "user_logged_in" || tipo == "user_authenticated") {
          Serial.println("[WS] 👤 Notificación de login recibida");

          if (doc.containsKey("user_id")) {
            // Si recibimos un user_id, asumimos que el login fue exitoso
            logeado = true;
            if (doc.containsKey("username")) {
              userName = doc["username"].as<String>();
            }

            lcd.clear();
            lcd.print("Sesion Activa");
            delayWithWebSocket(1000);
            mostrarMenuPrincipal();
          }
        }
        // Manejar acciones desde el backend
        else if (event == "new_action" || tipo == "action" || tipo == "action_execute") {
          int actionId = doc["action_id"] | doc["id"] | 0;
          String actionType = doc["action_type"] | doc["type"] | "";
          String command = doc["command"] | "";

          Serial.printf("[WS] ⚡ Acción recibida: %s (ID:%d)\n", actionType.c_str(), actionId);

          if (actionType == "LED_ON" || command == "LED_ON") {
            digitalWrite(ledPin1, HIGH);
            ledState1 = 1;
            
            // 🔥 SINCRONIZAR BLYNK
            if (Blynk.connected()) {
              Blynk.virtualWrite(V0, 1);  // Switch a ON
              Blynk.virtualWrite(V9, 1);  // LED Status a ON
            }
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("LED: ON");
            lcd.setCursor(0, 1);
            lcd.print("STATUS: ENCENDIDO");
            delayWithWebSocket(2000);
            delay(1000);
            mostrarMenuPrincipal();
          } else if (actionType == "LED_OFF" || command == "LED_OFF") {
            digitalWrite(ledPin1, LOW);
            ledState1 = 0;
            
            // 🔥 SINCRONIZAR BLYNK
            if (Blynk.connected()) {
              Blynk.virtualWrite(V0, 0);  // Switch a OFF
              Blynk.virtualWrite(V9, 0);  // LED Status a OFF
            }
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("LED: OFF");
            lcd.setCursor(0, 1);
            lcd.print("STATUS: APAGADO");
            delayWithWebSocket(2000);
            delay(1000);
            mostrarMenuPrincipal();
          } else if (actionType == "MOTOR_IZQ" || command == "MOTOR_IZQ") {
            iniciarGiroMotor(false);
            mostrarPantallaMotor(false);
          } else if (actionType == "MOTOR_DER" || command == "MOTOR_DER") {
            iniciarGiroMotor(true);
            mostrarPantallaMotor(true);
          } else if (actionType == "MOTOR_STOP" || command == "MOTOR_STOP") {
            pararMotor();
            mostrarMenuPrincipal();
          }
        }
        // Mensaje genérico de conexión
        else if (tipo == "connection" || tipo == "info") {
          String msg = doc["message"] | "";
          Serial.println("[WS] ℹ️ Mensaje del servidor: " + msg);
        }
        break;
      }

    case WStype_ERROR:
      Serial.println("[WS] ❌ Error en WebSocket");
      break;

    default:
      break;
  }
}

// ================== BACKEND ==================
bool verificarConexionBackend() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HEALTH] WiFi no conectado");
    return false;
  }
  
  HTTPClient http;
  String url = API_BASE + "/health";
  http.begin(url);
  http.setTimeout(10000); // 10 segundos de timeout
  
  Serial.println("[HEALTH] Probando: " + url);
  int code = http.GET();
  Serial.printf("[HEALTH] HTTP code: %d\n", code);
  
  if (code == 307) {
    String payload = http.getString();
    Serial.println("[HEALTH] payload: " + payload);
    http.end();
    return true;
  } else if (code > 0) {
    Serial.printf("[HEALTH] Error HTTP: %d\n", code);
    String error = http.getString();
    Serial.println("[HEALTH] Error: " + error);
  } else {
    Serial.printf("[HEALTH] Error de conexión: %d\n", code);
  }
  
  http.end();
  return false;
}

// ==========================================================================

bool loginESP32(String username, String password) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String url = API_BASE + "/api/auth/login";
  Serial.println("[LOGIN] URL: " + url);

  http.begin(url);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "username=" + username + "&password=" + password;
  Serial.println("[LOGIN] Body: " + body);

  int code = http.POST(body);
  Serial.printf("[LOGIN] HTTP %d\n", code);

  if (code == 200) {
    String resp = http.getString();
    Serial.println("[LOGIN] Respuesta: " + resp);

    StaticJsonDocument<512> response;
    DeserializationError err = deserializeJson(response, resp);
    if (err) {
      Serial.println("[LOGIN] JSON parse error");
      http.end();
      return false;
    }

    tokenActual = response["access_token"].as<String>();

    // Obtener nombre del usuario
    if (response.containsKey("user")) {
      if (response["user"].containsKey("name")) {
        userName = response["user"]["name"].as<String>();
      } else if (response["user"].containsKey("username")) {
        userName = response["user"]["username"].as<String>();
      }
    } else {
      userName = username;
    }

    logeado = true;

    // 🔥 NOTIFICAR AL WEBSOCKET DEL LOGIN EXITOSO
    if (webSocket.isConnected()) {
      String authMsg = "{\"type\":\"auth\",\"token\":\"" + tokenActual + "\"}";
      webSocket.sendTXT(authMsg);
      Serial.println("[LOGIN] 🔐 Autenticación enviada al WebSocket");
    }

    http.end();
    return true;
  } else {
    Serial.printf("[LOGIN] Error %d\n", code);
    String errorResp = http.getString();
    Serial.println("[LOGIN] Error respuesta: " + errorResp);
    http.end();
    return false;
  }
}

void enviarAccionBackend(String tipoAccion) {
  if (!logeado || tokenActual == "") {
    Serial.println("[ACTION] No logeado, ignorando acción");
    return;
  }

  HTTPClient http;
  String url = API_BASE + "/actions/";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + tokenActual);

  StaticJsonDocument<256> doc;
  doc["id_device"] = 1;
  doc["action"] = tipoAccion;
  String body;
  serializeJson(doc, body);

  Serial.println("[ACTION] Enviando: " + body);

  int code = http.POST(body);
  Serial.printf("[ACTION] HTTP Code: %d\n", code);

  if (code == 200 || code == 201) {
    String resp = http.getString();
    Serial.println("[ACTION] Respuesta: " + resp);
  } else {
    String error = http.getString();
    Serial.println("[ACTION] Error: " + error);
    Serial.println("[ACTION] Body enviado: " + body);  // Para debugging
  }

  http.end();
}

// ================== FUNCIONES DE VISUALIZACIÓN ==================
void mostrarPantallaMotor(bool direccion) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(direccion ? "Giro Derecha" : "Giro Izquierda");
  lcd.setCursor(0, 1);
  lcd.print("Avance: 0%");
  porcentajeActual = 0;
}

void actualizarPorcentajeEnPantalla() {
  if (porcentajeActual <= 100) {
    lcd.setCursor(8, 1);
    lcd.print(porcentajeActual);
    lcd.print("%   ");
    
    if (Blynk.connected()) {
      Blynk.virtualWrite(V7, porcentajeActual);
    }
    
    Serial.printf("[MOTOR] Progreso: %d%%\n", porcentajeActual);
  }
}

void mostrarMenuPrincipal() {
  menuNivel = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SELECT OPTION");
  lcd.setCursor(0, 1);
  lcd.print("1.LED 2.MOTOR");
}

void mostrarMenuLED() {
  menuNivel = 1;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LED MENU");
  lcd.setCursor(0, 1);
  lcd.print("1=ON 2=OFF *=BACK");
}

void mostrarMenuMotor() {
  menuNivel = 2;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MOTOR MENU");
  lcd.setCursor(0, 1);
  lcd.print("1.<= 2.STOP 3.=>");
}

void mostrarMenuLogin() {
  lcd.clear();
  lcd.print("LOGIN OPCIONES:");
  lcd.setCursor(0, 1);
  lcd.print("1=MANUAL, OR APP");
  loginManualActivo = false;
  userName = "";
  logeado = false;
}

// ================== TECLADO ==================
void procesarTeclado(char key) {
  switch (menuNivel) {
    case 0:
      if (key == '1') mostrarMenuLED();
      else if (key == '2') mostrarMenuMotor();
      break;
    case 1:
      if (key == '1') {
        digitalWrite(ledPin1, HIGH);
        ledState1 = 1;
        enviarAccionBackend("LED_ON");
        
        // 🔥 ACTUALIZAR LED STATUS EN BLYNK
        if (Blynk.connected()) {
          Blynk.virtualWrite(V9, 1); // Sincronizar LED status
        }
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("LED ENCENDIDO ");
        mostrarMenuLED();
      } else if (key == '2') {
        digitalWrite(ledPin1, LOW);
        ledState1 = 0;
        lcd.clear();
        enviarAccionBackend("LED_OFF");
        
        // 🔥 ACTUALIZAR LED STATUS EN BLYNK
        if (Blynk.connected()) {
          Blynk.virtualWrite(V9, 0); // Sincronizar LED status
        }
        
        lcd.setCursor(0, 0);
        lcd.print("LED APAGADO ");
        mostrarMenuLED();
      } else if (key == '*') mostrarMenuPrincipal();
      break;
    case 2:
      if (key == '1') {
        iniciarGiroMotor(false);
        mostrarPantallaMotor(false);
        enviarAccionBackend("MOTOR_IZQ");
      } else if (key == '2') {
        pararMotor();
        enviarAccionBackend("MOTOR_STOP");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Motor Detenido ");
        lcd.setCursor(0, 1);
        lcd.print("1.<= 2.STOP 3.=>");
      } else if (key == '3') {
        iniciarGiroMotor(true);
        mostrarPantallaMotor(true);
        enviarAccionBackend("MOTOR_DER");
      } else if (key == '*') {
        pararMotor();
        mostrarMenuPrincipal();
      }
      break;
  }
}

// ================== LOGIN MANUAL ==================
void procesarLoginManual(char key) {
  static String username = "";
  static String password = "";
  static bool ingresandoUsuario = true;

  if (key == '1' && !loginManualActivo) {
    loginManualActivo = true;
    username = "";
    password = "";
    ingresandoUsuario = true;
    lcd.clear();
    lcd.print("Usuario:");
    lcd.setCursor(0, 1);
    return;
  }

  if (key == '*') {
    username = "";
    password = "";
    ingresandoUsuario = true;
    loginManualActivo = false;
    mostrarMenuLogin();
    return;
  }

  if (!loginManualActivo) return;

  if (key == '#') {
    if (ingresandoUsuario) {
      ingresandoUsuario = false;
      lcd.clear();
      lcd.print("Clave:");
      lcd.setCursor(0, 1);
      return;
    } else {
      lcd.clear();
      lcd.print("Autenticando...");
      bool ok = loginESP32(username, password);
      if (ok) {
        lcd.clear();
        lcd.print("Login exitoso");
        delayWithWebSocket(800);
        lcd.clear();
        lcd.print("Bienvenido:");
        lcd.setCursor(0, 1);
        lcd.print(userName);
        delayWithWebSocket(1500);
        lcd.clear();
        lcd.print("SISTEMA CONTROL");
        lcd.setCursor(0, 1);
        lcd.print("LED Y MOTOR CUL");
        delayWithWebSocket(4000);
        mostrarMenuPrincipal();
      } else {
        lcd.clear();
        lcd.print("Login incorrecto");
        delayWithWebSocket(1500);
        mostrarMenuLogin();
      }
      username = "";
      password = "";
      ingresandoUsuario = true;
      loginManualActivo = false;
      return;
    }
  }

  if (ingresandoUsuario) {
    username += key;
    lcd.setCursor(0, 1);
    lcd.print(username);
  } else {
    password += key;
    lcd.setCursor(0, 1);
    // Mostrar asteriscos en lugar de los caracteres reales
    String asteriscos = "";
    for (int i = 0; i < password.length(); i++) {
      asteriscos += "*";
    }
    lcd.print(asteriscos);
  }
}

// ================== LED ==================
void toggleLed1() {
  ledState1 = !ledState1;
  digitalWrite(ledPin1, ledState1);
  enviarAccionBackend(ledState1 ? "LED_ON" : "LED_OFF");
  incrementarContadorAccion(); //ENVIO A THINGSPEAK
  
  // 🔥 ACTUALIZAR LED STATUS EN BLYNK
  if (Blynk.connected()) {
    Blynk.virtualWrite(V9, ledState1); // Sincronizar LED status
  }
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LED: ");
  lcd.print(ledState1 ? "ON" : "OFF");
  delay(1500);
  mostrarMenuPrincipal();
  
  Serial.printf("[BOTON] LED Status (V9) actualizado: %s\n", ledState1 ? "ON" : "OFF");
}

// ================== MOTOR ==================
void iniciarGiroMotor(bool direccion) {
  Serial.printf("[MOTOR] Iniciando giro - Dirección: %s\n", direccion ? "DERECHA" : "IZQUIERDA");
  incrementarContadorAccion();//ENVIO A THINGSPEAK
  
  // Reiniciar variables del motor
  motorGirando = true;
  direccionMotor = direccion;
  pasosRestantes = pasosPorVuelta;
  pasoActual = 0; // Reiniciar secuencia
  porcentajeActual = 0;
  
  // LEDs físicos - CORREGIDO
  digitalWrite(ledPin4, LOW);  // LED STOP apagado
  
  if (direccion) {
    // DERECHA: encender ledPin5, apagar ledPin3
    digitalWrite(ledPin5, HIGH);
    digitalWrite(ledPin3, LOW);
  } else {
    // IZQUIERDA: encender ledPin3, apagar ledPin5  
    digitalWrite(ledPin3, HIGH);
    digitalWrite(ledPin5, LOW);
  }
  
  // Sincronizar Blynk - CORREGIDO
  if (Blynk.connected()) {
    if (direccion) {
      // DERECHA: V4=ON, V5=OFF, V6=OFF
      Blynk.virtualWrite(V4, 1);  // LED Derecha ON
      Blynk.virtualWrite(V5, 0);  // LED Stop OFF
      Blynk.virtualWrite(V6, 0);  // LED Izquierda OFF
    } else {
      // IZQUIERDA: V4=OFF, V5=OFF, V6=ON  
      Blynk.virtualWrite(V4, 0);  // LED Derecha OFF
      Blynk.virtualWrite(V5, 0);  // LED Stop OFF
      Blynk.virtualWrite(V6, 1);  // LED Izquierda ON
    }
    Blynk.virtualWrite(V7, 0);  // Porcentaje a 0%
  }
  
  // Mostrar en LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(direccion ? "Giro Derecha" : "Giro Izquierda");
  lcd.setCursor(0, 1);
  lcd.print("Avance: 0%");
  
  Serial.println("[MOTOR] ✅ Giro iniciado correctamente");
}

void pararMotor() {
  motorGirando = false;
  incrementarContadorAccion();//ENVIO A THINGSPEAK
  
  // Apagar pines del motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // LEDs físicos - CORREGIDO
  digitalWrite(ledPin3, LOW);   // LED Izquierda apagado
  digitalWrite(ledPin5, LOW);   // LED Derecha apagado
  digitalWrite(ledPin4, HIGH);  // LED Stop encendido
  
  // Sincronizar Blynk - CORREGIDO
  if (Blynk.connected()) {
    Blynk.virtualWrite(V4, 0);  // LED Derecha OFF
    Blynk.virtualWrite(V5, 1);  // LED Stop ON
    Blynk.virtualWrite(V6, 0);  // LED Izquierda OFF
  }
  
  Serial.println("[MOTOR] 🛑 Motor detenido");
}

void ejecutarPaso(bool direccion) {
  pasoActual = (direccion) ? (pasoActual + 1) % 8 : (pasoActual + 7) % 8;
  digitalWrite(IN1, pasoSecuencia[pasoActual][0]);
  digitalWrite(IN2, pasoSecuencia[pasoActual][1]);
  digitalWrite(IN3, pasoSecuencia[pasoActual][2]);
  digitalWrite(IN4, pasoSecuencia[pasoActual][3]);
}

void delayWithWebSocket(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    webSocket.loop();
    delay(10);
  }
}

void reconectarWebSocket() {
  Serial.println("[WS] 🔄 Reconectando WebSocket...");
  webSocket.disconnect();
  delay(1000);
  webSocket.begin(API_HOST, API_PORT, "/ws/device/1");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
}


// ================== FUNCIONES THINGSPEAK ================== //
void setupThingSpeak() {
  ThingSpeak.begin(client);
  Serial.println("[THINGSPEAK] ✅ Configurado correctamente");
  Serial.printf("[THINGSPEAK] Channel ID: %d\n", CHANNEL_ID);
}

int calcularHealthScore() {
  int score = 100;
  
  // Penalizar por errores de conexión
  if (!webSocket.isConnected()) score -= 20;
  if (!Blynk.connected()) score -= 20;
  if (WiFi.status() != WL_CONNECTED) score -= 30;
  
  // Bonus por actividad reciente
  if (motorGirando) score += 10;
  if (ledState1) score += 5;
  
  // Asegurar que esté en rango 0-100
  return constrain(score, 0, 100);
}

void incrementarContadorAccion() {
  actionCounter++;
  if (actionCounter > 10000) actionCounter = 1; // Reset para evitar overflow
  Serial.printf("[THINGSPEAK] Contador acciones: %d\n", actionCounter);
}

void enviarDatosThingSpeak() {
  if (!thingSpeakEnabled) return;
  
  if (millis() - lastThingSpeakUpdate >= thingSpeakInterval) {
    Serial.println("[THINGSPEAK] 📤 Enviando datos...");
    
    // Calcular valores para ThingSpeak
    int motorState = !motorGirando ? 0 : (direccionMotor ? 2 : 1);
    int ledState = ledState1 ? 1 : 0;
    
    // Calcular duración de sesión de giro
    if (motorGirando && motorStartTime == 0) {
      motorStartTime = millis();
      Serial.println("[THINGSPEAK] Iniciando temporizador de giro");
    } else if (!motorGirando && motorStartTime > 0) {
      motorDuration = (millis() - motorStartTime) / 1000;
      Serial.printf("[THINGSPEAK] Giro completado. Duración: %d segundos\n", motorDuration);
      motorStartTime = 0;
    }
    
    // Calcular health score del sistema
    int systemHealth = calcularHealthScore();
    
    // Establecer campos de ThingSpeak
    ThingSpeak.setField(FIELD_MOTOR_STATE, motorState);
    ThingSpeak.setField(FIELD_MOTOR_PROGRESS, porcentajeActual);
    ThingSpeak.setField(FIELD_LED_STATE, ledState);
    ThingSpeak.setField(FIELD_MOTOR_DURATION, motorDuration);
    ThingSpeak.setField(FIELD_ACTION_COUNT, actionCounter);
    ThingSpeak.setField(FIELD_SYSTEM_HEALTH, systemHealth);
    
    // Enviar datos a ThingSpeak
    int response = ThingSpeak.writeFields(CHANNEL_ID, WRITE_API_KEY);
    
    if (response == 200) {
      Serial.println("[THINGSPEAK] ✅ Datos enviados correctamente");
      Serial.printf("[THINGSPEAK] Estado Motor: %d, Progreso: %d%%, LED: %d, Health: %d\n", 
                    motorState, porcentajeActual, ledState, systemHealth);
    } else {
      Serial.printf("[THINGSPEAK] ❌ Error enviando datos. Código: %d\n", response);
      
      // Intentar reconectar si hay error persistente
      if (response == -1 || response == -2) {
        Serial.println("[THINGSPEAK] 🔄 Reconectando...");
        ThingSpeak.begin(client);
      }
    }
    
    lastThingSpeakUpdate = millis();
  }
}