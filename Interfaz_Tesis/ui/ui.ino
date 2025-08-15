/***********************************************
 * TANARI DP - VERSIÓN 3.2 (Sistema de Monitoreo Avanzado)
 * 
 * Descripción: 
 *   Sistema portátil de monitoreo ambiental para medición de gases de efecto invernadero (CO2 y Metano),
 *   temperatura y humedad. Integra gestión de energía, interfaz gráfica avanzada y comunicación inalámbrica BLE.
 * 
 * Características principales:
 *   - Medición precisa de CO2 con sensor Sensirion SCD41
 *   - Detección de Metano con sensor MQ-4 calibrado
 *   - Circuito optimizado para medición de batería (ultra bajo consumo)
 *   - Interfaz gráfica fluida con LVGL y pantalla táctil ILI9488
 *   - Comunicación BLE 5.0 con notificaciones en tiempo real
 *   - Transmisión UART para integración con sistemas externos
 *   - Gestión dinámica de brillo de pantalla mediante PWM
 * 
 * Especificaciones técnicas:
 *   - Microcontrolador: ESP32-S3 WROON-1 R16N8
 *   - Pantalla: 3.5" táctil (320x480 píxeles)
 *   - Sensores: 
 *        - Sensirion SCD41 (CO2, temperatura, humedad)
 *        - MQ-4 (Metano)
 *   - Comunicación: BLE 5.0, UART (Serial2)
 *   - Alimentación: Batería LiPo 3.7V (rango operativo: 2.6V - 4.2V)
 * 
 * Autor: [Mendez. J  y  Rivera. M]
 * Fecha: [15-08-2025]
 * Licencia: MIT
 ***********************************************/

/***********************************************
 * SECCIÓN DE INCLUSIÓN DE BIBLIOTECAS
 ***********************************************/
#include <lvgl.h>               // Librería para interfaz gráfica (versión 9.1.0)
#include <TFT_eSPI.h>           // Controlador de pantalla TFT
#include <ui.h>                 // Interfaz de usuario generada por SquareLine Studio
#include <Arduino.h>            // Funciones básicas de Arduino
#include <Wire.h>               // Comunicación I2C para sensores
#include <SensirionI2cScd4x.h>  // Driver para sensor SCD41
#include <BLEDevice.h>          // Funcionalidad BLE (Bluetooth Low Energy)
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>     // Comunicación serial UART
#include <driver/ledc.h>        // Control PWM para gestión de brillo

/***********************************************
 * DEFINICIONES DE HARDWARE Y CONFIGURACIONES
 ***********************************************/
// UUIDs para servicio BLE personalizado
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_NOTIFY "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define NO_ERROR 0  // Código de operación exitosa

// Configuración de brillo de pantalla
#define BACKLIGHT_PIN 45        // GPIO para control de backlight
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TIMER LEDC_TIMER_0
#define PWM_FREQ 5000           // Frecuencia PWM en Hz (óptima para pantallas)
#define PWM_RESOLUTION LEDC_TIMER_13_BIT // 8192 niveles de brillo (13 bits)
#define MIN_DUTY_VALUE 100      // Valor mínimo para evitar apagado completo
#define MAX_DUTY_VALUE 8191     // Valor máximo (2^13 - 1)

// Configuración de medición de batería
#define BATTERY_ADC_PIN 6       // GPIO6 para lectura ADC
#define BATTERY_CONTROL_PIN 5   // GPIO5 para control de transistor
#define BATTERY_FULL_VOLTAGE 4.2f // Voltaje al 100% de carga (LiPo)
#define BATTERY_EMPTY_VOLTAGE 2.6f // Voltaje al 0% de carga
#define ADC_REF_VOLTAGE 3.3f    // Voltaje de referencia del ADC
#define ADC_RESOLUTION 4095.0f  // Resolución de 12 bits (0-4095)

/***********************************************
 * NAMESPACES PARA GESTIÓN DE ESTADOS GLOBALES
 ***********************************************/
namespace Sensor {
  bool dataReady = false;       // Bandera de datos disponibles del SCD41
  bool ErrorSCD = false;        // Estado de error del sensor SCD41
  int16_t error;                // Código de error específico
  uint16_t co2Concentration;    // CO2 en ppm
  float temperature;            // Temperatura en °C
  float relativeHumidity;       // Humedad relativa en %
  const double RL = 30;         // Resistencia de carga para MQ-4
  const int Ch4_pin = 1;        // Pin analógico para MQ-4 (GPIO1)
  double Ro = 3240;             // Resistencia en aire limpio (valor calibrado)
  double CH4_ppm = 0;           // Concentración de Metano en ppm
}

namespace Comms {
  bool bleConnected = false;    // Estado de conexión BLE activa
  bool Acople = true;           // Habilitar transmisión UART
  bool bleEnabled = false;      // Estado de módulo BLE habilitado
  BLEServer *pServer = NULL;    // Instancia de servidor BLE
  BLECharacteristic *pCharacteristicNotify; // Característica BLE para notificaciones
  BLEAdvertising *pAdvertising; // Objeto para publicidad BLE
}

namespace Display {
  static const uint16_t screenWidth = 320;   // Ancho de pantalla en píxeles
  static const uint16_t screenHeight = 480;  // Alto de pantalla en píxeles
  enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 }; // Tamaño buffer LVGL
  static lv_color_t buf[SCREENBUFFER_SIZE_PIXELS]; // Buffer gráfico para LVGL
  TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); // Controlador de pantalla
  int screenBrightness = 50;    // Brillo inicial (0-100%)
}

namespace Power {
  float batteryVoltage = 0.0f;  // Voltaje actual de batería
  int batteryPercentage = 0;    // Porcentaje de carga calculado (0-100%)
}

// Declaración adelantada de función
void updateBLEStatusLabel();

/***********************************************
 * MANEJADOR DE EVENTOS BLE (CALLBACKS)
 ***********************************************/
class MyServerCallbacks : public BLEServerCallbacks {
  /**
   * @brief Maneja eventos de conexión BLE
   * @param pServer Instancia del servidor BLE
   */
  void onConnect(BLEServer *pServer) {
    Comms::bleConnected = true;
    Serial.println("BLE conectado!");
    updateBLEStatusLabel();  // Actualizar UI inmediatamente
  };

  /**
   * @brief Maneja eventos de desconexión BLE
   * @param pServer Instancia del servidor BLE
   */
  void onDisconnect(BLEServer *pServer) {
    Comms::bleConnected = false;
    Serial.println("BLE desconectado!");
    updateBLEStatusLabel();  // Actualizar UI inmediatamente
    
    // Reiniciar publicidad si BLE sigue habilitado
    if (Comms::bleEnabled) {
      BLEDevice::startAdvertising();
    }
  };
};

/***********************************************
 * FUNCIONES DE LVGL (MANEJO DE PANTALLA)
 ***********************************************/
#if LV_USE_LOG != 0
/**
 * @brief Callback para logs de LVGL (vía Serial)
 * @param buf Mensaje a imprimir
 */
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif

/**
 * @brief Renderizado de gráficos en pantalla
 * @param disp Objeto display LVGL
 * @param area Área a actualizar
 * @param pixelmap Mapa de píxeles a renderizar
 */
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  Display::tft.startWrite();
  Display::tft.setAddrWindow(area->x1, area->y1, w, h);
  Display::tft.pushColors((uint16_t *)pixelmap, w * h, true);
  Display::tft.endWrite();

  lv_disp_flush_ready(disp);  // Notificar a LVGL que el renderizado completó
}

/**
 * @brief Lectura de entrada táctil
 * @param indev_driver Dispositivo de entrada LVGL
 * @param data Estructura para almacenar datos táctiles
 */
void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data) {
  uint16_t touchX, touchY;
  bool touched = Display::tft.getTouch(&touchX, &touchY, 600);  // Umbral de detección

  data->state = touched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  if (touched) {
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

/**
 * @brief Obtención de ticks del sistema para LVGL
 * @return Tiempo transcurrido en milisegundos
 */
static uint32_t my_tick_get_cb(void) {
  return millis();  // Usar función nativa de Arduino
}

/***********************************************
 * FUNCIONES DE CONTROL PRINCIPAL
 ***********************************************/
#ifdef __cplusplus
extern "C" {
  /**
   * @brief Ajusta el brillo de pantalla mediante PWM
   * @param percentage Porcentaje de brillo (0-100)
   */
  void setScreenBrightness(int percentage) {
    Display::screenBrightness = constrain(percentage, 0, 100);
    uint32_t duty = map(Display::screenBrightness, 0, 100, MIN_DUTY_VALUE, MAX_DUTY_VALUE);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
    Serial.printf("Brillo ajustado al %d%% (Ciclo PWM: %d)\n", Display::screenBrightness, duty);
  }

  /**
   * @brief Obtiene el nivel de brillo actual
   * @return Porcentaje de brillo (0-100)
   */
  int getScreenBrightness() {
    return Display::screenBrightness;
  }

  /**
   * @brief Habilita y activa el módulo BLE
   */
  void BLEOn() {
    if (Comms::bleEnabled) return;  // Evitar acciones redundantes
    Serial.println("Activando BLE...");
    Comms::pAdvertising->start();  // Iniciar publicidad
    Comms::bleEnabled = true;
    Serial.println("BLE activado y publicitando");
    updateBLEStatusLabel();  // Actualizar UI
  }

  /**
   * @brief Deshabilita el módulo BLE
   */
  void BLEOff() {
    if (!Comms::bleEnabled) return;  // Evitar acciones redundantes
    Serial.println("Desactivando BLE...");
    Comms::pAdvertising->stop();  // Detener publicidad
    Comms::bleEnabled = false;
    Comms::bleConnected = false;  // Forzar estado desconectado
    Serial.println("BLE desactivado");
    updateBLEStatusLabel();  // Actualizar UI
  }
}
#endif

/***********************************************
 * FUNCIONES DE LECTURA DE SENSORES
 ***********************************************/
/**
 * @brief Función de mapeo lineal entre rangos
 * @param x Valor a mapear
 * @param x1 Límite inferior de entrada
 * @param x2 Límite superior de entrada
 * @param y1 Límite inferior de salida
 * @param y2 Límite superior de salida
 * @return Valor mapeado al nuevo rango
 */
double mapear(double x, double x1, double x2, double y1, double y2) {
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

/**
 * @brief Lee y calcula concentración de Metano (MQ-4)
 * @note Realiza promedio de 10 lecturas para mayor estabilidad
 */
void LeerCH4() {
  double Slectura = 0;
  // Promedio de 10 lecturas para reducir ruido
  for (int i = 0; i < 10; i++) {
    Slectura += analogRead(Sensor::Ch4_pin);
    delay(1);  // Pequeño delay entre lecturas
  }
  
  double ADCsensorMQ = Slectura / 10.0;
  // Convertir valor ADC a voltaje (0-3.3V)
  double VsensorMQ = mapear(ADCsensorMQ, 0, 4095, 0, 3.3);
  // Calcular resistencia del sensor en presencia de gas
  double Rs = Sensor::RL * ((3.3 - VsensorMQ) / VsensorMQ);
  // Calcular ratio Rs/Ro
  double RatioMQ = Rs / Sensor::Ro;
  // Calcular concentración (fórmula específica para MQ-4)
  double exponente = (-log10(RatioMQ) + 1.052) / 0.3512;
  Sensor::CH4_ppm = pow(10, exponente);  // Resultado en ppm
}

// Instancia del sensor SCD41
SensirionI2cScd4x scd4x;

/**
 * @brief Lee datos del sensor SCD41 (CO2, temperatura, humedad)
 * @note Verifica disponibilidad de datos antes de leer
 */
void LeerCO2() {
  bool localDataReady = false;
  // Verificar si hay datos nuevos disponibles
  Sensor::error = scd4x.getDataReadyStatus(localDataReady);

  if (Sensor::error != NO_ERROR) {
    Sensor::ErrorSCD = true;  // Marcar error
    return;
  }

  if (localDataReady) {
    // Leer mediciones actuales
    Sensor::error = scd4x.readMeasurement(
      Sensor::co2Concentration,
      Sensor::temperature,
      Sensor::relativeHumidity);
    // Actualizar estado de error
    Sensor::ErrorSCD = (Sensor::error != NO_ERROR);
    Sensor::dataReady = localDataReady;  // Confirmar lectura
  }
}

/***********************************************
 * FUNCIONES DE INTERFAZ DE USUARIO
 ***********************************************/
/**
 * @brief Actualiza valores de sensores en la pantalla
 * @note Limitado a 1 actualización por segundo para evitar sobrecarga
 */
void ActualizarPantalla() {
  static uint32_t lastUpdate = 0;
  // Limitar actualizaciones a 1Hz
  if (millis() - lastUpdate < 1000) return;

  // Actualizar etiquetas con nuevos valores
  lv_label_set_text(ui_resulCO2Label, (String(Sensor::co2Concentration) + " ppm").c_str());
  lv_label_set_text(ui_resulCH4Label, (String(Sensor::CH4_ppm, 1) + " ppm").c_str());
  lv_label_set_text(ui_resulTempLabel, (String(Sensor::temperature, 1) + " °C").c_str());
  lv_label_set_text(ui_resulHumLabel, (String(Sensor::relativeHumidity, 1) + " %").c_str());

  lastUpdate = millis();  // Registrar último tiempo de actualización
}

/**
 * @brief Lee nivel de batería con circuito optimizado
 * @note Circuito se activa solo durante 150ms para minimizar consumo
 */
void readBatteryLevel() {
  // Activar circuito de medición
  digitalWrite(BATTERY_CONTROL_PIN, HIGH);
  delay(150); // Esperar estabilización (tiempo característico del circuito)
  
  int adcValue = analogRead(BATTERY_ADC_PIN);
  
  // Desactivar circuito inmediatamente después de leer
  digitalWrite(BATTERY_CONTROL_PIN, LOW);
  
  // Calcular voltaje real considerando divisor (R1=270K, R2=1M -> factor 1.27)
  float voltage = (adcValue / ADC_RESOLUTION) * ADC_REF_VOLTAGE * 1.27f;
  
  // Calcular porcentaje (mapeo lineal entre 2.6V y 4.2V)
  Power::batteryVoltage = voltage;
  Power::batteryPercentage = constrain(
    map(voltage * 100, BATTERY_EMPTY_VOLTAGE * 100, BATTERY_FULL_VOLTAGE * 100, 0, 100),
    0, 100
  );
}

/**
 * @brief Actualiza indicador de batería en todas las pantallas
 */
void updateBatteryIndicator() {
  char batText[8];
  snprintf(batText, sizeof(batText), "%d%%", Power::batteryPercentage);
  
  // Actualizar todas las instancias de la UI
  lv_label_set_text(ui_batPorcentajeLabel, batText);
  lv_label_set_text(ui_batPorcentajeLabel2, batText);
  lv_label_set_text(ui_batPorcentajeLabel3, batText);
}

/**
 * @brief Actualiza indicadores de estado de acople
 * @note Actualmente simulado (pendiente de integración con UGV)
 */
void updateAcopleIndicator() {
  bool acopleConnected = false; // Simulación - cambiar cuando se integre UGV
  
  // Actualizar estado en todas las pantallas
  if(acopleConnected) {
    // Estado "Conectado": mostrar panel verde, ocultar rojo
    lv_obj_clear_flag(ui_onAcoplePanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_offAcoplePanel, LV_OBJ_FLAG_HIDDEN);
    // Repetir para otras pantallas
    lv_obj_clear_flag(ui_onAcoplePanel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_offAcoplePanel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_onAcoplePanel3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_offAcoplePanel3, LV_OBJ_FLAG_HIDDEN);
  } else {
    // Estado "Desconectado": mostrar panel rojo, ocultar verde
    lv_obj_add_flag(ui_onAcoplePanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_offAcoplePanel, LV_OBJ_FLAG_HIDDEN);
    // Repetir para otras pantallas
    lv_obj_add_flag(ui_onAcoplePanel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_offAcoplePanel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_onAcoplePanel3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_offAcoplePanel3, LV_OBJ_FLAG_HIDDEN);
  }
}

/**
 * @brief Actualiza indicadores visuales de BLE
 * @note Maneja tres estados: Apagado, Desconectado, Conectado
 */
void updateBLEIndicators() {
  if (!Comms::bleEnabled) {
    // Estado APAGADO: mostrar panel gris
    lv_obj_clear_flag(ui_offBLEPanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_desconectadoBLEPanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_conectadoBLEPanel, LV_OBJ_FLAG_HIDDEN);
    // Repetir para otras pantallas
    lv_obj_clear_flag(ui_offBLEPanel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_desconectadoBLEPanel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_conectadoBLEPanel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_offBLEPanel3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_desconectadoBLEPanel3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_conectadoBLEPanel3, LV_OBJ_FLAG_HIDDEN);
  } else {
    if (Comms::bleConnected) {
      // Estado CONECTADO: mostrar panel verde
      lv_obj_add_flag(ui_offBLEPanel, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_desconectadoBLEPanel, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(ui_conectadoBLEPanel, LV_OBJ_FLAG_HIDDEN);
      // Repetir para otras pantallas
      lv_obj_add_flag(ui_offBLEPanel2, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_desconectadoBLEPanel2, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(ui_conectadoBLEPanel2, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_offBLEPanel3, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_desconectadoBLEPanel3, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(ui_conectadoBLEPanel3, LV_OBJ_FLAG_HIDDEN);
    } else {
      // Estado DESCONECTADO: mostrar panel amarillo
      lv_obj_add_flag(ui_offBLEPanel, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(ui_desconectadoBLEPanel, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_conectadoBLEPanel, LV_OBJ_FLAG_HIDDEN);
      // Repetir para otras pantallas
      lv_obj_add_flag(ui_offBLEPanel2, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(ui_desconectadoBLEPanel2, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_conectadoBLEPanel2, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_offBLEPanel3, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(ui_desconectadoBLEPanel3, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_conectadoBLEPanel3, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

/**
 * @brief Actualiza etiqueta de estado BLE en pantalla de configuración
 */
void updateBLEStatusLabel() {
  if (!Comms::bleEnabled) {
    lv_label_set_text(ui_bleOnOffLabel, "Apagado");
  } else {
    lv_label_set_text(ui_bleOnOffLabel, 
      Comms::bleConnected ? "Conectado" : "Desconectado");
  }
  updateBLEIndicators(); // Sincronizar indicadores visuales
}

/***********************************************
 * CONFIGURACIÓN INICIAL DEL SISTEMA (SETUP)
 ***********************************************/
void setup() {
  Serial.begin(115200); // Inicialización de serial para depuración
  Serial2.begin(115200, SERIAL_8N1, 11, 12); // UART para comunicación externa (GPIO11: TX, GPIO12: RX)

  // ===== CONFIGURACIÓN DE PINES =====
  // Circuito de medición de batería
  pinMode(BATTERY_CONTROL_PIN, OUTPUT);
  digitalWrite(BATTERY_CONTROL_PIN, LOW); // Iniciar con circuito desactivado
  pinMode(BATTERY_ADC_PIN, INPUT);

  // ===== INICIALIZACIÓN DE SENSORES =====
  Wire.begin(); // Iniciar comunicación I2C
  scd4x.begin(Wire, 0x62); // Dirección I2C del SCD41 (0x62)
  scd4x.startPeriodicMeasurement(); // Iniciar mediciones periódicas
  pinMode(Sensor::Ch4_pin, INPUT); // Configurar pin de MQ-4 como entrada

  // ===== CONFIGURACIÓN PWM PARA BRILLO =====
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = PWM_TIMER,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
    .gpio_num = BACKLIGHT_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = PWM_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = PWM_TIMER,
    .duty = map(Display::screenBrightness, 0, 100, MIN_DUTY_VALUE, MAX_DUTY_VALUE),
    .hpoint = 0,
  };
  ledc_channel_config(&channel_conf);

  // ===== INICIALIZACIÓN DE LVGL Y PANTALLA =====
  lv_init(); // Inicializar LVGL
  Display::tft.begin(); // Iniciar pantalla
  Display::tft.setRotation(2); // Ajustar según orientación física del display
  
  // Calibración de pantalla táctil (valores específicos de hardware)
  uint16_t calData[5] = { 282, 3469, 292, 3592, 2 };
  Display::tft.setTouch(calData);
  
  // Configuración de LVGL
  lv_display_t *disp = lv_display_create(Display::screenWidth, Display::screenHeight);
  lv_display_set_buffers(disp, Display::buf, NULL, sizeof(Display::buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(disp, my_disp_flush);
  
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);
  
  lv_tick_set_cb(my_tick_get_cb); // Configurar fuente de tiempo

  // Inicializar interfaz de usuario generada
  ui_init();
  
  // Registrar manejadores de eventos UI
  lv_obj_add_event_cb(ui_bleSwitch, ui_event_bleSwitch, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(ui_controlSlider, ui_event_controlSlider, LV_EVENT_VALUE_CHANGED, NULL);

  // ===== CONFIGURACIÓN INICIAL DE BLE =====
  Serial.println("Inicializando stack BLE...");
  BLEDevice::init("TANARI DP"); // Nombre visible del dispositivo Bluetooth
  Comms::pServer = BLEDevice::createServer();
  Comms::pServer->setCallbacks(new MyServerCallbacks());
  
  // Crear servicio y característica BLE
  BLEService *pService = Comms::pServer->createService(SERVICE_UUID);
  Comms::pCharacteristicNotify = pService->createCharacteristic(
    CHARACTERISTIC_UUID_NOTIFY,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  Comms::pCharacteristicNotify->addDescriptor(new BLE2902());
  
  pService->start(); // Iniciar servicio
  Comms::pAdvertising = BLEDevice::getAdvertising();
  Comms::pAdvertising->addServiceUUID(SERVICE_UUID); // Anunciar servicio
  Comms::pAdvertising->setScanResponse(true); // Permitir respuestas a escaneos
  Serial.println("Stack BLE inicializado.");

  // ===== ESTADO INICIAL DEL SISTEMA =====
  Comms::bleEnabled = false;
  lv_obj_clear_state(ui_bleSwitch, LV_STATE_CHECKED); // Switch BLE en posición apagado
  BLEOff(); // Asegurar estado inicial coherente
  lv_slider_set_value(ui_controlSlider, Display::screenBrightness, LV_ANIM_OFF); // Sincronizar slider
  
  // Actualizar UI con valores iniciales
  updateBLEStatusLabel(); // Estado BLE
  char initial_brightness_buffer[8];
  snprintf(initial_brightness_buffer, sizeof(initial_brightness_buffer), "%d%%", Display::screenBrightness);
  lv_label_set_text(ui_porcentajeLabel, initial_brightness_buffer); // Brillo inicial

  Serial.println("Sistema Tanari DP inicializado");

  // ===== CONFIGURACIÓN DE TEMPORIZADORES =====
  // Temporizador para lectura de sensores (500ms)
  lv_timer_create([](lv_timer_t *timer) {
    LeerCO2();
    LeerCH4();
  }, 500, NULL);

  // Temporizador para actualización de UI (1000ms)
  lv_timer_create([](lv_timer_t *timer) {
    ActualizarPantalla();
    updateBLEStatusLabel(); // Actualizar estado BLE periódicamente
  }, 1000, NULL);

  // Temporizador para medición de batería (30000ms = 30s)
  lv_timer_create([](lv_timer_t *timer) {
    readBatteryLevel();
    updateBatteryIndicator();
  }, 30000, NULL);

  // Temporizador para actualización de indicadores de estado (1000ms)
  lv_timer_create([](lv_timer_t *timer) {
    updateAcopleIndicator();
    updateBLEIndicators();
  }, 1000, NULL);
}

/***********************************************
 * BUCLE PRINCIPAL DEL SISTEMA (LOOP)
 ***********************************************/
void loop() {
  // ===== TRANSMISIÓN DE DATOS POR BLE =====
  if (Comms::bleEnabled && Comms::bleConnected && !Sensor::ErrorSCD) {
    String datosTX = String(Sensor::co2Concentration) + ";" + 
                    String(Sensor::CH4_ppm) + ";" + 
                    String(Sensor::temperature) + ";" + 
                    String(Sensor::relativeHumidity);
    Comms::pCharacteristicNotify->setValue(datosTX.c_str());
    Comms::pCharacteristicNotify->notify(); // Enviar a dispositivos conectados
  }

  // ===== TRANSMISIÓN POR UART (PARA ACOPLE) =====
  if (Comms::Acople && !Sensor::ErrorSCD) {
    String datosTX = String(Sensor::co2Concentration) + ";" + 
                    String(Sensor::CH4_ppm) + ";" + 
                    String(Sensor::temperature) + ";" + 
                    String(Sensor::relativeHumidity);
    Serial2.println(datosTX); // Enviar por Serial2
  }

  // ===== LECTURA PERIÓDICA DE BATERÍA =====
  static uint32_t lastBatteryRead = 0;
  if (millis() - lastBatteryRead > 30000) {
    readBatteryLevel();
    updateBatteryIndicator();
    lastBatteryRead = millis();
  }

  lv_timer_handler(); // Procesar eventos de LVGL (UI y temporizadores)
  delay(5); // Pequeño delay para permitir gestión de otras tareas
}