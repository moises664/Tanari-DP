/***********************************************
 * TANARI DP - VERSIÓN 3.1 (Sistema de Monitoreo)
 *
 * Descripción: Firmware para el sistema de monitoreo ambiental TANARI DP.
 * Permite la lectura de sensores de CO2 (Sensirion SCD41) y Metano (MQ-4),
 * control de brillo de pantalla y gestión de comunicación BLE.
 *
 * Cambios en esta versión:
 *
 * 1. Control de BLE mediante switch en la interfaz (se elimina botón físico).
 * 2. Control de brillo de pantalla (PWM) desde la interfaz de usuario.
 * 3. Visualización del estado del BLE (apagado, desconectado, conectado) en el panel principal.
 * 4. Refactorización de eventos de la UI en un archivo `ui_events.c`.
 *
 * Componentes:
 * - ESP32-S3 con pantalla TFT ILI9488 integrada
 * - Sensor Sensirion SCD41 (CO2, temperatura, humedad)
 * - Sensor MQ-4 (Metano)
 * - Comunicación BLE para conexión móvil
 ***********************************************/

#include <lvgl.h>      // Librería LVGL para la interfaz gráfica
#include <TFT_eSPI.h>    // Librería para el control de la pantalla TFT
#include <ui.h>          // Archivo de la UI generado por SquareLine Studio
#include <Arduino.h>     // Funciones básicas de Arduino
#include <Wire.h>        // Librería para comunicación I2C (para SCD41)
#include <SensirionI2cScd4x.h> // Librería para el sensor SCD41
#include <BLEDevice.h>   // Librerías para BLE
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h> // Para comunicación UART (Serial2)
#include <driver/ledc.h> // Usamos la biblioteca nativa del driver LEDC para PWM

// =============================
// DEFINICIONES Y CONFIGURACIONES
// =============================

// UUIDs para servicio y característica BLE (identificadores únicos)
#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_NOTIFY "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define NO_ERROR 0 // Definición para indicar que no hay error en lecturas

// --- Configuración de Brillo (CONTROL PWM) ---
// Pin GPIO 45 utilizado para el control del backlight de la pantalla
#define BACKLIGHT_PIN 45
// Canal PWM a usar (el ESP32 tiene varios canales, del 0 al 15)
#define PWM_CHANNEL LEDC_CHANNEL_0
// Temporizador LEDC a usar (el ESP32 tiene varios temporizadores, del 0 al 3)
#define PWM_TIMER LEDC_TIMER_0
// Frecuencia del PWM en Hz (5000 Hz es un valor común)
#define PWM_FREQ 5000
// Resolución de 13 bits (esto significa 2^13 = 8192 pasos, de 0 a 8191)
#define PWM_RESOLUTION LEDC_TIMER_13_BIT
// Valor mínimo del ciclo de trabajo PWM (para que la pantalla no se apague completamente)
#define MIN_DUTY_VALUE 100
// Valor máximo del ciclo de trabajo PWM (2^13 - 1)
#define MAX_DUTY_VALUE 8191


// ----------------------------------
// DECLARACIÓN DE VARIABLES GLOBALES
// ----------------------------------

// Namespace para variables relacionadas con los sensores
namespace Sensor {
    bool dataReady = false;    // Indica si los datos del sensor CO2 están listos
    bool ErrorSCD = false;     // Indica si hay un error con el sensor SCD41
    int16_t error;             // Código de error del sensor SCD41
    uint16_t co2Concentration; // Concentración de CO2 en ppm
    float temperature;         // Temperatura en °C
    float relativeHumidity;    // Humedad relativa en %

    const double RL = 30;      // Resistencia de carga para el sensor MQ-4
    const int Ch4_pin = 1;     // Pin analógico conectado al sensor MQ-4
    double Ro = 3240;          // Resistencia del sensor en aire limpio (calibración)
    double CH4_ppm = 0;        // Concentración de Metano en ppm
}

// Namespace para variables relacionadas con la comunicación
namespace Comms {
    bool bleConnected = false;   // Estado de conexión BLE (true si hay un cliente conectado)
    bool Acople = true;          // Flag para la transmisión por UART (Serial2)
    bool bleEnabled = false;     // Estado de activación del módulo BLE (true si está encendido/publicitando)

    BLEServer *pServer = NULL;             // Puntero al servidor BLE
    BLECharacteristic *pCharacteristicNotify; // Puntero a la característica de notificación BLE
    BLEAdvertising *pAdvertising;          // Puntero para la publicidad BLE
}

// Namespace para variables relacionadas con la pantalla y la UI
namespace Display {
    static const uint16_t screenWidth = 320;    // Ancho de la pantalla en píxeles
    static const uint16_t screenHeight = 480; // Alto de la pantalla en píxeles
    // Tamaño del buffer para LVGL (aproximadamente 1/10 del total de píxeles)
    enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 };
    static lv_color_t buf[SCREENBUFFER_SIZE_PIXELS]; // Buffer de dibujo de LVGL
    TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); // Objeto de la librería TFT_eSPI
    int screenBrightness = 50; // Brillo inicial en porcentaje (0-100)
}

// =================================
// CLASES PARA MANEJO DE EVENTOS BLE
// =================================
/**
 * @brief Clase de callback para manejar eventos del servidor BLE (conexión/desconexión).
 */
class MyServerCallbacks : public BLEServerCallbacks {
    /**
     * @brief Se llama cuando un cliente BLE se conecta al servidor.
     * @param pServer Puntero al servidor BLE.
     */
    void onConnect(BLEServer* pServer) {
        Comms::bleConnected = true; // Actualiza el estado de conexión
        Serial.println("BLE conectado!"); // Imprime un mensaje en el Serial
        updateBLEStatusLabel(); // Actualiza el texto en la UI de inmediato
    };

    /**
     * @brief Se llama cuando un cliente BLE se desconecta del servidor.
     * @param pServer Puntero al servidor BLE.
     */
    void onDisconnect(BLEServer* pServer) {
        Comms::bleConnected = false; // Actualiza el estado de conexión
        Serial.println("BLE desconectado!"); // Imprime un mensaje en el Serial
        updateBLEStatusLabel(); // Actualiza el texto en la UI de inmediato
        // Si el BLE aún está habilitado, reiniciar la publicidad para permitir reconexión
        if (Comms::bleEnabled) {
            BLEDevice::startAdvertising();
        }
    };
};

// =====================================
// FUNCIONES DE CONFIGURACIÓN DE LVGL
// =====================================

#if LV_USE_LOG != 0
/**
 * @brief Función de callback para imprimir logs de LVGL en el Serial.
 * @param buf Cadena de caracteres a imprimir.
 */
void my_print(const char * buf) {
    Serial.printf(buf);
    Serial.flush();
}
#endif

/**
 * @brief Función de callback para que LVGL dibuje en la pantalla TFT.
 * @param disp Puntero al objeto de display LVGL.
 * @param area Área de la pantalla a actualizar.
 * @param pixelmap Puntero al buffer de píxeles a dibujar.
 */
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap) {
    uint32_t w = area->x2 - area->x1 + 1; // Ancho del área
    uint32_t h = area->y2 - area->y1 + 1; // Alto del área

    Display::tft.startWrite(); // Iniciar escritura optimizada
    Display::tft.setAddrWindow(area->x1, area->y1, w, h); // Establecer ventana de dirección
    Display::tft.pushColors((uint16_t*)pixelmap, w * h, true); // Enviar los píxeles
    Display::tft.endWrite(); // Finalizar escritura

    lv_disp_flush_ready(disp); // Notificar a LVGL que el flush está completo
}

/**
 * @brief Función de callback para que LVGL lea los eventos táctiles.
 * @param indev_driver Puntero al driver de entrada LVGL.
 * @param data Estructura para almacenar los datos de la entrada (estado y coordenadas).
 */
void my_touchpad_read(lv_indev_t * indev_driver, lv_indev_data_t * data) {
    uint16_t touchX, touchY;
    // Leer el toque de la pantalla táctil. El 600 es un umbral de presión.
    bool touched = Display::tft.getTouch(&touchX, &touchY, 600);

    // Establecer el estado del touchpad: presionado o liberado
    data->state = touched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    // Si está tocado, actualizar las coordenadas
    if (touched) {
        data->point.x = touchX;
        data->point.y = touchY;
    }
}

/**
 * @brief Función de callback para que LVGL obtenga los ticks del sistema.
 * @return Tiempo transcurrido en milisegundos desde el inicio.
 */
static uint32_t my_tick_get_cb(void) {
    return millis(); // Retorna el tiempo en milisegundos usando la función de Arduino
}

// =======================================
// FUNCIONES DE CONTROL (BLE Y PANTALLA)
// =======================================

// Bloque extern "C" para asegurar el enlazado C para funciones llamadas desde ui_events.c
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ajusta el brillo de la pantalla usando PWM nativo (driver LEDC del ESP32).
 * @param percentage El brillo deseado en porcentaje (0-100).
 */
void setScreenBrightness(int percentage) {
    // Limitar el valor de porcentaje entre 0 y 100 para evitar desbordamientos
    Display::screenBrightness = constrain(percentage, 0, 100);
    
    // Mapear el porcentaje (0-100) al rango de ciclo de trabajo del PWM
    // El rango va de MIN_DUTY_VALUE (100) a MAX_DUTY_VALUE (8191)
    uint32_t duty = map(Display::screenBrightness, 0, 100, MIN_DUTY_VALUE, MAX_DUTY_VALUE);
    
    // Usar las funciones directas del driver LEDC para aplicar el nuevo ciclo de trabajo
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
    
    // Imprimir el brillo ajustado para depuración
    Serial.printf("Brillo ajustado al %d%% (PWM Duty Cycle: %d)\n", Display::screenBrightness, duty);
}

/**
 * @brief Devuelve el nivel de brillo actual en porcentaje.
 * @return El brillo actual almacenado (0-100).
 */
int getScreenBrightness() {
    return Display::screenBrightness;
}

/**
 * @brief Inicializa y activa el servicio BLE (Bluetooth Low Energy).
 * Si el BLE ya está activado, solo inicia la publicidad.
 */
void BLEOn() {
    if (Comms::bleEnabled) return; // Si ya está encendido, salir

    Serial.println("Activando BLE...");
    // Solo iniciar la publicidad, el stack BLE ya se inicializó en setup()
    Comms::pAdvertising->start(); 
    
    Comms::bleEnabled = true; // Marcar el BLE como habilitado
    Serial.println("BLE activado y publicitando.");
    updateBLEStatusLabel(); // Actualizar el estado en la interfaz de usuario
}

/**
 * @brief Desactiva el servicio BLE.
 * Si el BLE ya está desactivado, solo detiene la publicidad.
 */
void BLEOff() {
    if (!Comms::bleEnabled) return; // Si ya está apagado, salir

    Serial.println("Desactivando BLE...");
    // Solo detener la publicidad, no desinicializar el stack BLE
    Comms::pAdvertising->stop(); 
    
    Comms::bleEnabled = false; // Marcar el BLE como deshabilitado
    Comms::bleConnected = false; // Asegurar que el estado de conexión sea falso
    Serial.println("BLE desactivado.");
    updateBLEStatusLabel(); // Actualizar el estado en la interfaz de usuario
}

#ifdef __cplusplus
} // Cierra el bloque extern "C"
#endif

// =================================
// FUNCIONES DE LECTURA DE SENSORES
// =================================

/**
 * @brief Función para mapear un valor de un rango a otro (similar a la función map de Arduino).
 * @param x Valor a mapear.
 * @param x1 Límite inferior del rango de entrada.
 * @param x2 Límite superior del rango de entrada.
 * @param y1 Límite inferior del rango de salida.
 * @param y2 Límite superior del rango de salida.
 * @return El valor mapeado.
 */
double mapear(double x, double x1, double x2, double y1, double y2) {
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

/**
 * @brief Lee el sensor de Metano (MQ-4) y calcula la concentración en PPM.
 */
void LeerCH4() {
    double Slectura = 0;
    // Realizar 10 lecturas analógicas y promediar para mayor estabilidad
    for (int i = 0; i < 10; i++) {
        Slectura += analogRead(Sensor::Ch4_pin);
        delay(1);
    }
    
    double ADCsensorMQ = Slectura / 10.0; // Valor ADC promedio
    // Mapear el valor ADC (0-4095 para 12-bit) a voltaje (0-3.3V)
    double VsensorMQ = mapear(ADCsensorMQ, 0, 4095, 0, 3.3);
    // Calcular la resistencia del sensor (Rs)
    double Rs = Sensor::RL * ((3.3 - VsensorMQ) / VsensorMQ);
    // Calcular el ratio Rs/Ro (Ro es la resistencia del sensor en aire limpio)
    double RatioMQ = Rs / Sensor::Ro;
    // Usar la ecuación de la curva de calibración del MQ-4 para obtener PPM
    double exponente = (-log10(RatioMQ) + 1.052) / 0.3512;
    Sensor::CH4_ppm = pow(10, exponente);
}

SensirionI2cScd4x scd4x; // Instancia del objeto para el sensor SCD41

/**
 * @brief Lee el sensor de CO2 (SCD41) para obtener CO2, temperatura y humedad.
 */
void LeerCO2() {
    bool localDataReady = false;
    // Verificar si hay nuevos datos listos para leer del sensor SCD41
    Sensor::error = scd4x.getDataReadyStatus(localDataReady);
    
    if (Sensor::error != NO_ERROR) {
        Sensor::ErrorSCD = true; // Marcar error si la lectura falla
        return;
    }
    
    if (localDataReady) {
        // Leer la concentración de CO2, temperatura y humedad relativa
        Sensor::error = scd4x.readMeasurement(
            Sensor::co2Concentration, 
            Sensor::temperature, 
            Sensor::relativeHumidity
        );
        Sensor::ErrorSCD = (Sensor::error != NO_ERROR); // Actualizar estado de error
        Sensor::dataReady = localDataReady; // Marcar que hay nuevos datos
    }
}

// ==============================
// FUNCIONES DE INTERFAZ GRÁFICA
// ==============================

/**
 * @brief Actualiza los valores de los sensores en la pantalla de monitoreo.
 * Se actualiza cada 1 segundo para evitar sobrecargar la UI.
 */
void ActualizarPantalla() {
    static uint32_t lastUpdate = 0;
    // Limitar la actualización de la pantalla a una vez por segundo
    if (millis() - lastUpdate < 1000) return;
    
    // Actualizar las etiquetas de texto de la UI con los valores de los sensores
    lv_label_set_text(ui_resulCO2Label, (String(Sensor::co2Concentration) + " ppm").c_str());
    lv_label_set_text(ui_resulCH4Label, (String(Sensor::CH4_ppm, 1) + " ppm").c_str());
    lv_label_set_text(ui_resulTempLabel, (String(Sensor::temperature, 1) + " C").c_str());
    lv_label_set_text(ui_resulHumLabel, (String(Sensor::relativeHumidity, 1) + " %").c_str());
    
    lastUpdate = millis(); // Guardar el tiempo de la última actualización
}

/**
 * @brief Actualiza la etiqueta de estado del BLE en la pantalla principal.
 * Muestra si el BLE está "Apagado", "Desconectado" o "Conectado".
 */
void updateBLEStatusLabel() {
    if (!Comms::bleEnabled) {
        lv_label_set_text(ui_bleOnOffLabel, "Apagado"); // Mostrar estado "Apagado"
    } else {
        if (Comms::bleConnected) {
            lv_label_set_text(ui_bleOnOffLabel, "Conectado"); // Mostrar estado "Conectado"
        } else {
            lv_label_set_text(ui_bleOnOffLabel, "Desconectado"); // Mostrar estado "Desconectado"
        }
    }
    Serial.printf("Estado BLE actualizado en UI a: %s\n", lv_label_get_text(ui_bleOnOffLabel)); // Log para depuración
}

// ======================
// CONFIGURACIÓN INICIAL
// ======================
/**
 * @brief Función de configuración inicial del sistema (se ejecuta una vez al inicio).
 * Inicializa puertos seriales, sensores, PWM para el brillo y LVGL.
 */
void setup() {
    Serial.begin(115200); // Inicializar comunicación Serial para depuración
    Serial2.begin(115200, SERIAL_8N1, 11, 12); // Inicializar Serial2 para comunicación UART (pines 11 TX, 12 RX)
    
    Wire.begin(); // Inicializar bus I2C (para SCD41)
    scd4x.begin(Wire, 0x62); // Inicializar sensor SCD41 con dirección 0x62
    scd4x.startPeriodicMeasurement(); // Iniciar medición periódica del SCD41
    
    pinMode(Sensor::Ch4_pin, INPUT); // Configurar pin del sensor MQ-4 como entrada

    // --- Configuración del PWM para el brillo (USANDO DRIVER LEDC DIRECTO) ---
    // Configuración del temporizador LEDC
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,      // Modo de baja velocidad (generalmente para PWM de pantalla)
        .duty_resolution = PWM_RESOLUTION,           // Resolución del ciclo de trabajo (13 bits)
        .timer_num       = PWM_TIMER,                // Temporizador a usar
        .freq_hz         = PWM_FREQ,                 // Frecuencia del PWM
        .clk_cfg         = LEDC_AUTO_CLK,            // Configuración automática del reloj
    };
    ledc_timer_config(&timer_conf); // Aplicar configuración del temporizador

    // Configuración del canal LEDC
    ledc_channel_config_t channel_conf = {
        .gpio_num        = BACKLIGHT_PIN,            // Pin GPIO para el PWM (backlight de la pantalla)
        .speed_mode      = LEDC_LOW_SPEED_MODE,      // Modo de baja velocidad
        .channel         = PWM_CHANNEL,              // Canal PWM a usar
        .intr_type       = LEDC_INTR_DISABLE,        // Sin interrupciones
        .timer_sel       = PWM_TIMER,                // Temporizador asociado
        // Establecer el brillo inicial mapeando el porcentaje inicial al ciclo de trabajo
        .duty            = map(Display::screenBrightness, 0, 100, MIN_DUTY_VALUE, MAX_DUTY_VALUE),
        .hpoint          = 0,                        // Punto inicial del ciclo de trabajo
    };
    ledc_channel_config(&channel_conf); // Aplicar configuración del canal

    // Inicialización de LVGL (Light and Versatile Graphics Library)
    lv_init();
    Display::tft.begin();      // Inicializar la pantalla TFT
    Display::tft.setRotation(2); // Establecer la rotación de la pantalla (ajustar según tu montaje)

    // Datos de calibración para el touchpad (importante para que el toque funcione correctamente)
    uint16_t calData[5] = { 282, 3469, 292, 3592, 2 };
    Display::tft.setTouch(calData); // Aplicar calibración del touchpad

    // Crear el display LVGL
    lv_display_t* disp = lv_display_create(Display::screenWidth, Display::screenHeight);
    // Configurar los buffers de dibujo de LVGL
    lv_display_set_buffers(disp, Display::buf, NULL, sizeof(Display::buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
    // Asignar el callback de flush para que LVGL dibuje en la TFT
    lv_display_set_flush_cb(disp, my_disp_flush);

    // Crear el dispositivo de entrada (touchpad) LVGL
    lv_indev_t* indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); // Tipo de entrada: puntero (touch)
    lv_indev_set_read_cb(indev, my_touchpad_read); // Asignar el callback de lectura del touchpad

    // Asignar el callback para obtener los ticks del sistema para LVGL
    lv_tick_set_cb(my_tick_get_cb);
    
    ui_init(); // Inicializar la interfaz de usuario generada por SquareLine Studio (crea todos los objetos visuales)

    // ========================================================
    // ** AÑADE EL REGISTRAR DE LOS EVENTOS **
    // ========================================================
    lv_obj_add_event_cb(ui_bleSwitch, ui_event_bleSwitch, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui_controlSlider, ui_event_controlSlider, LV_EVENT_VALUE_CHANGED, NULL);
    // ========================================================

    // === Inicialización única del stack BLE ===
    Serial.println("Inicializando stack BLE...");
    BLEDevice::init("TANARI DP"); // Inicializa el dispositivo BLE una sola vez
    Comms::pServer = BLEDevice::createServer();
    Comms::pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = Comms::pServer->createService(SERVICE_UUID);
    Comms::pCharacteristicNotify = pService->createCharacteristic(
        CHARACTERISTIC_UUID_NOTIFY,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    Comms::pCharacteristicNotify->addDescriptor(new BLE2902());
    pService->start();
    Comms::pAdvertising = BLEDevice::getAdvertising();
    Comms::pAdvertising->addServiceUUID(SERVICE_UUID);
    Comms::pAdvertising->setScanResponse(true);
    Serial.println("Stack BLE inicializado.");
    // ===========================================

    // ** Sincronizar el estado inicial del Switch de BLE **
    // El switch inicialmente no está marcado por defecto, así que el BLE comienza apagado.
    // Esto es consistente con Comms::bleEnabled = false; por defecto.
    Comms::bleEnabled = false; // Asegurar que la variable global esté en el estado inicial correcto
    lv_obj_clear_state(ui_bleSwitch, LV_STATE_CHECKED); // Asegurar que el switch de la UI esté apagado
    BLEOff(); // Asegura que no haya publicidad al inicio

    // ** Sincronizar el valor inicial del Slider de Brillo **
    // Asegurarse de que el slider refleje el brillo inicial
    lv_slider_set_value(ui_controlSlider, Display::screenBrightness, LV_ANIM_OFF);

    // **IMPORTANTE**: Actualizar el estado inicial del BLE y el brillo en la UI
    updateBLEStatusLabel(); // Llama a la función para mostrar el estado inicial de BLE en la UI
    
    char initial_brightness_buffer[8];
    // Formatear el brillo inicial a "XX%" y establecerlo en la etiqueta LVGL
    snprintf(initial_brightness_buffer, sizeof(initial_brightness_buffer), "%d%%", Display::screenBrightness);
    lv_label_set_text(ui_Label2, initial_brightness_buffer);

    Serial.println("Sistema Tanari DP inicializado"); // Mensaje de inicialización completada

    // Crear temporizador LVGL para lecturas de sensores (cada 500 ms)
    lv_timer_create([](lv_timer_t * timer) {
        LeerCO2();
        LeerCH4();
    }, 500, NULL);

    // Crear temporizador LVGL para actualizar la pantalla (cada 1000 ms)
    // Este temporizador se encarga de refrescar los datos de los sensores y el estado de BLE en la UI
    lv_timer_create([](lv_timer_t * timer) {
        ActualizarPantalla();   // Actualiza los valores de los sensores
        updateBLEStatusLabel(); // Asegura que el estado del BLE en la UI se mantenga actualizado periódicamente
    }, 1000, NULL);
}

// =================
// BUCLE PRINCIPAL
// =================
/**
 * @brief Bucle principal del programa (se ejecuta repetidamente después de setup()).
 * Se encarga de la gestión de la interfaz gráfica y la transmisión de datos por BLE/UART.
 */
void loop() {
    // Transmisión de datos por BLE si está habilitado, conectado y no hay error de sensor
    if (Comms::bleEnabled && Comms::bleConnected && !Sensor::ErrorSCD) {
        // Formatear los datos de los sensores en una cadena de texto separada por ';'
        String datosTX = String(Sensor::co2Concentration) + ";" + 
                                String(Sensor::CH4_ppm) + ";" + 
                                String(Sensor::temperature) + ";" + 
                                String(Sensor::relativeHumidity);
        
        Comms::pCharacteristicNotify->setValue(datosTX.c_str()); // Establecer el valor de la característica
        Comms::pCharacteristicNotify->notify(); // Enviar notificación a los clientes BLE conectados
    }
    
    // Transmisión de datos por UART (Serial2) si 'Acople' está activo y no hay error de sensor
    if (Comms::Acople && !Sensor::ErrorSCD) {
        // En tu código original, 'datosTX' no está definido aquí si no se cumple la condición BLE.
        // Se recomienda definir 'datosTX' fuera de los 'if' o duplicar su creación si es necesario para UART.
        // Por simplicidad, asumiré que 'datosTX' ya ha sido creado en el bloque BLE anterior o lo recreamos.
        // Si no usas BLE o quieres que siempre se transmita por UART, puedes mover la creación de datosTX.
        String datosTX = String(Sensor::co2Concentration) + ";" + 
                                String(Sensor::CH4_ppm) + ";" + 
                                String(Sensor::temperature) + ";" + 
                                String(Sensor::relativeHumidity);
        Serial2.println(datosTX); // Enviar datos por Serial2
    }

    lv_timer_handler(); // Llamar a la función de manejo de temporizadores de LVGL (procesa eventos UI)
    delay(5); // Pequeño delay para permitir otras tareas del sistema
}