// ui_events.c (YA ESTÁ CORRECTO Y COMPLETO CON TUS ÚLTIMAS MODIFICACIONES)

#include "ui.h"
#include <stdio.h>

/**
 * @brief Este bloque es el "puente" entre el código C de la UI y el código C++ de Arduino.
 * Declara las funciones de tu archivo .ino principal para que podamos llamarlas
 * desde este archivo .c sin que haya errores de compilación.
 */
#ifdef __cplusplus
extern "C" {
#endif

void BLEOn(void);
void BLEOff(void);
void setScreenBrightness(int percentage);
int getScreenBrightness(void);

#ifdef __cplusplus
}
#endif


/* ==========================================================
 * EVENTOS DE LA PANTALLA DE MONITOREO (Tus funciones existentes)
 * ========================================================== */

void mostrarSensorCO2Valor(lv_event_t * e)
{
    // Tu código para este evento va aquí (si lo necesitas).
    // Por ahora lo dejamos como lo generó SquareLine.
}

void mostrarSensorCH4Valor(lv_event_t * e)
{
    // Tu código para este evento va aquí.
}

void mostrarSensorTEMPValor(lv_event_t * e)
{
    // Tu código para este evento va aquí.
}

void mostrarSensorHUMValor(lv_event_t * e)
{
    // Tu código para este evento va aquí.
}


/* ============================================================
 * EVENTOS DE LA PANTALLA DE CONFIGURACIÓN (Nuevas funciones)
 * ============================================================ */

/**
 * @brief Evento que se dispara cuando el switch del BLE cambia de estado.
 */
void ui_event_bleSwitch(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);

    if (event_code == LV_EVENT_VALUE_CHANGED) {
        // Comprobar si el switch está en estado "activado"
        if (lv_obj_has_state(target, LV_STATE_CHECKED)) {
            BLEOn(); // Llama a la función para encender el BLE
        } else {
            BLEOff(); // Llama a la función para apagar el BLE
        }
    }
}

/**
 * @brief Evento que se dispara cuando el valor del slider de brillo cambia.
 */
void ui_event_controlSlider(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);

    if (event_code == LV_EVENT_VALUE_CHANGED) {
        // Obtener el valor actual del slider (0-100)
        int32_t slider_value = lv_slider_get_value(target);
        
        // Llamar a la función principal para ajustar el brillo
        setScreenBrightness(slider_value);
        
        // Actualizar la etiqueta del porcentaje en la pantalla de configuración
        char buffer[8];
        snprintf(buffer, sizeof(buffer), "%d%%", slider_value);
        lv_label_set_text(ui_Label2, buffer); // ¡Aquí el cambio!
    }
}