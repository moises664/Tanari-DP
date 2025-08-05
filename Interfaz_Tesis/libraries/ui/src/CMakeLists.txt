SET(SOURCES ui_comp_configurebutton.c
    ui_comp_offbutton.c
    ui_comp_offbutton1.c
    ui_comp.c
    ui_splashScreen.c
    ui_homeScreen.c
    ui_monitoreoScreen.c
    ui_configScreen.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    ui_img_1571022687.c
    ui_img_bateria_cargando_png.c
    ui_img_bateria_0_png.c
    ui_img_bateria_25_png.c
    ui_img_bateria_50_png.c
    ui_img_bateria_75_png.c
    ui_img_bateria_100_png.c
    ui_img_acople_desconectado_png.c
    ui_img_send_data_png.c)

add_library(ui ${SOURCES})
