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
    ui_img_welcomebg_320x480_original_png.c)

add_library(ui ${SOURCES})
