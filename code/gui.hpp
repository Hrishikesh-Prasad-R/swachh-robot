#ifndef GUI_HPP
#define GUI_HPP

#include <gtk/gtk.h>

// GUI Widgets structure
struct HMIWidgets {
    GtkWidget *window;
    GtkWidget *status_label;
    GtkWidget *battery_label;
    GtkWidget *vacuum_button;
    GtkWidget *arm_button;
    GtkWidget *wiper_button;
    GtkWidget *uv_button;
    GtkWidget *autonomous_button;
    GtkWidget *emergency_button;
    GtkWidget *ai_toggle;
    GtkWidget *forward_button;
    GtkWidget *backward_button;
    GtkWidget *left_button;
    GtkWidget *right_button;
    GtkWidget *stop_button;
};

// Global widgets declaration
extern HMIWidgets g_widgets;

// GUI functions
void update_status_display();
void create_hmi_interface(GtkApplication *app);

#endif // GUI_HPP
