#ifndef CALLBACKS_HPP
#define CALLBACKS_HPP

#include <gtk/gtk.h>

// Emergency controls
void on_emergency_stop(GtkWidget *widget, gpointer data);
void on_reset_emergency(GtkWidget *widget, gpointer data);

// Component toggles
void on_vacuum_toggle(GtkWidget *widget, gpointer data);
void on_arm_toggle(GtkWidget *widget, gpointer data);
void on_wiper_toggle(GtkWidget *widget, gpointer data);
void on_uv_toggle(GtkWidget *widget, gpointer data);
void on_autonomous_toggle(GtkWidget *widget, gpointer data);
void on_ai_toggle(GtkWidget *widget, gpointer data);

// Movement controls
void on_forward(GtkWidget *widget, gpointer data);
void on_backward(GtkWidget *widget, gpointer data);
void on_left(GtkWidget *widget, gpointer data);
void on_right(GtkWidget *widget, gpointer data);
void on_stop_movement(GtkWidget *widget, gpointer data);

// Timer callback
gboolean update_battery(gpointer data);

#endif // CALLBACKS_HPP
