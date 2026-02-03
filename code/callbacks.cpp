#include "callbacks.hpp"
#include "system_state.hpp"
#include "serial_comm.hpp"
#include "gui.hpp"

// Emergency stop callback
void on_emergency_stop(GtkWidget *widget, gpointer data) {
    g_system_state.emergency_stop = true;
    g_system_state.vacuum_active = false;
    g_system_state.arm_active = false;
    g_system_state.wiper_active = false;
    g_system_state.uv_active = false;
    g_system_state.autonomous_mode = false;
    g_system_state.status_message = "EMERGENCY STOP ACTIVATED";
    
    g_serial.send_command("ESTOP");
    
    // Update all button states
    gtk_button_set_label(GTK_BUTTON(g_widgets.vacuum_button), "Vacuum: OFF");
    gtk_button_set_label(GTK_BUTTON(g_widgets.arm_button), "Arm: OFF");
    gtk_button_set_label(GTK_BUTTON(g_widgets.wiper_button), "Wiper: OFF");
    gtk_button_set_label(GTK_BUTTON(g_widgets.uv_button), "UV: OFF");
    gtk_button_set_label(GTK_BUTTON(g_widgets.autonomous_button), "Auto: OFF");
    
    update_status_display();
}

// Reset emergency stop
void on_reset_emergency(GtkWidget *widget, gpointer data) {
    g_system_state.emergency_stop = false;
    g_system_state.status_message = "System Ready";
    g_serial.send_command("RESET");
    update_status_display();
}

// Vacuum control callback
void on_vacuum_toggle(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop) return;
    
    g_system_state.vacuum_active = !g_system_state.vacuum_active;
    
    if (g_system_state.vacuum_active) {
        gtk_button_set_label(GTK_BUTTON(widget), "Vacuum: ON");
        g_serial.send_command("VACUUM:ON");
        g_system_state.status_message = "Vacuum Activated";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "Vacuum: OFF");
        g_serial.send_command("VACUUM:OFF");
        g_system_state.status_message = "Vacuum Deactivated";
    }
    
    update_status_display();
}

// Arm control callback
void on_arm_toggle(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop) return;
    
    g_system_state.arm_active = !g_system_state.arm_active;
    
    if (g_system_state.arm_active) {
        gtk_button_set_label(GTK_BUTTON(widget), "Arm: ON");
        g_serial.send_command("ARM:ON");
        g_system_state.status_message = "Arm Activated";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "Arm: OFF");
        g_serial.send_command("ARM:OFF");
        g_system_state.status_message = "Arm Deactivated";
    }
    
    update_status_display();
}

// Wiper control callback
void on_wiper_toggle(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop) return;
    
    g_system_state.wiper_active = !g_system_state.wiper_active;
    
    if (g_system_state.wiper_active) {
        gtk_button_set_label(GTK_BUTTON(widget), "Wiper: ON");
        g_serial.send_command("WIPER:ON");
        g_system_state.status_message = "Wiper Activated";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "Wiper: OFF");
        g_serial.send_command("WIPER:OFF");
        g_system_state.status_message = "Wiper Deactivated";
    }
    
    update_status_display();
}

// UV control callback
void on_uv_toggle(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop) return;
    
    g_system_state.uv_active = !g_system_state.uv_active;
    
    if (g_system_state.uv_active) {
        gtk_button_set_label(GTK_BUTTON(widget), "UV: ON");
        g_serial.send_command("UV:ON");
        g_system_state.status_message = "UV Sanitizer Activated";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "UV: OFF");
        g_serial.send_command("UV:OFF");
        g_system_state.status_message = "UV Sanitizer Deactivated";
    }
    
    update_status_display();
}

// Autonomous mode callback
void on_autonomous_toggle(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop) return;
    
    g_system_state.autonomous_mode = !g_system_state.autonomous_mode;
    
    if (g_system_state.autonomous_mode) {
        gtk_button_set_label(GTK_BUTTON(widget), "Auto: ON");
        g_serial.send_command("AUTO:ON");
        g_system_state.status_message = "Autonomous Mode Active";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "Auto: OFF");
        g_serial.send_command("AUTO:OFF");
        g_system_state.status_message = "Manual Mode Active";
    }
    
    update_status_display();
}

// AI Detection toggle
void on_ai_toggle(GtkWidget *widget, gpointer data) {
    g_system_state.ai_detection_enabled = gtk_switch_get_active(GTK_SWITCH(widget));
    
    if (g_system_state.ai_detection_enabled) {
        g_serial.send_command("AI:ON");
        g_system_state.status_message = "AI Detection Enabled";
    } else {
        g_serial.send_command("AI:OFF");
        g_system_state.status_message = "AI Detection Disabled";
    }
    
    update_status_display();
}

// Movement controls
void on_forward(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop || g_system_state.autonomous_mode) return;
    g_serial.send_command("MOVE:FORWARD");
    g_system_state.status_message = "Moving Forward";
    update_status_display();
}

void on_backward(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop || g_system_state.autonomous_mode) return;
    g_serial.send_command("MOVE:BACKWARD");
    g_system_state.status_message = "Moving Backward";
    update_status_display();
}

void on_left(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop || g_system_state.autonomous_mode) return;
    g_serial.send_command("MOVE:LEFT");
    g_system_state.status_message = "Turning Left";
    update_status_display();
}

void on_right(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop || g_system_state.autonomous_mode) return;
    g_serial.send_command("MOVE:RIGHT");
    g_system_state.status_message = "Turning Right";
    update_status_display();
}

void on_stop_movement(GtkWidget *widget, gpointer data) {
    if (g_system_state.emergency_stop) return;
    g_serial.send_command("MOVE:STOP");
    g_system_state.status_message = "Movement Stopped";
    update_status_display();
}

// Battery update timer
gboolean update_battery(gpointer data) {
    // In real implementation, read from battery management system
    // For now, simulate slow discharge
    if (g_system_state.battery_level > 0) {
        if (g_system_state.vacuum_active || g_system_state.arm_active || 
            g_system_state.wiper_active || g_system_state.uv_active) {
            g_system_state.battery_level -= 1;
        }
    }
    
    update_status_display();
    return TRUE; // Continue timer
}
