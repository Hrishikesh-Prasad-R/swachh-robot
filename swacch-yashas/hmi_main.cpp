#include <gtk/gtk.h>
#include <iostream>
#include <string>
#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>

// System state structure
struct SystemState {
    bool vacuum_active;
    bool arm_active;
    bool wiper_active;
    bool uv_active;
    bool autonomous_mode;
    bool emergency_stop;
    bool ai_detection_enabled;
    int battery_level;
    std::string status_message;
    
    SystemState() : 
        vacuum_active(false), 
        arm_active(false), 
        wiper_active(false),
        uv_active(false),
        autonomous_mode(false),
        emergency_stop(false),
        ai_detection_enabled(true),
        battery_level(100),
        status_message("System Ready") {}
};

// Global system state
SystemState g_system_state;

// GUI Widgets
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

HMIWidgets g_widgets;

// Serial communication class for Arduino
class SerialComm {
private:
    int serial_port;
    bool connected;
    
public:
    SerialComm() : serial_port(-1), connected(false) {}
    
    bool connect(const char* port = "/dev/ttyUSB0") {
        serial_port = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        
        if (serial_port < 0) {
            std::cerr << "Error opening serial port: " << port << std::endl;
            return false;
        }
        
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(serial_port, &tty) != 0) {
            std::cerr << "Error getting serial attributes" << std::endl;
            return false;
        }
        
        // Configure serial port (115200 baud, 8N1)
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ISIG;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        
        tty.c_cc[VTIME] = 1;
        tty.c_cc[VMIN] = 0;
        
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting serial attributes" << std::endl;
            return false;
        }
        
        connected = true;
        std::cout << "Connected to serial port: " << port << std::endl;
        return true;
    }
    
    bool send_command(const std::string& command) {
        if (!connected || serial_port < 0) {
            std::cerr << "Serial port not connected" << std::endl;
            return false;
        }
        
        std::string cmd = command + "\n";
        ssize_t bytes_written = write(serial_port, cmd.c_str(), cmd.length());
        
        if (bytes_written < 0) {
            std::cerr << "Error writing to serial port" << std::endl;
            return false;
        }
        
        std::cout << "Sent command: " << command << std::endl;
        return true;
    }
    
    void disconnect() {
        if (serial_port >= 0) {
            close(serial_port);
            serial_port = -1;
            connected = false;
        }
    }
    
    ~SerialComm() {
        disconnect();
    }
};

SerialComm g_serial;

// Update status display
void update_status_display() {
    std::string status_text = "<span font='12' weight='bold'>";
    status_text += g_system_state.status_message;
    status_text += "</span>";
    gtk_label_set_markup(GTK_LABEL(g_widgets.status_label), status_text.c_str());
    
    std::string battery_text = "<span font='11'>Battery: " + 
                               std::to_string(g_system_state.battery_level) + "%</span>";
    gtk_label_set_markup(GTK_LABEL(g_widgets.battery_label), battery_text.c_str());
}

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
    gtk_button_set_label(GTK_BUTTON(g_widgets.vacuum_button), "Vacuum");
    gtk_button_set_label(GTK_BUTTON(g_widgets.arm_button), "Arm");
    gtk_button_set_label(GTK_BUTTON(g_widgets.wiper_button), "Wiper");
    gtk_button_set_label(GTK_BUTTON(g_widgets.uv_button), "UV");
    gtk_button_set_label(GTK_BUTTON(g_widgets.autonomous_button), "Auto");
    
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
        gtk_button_set_label(GTK_BUTTON(widget), "Vacuum");
        g_serial.send_command("VACUUM:ON");
        g_system_state.status_message = "Vacuum Activated";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "Vacuum");
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
        gtk_button_set_label(GTK_BUTTON(widget), "Arm");
        g_serial.send_command("ARM:ON");
        g_system_state.status_message = "Arm Activated";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "Arm");
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
        gtk_button_set_label(GTK_BUTTON(widget), "Wiper");
        g_serial.send_command("WIPER:ON");
        g_system_state.status_message = "Wiper Activated";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "Wiper");
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
        gtk_button_set_label(GTK_BUTTON(widget), "UV");
        g_serial.send_command("UV:ON");
        g_system_state.status_message = "UV Sanitizer Activated";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "UV");
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
        gtk_button_set_label(GTK_BUTTON(widget), "Auto");
        g_serial.send_command("AUTO:ON");
        g_system_state.status_message = "Autonomous Mode Active";
    } else {
        gtk_button_set_label(GTK_BUTTON(widget), "Auto");
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

// Battery update timer (simulate battery monitoring)
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

// Create the GUI
void create_hmi_interface(GtkApplication *app) {
    // Main window
    g_widgets.window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(g_widgets.window), "Vacuum Cleaner HMI");
    gtk_window_set_default_size(GTK_WINDOW(g_widgets.window), 1024, 600);
    gtk_window_fullscreen(GTK_WINDOW(g_widgets.window)); // Fullscreen for embedded display
    
    // Main container
    GtkWidget *main_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_widget_set_margin_start(main_box, 10);
    gtk_widget_set_margin_end(main_box, 10);
    gtk_widget_set_margin_top(main_box, 5);
    gtk_widget_set_margin_bottom(main_box, 5);
    gtk_window_set_child(GTK_WINDOW(g_widgets.window), main_box);
    
    // Header section
    GtkWidget *header_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_widget_set_halign(header_box, GTK_ALIGN_CENTER);
    gtk_box_append(GTK_BOX(main_box), header_box);
    
    GtkWidget *title_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(title_label), 
                         "<span font='16' weight='bold'>Swacch Vacuum HMI</span>");
    gtk_box_append(GTK_BOX(header_box), title_label);
    
    // Status section
    GtkWidget *status_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    gtk_widget_set_halign(status_box, GTK_ALIGN_CENTER);
    gtk_box_append(GTK_BOX(main_box), status_box);
    
    g_widgets.status_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_widgets.status_label), 
                         "<span font='12' weight='bold'>System Ready</span>");
    gtk_box_append(GTK_BOX(status_box), g_widgets.status_label);
    
    g_widgets.battery_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_widgets.battery_label), 
                         "<span font='11'>Battery: 100%</span>");
    gtk_box_append(GTK_BOX(status_box), g_widgets.battery_label);
    
    // Emergency stop section
    GtkWidget *emergency_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_widget_set_halign(emergency_box, GTK_ALIGN_CENTER);
    gtk_box_append(GTK_BOX(main_box), emergency_box);
    
    g_widgets.emergency_button = gtk_button_new_with_label("🛑 E-STOP");
    gtk_widget_set_size_request(g_widgets.emergency_button, 240, 55);
    GtkStyleContext *context = gtk_widget_get_style_context(g_widgets.emergency_button);
    gtk_style_context_add_class(context, "destructive-action");
    g_signal_connect(g_widgets.emergency_button, "clicked", G_CALLBACK(on_emergency_stop), NULL);
    gtk_box_append(GTK_BOX(emergency_box), g_widgets.emergency_button);
    
    GtkWidget *reset_button = gtk_button_new_with_label("🔄 Reset");
    gtk_widget_set_size_request(reset_button, 120, 55);
    g_signal_connect(reset_button, "clicked", G_CALLBACK(on_reset_emergency), NULL);
    gtk_box_append(GTK_BOX(emergency_box), reset_button);
    
    // Component controls section
    GtkWidget *controls_frame = gtk_frame_new("Component Controls");
    gtk_box_append(GTK_BOX(main_box), controls_frame);
    
    GtkWidget *controls_grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(controls_grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(controls_grid), 5);
    gtk_widget_set_margin_start(controls_grid, 5);
    gtk_widget_set_margin_end(controls_grid, 5);
    gtk_widget_set_margin_top(controls_grid, 5);
    gtk_widget_set_margin_bottom(controls_grid, 5);
    gtk_frame_set_child(GTK_FRAME(controls_frame), controls_grid);
    
    // Vacuum button
    g_widgets.vacuum_button = gtk_button_new_with_label("🌪️ Vacuum");
    gtk_widget_set_size_request(g_widgets.vacuum_button, 160, 50);
    g_signal_connect(g_widgets.vacuum_button, "clicked", G_CALLBACK(on_vacuum_toggle), NULL);
    gtk_grid_attach(GTK_GRID(controls_grid), g_widgets.vacuum_button, 0, 0, 1, 1);
    
    // Arm button
    g_widgets.arm_button = gtk_button_new_with_label("🦾 Arm");
    gtk_widget_set_size_request(g_widgets.arm_button, 160, 50);
    g_signal_connect(g_widgets.arm_button, "clicked", G_CALLBACK(on_arm_toggle), NULL);
    gtk_grid_attach(GTK_GRID(controls_grid), g_widgets.arm_button, 1, 0, 1, 1);
    
    // Wiper button
    g_widgets.wiper_button = gtk_button_new_with_label("🧹 Wiper");
    gtk_widget_set_size_request(g_widgets.wiper_button, 160, 50);
    g_signal_connect(g_widgets.wiper_button, "clicked", G_CALLBACK(on_wiper_toggle), NULL);
    gtk_grid_attach(GTK_GRID(controls_grid), g_widgets.wiper_button, 2, 0, 1, 1);
    
    // UV button
    g_widgets.uv_button = gtk_button_new_with_label("💡 UV");
    gtk_widget_set_size_request(g_widgets.uv_button, 160, 50);
    g_signal_connect(g_widgets.uv_button, "clicked", G_CALLBACK(on_uv_toggle), NULL);
    gtk_grid_attach(GTK_GRID(controls_grid), g_widgets.uv_button, 3, 0, 1, 1);
    
    // Autonomous mode button
    g_widgets.autonomous_button = gtk_button_new_with_label("🤖 Auto");
    gtk_widget_set_size_request(g_widgets.autonomous_button, 160, 50);
    g_signal_connect(g_widgets.autonomous_button, "clicked", G_CALLBACK(on_autonomous_toggle), NULL);
    gtk_grid_attach(GTK_GRID(controls_grid), g_widgets.autonomous_button, 0, 1, 2, 1);
    
    // AI Detection toggle
    GtkWidget *ai_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_widget_set_halign(ai_box, GTK_ALIGN_CENTER);
    GtkWidget *ai_label = gtk_label_new("AI Detection:");
    gtk_box_append(GTK_BOX(ai_box), ai_label);
    g_widgets.ai_toggle = gtk_switch_new();
    gtk_switch_set_active(GTK_SWITCH(g_widgets.ai_toggle), TRUE);
    g_signal_connect(g_widgets.ai_toggle, "state-set", G_CALLBACK(on_ai_toggle), NULL);
    gtk_box_append(GTK_BOX(ai_box), g_widgets.ai_toggle);
    gtk_grid_attach(GTK_GRID(controls_grid), ai_box, 2, 1, 2, 1);
    
    // Movement controls section
    GtkWidget *movement_frame = gtk_frame_new("Manual Movement Controls");
    gtk_box_append(GTK_BOX(main_box), movement_frame);
    
    GtkWidget *movement_grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(movement_grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(movement_grid), 5);
    gtk_widget_set_margin_start(movement_grid, 5);
    gtk_widget_set_margin_end(movement_grid, 5);
    gtk_widget_set_margin_top(movement_grid, 5);
    gtk_widget_set_margin_bottom(movement_grid, 5);
    gtk_widget_set_halign(movement_grid, GTK_ALIGN_CENTER);
    gtk_frame_set_child(GTK_FRAME(movement_frame), movement_grid);
    
    // Forward button
    g_widgets.forward_button = gtk_button_new_with_label("⬆️");
    gtk_widget_set_size_request(g_widgets.forward_button, 100, 50);
    g_signal_connect(g_widgets.forward_button, "clicked", G_CALLBACK(on_forward), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.forward_button, 1, 0, 1, 1);
    
    // Left button
    g_widgets.left_button = gtk_button_new_with_label("⬅️");
    gtk_widget_set_size_request(g_widgets.left_button, 100, 50);
    g_signal_connect(g_widgets.left_button, "clicked", G_CALLBACK(on_left), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.left_button, 0, 1, 1, 1);
    
    // Stop button
    g_widgets.stop_button = gtk_button_new_with_label("⏹️");
    gtk_widget_set_size_request(g_widgets.stop_button, 100, 50);
    GtkStyleContext *stop_context = gtk_widget_get_style_context(g_widgets.stop_button);
    gtk_style_context_add_class(stop_context, "destructive-action");
    g_signal_connect(g_widgets.stop_button, "clicked", G_CALLBACK(on_stop_movement), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.stop_button, 1, 1, 1, 1);
    
    // Right button
    g_widgets.right_button = gtk_button_new_with_label("➡️");
    gtk_widget_set_size_request(g_widgets.right_button, 100, 50);
    g_signal_connect(g_widgets.right_button, "clicked", G_CALLBACK(on_right), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.right_button, 2, 1, 1, 1);
    
    // Backward button
    g_widgets.backward_button = gtk_button_new_with_label("⬇️");
    gtk_widget_set_size_request(g_widgets.backward_button, 100, 50);
    g_signal_connect(g_widgets.backward_button, "clicked", G_CALLBACK(on_backward), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.backward_button, 1, 2, 1, 1);
    
    // Show window
    gtk_widget_show(g_widgets.window);
    
    // Start battery update timer (every 5 seconds)
    g_timeout_add_seconds(5, update_battery, NULL);
}

// Application activation callback
static void activate(GtkApplication *app, gpointer user_data) {
    create_hmi_interface(app);
}

int main(int argc, char *argv[]) {
    // Try to connect to Arduino
    if (!g_serial.connect("/dev/ttyUSB0")) {
        std::cout << "Warning: Could not connect to Arduino. HMI will run without hardware control." << std::endl;
        std::cout << "Make sure Arduino is connected to /dev/ttyUSB0 or update the port in the code." << std::endl;
    }
    
    // Create GTK application
    GtkApplication *app = gtk_application_new("com.swacch.hmi", G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);
    
    int status = g_application_run(G_APPLICATION(app), argc, argv);
    
    g_object_unref(app);
    g_serial.disconnect();
    
    return status;
}
