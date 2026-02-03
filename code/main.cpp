#include <iostream>
#include "system_state.hpp"
#include "serial_comm.hpp"
#include "gui.hpp"

// Application activation callback
static void activate(GtkApplication *app, gpointer user_data) {
    create_hmi_interface(app);
}

int main(int argc, char *argv[]) {
    // Try to connect to Arduino
    if (!g_serial.connect("/dev/ttyCH341USB0")) {
        std::cout << "Warning: Could not connect to Arduino. HMI will run without hardware control." << std::endl;
        std::cout << "Make sure Arduino is connected to /dev/ttyCH341USB0 or update the port in the code." << std::endl;
    }
    
    // Create GTK application
    GtkApplication *app = gtk_application_new("com.swacch.hmi", G_APPLICATION_FLAGS_NONE);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);
    
    int status = g_application_run(G_APPLICATION(app), argc, argv);
    
    g_object_unref(app);
    g_serial.disconnect();
    
    return status;
}
