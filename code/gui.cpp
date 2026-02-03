#include "gui.hpp"
#include "system_state.hpp"
#include "callbacks.hpp"

// Global widgets definition
HMIWidgets g_widgets;

// Global system state definition
SystemState g_system_state;

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
                         "<span font='16' weight='bold'>Swacch HMI</span>");
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
    
    g_widgets.emergency_button = gtk_button_new_with_label("EMERGENCY STOP");
    gtk_widget_set_size_request(g_widgets.emergency_button, 240, 55);
    GtkStyleContext *context = gtk_widget_get_style_context(g_widgets.emergency_button);
    gtk_style_context_add_class(context, "destructive-action");
    g_signal_connect(g_widgets.emergency_button, "clicked", G_CALLBACK(on_emergency_stop), NULL);
    gtk_box_append(GTK_BOX(emergency_box), g_widgets.emergency_button);
    
    GtkWidget *reset_button = gtk_button_new_with_label("Reset");
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
    gtk_widget_set_margin_bottom(controls_grid, 10);
    gtk_frame_set_child(GTK_FRAME(controls_frame), controls_grid);
    
    // Vacuum button
    g_widgets.vacuum_button = gtk_button_new_with_label("Vacuum: OFF");
    gtk_widget_set_size_request(g_widgets.vacuum_button, 160, 50);
    g_signal_connect(g_widgets.vacuum_button, "clicked", G_CALLBACK(on_vacuum_toggle), NULL);
    gtk_grid_attach(GTK_GRID(controls_grid), g_widgets.vacuum_button, 0, 0, 1, 1);
    
    // Arm button
    g_widgets.arm_button = gtk_button_new_with_label("Arm: OFF");
    gtk_widget_set_size_request(g_widgets.arm_button, 160, 50);
    g_signal_connect(g_widgets.arm_button, "clicked", G_CALLBACK(on_arm_toggle), NULL);
    gtk_grid_attach(GTK_GRID(controls_grid), g_widgets.arm_button, 1, 0, 1, 1);
    
    // Wiper button
    g_widgets.wiper_button = gtk_button_new_with_label("Wiper: OFF");
    gtk_widget_set_size_request(g_widgets.wiper_button, 160, 50);
    g_signal_connect(g_widgets.wiper_button, "clicked", G_CALLBACK(on_wiper_toggle), NULL);
    gtk_grid_attach(GTK_GRID(controls_grid), g_widgets.wiper_button, 2, 0, 1, 1);
    
    // UV button
    g_widgets.uv_button = gtk_button_new_with_label("UV: OFF");
    gtk_widget_set_size_request(g_widgets.uv_button, 160, 50);
    g_signal_connect(g_widgets.uv_button, "clicked", G_CALLBACK(on_uv_toggle), NULL);
    gtk_grid_attach(GTK_GRID(controls_grid), g_widgets.uv_button, 3, 0, 1, 1);
    
    // Autonomous mode button
    g_widgets.autonomous_button = gtk_button_new_with_label("Auto: OFF");
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
    g_widgets.forward_button = gtk_button_new_with_label("Forward");
    gtk_widget_set_size_request(g_widgets.forward_button, 100, 50);
    g_signal_connect(g_widgets.forward_button, "clicked", G_CALLBACK(on_forward), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.forward_button, 1, 0, 1, 1);
    
    // Left button
    g_widgets.left_button = gtk_button_new_with_label("Left");
    gtk_widget_set_size_request(g_widgets.left_button, 100, 50);
    g_signal_connect(g_widgets.left_button, "clicked", G_CALLBACK(on_left), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.left_button, 0, 1, 1, 1);
    
    // Stop button
    g_widgets.stop_button = gtk_button_new_with_label("STOP");
    gtk_widget_set_size_request(g_widgets.stop_button, 100, 50);
    GtkStyleContext *stop_context = gtk_widget_get_style_context(g_widgets.stop_button);
    gtk_style_context_add_class(stop_context, "destructive-action");
    g_signal_connect(g_widgets.stop_button, "clicked", G_CALLBACK(on_stop_movement), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.stop_button, 1, 1, 1, 1);
    
    // Right button
    g_widgets.right_button = gtk_button_new_with_label("Right");
    gtk_widget_set_size_request(g_widgets.right_button, 100, 50);
    g_signal_connect(g_widgets.right_button, "clicked", G_CALLBACK(on_right), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.right_button, 2, 1, 1, 1);
    
    // Backward button
    g_widgets.backward_button = gtk_button_new_with_label("Backward");
    gtk_widget_set_size_request(g_widgets.backward_button, 100, 50);
    g_signal_connect(g_widgets.backward_button, "clicked", G_CALLBACK(on_backward), NULL);
    gtk_grid_attach(GTK_GRID(movement_grid), g_widgets.backward_button, 1, 2, 1, 1);
    
    // Show window
    gtk_widget_show(g_widgets.window);
    
    // Start battery update timer (every 5 seconds)
    g_timeout_add_seconds(5, update_battery, NULL);
}
