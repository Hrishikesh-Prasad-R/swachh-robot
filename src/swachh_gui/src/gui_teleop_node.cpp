/**
 * Swachh Robot — GTK4 GUI Teleop Node
 *
 * C++ ROS 2 node with GTK4 GUI that replaces terminal-based arrow_teleop.
 * Replicates HMI behaviour from swacch-yashas and adds:
 *   - Distance moved display (from /odom)
 *   - Obstacle distance display (from /scan)
 *   - Collision avoidance (proportional deceleration)
 *   - Terminal logging of all actions
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gtk/gtk.h>

#include <cmath>
#include <algorithm>
#include <deque>
#include <numeric>
#include <string>
#include <mutex>

// ============== CONSTANTS ==============

static constexpr double ROBOT_RADIUS  = 0.105;  // meters (TurtleBot3 Burger)
static constexpr double SLOW_ZONE     = 0.5;    // start decelerating at this gap
static constexpr double COLLISION_CONE_DEG = 120.0; // front cone for collision
static constexpr double SPEED_LEVELS[] = {0.10, 0.22, 0.50};  // m/s tiers
static constexpr double TURN_SPEED    = 0.5;    // rad/s
static constexpr int    MEDIAN_SIZE   = 3;

// ============== ROS NODE ==============

class GuiTeleopNode : public rclcpp::Node {
public:
    GuiTeleopNode() : Node("gui_teleop") {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_callback(msg); }
        );
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); }
        );
        RCLCPP_INFO(this->get_logger(), "GUI Teleop node started");
    }

    // Publish velocity command
    void publish_cmd(double linear, double angular) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear;
        msg.angular.z = angular;
        cmd_pub_->publish(msg);
        current_linear_ = linear;
        current_angular_ = angular;
    }

    // Get effective forward speed with collision avoidance
    double get_effective_speed() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        double gap = collision_dist_ - ROBOT_RADIUS;
        if (gap <= 0.0) return 0.0;
        if (gap >= SLOW_ZONE) return SPEED_LEVELS[speed_level_];
        double factor = gap / SLOW_ZONE;
        return SPEED_LEVELS[speed_level_] * factor;
    }

    // Getters (thread-safe)
    double get_collision_dist() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return collision_dist_;
    }

    double get_total_distance() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return total_distance_;
    }

    double get_position_x() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return pos_x_;
    }

    double get_position_y() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return pos_y_;
    }

    int speed_level_ = 1;       // default: medium (0.22 m/s)
    double current_linear_ = 0.0;
    double current_angular_ = 0.0;

private:
    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        int n = static_cast<int>(msg->ranges.size());
        if (n == 0) return;

        // Compute collision cone: front COLLISION_CONE_DEG degrees
        double half_cone_rad = (COLLISION_CONE_DEG / 2.0) * M_PI / 180.0;
        double min_dist = std::numeric_limits<double>::infinity();

        for (int i = 0; i < n; i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            // Normalize to [-pi, pi]
            while (angle > M_PI) angle -= 2 * M_PI;
            while (angle < -M_PI) angle += 2 * M_PI;

            if (std::abs(angle) <= half_cone_rad) {
                double r = msg->ranges[i];
                if (!std::isinf(r) && !std::isnan(r) && r > 0.01) {
                    min_dist = std::min(min_dist, static_cast<double>(r));
                }
            }
        }

        // Median filter
        collision_history_.push_back(min_dist);
        if (collision_history_.size() > MEDIAN_SIZE) {
            collision_history_.pop_front();
        }

        // Compute median
        std::vector<double> sorted(collision_history_.begin(), collision_history_.end());
        std::sort(sorted.begin(), sorted.end());
        collision_dist_ = sorted[sorted.size() / 2];
    }

    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        if (odom_initialized_) {
            double dx = x - prev_x_;
            double dy = y - prev_y_;
            total_distance_ += std::sqrt(dx * dx + dy * dy);
        }

        prev_x_ = x;
        prev_y_ = y;
        pos_x_ = x;
        pos_y_ = y;
        odom_initialized_ = true;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::mutex data_mutex_;
    std::deque<double> collision_history_;
    double collision_dist_ = std::numeric_limits<double>::infinity();
    double total_distance_ = 0.0;
    double prev_x_ = 0.0, prev_y_ = 0.0;
    double pos_x_ = 0.0, pos_y_ = 0.0;
    bool odom_initialized_ = false;
};

// ============== GTK4 APPLICATION ==============

// Global state
static std::shared_ptr<GuiTeleopNode> g_node;

struct SystemState {
    bool vacuum_active = false;
    bool arm_active = false;
    bool wiper_active = false;
    bool uv_active = false;
    bool autonomous_mode = false;
    bool emergency_stop = false;
    bool ai_detection = true;
    int battery_level = 100;
    std::string status = "System Ready";
};
static SystemState g_state;

struct Widgets {
    GtkWidget *status_label;
    GtkWidget *battery_label;
    GtkWidget *distance_label;
    GtkWidget *obstacle_label;
    GtkWidget *position_label;
    GtkWidget *speed_label;
    GtkWidget *vacuum_btn;
    GtkWidget *arm_btn;
    GtkWidget *wiper_btn;
    GtkWidget *uv_btn;
    GtkWidget *auto_btn;
    GtkWidget *estop_btn;
};
static Widgets g_w;

// ---- Helper: update status display ----
static void update_status() {
    std::string markup = "<span font='12' weight='bold'>" + g_state.status + "</span>";
    gtk_label_set_markup(GTK_LABEL(g_w.status_label), markup.c_str());

    std::string bat = "<span font='11'>🔋 Battery: " + std::to_string(g_state.battery_level) + "%</span>";
    gtk_label_set_markup(GTK_LABEL(g_w.battery_label), bat.c_str());
}

static void update_telemetry() {
    double dist = g_node->get_total_distance();
    double obs = g_node->get_collision_dist();
    double px = g_node->get_position_x();
    double py = g_node->get_position_y();

    char buf[128];
    snprintf(buf, sizeof(buf),
             "<span font='14' weight='bold'>📏 Distance Moved: %.2f m</span>", dist);
    gtk_label_set_markup(GTK_LABEL(g_w.distance_label), buf);

    // Obstacle color: red if close, green if far
    double gap = obs - ROBOT_RADIUS;
    const char *color = (gap < 0.3) ? "red" : (gap < SLOW_ZONE) ? "orange" : "green";
    if (std::isinf(obs)) {
        snprintf(buf, sizeof(buf),
                 "<span font='13' weight='bold' foreground='green'>🛡️ Obstacle: > 3.5 m (clear)</span>");
    } else {
        snprintf(buf, sizeof(buf),
                 "<span font='13' weight='bold' foreground='%s'>🛡️ Obstacle: %.2f m</span>",
                 color, obs);
    }
    gtk_label_set_markup(GTK_LABEL(g_w.obstacle_label), buf);

    snprintf(buf, sizeof(buf),
             "<span font='11'>📍 Position: (%.2f, %.2f)</span>", px, py);
    gtk_label_set_markup(GTK_LABEL(g_w.position_label), buf);

    snprintf(buf, sizeof(buf),
             "<span font='11'>⚡ Speed: %.2f m/s  |  Level: %d/3</span>",
             SPEED_LEVELS[g_node->speed_level_], g_node->speed_level_ + 1);
    gtk_label_set_markup(GTK_LABEL(g_w.speed_label), buf);
}

// ---- Timer: spin ROS + update GUI ----
static gboolean ros_spin_tick(gpointer) {
    if (rclcpp::ok()) {
        rclcpp::spin_some(g_node);
        update_telemetry();

        // Continuous collision enforcement while moving forward
        if (g_node->current_linear_ > 0.0 && !g_state.emergency_stop) {
            double eff = g_node->get_effective_speed();
            if (eff < g_node->current_linear_) {
                g_node->publish_cmd(eff, g_node->current_angular_);
                if (eff <= 0.0) {
                    g_state.status = "⛔ BLOCKED — Wall too close";
                    update_status();
                }
            }
        }
    }
    return TRUE;
}

// ---- Battery simulation timer ----
static gboolean battery_tick(gpointer) {
    if (g_state.battery_level > 0) {
        if (g_state.vacuum_active || g_state.arm_active ||
            g_state.wiper_active || g_state.uv_active) {
            g_state.battery_level--;
        }
    }
    update_status();
    return TRUE;
}

// ---- Movement callbacks ----
static void on_forward(GtkWidget*, gpointer) {
    if (g_state.emergency_stop || g_state.autonomous_mode) return;
    double eff = g_node->get_effective_speed();
    g_node->publish_cmd(eff, 0.0);
    double dist = g_node->get_total_distance();
    if (eff <= 0.0) {
        g_state.status = "⛔ BLOCKED — Wall too close";
    } else {
        g_state.status = "▶️ Moving Forward";
    }
    RCLCPP_INFO(g_node->get_logger(), "FORWARD speed=%.2f dist=%.2fm obstacle=%.2fm",
                eff, dist, g_node->get_collision_dist());
    update_status();
}

static void on_backward(GtkWidget*, gpointer) {
    if (g_state.emergency_stop || g_state.autonomous_mode) return;
    g_node->publish_cmd(-SPEED_LEVELS[g_node->speed_level_], 0.0);
    g_state.status = "◀️ Moving Backward";
    RCLCPP_INFO(g_node->get_logger(), "BACKWARD speed=%.2f dist=%.2fm",
                SPEED_LEVELS[g_node->speed_level_], g_node->get_total_distance());
    update_status();
}

static void on_left(GtkWidget*, gpointer) {
    if (g_state.emergency_stop || g_state.autonomous_mode) return;
    g_node->publish_cmd(0.0, TURN_SPEED);
    g_state.status = "↩️ Turning Left";
    RCLCPP_INFO(g_node->get_logger(), "LEFT turn dist=%.2fm", g_node->get_total_distance());
    update_status();
}

static void on_right(GtkWidget*, gpointer) {
    if (g_state.emergency_stop || g_state.autonomous_mode) return;
    g_node->publish_cmd(0.0, -TURN_SPEED);
    g_state.status = "↪️ Turning Right";
    RCLCPP_INFO(g_node->get_logger(), "RIGHT turn dist=%.2fm", g_node->get_total_distance());
    update_status();
}

static void on_stop(GtkWidget*, gpointer) {
    if (g_state.emergency_stop) return;
    g_node->publish_cmd(0.0, 0.0);
    g_state.status = "⏹️ Stopped";
    RCLCPP_INFO(g_node->get_logger(), "STOP dist=%.2fm", g_node->get_total_distance());
    update_status();
}

// ---- Emergency stop ----
static void on_estop(GtkWidget*, gpointer) {
    g_state.emergency_stop = true;
    g_state.vacuum_active = false;
    g_state.arm_active = false;
    g_state.wiper_active = false;
    g_state.uv_active = false;
    g_state.autonomous_mode = false;
    g_node->publish_cmd(0.0, 0.0);
    g_state.status = "🛑 EMERGENCY STOP";
    RCLCPP_WARN(g_node->get_logger(), "EMERGENCY STOP ACTIVATED");

    gtk_button_set_label(GTK_BUTTON(g_w.vacuum_btn), "🌪️ Vacuum");
    gtk_button_set_label(GTK_BUTTON(g_w.arm_btn), "🦾 Arm");
    gtk_button_set_label(GTK_BUTTON(g_w.wiper_btn), "🧹 Wiper");
    gtk_button_set_label(GTK_BUTTON(g_w.uv_btn), "💡 UV");
    gtk_button_set_label(GTK_BUTTON(g_w.auto_btn), "🤖 Auto");
    update_status();
}

static void on_reset(GtkWidget*, gpointer) {
    g_state.emergency_stop = false;
    g_state.status = "✅ System Ready";
    RCLCPP_INFO(g_node->get_logger(), "System RESET");
    update_status();
}

// ---- Component toggle callbacks ----
static void on_vacuum(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.vacuum_active = !g_state.vacuum_active;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.vacuum_active ? "🌪️ Vacuum ✅" : "🌪️ Vacuum");
    g_state.status = g_state.vacuum_active ? "Vacuum ON" : "Vacuum OFF";
    RCLCPP_INFO(g_node->get_logger(), "VACUUM %s", g_state.vacuum_active ? "ON" : "OFF");
    update_status();
}

static void on_arm(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.arm_active = !g_state.arm_active;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.arm_active ? "🦾 Arm ✅" : "🦾 Arm");
    g_state.status = g_state.arm_active ? "Arm ON" : "Arm OFF";
    RCLCPP_INFO(g_node->get_logger(), "ARM %s", g_state.arm_active ? "ON" : "OFF");
    update_status();
}

static void on_wiper(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.wiper_active = !g_state.wiper_active;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.wiper_active ? "🧹 Wiper ✅" : "🧹 Wiper");
    g_state.status = g_state.wiper_active ? "Wiper ON" : "Wiper OFF";
    RCLCPP_INFO(g_node->get_logger(), "WIPER %s", g_state.wiper_active ? "ON" : "OFF");
    update_status();
}

static void on_uv(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.uv_active = !g_state.uv_active;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.uv_active ? "💡 UV ✅" : "💡 UV");
    g_state.status = g_state.uv_active ? "UV ON" : "UV OFF";
    RCLCPP_INFO(g_node->get_logger(), "UV %s", g_state.uv_active ? "ON" : "OFF");
    update_status();
}

static void on_auto_toggle(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.autonomous_mode = !g_state.autonomous_mode;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.autonomous_mode ? "🤖 Auto ✅" : "🤖 Auto");
    if (g_state.autonomous_mode) {
        g_state.status = "🤖 Autonomous Mode";
        g_node->publish_cmd(0.0, 0.0);
    } else {
        g_state.status = "🕹️ Manual Mode";
    }
    RCLCPP_INFO(g_node->get_logger(), "AUTO %s", g_state.autonomous_mode ? "ON" : "OFF");
    update_status();
}

// ---- Speed level callbacks ----
static void on_speed1(GtkWidget*, gpointer) {
    g_node->speed_level_ = 0;
    RCLCPP_INFO(g_node->get_logger(), "Speed level 1 (0.10 m/s)");
}
static void on_speed2(GtkWidget*, gpointer) {
    g_node->speed_level_ = 1;
    RCLCPP_INFO(g_node->get_logger(), "Speed level 2 (0.22 m/s)");
}
static void on_speed3(GtkWidget*, gpointer) {
    g_node->speed_level_ = 2;
    RCLCPP_INFO(g_node->get_logger(), "Speed level 3 (0.50 m/s)");
}

// ---- CSS styling ----
static void apply_css() {
    GtkCssProvider *provider = gtk_css_provider_new();
    const char *css =
        "window { background-color: #1a1a2e; }\n"
        "label { color: #e0e0e0; }\n"
        "button { border-radius: 8px; padding: 8px 16px; font-weight: bold;\n"
        "         background: linear-gradient(#2d3436, #636e72);\n"
        "         color: white; border: 1px solid #555; }\n"
        "button:hover { background: linear-gradient(#636e72, #2d3436); }\n"
        "button.destructive-action { background: linear-gradient(#c0392b, #e74c3c);\n"
        "                            color: white; }\n"
        "button.destructive-action:hover { background: linear-gradient(#e74c3c, #c0392b); }\n"
        "button.suggested-action { background: linear-gradient(#2980b9, #3498db);\n"
        "                          color: white; }\n"
        "button.suggested-action:hover { background: linear-gradient(#3498db, #2980b9); }\n"
        "frame { border: 1px solid #444; border-radius: 8px; padding: 4px; }\n"
        "frame > label { color: #aaa; font-size: 11px; }\n"
        ".speed-active { background: linear-gradient(#27ae60, #2ecc71); color: white; }\n";

    gtk_css_provider_load_from_data(provider, css, -1);
    gtk_style_context_add_provider_for_display(
        gdk_display_get_default(),
        GTK_STYLE_PROVIDER(provider),
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION
    );
    g_object_unref(provider);
}

// ---- Build the GUI ----
static void activate(GtkApplication *app, gpointer) {
    apply_css();

    GtkWidget *window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(window), "Swachh Robot — GUI Control");
    gtk_window_set_default_size(GTK_WINDOW(window), 800, 700);

    // Main vertical box
    GtkWidget *main_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_margin_start(main_box, 12);
    gtk_widget_set_margin_end(main_box, 12);
    gtk_widget_set_margin_top(main_box, 8);
    gtk_widget_set_margin_bottom(main_box, 8);
    gtk_window_set_child(GTK_WINDOW(window), main_box);

    // ---- Title ----
    GtkWidget *title = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(title),
        "<span font='18' weight='bold' foreground='#3498db'>🤖 Swachh Robot HMI</span>");
    gtk_box_append(GTK_BOX(main_box), title);

    // ---- Status + Battery ----
    g_w.status_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.status_label),
        "<span font='12' weight='bold'>✅ System Ready</span>");
    gtk_box_append(GTK_BOX(main_box), g_w.status_label);

    g_w.battery_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.battery_label),
        "<span font='11'>🔋 Battery: 100%</span>");
    gtk_box_append(GTK_BOX(main_box), g_w.battery_label);

    // ---- Telemetry section ----
    GtkWidget *telemetry_frame = gtk_frame_new("Telemetry");
    gtk_box_append(GTK_BOX(main_box), telemetry_frame);

    GtkWidget *tbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_margin_start(tbox, 8);
    gtk_widget_set_margin_end(tbox, 8);
    gtk_widget_set_margin_top(tbox, 4);
    gtk_widget_set_margin_bottom(tbox, 4);
    gtk_frame_set_child(GTK_FRAME(telemetry_frame), tbox);

    g_w.distance_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.distance_label),
        "<span font='14' weight='bold'>📏 Distance Moved: 0.00 m</span>");
    gtk_box_append(GTK_BOX(tbox), g_w.distance_label);

    g_w.obstacle_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.obstacle_label),
        "<span font='13' weight='bold' foreground='green'>🛡️ Obstacle: — m</span>");
    gtk_box_append(GTK_BOX(tbox), g_w.obstacle_label);

    g_w.position_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.position_label),
        "<span font='11'>📍 Position: (0.00, 0.00)</span>");
    gtk_box_append(GTK_BOX(tbox), g_w.position_label);

    g_w.speed_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.speed_label),
        "<span font='11'>⚡ Speed: 0.22 m/s  |  Level: 2/3</span>");
    gtk_box_append(GTK_BOX(tbox), g_w.speed_label);

    // ---- Emergency Stop ----
    GtkWidget *estop_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_widget_set_halign(estop_box, GTK_ALIGN_CENTER);
    gtk_box_append(GTK_BOX(main_box), estop_box);

    g_w.estop_btn = gtk_button_new_with_label("🛑 EMERGENCY STOP");
    gtk_widget_set_size_request(g_w.estop_btn, 260, 55);
    gtk_widget_add_css_class(g_w.estop_btn, "destructive-action");
    g_signal_connect(g_w.estop_btn, "clicked", G_CALLBACK(on_estop), NULL);
    gtk_box_append(GTK_BOX(estop_box), g_w.estop_btn);

    GtkWidget *reset_btn = gtk_button_new_with_label("🔄 Reset");
    gtk_widget_set_size_request(reset_btn, 120, 55);
    gtk_widget_add_css_class(reset_btn, "suggested-action");
    g_signal_connect(reset_btn, "clicked", G_CALLBACK(on_reset), NULL);
    gtk_box_append(GTK_BOX(estop_box), reset_btn);

    // ---- Component Controls ----
    GtkWidget *comp_frame = gtk_frame_new("Component Controls");
    gtk_box_append(GTK_BOX(main_box), comp_frame);

    GtkWidget *comp_grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(comp_grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(comp_grid), 5);
    gtk_widget_set_margin_start(comp_grid, 5);
    gtk_widget_set_margin_end(comp_grid, 5);
    gtk_widget_set_margin_top(comp_grid, 5);
    gtk_widget_set_margin_bottom(comp_grid, 5);
    gtk_widget_set_halign(comp_grid, GTK_ALIGN_CENTER);
    gtk_frame_set_child(GTK_FRAME(comp_frame), comp_grid);

    g_w.vacuum_btn = gtk_button_new_with_label("🌪️ Vacuum");
    gtk_widget_set_size_request(g_w.vacuum_btn, 150, 48);
    g_signal_connect(g_w.vacuum_btn, "clicked", G_CALLBACK(on_vacuum), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.vacuum_btn, 0, 0, 1, 1);

    g_w.arm_btn = gtk_button_new_with_label("🦾 Arm");
    gtk_widget_set_size_request(g_w.arm_btn, 150, 48);
    g_signal_connect(g_w.arm_btn, "clicked", G_CALLBACK(on_arm), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.arm_btn, 1, 0, 1, 1);

    g_w.wiper_btn = gtk_button_new_with_label("🧹 Wiper");
    gtk_widget_set_size_request(g_w.wiper_btn, 150, 48);
    g_signal_connect(g_w.wiper_btn, "clicked", G_CALLBACK(on_wiper), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.wiper_btn, 2, 0, 1, 1);

    g_w.uv_btn = gtk_button_new_with_label("💡 UV");
    gtk_widget_set_size_request(g_w.uv_btn, 150, 48);
    g_signal_connect(g_w.uv_btn, "clicked", G_CALLBACK(on_uv), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.uv_btn, 3, 0, 1, 1);

    g_w.auto_btn = gtk_button_new_with_label("🤖 Auto");
    gtk_widget_set_size_request(g_w.auto_btn, 150, 48);
    g_signal_connect(g_w.auto_btn, "clicked", G_CALLBACK(on_auto_toggle), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.auto_btn, 0, 1, 2, 1);

    // ---- Speed Controls ----
    GtkWidget *speed_frame = gtk_frame_new("Speed Level");
    gtk_box_append(GTK_BOX(main_box), speed_frame);

    GtkWidget *speed_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_widget_set_halign(speed_box, GTK_ALIGN_CENTER);
    gtk_widget_set_margin_start(speed_box, 5);
    gtk_widget_set_margin_end(speed_box, 5);
    gtk_widget_set_margin_top(speed_box, 5);
    gtk_widget_set_margin_bottom(speed_box, 5);
    gtk_frame_set_child(GTK_FRAME(speed_frame), speed_box);

    GtkWidget *s1 = gtk_button_new_with_label("🐢 Slow (0.10)");
    gtk_widget_set_size_request(s1, 150, 40);
    g_signal_connect(s1, "clicked", G_CALLBACK(on_speed1), NULL);
    gtk_box_append(GTK_BOX(speed_box), s1);

    GtkWidget *s2 = gtk_button_new_with_label("🚶 Normal (0.22)");
    gtk_widget_set_size_request(s2, 150, 40);
    gtk_widget_add_css_class(s2, "suggested-action");
    g_signal_connect(s2, "clicked", G_CALLBACK(on_speed2), NULL);
    gtk_box_append(GTK_BOX(speed_box), s2);

    GtkWidget *s3 = gtk_button_new_with_label("🏃 Fast (0.50)");
    gtk_widget_set_size_request(s3, 150, 40);
    g_signal_connect(s3, "clicked", G_CALLBACK(on_speed3), NULL);
    gtk_box_append(GTK_BOX(speed_box), s3);

    // ---- Movement Controls ----
    GtkWidget *move_frame = gtk_frame_new("Movement Controls");
    gtk_box_append(GTK_BOX(main_box), move_frame);

    GtkWidget *move_grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(move_grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(move_grid), 5);
    gtk_widget_set_margin_start(move_grid, 5);
    gtk_widget_set_margin_end(move_grid, 5);
    gtk_widget_set_margin_top(move_grid, 5);
    gtk_widget_set_margin_bottom(move_grid, 5);
    gtk_widget_set_halign(move_grid, GTK_ALIGN_CENTER);
    gtk_frame_set_child(GTK_FRAME(move_frame), move_grid);

    GtkWidget *fwd = gtk_button_new_with_label("⬆️  Forward");
    gtk_widget_set_size_request(fwd, 140, 55);
    g_signal_connect(fwd, "clicked", G_CALLBACK(on_forward), NULL);
    gtk_grid_attach(GTK_GRID(move_grid), fwd, 1, 0, 1, 1);

    GtkWidget *left = gtk_button_new_with_label("⬅️  Left");
    gtk_widget_set_size_request(left, 140, 55);
    g_signal_connect(left, "clicked", G_CALLBACK(on_left), NULL);
    gtk_grid_attach(GTK_GRID(move_grid), left, 0, 1, 1, 1);

    GtkWidget *stop = gtk_button_new_with_label("⏹️  Stop");
    gtk_widget_set_size_request(stop, 140, 55);
    gtk_widget_add_css_class(stop, "destructive-action");
    g_signal_connect(stop, "clicked", G_CALLBACK(on_stop), NULL);
    gtk_grid_attach(GTK_GRID(move_grid), stop, 1, 1, 1, 1);

    GtkWidget *right = gtk_button_new_with_label("➡️  Right");
    gtk_widget_set_size_request(right, 140, 55);
    g_signal_connect(right, "clicked", G_CALLBACK(on_right), NULL);
    gtk_grid_attach(GTK_GRID(move_grid), right, 2, 1, 1, 1);

    GtkWidget *bwd = gtk_button_new_with_label("⬇️  Backward");
    gtk_widget_set_size_request(bwd, 140, 55);
    g_signal_connect(bwd, "clicked", G_CALLBACK(on_backward), NULL);
    gtk_grid_attach(GTK_GRID(move_grid), bwd, 1, 2, 1, 1);

    // Show window
    gtk_widget_set_visible(window, TRUE);

    // Timers
    g_timeout_add(50, ros_spin_tick, NULL);         // 20Hz ROS spin + GUI update
    g_timeout_add_seconds(5, battery_tick, NULL);   // Battery sim
}

// ============== MAIN ==============

int main(int argc, char *argv[]) {
    // Init ROS 2
    rclcpp::init(argc, argv);
    g_node = std::make_shared<GuiTeleopNode>();

    // Create GTK application
    GtkApplication *app = gtk_application_new("com.swachh.gui_teleop",
                                               G_APPLICATION_FLAGS_NONE);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);

    int status = g_application_run(G_APPLICATION(app), argc, argv);

    // Cleanup
    g_node->publish_cmd(0.0, 0.0);  // Stop robot
    g_object_unref(app);
    rclcpp::shutdown();

    return status;
}
