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
static constexpr int    NUM_SPEED_LEVELS = 9;
static constexpr double SPEED_LEVELS[NUM_SPEED_LEVELS] = {
    0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.65, 0.80
};
static const char* SPEED_NAMES[NUM_SPEED_LEVELS] = {
    "1: Crawl  (0.05)", "2: Slow   (0.10)", "3: Gentle (0.15)",
    "4: Medium (0.20)", "5: Normal (0.30)", "6: Fast   (0.40)",
    "7: Faster (0.50)", "8: Rush   (0.65)", "9: Max    (0.80)"
};
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

    int speed_level_ = 4;       // default: level 5 = Normal (0.30 m/s)
    double current_linear_ = 0.0;
    double current_angular_ = 0.0;

private:
    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        int n = static_cast<int>(msg->ranges.size());
        if (n == 0) return;

        double half_cone_rad = (COLLISION_CONE_DEG / 2.0) * M_PI / 180.0;
        double min_dist = std::numeric_limits<double>::infinity();

        for (int i = 0; i < n; i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            while (angle > M_PI) angle -= 2 * M_PI;
            while (angle < -M_PI) angle += 2 * M_PI;

            if (std::abs(angle) <= half_cone_rad) {
                double r = msg->ranges[i];
                if (!std::isinf(r) && !std::isnan(r) && r > 0.01) {
                    min_dist = std::min(min_dist, static_cast<double>(r));
                }
            }
        }

        collision_history_.push_back(min_dist);
        if (collision_history_.size() > MEDIAN_SIZE) {
            collision_history_.pop_front();
        }

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

static std::shared_ptr<GuiTeleopNode> g_node;
static bool g_moving = false;  // true while any movement button is held

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
    GtkWidget *speed_buttons[NUM_SPEED_LEVELS];
};
static Widgets g_w;

// ---- Helper: update status display ----
static void update_status() {
    std::string markup = "<span font='12' weight='bold'>" + g_state.status + "</span>";
    gtk_label_set_markup(GTK_LABEL(g_w.status_label), markup.c_str());

    std::string bat = "<span font='11'>Battery: " + std::to_string(g_state.battery_level) + "%</span>";
    gtk_label_set_markup(GTK_LABEL(g_w.battery_label), bat.c_str());
}

// ---- Helper: highlight active speed button ----
static void update_speed_buttons() {
    for (int i = 0; i < NUM_SPEED_LEVELS; i++) {
        if (i == g_node->speed_level_) {
            gtk_widget_add_css_class(g_w.speed_buttons[i], "speed-active");
        } else {
            gtk_widget_remove_css_class(g_w.speed_buttons[i], "speed-active");
        }
    }
}

static void update_telemetry() {
    double dist = g_node->get_total_distance();
    double obs = g_node->get_collision_dist();
    double px = g_node->get_position_x();
    double py = g_node->get_position_y();

    char buf[128];
    snprintf(buf, sizeof(buf),
             "<span font='14' weight='bold'>Distance Moved: %.2f m</span>", dist);
    gtk_label_set_markup(GTK_LABEL(g_w.distance_label), buf);

    double gap = obs - ROBOT_RADIUS;
    const char *color = (gap < 0.3) ? "red" : (gap < SLOW_ZONE) ? "orange" : "green";
    if (std::isinf(obs)) {
        snprintf(buf, sizeof(buf),
                 "<span font='13' weight='bold' foreground='green'>Obstacle: &gt; 3.5 m (clear)</span>");
    } else {
        snprintf(buf, sizeof(buf),
                 "<span font='13' weight='bold' foreground='%s'>Obstacle: %.2f m</span>",
                 color, obs);
    }
    gtk_label_set_markup(GTK_LABEL(g_w.obstacle_label), buf);

    snprintf(buf, sizeof(buf),
             "<span font='11'>Position: (%.2f, %.2f)</span>", px, py);
    gtk_label_set_markup(GTK_LABEL(g_w.position_label), buf);

    snprintf(buf, sizeof(buf),
             "<span font='11'>Speed: %.2f m/s  |  Level: %d/%d</span>",
             SPEED_LEVELS[g_node->speed_level_], g_node->speed_level_ + 1, NUM_SPEED_LEVELS);
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
                    g_state.status = "BLOCKED - Wall too close";
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

// ---- Movement: press = start, release = stop ----

// Generic release handler: stop the robot
static void on_move_release(GtkGestureClick*, int, double, double, gpointer) {
    g_moving = false;
    g_node->publish_cmd(0.0, 0.0);
    g_state.status = "Stopped";
    RCLCPP_INFO(g_node->get_logger(), "RELEASED dist=%.2fm", g_node->get_total_distance());
    update_status();
}

// Press handlers for each direction
static void on_forward_press(GtkGestureClick*, int, double, double, gpointer) {
    if (g_state.emergency_stop || g_state.autonomous_mode) return;
    double eff = g_node->get_effective_speed();
    if (eff <= 0.0) {
        g_state.status = "BLOCKED - Wall too close";
        update_status();
        return;
    }
    g_moving = true;
    g_node->publish_cmd(eff, 0.0);
    g_state.status = "Moving Forward (hold)";
    RCLCPP_INFO(g_node->get_logger(), "FORWARD speed=%.2f dist=%.2fm obstacle=%.2fm",
                eff, g_node->get_total_distance(), g_node->get_collision_dist());
    update_status();
}

static void on_backward_press(GtkGestureClick*, int, double, double, gpointer) {
    if (g_state.emergency_stop || g_state.autonomous_mode) return;
    g_moving = true;
    g_node->publish_cmd(-SPEED_LEVELS[g_node->speed_level_], 0.0);
    g_state.status = "Moving Backward (hold)";
    RCLCPP_INFO(g_node->get_logger(), "BACKWARD speed=%.2f dist=%.2fm",
                SPEED_LEVELS[g_node->speed_level_], g_node->get_total_distance());
    update_status();
}

static void on_left_press(GtkGestureClick*, int, double, double, gpointer) {
    if (g_state.emergency_stop || g_state.autonomous_mode) return;
    g_moving = true;
    g_node->publish_cmd(0.0, TURN_SPEED);
    g_state.status = "Turning Left (hold)";
    RCLCPP_INFO(g_node->get_logger(), "LEFT turn dist=%.2fm", g_node->get_total_distance());
    update_status();
}

static void on_right_press(GtkGestureClick*, int, double, double, gpointer) {
    if (g_state.emergency_stop || g_state.autonomous_mode) return;
    g_moving = true;
    g_node->publish_cmd(0.0, -TURN_SPEED);
    g_state.status = "Turning Right (hold)";
    RCLCPP_INFO(g_node->get_logger(), "RIGHT turn dist=%.2fm", g_node->get_total_distance());
    update_status();
}

static void on_stop(GtkWidget*, gpointer) {
    if (g_state.emergency_stop) return;
    g_moving = false;
    g_node->publish_cmd(0.0, 0.0);
    g_state.status = "Stopped";
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
    g_moving = false;
    g_node->publish_cmd(0.0, 0.0);
    g_state.status = "EMERGENCY STOP";
    RCLCPP_WARN(g_node->get_logger(), "EMERGENCY STOP ACTIVATED");

    gtk_button_set_label(GTK_BUTTON(g_w.vacuum_btn), "Vacuum");
    gtk_button_set_label(GTK_BUTTON(g_w.arm_btn), "Arm");
    gtk_button_set_label(GTK_BUTTON(g_w.wiper_btn), "Wiper");
    gtk_button_set_label(GTK_BUTTON(g_w.uv_btn), "UV");
    gtk_button_set_label(GTK_BUTTON(g_w.auto_btn), "Auto");
    update_status();
}

static void on_reset(GtkWidget*, gpointer) {
    g_state.emergency_stop = false;
    g_state.status = "System Ready";
    RCLCPP_INFO(g_node->get_logger(), "System RESET");
    update_status();
}

// ---- Component toggle callbacks ----
static void on_vacuum(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.vacuum_active = !g_state.vacuum_active;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.vacuum_active ? "Vacuum [ON]" : "Vacuum");
    g_state.status = g_state.vacuum_active ? "Vacuum ON" : "Vacuum OFF";
    RCLCPP_INFO(g_node->get_logger(), "VACUUM %s", g_state.vacuum_active ? "ON" : "OFF");
    update_status();
}

static void on_arm(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.arm_active = !g_state.arm_active;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.arm_active ? "Arm [ON]" : "Arm");
    g_state.status = g_state.arm_active ? "Arm ON" : "Arm OFF";
    RCLCPP_INFO(g_node->get_logger(), "ARM %s", g_state.arm_active ? "ON" : "OFF");
    update_status();
}

static void on_wiper(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.wiper_active = !g_state.wiper_active;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.wiper_active ? "Wiper [ON]" : "Wiper");
    g_state.status = g_state.wiper_active ? "Wiper ON" : "Wiper OFF";
    RCLCPP_INFO(g_node->get_logger(), "WIPER %s", g_state.wiper_active ? "ON" : "OFF");
    update_status();
}

static void on_uv(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.uv_active = !g_state.uv_active;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.uv_active ? "UV [ON]" : "UV");
    g_state.status = g_state.uv_active ? "UV ON" : "UV OFF";
    RCLCPP_INFO(g_node->get_logger(), "UV %s", g_state.uv_active ? "ON" : "OFF");
    update_status();
}

static void on_auto_toggle(GtkWidget *w, gpointer) {
    if (g_state.emergency_stop) return;
    g_state.autonomous_mode = !g_state.autonomous_mode;
    gtk_button_set_label(GTK_BUTTON(w),
        g_state.autonomous_mode ? "Auto [ON]" : "Auto");
    if (g_state.autonomous_mode) {
        g_state.status = "Autonomous Mode";
        g_node->publish_cmd(0.0, 0.0);
    } else {
        g_state.status = "Manual Mode";
    }
    RCLCPP_INFO(g_node->get_logger(), "AUTO %s", g_state.autonomous_mode ? "ON" : "OFF");
    update_status();
}

// ---- Speed level callback (single handler, level passed as data) ----
static void on_speed_select(GtkWidget*, gpointer data) {
    int level = GPOINTER_TO_INT(data);
    g_node->speed_level_ = level;
    update_speed_buttons();
    RCLCPP_INFO(g_node->get_logger(), "Speed level %d (%.2f m/s)", level + 1, SPEED_LEVELS[level]);
}

// ---- CSS styling ----
static void apply_css() {
    GtkCssProvider *provider = gtk_css_provider_new();
    const char *css =
        "window { background-color: #1a1a2e; }\n"
        "label { color: #e0e0e0; }\n"
        "button { border-radius: 6px; padding: 6px 12px; font-weight: bold;\n"
        "         background: linear-gradient(#2d3436, #636e72);\n"
        "         color: white; border: 1px solid #555; font-size: 12px; }\n"
        "button:hover { background: linear-gradient(#636e72, #2d3436); }\n"
        "button.destructive-action { background: linear-gradient(#c0392b, #e74c3c);\n"
        "                            color: white; }\n"
        "button.destructive-action:hover { background: linear-gradient(#e74c3c, #c0392b); }\n"
        "button.suggested-action { background: linear-gradient(#2980b9, #3498db);\n"
        "                          color: white; }\n"
        "button.suggested-action:hover { background: linear-gradient(#3498db, #2980b9); }\n"
        "button.speed-active { background: linear-gradient(#27ae60, #2ecc71);\n"
        "                      color: white; border: 2px solid #1e8449; }\n"
        "frame { border: 1px solid #444; border-radius: 8px; padding: 4px; }\n"
        "frame > label { color: #aaa; font-size: 11px; }\n";

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
    gtk_window_set_title(GTK_WINDOW(window), "Swachh Robot - GUI Control");
    gtk_window_set_default_size(GTK_WINDOW(window), 820, 750);

    // Scrollable wrapper
    GtkWidget *scroll = gtk_scrolled_window_new();
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll),
                                   GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);
    gtk_window_set_child(GTK_WINDOW(window), scroll);

    // Main vertical box
    GtkWidget *main_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_widget_set_margin_start(main_box, 12);
    gtk_widget_set_margin_end(main_box, 12);
    gtk_widget_set_margin_top(main_box, 8);
    gtk_widget_set_margin_bottom(main_box, 8);
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(scroll), main_box);

    // ---- Title ----
    GtkWidget *title = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(title),
        "<span font='16' weight='bold' foreground='#3498db'>Swachh Robot HMI</span>");
    gtk_box_append(GTK_BOX(main_box), title);

    // ---- Status + Battery ----
    g_w.status_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.status_label),
        "<span font='12' weight='bold'>System Ready</span>");
    gtk_box_append(GTK_BOX(main_box), g_w.status_label);

    g_w.battery_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.battery_label),
        "<span font='11'>Battery: 100%</span>");
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
        "<span font='14' weight='bold'>Distance Moved: 0.00 m</span>");
    gtk_box_append(GTK_BOX(tbox), g_w.distance_label);

    g_w.obstacle_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.obstacle_label),
        "<span font='13' weight='bold' foreground='green'>Obstacle: -- m</span>");
    gtk_box_append(GTK_BOX(tbox), g_w.obstacle_label);

    g_w.position_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.position_label),
        "<span font='11'>Position: (0.00, 0.00)</span>");
    gtk_box_append(GTK_BOX(tbox), g_w.position_label);

    g_w.speed_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(g_w.speed_label),
        "<span font='11'>Speed: 0.30 m/s  |  Level: 5/9</span>");
    gtk_box_append(GTK_BOX(tbox), g_w.speed_label);

    // ---- Emergency Stop ----
    GtkWidget *estop_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_widget_set_halign(estop_box, GTK_ALIGN_CENTER);
    gtk_box_append(GTK_BOX(main_box), estop_box);

    g_w.estop_btn = gtk_button_new_with_label("EMERGENCY STOP");
    gtk_widget_set_size_request(g_w.estop_btn, 240, 50);
    gtk_widget_add_css_class(g_w.estop_btn, "destructive-action");
    g_signal_connect(g_w.estop_btn, "clicked", G_CALLBACK(on_estop), NULL);
    gtk_box_append(GTK_BOX(estop_box), g_w.estop_btn);

    GtkWidget *reset_btn = gtk_button_new_with_label("Reset");
    gtk_widget_set_size_request(reset_btn, 100, 50);
    gtk_widget_add_css_class(reset_btn, "suggested-action");
    g_signal_connect(reset_btn, "clicked", G_CALLBACK(on_reset), NULL);
    gtk_box_append(GTK_BOX(estop_box), reset_btn);

    // ---- Component Controls ----
    GtkWidget *comp_frame = gtk_frame_new("Component Controls");
    gtk_box_append(GTK_BOX(main_box), comp_frame);

    GtkWidget *comp_grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(comp_grid), 4);
    gtk_grid_set_column_spacing(GTK_GRID(comp_grid), 4);
    gtk_widget_set_margin_start(comp_grid, 5);
    gtk_widget_set_margin_end(comp_grid, 5);
    gtk_widget_set_margin_top(comp_grid, 4);
    gtk_widget_set_margin_bottom(comp_grid, 4);
    gtk_widget_set_halign(comp_grid, GTK_ALIGN_CENTER);
    gtk_frame_set_child(GTK_FRAME(comp_frame), comp_grid);

    g_w.vacuum_btn = gtk_button_new_with_label("Vacuum");
    gtk_widget_set_size_request(g_w.vacuum_btn, 140, 42);
    g_signal_connect(g_w.vacuum_btn, "clicked", G_CALLBACK(on_vacuum), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.vacuum_btn, 0, 0, 1, 1);

    g_w.arm_btn = gtk_button_new_with_label("Arm");
    gtk_widget_set_size_request(g_w.arm_btn, 140, 42);
    g_signal_connect(g_w.arm_btn, "clicked", G_CALLBACK(on_arm), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.arm_btn, 1, 0, 1, 1);

    g_w.wiper_btn = gtk_button_new_with_label("Wiper");
    gtk_widget_set_size_request(g_w.wiper_btn, 140, 42);
    g_signal_connect(g_w.wiper_btn, "clicked", G_CALLBACK(on_wiper), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.wiper_btn, 2, 0, 1, 1);

    g_w.uv_btn = gtk_button_new_with_label("UV");
    gtk_widget_set_size_request(g_w.uv_btn, 140, 42);
    g_signal_connect(g_w.uv_btn, "clicked", G_CALLBACK(on_uv), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.uv_btn, 3, 0, 1, 1);

    g_w.auto_btn = gtk_button_new_with_label("Auto");
    gtk_widget_set_size_request(g_w.auto_btn, 140, 42);
    g_signal_connect(g_w.auto_btn, "clicked", G_CALLBACK(on_auto_toggle), NULL);
    gtk_grid_attach(GTK_GRID(comp_grid), g_w.auto_btn, 0, 1, 2, 1);

    // ---- Speed Controls (9 levels in a grid) ----
    GtkWidget *speed_frame = gtk_frame_new("Speed Level (1-9)");
    gtk_box_append(GTK_BOX(main_box), speed_frame);

    GtkWidget *speed_grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(speed_grid), 3);
    gtk_grid_set_column_spacing(GTK_GRID(speed_grid), 3);
    gtk_widget_set_margin_start(speed_grid, 5);
    gtk_widget_set_margin_end(speed_grid, 5);
    gtk_widget_set_margin_top(speed_grid, 4);
    gtk_widget_set_margin_bottom(speed_grid, 4);
    gtk_widget_set_halign(speed_grid, GTK_ALIGN_CENTER);
    gtk_frame_set_child(GTK_FRAME(speed_frame), speed_grid);

    for (int i = 0; i < NUM_SPEED_LEVELS; i++) {
        g_w.speed_buttons[i] = gtk_button_new_with_label(SPEED_NAMES[i]);
        gtk_widget_set_size_request(g_w.speed_buttons[i], 160, 34);
        g_signal_connect(g_w.speed_buttons[i], "clicked",
                         G_CALLBACK(on_speed_select), GINT_TO_POINTER(i));
        // 3 columns layout: row = i/3, col = i%3
        gtk_grid_attach(GTK_GRID(speed_grid), g_w.speed_buttons[i],
                         i % 3, i / 3, 1, 1);
    }
    // Highlight the default speed button
    update_speed_buttons();

    // ---- Movement Controls (hold to move, release to stop) ----
    GtkWidget *move_frame = gtk_frame_new("Movement Controls (hold to move)");
    gtk_box_append(GTK_BOX(main_box), move_frame);

    GtkWidget *move_grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(move_grid), 4);
    gtk_grid_set_column_spacing(GTK_GRID(move_grid), 4);
    gtk_widget_set_margin_start(move_grid, 5);
    gtk_widget_set_margin_end(move_grid, 5);
    gtk_widget_set_margin_top(move_grid, 4);
    gtk_widget_set_margin_bottom(move_grid, 4);
    gtk_widget_set_halign(move_grid, GTK_ALIGN_CENTER);
    gtk_frame_set_child(GTK_FRAME(move_frame), move_grid);

    // Helper lambda to attach press/release gesture to a button
    auto attach_move_gesture = [](GtkWidget *btn, GCallback press_cb) {
        GtkGesture *gesture = gtk_gesture_click_new();
        g_signal_connect(gesture, "pressed", press_cb, NULL);
        g_signal_connect(gesture, "released", G_CALLBACK(on_move_release), NULL);
        gtk_widget_add_controller(btn, GTK_EVENT_CONTROLLER(gesture));
    };

    GtkWidget *fwd = gtk_button_new_with_label("Forward");
    gtk_widget_set_size_request(fwd, 130, 50);
    attach_move_gesture(fwd, G_CALLBACK(on_forward_press));
    gtk_grid_attach(GTK_GRID(move_grid), fwd, 1, 0, 1, 1);

    GtkWidget *left = gtk_button_new_with_label("Left");
    gtk_widget_set_size_request(left, 130, 50);
    attach_move_gesture(left, G_CALLBACK(on_left_press));
    gtk_grid_attach(GTK_GRID(move_grid), left, 0, 1, 1, 1);

    GtkWidget *stop = gtk_button_new_with_label("STOP");
    gtk_widget_set_size_request(stop, 130, 50);
    gtk_widget_add_css_class(stop, "destructive-action");
    g_signal_connect(stop, "clicked", G_CALLBACK(on_stop), NULL);
    gtk_grid_attach(GTK_GRID(move_grid), stop, 1, 1, 1, 1);

    GtkWidget *right = gtk_button_new_with_label("Right");
    gtk_widget_set_size_request(right, 130, 50);
    attach_move_gesture(right, G_CALLBACK(on_right_press));
    gtk_grid_attach(GTK_GRID(move_grid), right, 2, 1, 1, 1);

    GtkWidget *bwd = gtk_button_new_with_label("Backward");
    gtk_widget_set_size_request(bwd, 130, 50);
    attach_move_gesture(bwd, G_CALLBACK(on_backward_press));
    gtk_grid_attach(GTK_GRID(move_grid), bwd, 1, 2, 1, 1);

    // Show window
    gtk_widget_set_visible(window, TRUE);

    // Timers
    g_timeout_add(50, ros_spin_tick, NULL);         // 20Hz ROS spin + GUI update
    g_timeout_add_seconds(5, battery_tick, NULL);   // Battery sim
}

// ============== MAIN ==============

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    g_node = std::make_shared<GuiTeleopNode>();

    GtkApplication *app = gtk_application_new("com.swachh.gui_teleop",
                                               G_APPLICATION_FLAGS_NONE);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);

    int status = g_application_run(G_APPLICATION(app), argc, argv);

    g_node->publish_cmd(0.0, 0.0);
    g_object_unref(app);
    rclcpp::shutdown();

    return status;
}
