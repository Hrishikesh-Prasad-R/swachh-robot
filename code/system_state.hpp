#ifndef SYSTEM_STATE_HPP
#define SYSTEM_STATE_HPP

#include <string>

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

// Global system state declaration
extern SystemState g_system_state;

#endif // SYSTEM_STATE_HPP
