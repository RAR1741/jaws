package frc.robot;

import java.util.Map;
import java.util.HashMap;
import java.lang.reflect.Field;

/**
 * 
 */
public class Config {
    /**
     * Map holding a keyed list of all the settings.
     */
    static Map<String, Double> mSettings = new HashMap<String, Double>();

    static class ConfigValues {
        // I didn't feel like doing any file i/o stuff, so this is what you get.
        // BEGIN: config.txt circa May 4, 2014
        public static double
        //# config.txt
        //# CAM PID
        //cam_p = .08
        //cam_i = .0001
        //cam_d = 0.2
        
        cam_p = 0.08,
        cam_i = 0.00005,
        cam_d = 0.8,
        
        //# Robot Loop period
        robot_loop_period = .02,
        
        //# CAM
        cam_loop_period = 0.004,
        robot_cam_rearm_rate = 25,
        
        // cam_ready_to_fire_position = 46.7,
        // cam_point_of_no_return = 51,
        // cam_fire_to_position = 52.3,
        
        cam_ready_to_fire_position = 54,
        cam_fire_to_position = 63,
        cam_point_of_no_return = 58,
        
        //cam_fire_position_tolerance = 1, //This was the original value, changed to 3 to make more responsive
        cam_fire_position_tolerance = 3,
        cam_setpoint_error_limit = 25,
        cam_home_speed = .6,
        cam_home_speed_slow = .25,
        cam_home_speed_cutoff = 50,
        cam_eject_position = 21,
        
        //#Odometer stuff
        //auto_firing_distance = 96, //#Distance from the goal that robot will be shooting from in inches
        auto_collection_delay = 1.0,
        odometer_auto_speed = -.1,
        
        //# Autonomous
        auto_case = 5,
        auto_heading_p = 0.04,
        auto_speed = -1.0, //#-0.6 
        //#0.5
        auto_firing_distance = 132,
        auto_drive_distance = 100,
        auto_target_timeout = 1.0,
        auto_target_hot_timeout = 5.0,
        
        //#Driving variables
        // xaxis_percent = 1,
        
        
        //# Autonomous mode 5 variables - Two ball drag mode
        auton5_drive_speed = -1.0,
        auton5_gyro_reset_delay = 2.0,
        auton5_extend_delay = 1.0,
        auton5_drag_speed = -0.4,
        auton5_drive_distance = 120.0,
        auton5_eject_speed = 0.0,
        auton5_ball_1_launch_delay = 0.5,
        auton5_ball_2_ready2fire_delay = 1.5,
        auton5_intake_roller_speed = -0.7,
        auton5_ball_2_settle_delay = 2.5,
        auton5_uneject_time = 1.0,
        auton5_backup_distance = 6,
        
        //# Autonomous mode 6 variables - Two ball retrieve mode
        auton6_drive_forward_distance = 96.0,
        auton6_ball_1_fire_delay = 1.0,
        auton6_intake_roller_speed = 1.0,
        auton6_ball_2_load_delay = 1.0,
        
        //# Autonomous mode 7 variables - Two ball drag mode
        auton7_gyro_reset_delay = 2.0,
        auton7_extend_delay = 1.0,
        auton7_drag_speed = -0.39,
        auton7_drive_distance = 108.0,
        auton7_eject_speed = 0.0,
        auton7_ball_1_launch_delay = 1.5,
        auton7_ball_2_ready2fire_delay = 1.5,
        auton7_intake_roller_speed = -0.7,
        auton7_ball_2_settle_delay = 2.5,
        auton7_uneject_time = 0.5,
        auton7_backup_distance = 6,
        
        
        
        //# collection
        intake_roller_speed = 1,
        
        //#Driving variables
        xaxis_percent = .75,
        
        //# Camera meta-config (meant for dashboard, not robot)
        target_r = 55,
        target_g = 231,
        target_b = 221,
        
        target_r_down_limit = 0,
        target_r_up_limit = 60,
        
        target_g_down_limit = 200,
        target_g_up_limit = 255,
        
        target_b_down_limit = 200,
        target_b_up_limit = 255,
        
        drive_encoderlines = 250,
        drive_max_speed = 5000
        ;
    }

    public static void loadFromFile(String filename) {
        // "backwards compatibility"
        initSettings();
    }

    public static void initSettings() {
        Field[] values = ConfigValues.class.getFields();

        for (Field field : values) {
            try {
                mSettings.put(field.getName(), field.getDouble(ConfigValues.class));
            } catch (IllegalArgumentException | IllegalAccessException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public static void dump() {
        // TODO: I decided this wasn't really needed for now.
    }

    public static double getSetting(String name) {
        return getSetting(name, -0d); // yup, floats can be -0
    }
    
    public static double getSetting(String name, double reasonableDefault) {
        // The only reason I wanted a map lol
        return mSettings.getOrDefault(name, reasonableDefault);
    }
}
