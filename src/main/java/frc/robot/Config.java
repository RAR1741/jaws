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
        // BEGIN: config.txt circa Nov 26, 2013
        public static double
                //#drive section
                //drive_kp = 0.000130, //original 4-16-13
                drive_kp = 0.000505,
                //drive_ki = 0.0000080, //original 4-16-13
                drive_ki = 0.0000330,
                //drive_kd = 0.000300, //original 4-16-13
                drive_kd = 0.000405, drive_encoderlines = 250, drive_max_speed = 4500,
                //    1=voltage, 2=speed
                control_mode = 2, drive_ratelimit_perTick = 900, drive_speed_modify = 0.5,
                //#end drive section

                //#shooter section
                angle_power = .75, angle_power_slow = 0.25, loader_power = -0.75,
                //shooter_kp = -0.25 orginal 4-20-13
                shooter_kp = -1,
                //shooter_ki = -0.005 original 4-20-13
                shooter_ki = -0.02, shooter_kd = -0.0001,

                rear_kp = -0.25, rear_ki = -0.005, rear_kd = -0.0001,

                lever_kp = 20, lever_ki = 0, lever_kd = 0,

                shooter_homing_turnaround_time = 2, shooter_homing_up_speed = 1, shooter_homing_down_speed = -1,
                shooter_angle_tolerance = 10,

                home_position_shaft_length = 9.5, crouch_angle = 10,

                wheel_deadzone = .03,

                //#### WHISBEE
                pyramid_shooting_angle = 22.5, shooter_loading_angle = 10, load_zone_shooting_angle = 14.25,
                pyramid_basket_angle = 30,
                //####

                //### RIZZLER
                //pyramid_shooting_angle = 26.5,
                //shooter_loading_angle = 10,
                //load_zone_shooting_angle = 12,
                //pyramid_basket_angle = 35,
                //###

                //#RPM
                //shooter_main_speed = 4500, //original 4-20-13
                shooter_main_speed = 4000,
                //shooter_secondary_speed = 3000, //original 4-20-13
                shooter_secondary_speed = 2000, shooter_main_speed_pyramid = 2300,
                shooter_secondary_speed_pyramid = 2000, shooter_angle_bias = 0,

                //#Percentage
                shooter_front_wheel_manual = 1.0, shooter_rear_wheel_manual = 0.7,

                //#end shooter section

                target_camera_fov = 57, target_rotate_scale = .5, target_rotate_tolerance = 1,

                //#Autonomous section
                autoProgram = 1, auto_spinup_delay = 2.0,
                //auto_Front_Wheel_Speed = 4500, //original 4-20-13
                //auto_Rear_Wheel_Speed = 3000, //original 4-20-13
                auto_Front_Wheel_Speed = 4000, auto_Rear_Wheel_Speed = 2500, auto_Shooter_Tolerance = 200,
                auto_delay = 1, auto_angle = 22.5,
                //auto_angle = 22.75,
                //#End autonomous section

                //#Funnel Control
                funnel_release_angle = 35
                //#End funnel control
        ; // END: config.txt
        // Close enough lol...
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
