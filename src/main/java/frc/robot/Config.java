package frc.robot;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

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
        // BEGIN: config.txt circa Oct. 2022
        // im sorry for continuing to use this as a config file
        public static double
                cam_p = 0.08,
                cam_i = 0.00005,
                cam_d = 0.8,

                robot_loop_period = .02,

                cam_loop_period = 0.004,
                robot_cam_rearm_rate = 25,

                cam_ready_to_fire_position = 54,
                cam_fire_to_position = 63,
                cam_point_of_no_return = 58,

                cam_fire_position_tolerance = 3,
                cam_setpoint_error_limit = 25,
                cam_home_speed = .6,
                cam_home_speed_slow = .25,
                cam_home_speed_cutoff = 50,
                cam_eject_position = 21,

                auto_firing_distance = 96,
                auto_collection_delay = 1.0,
                odometer_auto_speed = -.1,

                intake_roller_speed = 1,

                xaxis_percent = .75,

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
                drive_max_speed = 5000;
        // END: config.txt
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
                System.out.println(Config.getSetting(field.getName()));
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
