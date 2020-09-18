import frc.robot.Config;

import static org.junit.Assert.assertEquals;

import org.junit.BeforeClass;

public class ConfigTest {
    private static double CLOSE_ENOUGH = 0.01;

    @BeforeClass
    public static void configSetup() {
        Config.initSettings();
    }

    @org.junit.Test
    public void configGetSetting() {
        assertEquals(Config.getSetting("auto_delay"), 1, CLOSE_ENOUGH);
    }

    @org.junit.Test
    public void configGetDefault() {
        assertEquals(Config.getSetting("auto_crash", 3600), 3600, CLOSE_ENOUGH);
    }

    @org.junit.Test
    public void configGetUnknownSetting() {
        assertEquals(Config.getSetting("UNKNOWN_SETTING"), -0d, CLOSE_ENOUGH);
    }
}
