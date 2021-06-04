import frc.robot.Config;

import org.junit.Test;
import org.junit.BeforeClass;

import static org.junit.Assert.assertEquals;

public class ConfigTest {
    private static double CLOSE_ENOUGH = 0.01;
    @BeforeClass
    public static void configSetup() {
        Config.initSettings();
    }

    @Test
    public void configGetSetting() {
        assertEquals(1, Config.getSetting("auto_delay"), CLOSE_ENOUGH);
    }

    @Test
    public void configGetDefault() {
        assertEquals(3600, Config.getSetting("auto_crash", 3600), CLOSE_ENOUGH);
    }

    @Test
    public void configGetUnknownSetting() {
        assertEquals(-0d, Config.getSetting("UNKNOWN_SETTING"), CLOSE_ENOUGH);
    }
    
}
