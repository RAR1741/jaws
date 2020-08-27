import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;

import frc.robot.Config;

import static org.junit.Assert.assertEquals;

public class ConfigTest {
    private static double CLOSE_ENOUGH = 0.01;
    @BeforeAll
    public configSetup() {
        Config.initSettings();
    }

    @Test
    public void configGetSetting() {
        assertEquals(Config.getSetting("auto_delay"), 1, CLOSE_ENOUGH);
    }

    @Test
    public void configGetDefault() {
        assertEquals(Config.getSetting("auto_crash", 3600), 3600, CLOSE_ENOUGH);
    }

    @Test
    public void configGetUnknownSetting() {
        assertEquals(Config.getSetting("UNKNOWN_SETTING"), -0d, CLOSE_ENOUGH);
    }
    
}
