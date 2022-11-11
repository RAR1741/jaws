import frc.robot.Config;

import org.junit.Test;
import org.junit.BeforeClass;

import static org.junit.Assert.assertEquals;

public class ConfigTest {
    private static final double CLOSE_ENOUGH = 0.01;

    @BeforeClass
    public static void configSetup() {
        Config.loadFromFile();
    }

    @Test
    public void test() {
        assertEquals(1, 1);
    }

    @Test
    public void configGetSetting() {
        //assertEquals(1, Config.getSetting("auto_delay"), CLOSE_ENOUGH);
    }

    @Test
    public void configGetDefault() {
        assertEquals(3600, Config.getSetting("auto_crash", 3600), CLOSE_ENOUGH);
    }

    @Test
    public void configGetUnknownSetting() {
        assertEquals(-0d, Config.getSetting("UNKNOWN_SETTING"), CLOSE_ENOUGH);
    }

    @Test
    public void configLoadFromFile() {
        // This doesn't actually do anything
        // other than the default init right now
        Config.loadFromFile();

        //assertEquals(35, Config.getSetting("funnel_release_angle"), CLOSE_ENOUGH);
    }
}
