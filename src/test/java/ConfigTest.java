import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.Config;

public class ConfigTest {
    private static final double CLOSE_ENOUGH = 0.01;

    @BeforeAll
    public static void configSetup() {
        Config.loadFromFile();
    }

    @Test
    public void configGetSetting() {
        assertEquals(Math.PI, Config.getSetting("test.value1", 0), CLOSE_ENOUGH);
    }

    @Test
    public void configGetSettingWithMissingSetting() {
        assertEquals(30.0, Config.getSetting("test.value_does_not_exist", 30.0), CLOSE_ENOUGH);
    }

    @Test
    public void configLoadFromFile() {
        // This doesn't actually do anything
        // other than the default init right now
        Config.loadFromFile();

        //assertEquals(35, Config.getSetting("funnel_release_angle"), CLOSE_ENOUGH);
    }
}
