import org.junit.jupiter.api.Test;

import frc.robot.Config;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class ConfigTests {
 
    private static final String testPath = "./src/test/resources/test.properties";

    @Test
    void canLoadFromRoot() {
        Config.readFileConfig(testPath);
    }

    @Test
    void throwsErrorWhenNotFound() {
        assertThrows(Error.class, () -> {
            Config.readFileConfig("./src/test/resources/not-found.properties");
        });
    }

    @Test
    void readProperty() {
        var cfg = Config.readFileConfig(testPath);
        var testAmps = cfg.getProperty("example.motor.amps");
        assertEquals("40", testAmps);
    }
}
