package frc.robot;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertThrows;

import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ConfigTests {
 
    public static final String testConfigPath = Paths.get("src","test","resources", "test.properties").toString();

    @Test
    void canLoadFromRoot() {
        Config.readFileConfig(testConfigPath);
    }

    @Test
    void throwsErrorWhenNotFound() {
        assertThrows(Error.class, () -> {
            Config.readFileConfig("./resources/not-found.properties");
        });
    }

    @Test
    void readProperty() {
        var cfg = Config.readFileConfig(testConfigPath);
        var testAmps = cfg.getProperty("example.motor.amps");
        assertEquals("40", testAmps);
    }
}
