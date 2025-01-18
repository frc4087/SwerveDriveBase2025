package frc.robot;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;

public class Config {
    public final RobotConfig generatedConfig;
    public final Properties fileConfig;

    public Config(String fileLocation) {
        generatedConfig = readGeneratedConfig();
        fileConfig = readFileConfig(fileLocation);
    }

    public Config() {
        this("./src/main/resources/robot.properties");
    }

    public Integer readIntegerProperty(String property) {
        var rawProperty = fileConfig.getProperty(property);
        if (rawProperty == null) {
            throw new RuntimeException(property + " was not found.");
        }
        return Integer.parseInt(rawProperty);
    }

    protected static RobotConfig readGeneratedConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new Error("Unable to load Robot Config: %s".formatted(e.getMessage()));
        }
    }

    public static Properties readFileConfig(String path) {
        var properties = new Properties();
        try {
            var reader = new FileReader(path);
            properties.load(reader);
            return properties;
        } catch (FileNotFoundException e) {
            throw new Error("Unable to find property file: %s".formatted(e.getMessage()));
        } catch (IOException e) {
            throw new Error("IOException when reading property file: %s".formatted(e.getMessage()));
        }
    } 
}
