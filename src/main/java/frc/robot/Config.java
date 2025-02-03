package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.generated.CompBotTunerConstants;
import frc.robot.generated.PracticeBotTunerConstants;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.ctre.phoenix6.configs.CANcoderConfiguration;;

public class Config {
    public final RobotConfig generatedConfig;
    public final Properties fileConfig;
    private boolean inPracticeMode;

    public Config(String fileLocation) {
        generatedConfig = readGeneratedConfig();
        fileConfig = readFileConfig(fileLocation);
        inPracticeMode = readBooleanProperty("robot.practice.mode");
    }

    public Config() {
        this(
            new File(Filesystem.getDeployDirectory(), "robot.properties").getPath()
        );
    }

    public Integer readIntegerProperty(String property) {
        return Integer.parseInt(readOrThrow(property));
    }

    public Boolean readBooleanProperty(String property) {
        return Boolean.parseBoolean(readOrThrow(property));
    }

    public Double readDoubleProperty(String property) {
        return Double.parseDouble(readOrThrow(property));
    }

    private String readOrThrow(String property) {
        var rawProperty = fileConfig.getProperty(property);
        if (rawProperty == null) {
            throw new RuntimeException(property + " was not found.");
        }
        return rawProperty;
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

    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftModule() {
        if (inPracticeMode) {
            return PracticeBotTunerConstants.FrontLeft;
        } else {
            return CompBotTunerConstants.FrontLeft;
        }
    }

    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightModule() {
        if (inPracticeMode) {
            return PracticeBotTunerConstants.FrontRight;
        } else {
            return CompBotTunerConstants.FrontRight;
        }

    }

    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftModule() {
        if (inPracticeMode) {
            return PracticeBotTunerConstants.BackLeft;
        } else {
            return CompBotTunerConstants.BackLeft;
        }
    }

    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightModule() {
        if (inPracticeMode) {
            return PracticeBotTunerConstants.BackRight;
        } else {
            return CompBotTunerConstants.BackRight;
        }
    }

    public SwerveDrivetrainConstants drivetrainConstants(){
        if (inPracticeMode) {
            return PracticeBotTunerConstants.DrivetrainConstants;
        } else {
            return CompBotTunerConstants.DrivetrainConstants;
        }
    }

    public LinearVelocity kSpeedAt12Volts(){
        if (inPracticeMode){
            return PracticeBotTunerConstants.kSpeedAt12Volts;
    
        } else {
            return CompBotTunerConstants.kSpeedAt12Volts;
        }
    }
}
