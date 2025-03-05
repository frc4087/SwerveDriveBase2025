package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
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
    public boolean inPracticeMode;
    public TunerConstants TunerConstants;

    public Config(String fileLocation) {
        generatedConfig = readGeneratedConfig();
        fileConfig = readFileConfig(fileLocation);
        inPracticeMode = readBooleanProperty("robot.practice.mode");
        if (inPracticeMode) {
            this.TunerConstants = new TunerConstants()
                    .withFrontLeftModule(PracticeBotTunerConstants.FrontLeft)
                    .withFrontRightModule(PracticeBotTunerConstants.FrontRight)
                    .withBackLeftModule(PracticeBotTunerConstants.BackLeft)
                    .withBackRightModule(PracticeBotTunerConstants.BackRight)
                    .withDrivetrainConstants(PracticeBotTunerConstants.DrivetrainConstants)
                    .withKSpeedAt12Volts(PracticeBotTunerConstants.kSpeedAt12Volts);
        } else {
            this.TunerConstants = new TunerConstants()
                    .withFrontLeftModule(CompBotTunerConstants.FrontLeft)
                    .withFrontRightModule(CompBotTunerConstants.FrontRight)
                    .withBackLeftModule(CompBotTunerConstants.BackLeft)
                    .withBackRightModule(CompBotTunerConstants.BackRight)
                    .withDrivetrainConstants(CompBotTunerConstants.DrivetrainConstants)
                    .withKSpeedAt12Volts(CompBotTunerConstants.kSpeedAt12Volts);
        }
    }

    public Config() {
        this(
                new File(Filesystem.getDeployDirectory(), "robot.properties").getPath());
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

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    // Snap to Heading Constants

    public static final double ROTATE_TO_TARGET_FF = 0.01;

    public static final double ROTATION_DEADBAND_THRESHOLD = 0.04;

    // Snap to Heading Constants

    static class TunerConstants {
        private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftModule;
        private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightModule;
        private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftModule;
        private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightModule;
        private SwerveDrivetrainConstants drivetrainConstants;
        private LinearVelocity kSpeedAt12Volts;

        public TunerConstants() {
        }

        public TunerConstants withFrontLeftModule(
                SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftModule) {
            this.frontLeftModule = frontLeftModule;
            return this;
        }

        public TunerConstants withFrontRightModule(
                SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightModule) {
            this.frontRightModule = frontRightModule;
            return this;
        }

        public TunerConstants withBackLeftModule(
                SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftModule) {
            this.backLeftModule = backLeftModule;
            return this;
        }

        public TunerConstants withBackRightModule(
                SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightModule) {
            this.backRightModule = backRightModule;
            return this;
        }

        public TunerConstants withDrivetrainConstants(SwerveDrivetrainConstants drivetrainConstants) {
            this.drivetrainConstants = drivetrainConstants;
            return this;
        }

        public TunerConstants withKSpeedAt12Volts(LinearVelocity kSpeedAt12Volts) {
            this.kSpeedAt12Volts = kSpeedAt12Volts;
            return this;
        }

        public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontLeftModule() {
            if (frontLeftModule == null) {
                throw new IllegalStateException("FrontLeft module was not provided");
            }
            return frontLeftModule;
        }

        public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontRightModule() {
            if (frontRightModule == null) {
                throw new IllegalStateException("FrontRight module was not provided");
            }
            return frontRightModule;
        }

        public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackLeftModule() {
            if (backLeftModule == null) {
                throw new IllegalStateException("BackLeft module was not provided");
            }
            return backLeftModule;
        }

        public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackRightModule() {
            if (backRightModule == null) {
                throw new IllegalStateException("BackRight module was not provided");
            }
            return backRightModule;
        }

        public SwerveDrivetrainConstants getDrivetrainConstants() {
            if (drivetrainConstants == null) {
                throw new IllegalStateException("Drivetrain constants were not provided");
            }
            return drivetrainConstants;
        }

        public LinearVelocity getKSpeedAt12Volts() {
            if (kSpeedAt12Volts == null) {
                throw new IllegalStateException("K speed at 12 volts was not provided");
            }
            return kSpeedAt12Volts;
        }
    }
}
