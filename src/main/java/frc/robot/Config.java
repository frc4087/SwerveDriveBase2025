package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Config.TunerConstants;
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

    //doing photonvis allignment stuff
    static class Constants{
        public static class Vision{
        public static final String kCameraName = "photonvision";
        //cam mounted facing fwd, half a meter fwd of center, half a meter up from ctr, pitched upwds
        private static final double camPitch = Units.degreesToRadians(30.0);
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -camPitch, 0));
                
                // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public Vision(){}
        }
            public static class Swerve {
        // Physical properties
        public static final double kTrackWidth = Units.inchesToMeters(18.5);
        public static final double kTrackLength = Units.inchesToMeters(18.5);
        public static final double kRobotWidth = Units.inchesToMeters(25 + 3.25 * 2);
        public static final double kRobotLength = Units.inchesToMeters(25 + 3.25 * 2);
        public static final double kMaxLinearSpeed = Units.feetToMeters(15.5);
        public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        public static final double kDriveGearRatio = 6.75; // 6.75:1 SDS MK4 L2 ratio
        public static final double kSteerGearRatio = 12.8; // 12.8:1

        public static final double kDriveDistPerPulse = kWheelCircumference / 1024 / kDriveGearRatio;
        public static final double kSteerRadPerPulse = 2 * Math.PI / 1024;
public enum ModuleConstants {
            FL( // Front left
                    1, 0, 0, 1, 1, 2, 3, 0, kTrackLength / 2, kTrackWidth / 2),
            FR( // Front Right
                    2, 2, 4, 5, 3, 6, 7, 0, kTrackLength / 2, -kTrackWidth / 2),
            BL( // Back Left
                    3, 4, 8, 9, 5, 10, 11, 0, -kTrackLength / 2, kTrackWidth / 2),
            BR( // Back Right
                    4, 6, 12, 13, 7, 14, 15, 0, -kTrackLength / 2, -kTrackWidth / 2);

            public final int moduleNum;
            public final int driveMotorID;
            public final int driveEncoderA;
            public final int driveEncoderB;
            public final int steerMotorID;
            public final int steerEncoderA;
            public final int steerEncoderB;
            public final double angleOffset;
            public final Translation2d centerOffset;

            private ModuleConstants(
                    int moduleNum,
                    int driveMotorID,
                    int driveEncoderA,
                    int driveEncoderB,
                    int steerMotorID,
                    int steerEncoderA,
                    int steerEncoderB,
                    double angleOffset,
                    double xOffset,
                    double yOffset) {
                this.moduleNum = moduleNum;
                this.driveMotorID = driveMotorID;
                this.driveEncoderA = driveEncoderA;
                this.driveEncoderB = driveEncoderB;
                this.steerMotorID = steerMotorID;
                this.steerEncoderA = steerEncoderA;
                this.steerEncoderB = steerEncoderB;
                this.angleOffset = angleOffset;
                centerOffset = new Translation2d(xOffset, yOffset);
            }
            // Feedforward
        // Linear drive feed forward
        public static final SimpleMotorFeedforward kDriveFF =
                new SimpleMotorFeedforward( // real
                        0.25, // Voltage to break static friction
                        2.5, // Volts per meter per second
                        0.3 // Volts per meter per second squared
                        );
        // Steer feed forward
        public static final SimpleMotorFeedforward kSteerFF =
                new SimpleMotorFeedforward( // real
                        0.5, // Voltage to break static friction
                        0.25, // Volts per radian per second
                        0.01 // Volts per radian per second squared
                        );

        // PID
        public static final double kDriveKP = 1;
        public static final double kDriveKI = 0;
        public static final double kDriveKD = 0;

        public static final double kSteerKP = 20;
        public static final double kSteerKI = 0;
        public static final double kSteerKD = 0.25;
    }
            }
                public Constants(){}
                //missing?

    }

    static class TunerConstants {
        private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftModule;
        private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightModule;
        private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftModule;
        private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightModule;
        private SwerveDrivetrainConstants drivetrainConstants;
        private LinearVelocity kSpeedAt12Volts;
    
        public TunerConstants() {}
    
        public TunerConstants withFrontLeftModule(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftModule) {
            this.frontLeftModule = frontLeftModule;
            return this;
        }
    
        public TunerConstants withFrontRightModule(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightModule) {
            this.frontRightModule = frontRightModule;
            return this;
        }
    
        public TunerConstants withBackLeftModule(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftModule) {
            this.backLeftModule = backLeftModule;
            return this;
        }
    
        public TunerConstants withBackRightModule(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightModule) {
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
