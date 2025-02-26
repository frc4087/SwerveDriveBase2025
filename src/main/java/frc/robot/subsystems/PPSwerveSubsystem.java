package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Wraps a target drive system to provide PPDrivable support.
 */
public class PPSwerveSubsystem extends SubsystemBase implements PPDrivable {
    public PPSwerveSubsystem(DriveSpecs specs, CommandSwerveDrivetrain drive) {
        _specs = specs;
        _drive = drive;
        _subsystems.add(_drive);
        _subsystems.add(this);

        _speedsMax = new ChassisSpeeds(_specs.kMaxLinearVelMps(),
                _specs.kMaxLinearVelMps(),
                _specs.kMaxAngularVelRps());

        // connect PP to drivetrain
        /// resolve PP robot config
        double moi = DriveSpecs.getMomentOfInertia(
                _specs.kTotalMassK(),
                _specs.kWheelBaseM(),
                _specs.kTrackWidthM()); // est
        double wheelCof = 0.9; // guess
        double gearRatio = 6.75; // Kraken X60, L2 ratio
        ModuleConfig moduleConfig = new ModuleConfig(Units.inchesToMeters(2.0),
                _specs.kMaxLinearVelMps(), wheelCof, DCMotor.getKrakenX60(1), gearRatio, 60.0, 1);

        RobotConfig robotConfig = new RobotConfig(
                _specs.kTotalMassK(), moi, moduleConfig, _drive.getModuleLocations());

        /// build path controller
        PIDConstants pidXY = new PIDConstants(5.0, 0.0, 0.2);
        PIDConstants pidR = new PIDConstants(5.0, 0.0, 0.2);
        PathFollowingController controller = new PPHolonomicDriveController(pidXY, pidR);

        /// config PP command factory
        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getTrueSpeeds,
                (speeds, feedforwards) -> this.setDriveSpeeds(speeds),
                controller,
                robotConfig,
                PPSwerveSubsystem::isPathFlipped,
                this.getSubsystems().toArray(new Subsystem[0]));
    }

    public DriveSpecs getSpecs() {
        return _specs;
    }

    // PPDrivable

    @Override
    public ChassisSpeeds getTrueSpeeds() {
        return _drive.getRobotRelativeChassisSpeeds();
    }

    // CommandDrivable

    @Override
    public void setDriveSpeeds(ChassisSpeeds speeds) {
        _drive.setControl(new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(speeds));
    }

    @Override
    public ChassisSpeeds getMaxSpeeds() {
        return _speedsMax;
    }

    @Override
    public void stop() {
        _drive.setControl(new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }

    @Override
    public boolean isHolonomic() {
        return true;
    }

    @Override
    public void resetPose(Pose2d pose) {
        _drive.resetPose(pose);
    }

    @Override
    public Pose2d getPose() {
        return _drive.getPose();
    }

    @Override
    public List<Subsystem> getSubsystems() {
        return Collections.unmodifiableList(_subsystems);
    }

    // personal

    private final DriveSpecs _specs;
    private final CommandSwerveDrivetrain _drive;
    private final List<Subsystem> _subsystems = new ArrayList<>();
    private final ChassisSpeeds _speedsMax;

    // class

    /**
     * Returns true if the path being followed should be mirrored (i.e. for the
     * red alliance)
     * 
     * @return The state.
     */
    public static boolean isPathFlipped() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public static DriveSpecs PRACTICE_SPECS = newPracticeSpecs();

    // class, personal

    private static DriveSpecs newPracticeSpecs() {
        double kTrackWidthM = Units.inchesToMeters(22.75);
        double kWheelBaseM = Units.inchesToMeters(22.75);
        double kTotalMassK = Units.lbsToKilograms(80); // guess
        double radius = DriveSpecs.getChassisRadius(kTrackWidthM, kWheelBaseM);
        double kMaxLinearVelMps = 0.85 * Units.feetToMeters(15.5); // est
        double kMaxLinearAccMpss = 2.0; // guess

        return new DriveSpecs(kTrackWidthM, kWheelBaseM, kTotalMassK, Units.inchesToMeters(18.0),
                Units.inchesToMeters(18.0), Units.inchesToMeters(18.0), Units.inchesToMeters(18.0),
                kMaxLinearVelMps, kMaxLinearAccMpss, DriveSpecs.getMaxAngularVel(kMaxLinearVelMps, radius),
                DriveSpecs.getMaxAngularAcc(true, kTotalMassK, kWheelBaseM, kTrackWidthM, kMaxLinearAccMpss));
    }
}