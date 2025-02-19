package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Commands the robot to drive from its current pose to a new pose at a given
 * speed. The nature of the command does not lend itself to chaining, as in a
 * path. Use PathPlanner instead. As such, zeros the speed at the beginning and
 * end of the command, but does not reset pose. As needed, use separate commands
 * before or after this call to reset the pose (e.g. ZeroPose).
 * <p>
 * Makes generous assumptions about error tolerance and PID tuning.
 * <p>
 * TODO: Delegate profile and controller building to the drive since it best
 * knows its needs and constraints.
 */
public class DriveToPoseCommand extends Command {
    /**
     * Creates an instance.
     * 
     * @param drivetrain  The target drivetrain.
     * @param speedFactor Speed factor relative to max [0, +1]. Sign is ignored.
     * @param pose        New pose.
     */
    public DriveToPoseCommand(CommandSwerveDrivetrain drive, Pose2d pose,
            double speedFactor) {
        _drive = drive;
        addRequirements(drive);

        _poseEnd = pose;

        SwerveDriveSpecs specs = new SwerveDriveSpecs();
        _speeds = new ChassisSpeeds(specs.kMaxLinearVelMps, specs.kMaxLinearVelMps, specs.kMaxAngularVelRps)
                .times(speedFactor);

        // build controller
        // TODO: Consider using profiles for X and Y
        Pose2d tolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(3.0));
        PIDController pidX = new PIDController(1.0, 0.0, 0.0);
        PIDController pidY = new PIDController(1.0, 0.0, 0.0);
        ProfiledPIDController pidR = new ProfiledPIDController(2.0, 0.0,
                0.0, new TrapezoidProfile.Constraints(specs.kMaxAngularVelRps, specs.kMaxAngularAccRpss));
        pidR.enableContinuousInput(-Math.PI, +Math.PI);

        _holoPid = new HolonomicDriveController(pidX, pidY, pidR);
        _holoPid.setTolerance(tolerance);

        reportInit();
    }

    @Override
    public void initialize() {
        _drive.setControl(new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }

    @Override
    public void execute() {
        Pose2d poseNow = _drive.getPose();
        ChassisSpeeds speeds = _holoPid.calculate(poseNow, _poseEnd,
                _speeds.vxMetersPerSecond, _poseEnd.getRotation());
        reportExecute(speeds);
        _drive.setControl(new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(speeds)
                .withDesaturateWheelSpeeds(true));
    }

    @Override
    public void end(boolean interrupted) {
        _drive.setControl(new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }

    @Override
    public boolean isFinished() {
        boolean isDone = _holoPid.atReference();
        if (isDone)
            reportDone();
        return isDone;
    }

    // personal

    private void reportInit() {
        Pose2d poseNow = _drive.getPose();

        System.out.printf("DriveToPoseCommand: poseEnd[%6.2f %6.2f %6.1f], poseNow[%6.2f %6.2f %6.1f]\n",
                _poseEnd.getX(), _poseEnd.getY(), _poseEnd.getRotation().getDegrees(),
                poseNow.getX(), poseNow.getY(), poseNow.getRotation().getDegrees());
    }

    private void reportExecute(ChassisSpeeds speeds) {
        Pose2d poseNow = _drive.getPose();
        Pose2d poseErr = _poseEnd.relativeTo(poseNow);

        System.out.printf("DriveToPoseCommand: poseErr[%6.2f %6.2f %6.1f], poseSpd[%6.2f %6.2f %6.1f]\n",
                poseErr.getX(), poseErr.getY(), poseErr.getRotation().getDegrees(),
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, Math.toDegrees(speeds.omegaRadiansPerSecond));
    }

    private void reportDone() {
        Pose2d poseNow = _drive.getPose();
        Pose2d poseErr = _poseEnd.relativeTo(poseNow);

        System.out.printf("DriveToPoseCommand: poseEnd[%6.2f %6.2f %6.1f], poseErr[%6.2f %6.2f %6.1f]\n",
                _poseEnd.getX(), _poseEnd.getY(), _poseEnd.getRotation().getDegrees(),
                poseErr.getX(), poseErr.getY(), poseErr.getRotation().getDegrees());
    }

    private final CommandSwerveDrivetrain _drive;
    private final Pose2d _poseEnd;
    private final ChassisSpeeds _speeds;
    private HolonomicDriveController _holoPid;

    // class

    public class SwerveDriveSpecs {
        public final double kTrackWidthM = Units.inchesToMeters(24); // 24-inch width???
        public final double kWheelBaseM = Units.inchesToMeters(24); // 24-inch length???
        public final double kTotalMassK = Units.lbsToKilograms(20); // 20 lbs???

        public final double kMaxLinearVelMps = 4.5; // Max speed in m/s
        public final double kMaxLinearAccMpss = 2.0; // Max acceleration in m/s²

        public final double kMaxAngularVelRps = getMaxAngularVel();
        public final double kMaxAngularAccRpss = getMaxAngularAcc();

        // class personal

        private double getChassisRadius() {
            return Math.hypot(kWheelBaseM / 2.0, kTrackWidthM / 2.0);
        }

        private double getMaxAngularVel() {
            return kMaxLinearVelMps / getChassisRadius();
        }

        private double getMaxAngularAcc() {
            // assume mass is equally distributed
            double momentOfInertia = (1.0 / 12.0) * kTotalMassK
                    * (Math.pow(kTrackWidthM, 2) + Math.pow(kWheelBaseM, 2));

            // compute total torque from all 4 modules
            double torque = 4 * ((kTotalMassK / 4) * kMaxLinearAccMpss * getChassisRadius());

            // compute maximum angular acceleration
            return torque / momentOfInertia;
        }
    }
}
