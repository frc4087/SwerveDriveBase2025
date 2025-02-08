package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.CompBotTunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class StrikeAPose extends SubsystemBase {
    private final PIDController headingController;
    public double desiredHeading;
    private final CommandSwerveDrivetrain drivetrain;

    public StrikeAPose(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.headingController = new PIDController(0.015, 0, 0.000); // PID constants from the Chief Delphi Team
        this.headingController.enableContinuousInput(-180, 180); // This was recommended so we don't correct error from the more inefficient side.
        this.desiredHeading = 0.0;
    }

    public void setDesiredHeading(double heading) {
        this.desiredHeading = heading;
    }

    public double getDesiredHeading() {
        return desiredHeading;
    }

    public double calculateCorrection() {
        double currentHeading = drivetrain.getHeadingDegrees();
        return headingController.calculate(currentHeading, desiredHeading);
    }

    public Command snapTo(Double degrees, Boolean isFieldRelative) {

    }

    public void maintainHeading(double x, double y, boolean fieldRelative) {
        double correction = calculateCorrection();
        double ffRotation = Math.signum(correction) * CompBotTunerConstants.ROTATE_TO_TARGET_FF;
        double desiredRotation = correction - ffRotation;

        if (Math.abs(desiredRotation) < CompBotTunerConstants.ROTATION_DEADBAND_THRESHOLD) {
            desiredRotation = 0;
        }

        drivetrain.drive(x, y, desiredRotation);
    }

    public int convertCardinalDirections(int povAngleDeg) {
        if (povAngleDeg == 270) {
            povAngleDeg += 77;
        } else if (povAngleDeg == 90) {
            povAngleDeg -= 77;
        }
        return 360 - povAngleDeg;
    }

    public void rotateOrMaintainHeading(double x, double y, double rot, boolean fieldRelative, int povAngleDeg) {
        if (povAngleDeg != -1) {
            setDesiredHeading(convertCardinalDirections(povAngleDeg));
            maintainHeading(x, y, fieldRelative);
        } else if (rot == 0) {
            maintainHeading(x, y, fieldRelative);
        } else {
            setDesiredHeading(drivetrain.getHeadingDegrees());
            drivetrain.drive(x, y, rot);
        }
    }
}