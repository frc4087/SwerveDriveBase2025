package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.CompBotTunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class StrikeAPose extends SubsystemBase {
    private final PIDController headingController;
    private final CommandSwerveDrivetrain drivetrain;
    public double desiredHeading;

    public StrikeAPose(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.headingController = new PIDController(1.0, 0.0, 0.0); // PID constants from the Chief Delphi Team
        this.headingController.enableContinuousInput(-Math.PI, Math.PI); // This was recommended so we don't correct error from the more inefficient side.
    }

    public Command snappy(Double inputTarget, Boolean isFieldRelative) {
        var targetRads = determineTargetRadFromRelative(inputTarget);
        System.out.println(String.format("Target: %s", targetRads));
        return new Command() {
            @Override
            public void execute() {
                System.out.println("RUNNING");
                double current = drivetrain.getState().Pose.getRotation().getRadians();
                double next = headingController.calculate(current, targetRads);
                drivetrain.drive(0, 0, next);
            }

            @Override
            public boolean isFinished() {
                // TODO: Figure out why delta is bigger than expected
                var delta = Math.abs(targetRads - drivetrain.getState().Pose.getRotation().getRadians());
                System.out.println(String.format("Delta: %s", delta));
                return delta < 1;
            }
        };
    }

    // TODO: Validate this method
    // We need this returned value to be in -pi, pi
    private Double determineTargetRadFromRelative(Double input) {
        var targetRads = drivetrain.getState().Pose.getRotation().getRadians() + input;
        return MathUtil.angleModulus(targetRads);
    }
}