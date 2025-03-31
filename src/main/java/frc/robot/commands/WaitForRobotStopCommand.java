package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class WaitForRobotStopCommand extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private final PIDController translationVelocityController;
    private final PIDController rotationVelocityController;

    public WaitForRobotStopCommand(CommandSwerveDrivetrain drivetrain, Config config) {
        super();
        this.addRequirements(drivetrain);
        this.drivetrain = drivetrain;

        translationVelocityController = new PIDController(1, 0, 0);
        translationVelocityController.setTolerance(0.1);

        rotationVelocityController = new PIDController(1, 0, 0);
        rotationVelocityController.setTolerance(0.1);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = drivetrain.getRobotRelativeChassisSpeeds();
        var translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        this.translationVelocityController.calculate(translationSpeed, 0);
        this.rotationVelocityController.calculate(speeds.omegaRadiansPerSecond, 0);
    }


    @Override
    public boolean isFinished() {
        return this.translationVelocityController.atSetpoint() && rotationVelocityController.atSetpoint();
    }
}
