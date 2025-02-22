package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateBotCommand extends Command {
    private final PIDController headingController;
    private CommandSwerveDrivetrain drivetrain;
    private double targetRads = 0;
    private double rotationalTolerance;

    public RotateBotCommand(CommandSwerveDrivetrain drivetrain,Config config) {
        super();
        this.addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.headingController = new PIDController(5, 0, 0); 
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationalTolerance = config.readDoubleProperty("RotateBotCommand.motor.rotate.angle.");
    }

    public RotateBotCommand withRobotRelativeCurrentRads(double targetRads) {
        this.targetRads = targetRads + drivetrain.getState().Pose.getRotation().getRadians();
        return this;
    }
    public RotateBotCommand withRobotRelativeStartRads(double targetRads) {
        this.targetRads = targetRads + drivetrain.getState().Pose.getRotation().getRadians();
        return this;
    }

    @Override
    public void execute() {
        double current = drivetrain.getState().Pose.getRotation().getRadians();
        double next = headingController.calculate(current, targetRads);
        // drivetrain.drive(0, 0, next);
    }

    @Override
    public boolean isFinished() {
        var delta = Math.abs(targetRads - drivetrain.getState().Pose.getRotation().getRadians());
        return delta < rotationalTolerance;
    }
}
