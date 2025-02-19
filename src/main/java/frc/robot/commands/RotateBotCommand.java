package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateBotCommand extends Command {
    private final PIDController headingController;
    private CommandSwerveDrivetrain drivetrain;
    private double targetRads = 0;
    private double rotationalTolerance;
    private String turnType = "(undefined)";

    public RotateBotCommand(CommandSwerveDrivetrain drivetrain, Config config) {
        super();
        this.addRequirements(drivetrain);
        this.drivetrain = drivetrain;

        this.headingController = new PIDController(5, 0, 0);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        var degreeTolerance = config.readDoubleProperty("drivetrain.rotationalTolerance.degrees");
        this.rotationalTolerance = Radians.convertFrom(degreeTolerance, Degree);
        this.headingController.setTolerance(this.rotationalTolerance);
    }

    public RotateBotCommand withRobotRelativeCurrentRads(double targetRads) {
        this.turnType = "robot relative";
        this.targetRads = MathUtil.angleModulus(targetRads + drivetrain.getRotationRads());
        return this;
    }

    public RotateBotCommand withRobotRelativeStartRads(double targetRads) {
        this.turnType = "field relative";
        this.targetRads = MathUtil.angleModulus(targetRads);
        return this;
    }

    @Override
    public void initialize() {
        System.out.println(
                String.format("RotateBotCommand: type[%s], now[%6.2f], target[%6.2f]",
                        this.turnType, Math.toDegrees(drivetrain.getRotationRads()),
                        Math.toDegrees(targetRads)));
    }

    @Override
    public void execute() {
        double ccwRad = drivetrain.getRotationRads();
        double next = this.headingController.calculate(ccwRad, targetRads);

        // System.out.println(String.format("    now[%6.2f] error[%6.2f]",
        //         Math.toDegrees(ccwRad), Math.toDegrees(this.headingController.getError())));

        drivetrain.spinWithSpeedRad(next);
    }

    @Override
    public boolean isFinished() {
        boolean isDone = this.headingController.atSetpoint();
        if (isDone) {
            double ccwRad = drivetrain.getRotationRads();
            System.out.println(String.format("    Done!!! now[%6.2f] error[%6.2f]",
                    Math.toDegrees(ccwRad), Math.toDegrees(this.headingController.getError())));
        }
        return isDone;

        // var delta = Math.abs(targetRads - drivetrain.getRotationRads());
        // System.out.println(String.format("Stopped? %s", delta <
        // rotationalTolerance));
        // return delta < rotationalTolerance;
    }
}
