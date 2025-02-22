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

    public RotateBotCommand(CommandSwerveDrivetrain drivetrain, Config config) {
        super();
        this.addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        
        this.headingController = new PIDController(5, 0, 0); 
        headingController.enableContinuousInput(-Math.PI, Math.PI);
       
        var degreeTolerance = config.readDoubleProperty("drivetrain.rotationalTolerance.degrees");
        this.rotationalTolerance = Radians.convertFrom(degreeTolerance, Degree);
    }

    public RotateBotCommand withRobotRelativeCurrentRads(double targetRads) {
        this.targetRads = MathUtil.angleModulus(targetRads + drivetrain.getRotationRads());
        return this;
    }
    public RotateBotCommand withRobotRelativeStartRads(double targetRads) {
        this.targetRads = MathUtil.angleModulus(targetRads);
        return this;
    }

    @Override
    public void initialize() {
        System.out.println(
            String.format("Current: %s; Target: %s;",
            drivetrain.getRotationRads(),
            targetRads
        ));
    }

    @Override
    public void execute() {
        System.out.println(String.format("Current: %s", drivetrain.getRotationRads()));
        double next = headingController.calculate(drivetrain.getRotationRads(), targetRads);
        drivetrain.spinWithSpeedRad(next);
    }

    @Override
    public boolean isFinished() {
        var delta = Math.abs(targetRads - drivetrain.getRotationRads());
        System.out.println(String.format("Stopped? %s", delta < rotationalTolerance));
        return delta < rotationalTolerance;
    }
}