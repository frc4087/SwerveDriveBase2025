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
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        var degreeTolerance = config.readDoubleProperty("drivetrain.rotationalTolerance.degrees");
        this.rotationalTolerance = Radians.convertFrom(degreeTolerance, Degree);
        this.headingController.setTolerance(this.rotationalTolerance);
    }

    public RotateBotCommand withFieldRelativeAngle(double targetDegrees) {
        this.targetRads = MathUtil.angleModulus(Radians.convertFrom(targetDegrees, Degree));
        return this;
    }

    @Override
    public void execute() {
        double ccwRad = drivetrain.getRotationRads();
        double next = this.headingController.calculate(ccwRad, targetRads);

        drivetrain.spinWithSpeedRad(next);
    }

    @Override
    public boolean isFinished() {
        return this.headingController.atSetpoint();
    }
}