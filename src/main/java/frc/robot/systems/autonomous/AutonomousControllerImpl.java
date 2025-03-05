package frc.robot.systems.autonomous;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Config;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrankenArm;
import frc.robot.subsystems.RollsRUs;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * Initial implementation of the Autonomous mode behavior
 */
public class AutonomousControllerImpl implements AutonomousController {

    private static AutonomousControllerImpl controller;

    private final SendableChooser<Command> autoChooser;

    private FrankenArm arm;

    private AutonomousControllerImpl(Config config, CommandSwerveDrivetrain driveSystem, FrankenArm arm, RollsRUs intake) {
        
        this.arm = arm;

        var controller = new PPHolonomicDriveController(
            readPidConstants(config, "translation"),
            readPidConstants(config, "rotation")
        );

        // Register Triggers Here
        new EventTrigger("RunIntake").whileTrue(intake.runIntake());
        new EventTrigger("RunOutput").whileTrue(intake.runOutput());

        new EventTrigger("MoveArmUp").onTrue(arm.snapUp());
        new EventTrigger("MoveArmDown").onTrue(arm.snapDown());

        // Register Commands Here
        NamedCommands.registerCommand("runIntake", intake.runIntake());
        NamedCommands.registerCommand("runOutput", intake.runOutput());
        
        // Boolean supplier that controls when the path will be mirrored for the red
        // alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        BooleanSupplier requiresFlip = Config::isRedAlliance;

        AutoBuilder.configure(
            driveSystem::getPose,
            driveSystem::resetPose,
            driveSystem::getRobotRelativeChassisSpeeds,
            driveSystem::driveRobotRelative,
            controller,
            config.generatedConfig,
            requiresFlip,
            driveSystem
        );
    
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public static synchronized AutonomousControllerImpl initialize(Config config, CommandSwerveDrivetrain driveSystem, FrankenArm arm, RollsRUs intake) {
        controller = new AutonomousControllerImpl(config, driveSystem, arm, intake);
        return controller;
    }

    public static synchronized AutonomousControllerImpl instance() {
        if (controller == null) {
            throw new IllegalStateException("AutoController not initialized");
        }
        return controller;
    }

    private PIDConstants readPidConstants(Config config, String type) {
        return new PIDConstants(
            config.readDoubleProperty(String.format("pfc.pid.%s.kp", type)),
            config.readDoubleProperty(String.format("pfc.pid.%s.ki", type)),
            config.readDoubleProperty(String.format("pfc.pid.%s.kd", type))
        );
    }

    @Override
    public void runInit() {
        arm.armMotor.setPosition(0.0);
        CommandScheduler.getInstance().cancelAll();
        autoChooser.getSelected().schedule();
    }

    @Override
    public void runPeriodic() {}

    @Override
    public void runExit() {
        Commands.print("No autonomous exit configured").schedule();
    }
}
