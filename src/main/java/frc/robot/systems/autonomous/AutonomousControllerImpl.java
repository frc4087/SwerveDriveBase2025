package frc.robot.systems.autonomous;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Config;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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

    private AutonomousControllerImpl(Config config, CommandSwerveDrivetrain driveSystem) {
        var controller = new PPHolonomicDriveController(
            readPidConstants(config, "translation"),
            readPidConstants(config, "rotation")
        );

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

    public static synchronized AutonomousControllerImpl initialize(Config config, CommandSwerveDrivetrain driveSystem) {
        controller = new AutonomousControllerImpl(config, driveSystem);
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
