package frc.robot.systems.autonomous;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * Initial implementation of the Autonomous mode behavior
 */
public class AutonomousControllerImpl implements AutonomousController {

    private static AutonomousControllerImpl controller;

    private CommandSwerveDrivetrain drivetrain;

    private Map<String, PathPlannerAuto> autos;

    private final SendableChooser<Command> autoChooser;

    private final Timer timer = new Timer();


    private AutonomousControllerImpl(Config config, CommandSwerveDrivetrain driveSystem) {
        this.drivetrain = driveSystem;
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
            createPathFollowingController(),
            config.generatedConfig,
            requiresFlip,
            driveSystem
        );
        loadAutos();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void loadAutos() {
        autos = Map.of(
            "Config", new PathPlannerAuto("Config"),
            "Test Auto", new PathPlannerAuto("Test Auto"),
            "Simple Coral Auto", new PathPlannerAuto("Simple Coral Auto")
        );
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

    @Override
    public void runInit() {
        timer.restart();
        CommandScheduler.getInstance().cancelAll();
        PathfindingCommand.warmupCommand().schedule();
        autoChooser.getSelected().schedule();
    }

    @Override
    public void runPeriodic() {
        /*if(timer.get() < 2.0) {
            drivetrain.setControl(
                new SwerveRequest.RobotCentric().withVelocityX(1.0)
            );
        } else {
            drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        }*/
    }

    @Override
    public void runExit() {
        Commands.print("No autonomous exit configured").schedule();
    }

    private PathFollowingController createPathFollowingController() {
        return new PPHolonomicDriveController(
            new PIDConstants(5, 0, 0),
            new PIDConstants(5, 0, 0)
        );
    }
}
