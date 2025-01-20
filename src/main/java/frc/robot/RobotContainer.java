// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrankenArm;
import frc.robot.subsystems.RollsRUs;
import frc.robot.subsystems.autonomous.AutonomousController;
import frc.robot.subsystems.autonomous.AutonomousControllerImpl;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                    // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final Config config = new Config();

  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController driverJoystick = new CommandXboxController(0);

  private final CommandXboxController operatorJoystick = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
      config,
      TunerConstants.DrivetrainConstants, 
      TunerConstants.FrontLeft, 
      TunerConstants.FrontRight, 
      TunerConstants.BackLeft, 
      TunerConstants.BackRight
  );

  public final AutonomousController autonomousController = AutonomousControllerImpl.initialize(config, drivetrain);

  public final FrankenArm frankenArm = new FrankenArm(config);

  public final RollsRUs rollsRUs = new RollsRUs(config);

  public TalonFX IntakeMotor = new TalonFX(TunerConstants.IntakeMotor);

  public RobotContainer() {
    setUpDriverController();
    setUpOpController();
    setUpTelemetry();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

  }

  private void setUpDriverController() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive
                                                                                                 // forward with
                                                                                                 // negative Y
                                                                                                 // (forward)
            .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                              // negative X (left)
        ));

    driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> point
        .withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  }

  public void setUpOpController() {
    // Controll intake
    operatorJoystick.povRight().whileTrue(rollsRUs.runOutput());
    operatorJoystick.povLeft().whileTrue(rollsRUs.runIntake());

    // Arm Controll
    operatorJoystick.povUp().whileTrue(frankenArm.runFoward());
    operatorJoystick.povDown().whileTrue(frankenArm.runBack());
  }

  private void setUpTelemetry() {
    drivetrain.registerTelemetry(logger::telemeterize);
    // Add a button to run the example auto to SmartDashboard, this will also be in
    // the auto chooser built above
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
        new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
        new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
        new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
        new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = drivetrain.getPose();

      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          new PathConstraints(
              4.0, 4.0,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          null, // Ideal starting state can be null for on-the-fly paths
          new GoalEndState(0.0, currentPose.getRotation()));

      // Prevent this path from being flipped on the red alliance, since the given
      // positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
  }

  public AutonomousController auto() {
    return autonomousController;
  }
}
