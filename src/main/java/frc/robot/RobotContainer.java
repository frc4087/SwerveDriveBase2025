// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.RotateBotCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrankenArm;
import frc.robot.subsystems.RollsRUs;
import frc.robot.systems.autonomous.AutonomousController;
import frc.robot.systems.autonomous.AutonomousControllerImpl;

public class RobotContainer {

  public final Config config = new Config();
  //private final RobotContainer m_robotContainer;

  private double MaxSpeed = config.TunerConstants.getKSpeedAt12Volts().in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                    // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);


  // private Integer fieldDirectionInDegrees;

  // private final SendableChooser<Command> autoChooser;

  private final CommandXboxController driverController = new CommandXboxController(0);

  private final CommandXboxController operatorController = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
      config,
      config.TunerConstants.getDrivetrainConstants(), 
      config.TunerConstants.getFrontLeftModule(), 
      config.TunerConstants.getFrontRightModule(),
      config.TunerConstants.getBackLeftModule(),
      config.TunerConstants.getBackRightModule()
  );

  public final FrankenArm frankenArm = new FrankenArm(config);

  public final AutonomousController autonomousController = AutonomousControllerImpl.initialize(
    config, 
    drivetrain,
    frankenArm
  );


  public final RollsRUs rollsRUs = new RollsRUs(config);

  Integer intakeMotorPort = config.readIntegerProperty("ports.intake.motor");
  public TalonFX IntakeMotor = new TalonFX(intakeMotorPort);

  public RobotContainer() {
    setUpDriverController();
    setUpOpController();
    setUpTelemetry();

    NamedCommands.registerCommand("goToZero", frankenArm.goZero());
    NamedCommands.registerCommand("goToNine", frankenArm.goNine());

    NamedCommands.registerCommand("runIntake", rollsRUs.runIntake());
    NamedCommands.registerCommand("runOutput", rollsRUs.runOutput());

  }

  private void setUpDriverController() {
    drivetrain.configNeutralMode(NeutralModeValue.Brake);
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive
                                                                                                 // forward with
                                                                                                 // negative Y
                                                                                                 // (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                              // negative X (left)
        ));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain.applyRequest(() -> point
        .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    driverController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // new RotateBotCommand(m_robotContainer.drivetrain, m_robotContainer.config)
    // .withRobotRelativeCurrentRads(Radians.convertFrom(90, Degree))
    // .schedule();
    
    // driverController.povUp().onTrue(RotateBotCommand.(0.0, false));
    // driverController.povRight().onTrue(RotateBotCommand.onTrue(90.0, false));
    // driverController.povDown().onTrue(RotateBotCommand.onTrue(180.0, false));
    // driverController.povLeft().onTrue(RotateBotCommand.onTrue(270.0, false));
  }

  public void setUpOpController() {

    // Intake Control
    operatorController.leftBumper().whileTrue(rollsRUs.runOutput());
    operatorController.rightBumper().whileTrue(rollsRUs.runIntake());

    // Arm Controll
    operatorController.x().onTrue(frankenArm.goZero());
    operatorController.b().onTrue(frankenArm.goNine());

  }

  private void setUpTelemetry() {
    drivetrain.registerTelemetry(logger::telemeterize);
    // Add a button to run the example auto to SmartDashboard, this will also be in
    // the auto chooser built above
    // SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // // Add a button to run pathfinding commands to SmartDashboard
    // SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
    //     new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
    //     new PathConstraints(
    //         4.0, 4.0,
    //         Units.degreesToRadians(360), Units.degreesToRadians(540)),
    //     0));
    // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    //     new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
    //     new PathConstraints(
    //         4.0, 4.0,
    //         Units.degreesToRadians(360), Units.degreesToRadians(540)),
    //     0));

    // // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // // This example will simply move the robot 2m in the +X field direction
    // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
    //   Pose2d currentPose = drivetrain.getPose();

    //   // The rotation component in these poses represents the direction of travel
    //   Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    //   Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

    //   List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
    //   PathPlannerPath path = new PathPlannerPath(
    //       waypoints,
    //       new PathConstraints(
    //           4.0, 4.0,
    //           Units.degreesToRadians(360), Units.degreesToRadians(540)),
    //       null, // Ideal starting state can be null for on-the-fly paths
    //       new GoalEndState(0.0, currentPose.getRotation()));

    //   // Prevent this path from being flipped on the red alliance, since the given
    //   // positions are already correct
    //   path.preventFlipping = true;

    //   AutoBuilder.followPath(path).schedule();

  }

  public AutonomousController auto() {
    return autonomousController;
  }
}
