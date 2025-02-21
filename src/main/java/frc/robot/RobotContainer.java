// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrankenArm;
import frc.robot.subsystems.RollsRUs;
import frc.robot.subsystems.SwerveDriveSpecs;
import frc.robot.systems.autonomous.AutonomousController;
import frc.robot.systems.autonomous.AutonomousControllerImpl;

public class RobotContainer {

  public final Config config = new Config();

  private double MaxSpeed = config.TunerConstants.getKSpeedAt12Volts().in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                    // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driverController = new CommandXboxController(0);

  private final CommandXboxController operatorController = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
      config,
      config.TunerConstants.getDrivetrainConstants(),
      config.TunerConstants.getFrontLeftModule(),
      config.TunerConstants.getFrontRightModule(),
      config.TunerConstants.getBackLeftModule(),
      config.TunerConstants.getBackRightModule());

  public final AutonomousController autonomousController = AutonomousControllerImpl.initialize(config, drivetrain);

  public final FrankenArm frankenArm = new FrankenArm(config);

  public final RollsRUs rollsRUs = new RollsRUs(config);

  Integer intakeMotorPort = config.readIntegerProperty("ports.intake.motor");
  public TalonFX IntakeMotor = new TalonFX(intakeMotorPort);

  public RobotContainer() {
    setUpDriverController();
    setUpOpController();
    setUpTelemetry();
  }

  private void setUpDriverController() {
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
    //// driverController.leftBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldCentric()));
    //// driverController.rightBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldCentric()));

    // driverController.povUp().onTrue(
    // new RotateBotCommand(drivetrain, config)
    // .withRobotRelativeCurrentRads(Radians.convertFrom(-180, Degree))
    // );

    /////////////////////////////////
    DriverStation.silenceJoystickConnectionWarning(true);
    /////////////////////////////////

    // All is per the field drawing
    // (https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf)
    // and relative to the view from the blue alliance. POV controls correspond to
    // the blue reef faces as viewed
    // by the driver. Assumes the robot is started at the field origin (i.e. near
    // right corner, behind coral feed).
    SwerveDriveSpecs specs = SwerveDriveSpecs.getInstance();

    Pose2d poseStart = offsetFieldPose(new Pose2d(Units.inchesToMeters(0.00), Units.inchesToMeters(158.50),
        Rotation2d.fromDegrees(0.0)), specs.kBackOffset); // X: back against blue line, Y: field mid point
    drivetrain.resetPose(poseStart);

    Pose2d pose12 = offsetFieldPose(new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80),
        Rotation2d.fromDegrees(-126.0)), specs.kFrontOffset);
    Pose2d pose13 = offsetFieldPose(new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20),
        Rotation2d.fromDegrees(126.0)), specs.kFrontOffset);

    Pose2d pose18 = offsetFieldPose(new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50),
        Rotation2d.fromDegrees(0.0)), specs.kFrontOffset);
    Pose2d pose17 = offsetFieldPose(new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17),
        Rotation2d.fromDegrees(+60.0)), specs.kFrontOffset);
    Pose2d pose19 = offsetFieldPose(new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83),
        Rotation2d.fromDegrees(-60.0)), specs.kFrontOffset);
    Pose2d pose21 = offsetFieldPose(new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50),
        Rotation2d.fromDegrees(180.0)), specs.kFrontOffset);
    Pose2d pose22 = offsetFieldPose(new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17),
        Rotation2d.fromDegrees(+120.0)), specs.kFrontOffset);
    Pose2d pose20 = offsetFieldPose(new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83),
        Rotation2d.fromDegrees(-120.0)), specs.kFrontOffset);

    // pose reset commands
    driverController.povLeft().onTrue(new InstantCommand(() -> drivetrain.resetPose(pose13), drivetrain));
    driverController.povDown().onTrue(new InstantCommand(() -> drivetrain.resetPose(poseStart), drivetrain));
    driverController.povRight().onTrue(new InstantCommand(() -> drivetrain.resetPose(pose12), drivetrain));

    // driveto commands
    double speedFac = 1.0;

    /// driveto loading stations
    driverController.x().and(driverController.rightBumper()).whileTrue(
        new DriveToPoseCommand(drivetrain, pose13, speedFac));
    driverController.b().and(driverController.rightBumper()).whileTrue(
        new DriveToPoseCommand(drivetrain, pose12, speedFac));

    /// driveto near reef faces
    driverController.a().and(driverController.rightBumper()).whileTrue(
        new DriveToPoseCommand(drivetrain, pose18, speedFac));
    driverController.a().and(driverController.b()).and(driverController.rightBumper()).whileTrue(
        new DriveToPoseCommand(drivetrain, pose17, speedFac));
    driverController.a().and(driverController.x()).and(driverController.rightBumper()).whileTrue(
        new DriveToPoseCommand(drivetrain, pose19, speedFac));

    /// driveto far reef faces
    driverController.y().and(driverController.rightBumper()).whileTrue(
        new DriveToPoseCommand(drivetrain, pose21, speedFac));
    driverController.y().and(driverController.b()).and(driverController.rightBumper()).whileTrue(
        new DriveToPoseCommand(drivetrain, pose22, speedFac));
    driverController.y().and(driverController.x()).and(driverController.rightBumper()).whileTrue(
        new DriveToPoseCommand(drivetrain, pose20, speedFac));

    // driverController.povDown().onTrue(
    // new DriveToPoseCommand(drivetrain, pose18, speedFac));
    // driverController.povDownRight().onTrue(
    // new DriveToPoseCommand(drivetrain, pose17, speedFac));
    // driverController.povDownLeft().onTrue(
    // new DriveToPoseCommand(drivetrain, pose19, speedFac));
    // driverController.povUp().onTrue(
    // new DriveToPoseCommand(drivetrain, pose21, speedFac));
    // driverController.povUpRight().onTrue(
    // new DriveToPoseCommand(drivetrain, pose22, speedFac));
    // driverController.povUpLeft().onTrue(
    // new DriveToPoseCommand(drivetrain, pose20, speedFac));

  }

  /**
   * Returns a field pose that is offset by a given amount from a given input
   * field pose in the direction of the pose. The field coordinate system is
   * assumed right-handed with Z up.
   * 
   * @param poseIn Input pose.
   * @param offset Offset (m) relative to the input pose direction (positive
   *               offset is "forward").
   * @return Output pose.
   */
  private static Pose2d offsetFieldPose(Pose2d poseIn, double offset) {
    double resultX = poseIn.getX() + offset * poseIn.getRotation().getCos();
    double resultY = poseIn.getY() + offset * poseIn.getRotation().getSin();
    return new Pose2d(resultX, resultY, poseIn.getRotation());
  }

  public void setUpOpController() {
    // Intake Control
    operatorController.leftBumper().whileTrue(rollsRUs.runOutput());
    operatorController.rightBumper().whileTrue(rollsRUs.runIntake());

    // Arm Controll
    operatorController.x().whileTrue(frankenArm.runFoward());
    operatorController.b().whileTrue(frankenArm.runBack());
  }

  private void setUpTelemetry() {
    drivetrain.registerTelemetry(logger::telemeterize);
    // Add a button to run the example auto to SmartDashboard, this will also be in
    // the auto chooser built above
    // SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // // Add a button to run pathfinding commands to SmartDashboard
    // SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
    // new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
    // new PathConstraints(
    // 4.0, 4.0,
    // Units.degreesToRadians(360), Units.degreesToRadians(540)),
    // 0));
    // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    // new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
    // new PathConstraints(
    // 4.0, 4.0,
    // Units.degreesToRadians(360), Units.degreesToRadians(540)),
    // 0));

    // // Add a button to SmartDashboard that will create and follow an on-the-fly
    // path
    // // This example will simply move the robot 2m in the +X field direction
    // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
    // Pose2d currentPose = drivetrain.getPose();

    // // The rotation component in these poses represents the direction of travel
    // Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    // Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new
    // Translation2d(2.0, 0.0)), new Rotation2d());

    // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos,
    // endPos);
    // PathPlannerPath path = new PathPlannerPath(
    // waypoints,
    // new PathConstraints(
    // 4.0, 4.0,
    // Units.degreesToRadians(360), Units.degreesToRadians(540)),
    // null, // Ideal starting state can be null for on-the-fly paths
    // new GoalEndState(0.0, currentPose.getRotation()));

    // // Prevent this path from being flipped on the red alliance, since the given
    // // positions are already correct
    // path.preventFlipping = true;

    // AutoBuilder.followPath(path).schedule();

  }

  public AutonomousController auto() {
    return autonomousController;
  }
}
