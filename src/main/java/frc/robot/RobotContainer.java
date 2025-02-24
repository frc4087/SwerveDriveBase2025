// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.commands.RotateBotCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrankenArm;
import frc.robot.subsystems.RollsRUs;
import frc.robot.subsystems.SirLiftsALot;
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
      config.TunerConstants.getBackRightModule()
    );

  public final FrankenArm frankenArm = new FrankenArm(config);

  public final RollsRUs rollsRUs = new RollsRUs(config);

  public final SirLiftsALot sirLiftsALot = new SirLiftsALot(config);

  public final AutonomousController autonomousController = AutonomousControllerImpl.initialize(config, drivetrain, frankenArm, rollsRUs);

  Integer intakeMotorPort = config.readIntegerProperty("ports.intake.motor");
  public TalonFX IntakeMotor = new TalonFX(intakeMotorPort);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
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
        )
    );

    driverController.povDown().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.povLeft().whileTrue(drivetrain.applyRequest(() -> point
        .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Bottom
    driverController.a().onTrue(
      new RotateBotCommand(drivetrain, config)
        .withFieldRelativeAngle(180.0)
    );

    // Bottom Left
    driverController.x().and(driverController.rightBumper().negate()).onTrue(
      new RotateBotCommand(drivetrain, config)
        .withFieldRelativeAngle(120.0)
    );

    // Bottom Right
    driverController.b().and(driverController.rightBumper().negate()).onTrue(
      new RotateBotCommand(drivetrain, config)
        .withFieldRelativeAngle(-120.0)
    );

    // Top
    driverController.y().onTrue(
      new RotateBotCommand(drivetrain, config)
        .withFieldRelativeAngle(0.0)
    );
    
    // Top Left
    driverController.x().and(driverController.rightBumper()).onTrue(
      new RotateBotCommand(drivetrain, config)
        .withFieldRelativeAngle(60.0)
    );

    // Top Right
    driverController.b().and(driverController.rightBumper()).onTrue(
      new RotateBotCommand(drivetrain, config)
        .withFieldRelativeAngle(-60.0)
    );
    
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
    operatorController.leftBumper().whileTrue(rollsRUs.runIntake());
    operatorController.rightBumper().whileTrue(rollsRUs.runOutput());

    operatorController.leftTrigger().whileTrue(frankenArm.runContinuous());

    // Arm Control
    operatorController.x().onTrue(frankenArm.snapUp());
    operatorController.b().onTrue(frankenArm.snapDown());

    // 
    operatorController.povUp().whileTrue(sirLiftsALot.climberGo());
    operatorController.povDown().whileTrue(sirLiftsALot.climberStop());
  }

  private void setUpTelemetry() {
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public AutonomousController auto() {
    return autonomousController;
  }
}
