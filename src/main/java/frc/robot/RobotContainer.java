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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RotateBotCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrankenArm;
import frc.robot.subsystems.RollsRUs;
import frc.robot.systems.autonomous.AutonomousController;
import frc.robot.systems.autonomous.AutonomousControllerImpl;

public class RobotContainer {

	public final Config config = new Config();

	private double MaxSpeed = config.TunerConstants.getKSpeedAt12Volts().in(MetersPerSecond);
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

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

	public final AutonomousController autonomousController = 
		AutonomousControllerImpl.initialize(config, drivetrain, frankenArm, rollsRUs);

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
		// For the controller, Y is up/down and X is left/right.
		
		// Drivetrain will execute this command periodically
		drivetrain.setDefaultCommand(
			drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
				.withVelocityY(-driverController.getLeftX() * MaxSpeed)
				.withRotationalRate(-driverController.getRightX() * MaxAngularRate)
			)
		);

		driverController.povDown().whileTrue(drivetrain.applyRequest(() -> brake));
		driverController.povLeft().whileTrue(drivetrain.applyRequest(() -> point
				.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

		// Reset the field-centric heading on left bumper press
		driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		// The following commands rotate the robot to face the reef

		// Back
		driverController.a().onTrue(
			new RotateBotCommand(drivetrain, config)
				.withFieldRelativeAngle(180.0));

		// Back Left
		driverController.x().and(driverController.rightBumper().negate()).onTrue(
			new RotateBotCommand(drivetrain, config)
				.withFieldRelativeAngle(120.0));

		// Back Right
		driverController.b().and(driverController.rightBumper().negate()).onTrue(
			new RotateBotCommand(drivetrain, config)
				.withFieldRelativeAngle(-120.0));

		// Front
		driverController.y().onTrue(
			new RotateBotCommand(drivetrain, config)
				.withFieldRelativeAngle(0.0));

		// Front Left
		driverController.x().and(driverController.rightBumper()).onTrue(
			new RotateBotCommand(drivetrain, config)
				.withFieldRelativeAngle(60.0));

		// Front Right
		driverController.b().and(driverController.rightBumper()).onTrue(
			new RotateBotCommand(drivetrain, config)
				.withFieldRelativeAngle(-60.0));

	}

	public void setUpOpController() {
		// Intake Control
		operatorController.leftBumper().whileTrue(rollsRUs.runIntake());
		operatorController.rightBumper().whileTrue(rollsRUs.runOutput());

		// Arm Control
		operatorController.x().onTrue(frankenArm.snapUp());
		operatorController.b().onTrue(frankenArm.snapDown());

		// Arm Reset
		operatorController.leftTrigger().whileTrue(frankenArm.runUp());
	}

	private void setUpTelemetry() {
		drivetrain.registerTelemetry(logger::telemeterize);
	}

	public AutonomousController auto() {
		return autonomousController;
	}
}
