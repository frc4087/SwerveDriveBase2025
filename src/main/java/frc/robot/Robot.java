// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RotateBotCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrankenArm;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {

  private final RobotContainer m_robotContainer;
  //private final FrankenArm m_frankenArm;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Timer.delay(5);
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_robotContainer.auto().runInit();
    // m_robotContainer.strikeAPose
    //   .snappy(Radians.convertFrom(90, Degree), true)
    //   .schedule();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    
    new RotateBotCommand(m_robotContainer.drivetrain, m_robotContainer.config)
      .withRobotRelativeCurrentRads(Radians.convertFrom(90, Degree))
      .schedule();

    // m_robotContainer.auto().runInit();
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.auto().runPeriodic();
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.auto().runExit();
  }

  @Override
  public void teleopInit() {
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void teleopPeriodic() {
    Commands.run(() -> {
      m_robotContainer.frankenArm.periodicConfig();
    }, m_robotContainer.frankenArm).schedule();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
