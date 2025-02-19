// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  public final RobotContainer m_robotContainer;

  public Robot() {
   m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.auto().runInit();
    // m_robotContainer.strikeAPose
    //   .snappy(Radians.convertFrom(90, Degree), true)
    //   .schedule();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    
    // new RotateBotCommand(m_robotContainer.drivetrain, m_robotContainer.config)
    //   .withRobotRelativeCurrentRads(Radians.convertFrom(90, Degree))
    //   .schedule();
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
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

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
