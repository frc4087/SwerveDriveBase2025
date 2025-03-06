// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;

public class Robot extends TimedRobot {

    public final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
        Timer.delay(5);
        CameraServer.startAutomaticCapture();
    }
    @Override
    public void robotInit() {
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        
    }
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Read battery voltage
        double voltage = RobotController.getBatteryVoltage();
        // Send voltage to SmartDashboard
        SmartDashboard.putNumber("Battery Voltage", voltage);

         // Retrieve arm position
        double armPosition = m_robotContainer.frankenArm.armMotor.getPosition().getValueAsDouble();
        // Send arm position to SmartDashboard
        SmartDashboard.putNumber("Arm Position", armPosition);

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
        m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);
        m_robotContainer.auto().runInit();
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
