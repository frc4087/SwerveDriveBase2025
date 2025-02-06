
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class FrankenArm extends SubsystemBase {
  public TalonFX armMotor;
  private final Integer fwdSpeed;
  private final Integer backwardSpeed;

  public FrankenArm(Config config) {
    
    var armMotorPort = config.readIntegerProperty("ports.arm.motor");
    armMotor = new TalonFX(armMotorPort);

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = config.readIntegerProperty("rollsRUs.motor.current.limit.amps");
    limitConfigs.StatorCurrentLimitEnable = true;
    armMotor.getConfigurator().apply(limitConfigs);

    fwdSpeed = config.readIntegerProperty("frankenarm.motor.forwards.speed");
    backwardSpeed = config.readIntegerProperty("frankenarm.motor.backwards.speed");
    
  }

  public Command runFoward() {
    return this.runEnd(() -> armMotor.set(fwdSpeed), this::stop);
  }

  public Command runBack() {
    return this.runEnd(() -> armMotor.set(backwardSpeed), this::stop);
  }

  private void stop() {
    armMotor.set(0);
  }
}
