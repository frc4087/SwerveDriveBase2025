
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.generated.TunerConstants;

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
  public TalonFX armMotor = new TalonFX(TunerConstants.ArmMotor);
  private final Double fwdSpeed;
  private final Double backwardSpeed;
  private final Double fwdStall;
  private final Double backwardStall;

  public FrankenArm(Config config) {
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("frankenarm.motor.statorCurrent.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = config.readIntegerProperty("frankenarm.motor.supplyCurrent.limit.amps");
        limitConfigs.SupplyCurrentLimitEnable = true;

        armMotor.getConfigurator().apply(limitConfigs);

        fwdSpeed = config.readDoubleProperty("frankenarm.motor.forwards.speed");
        backwardSpeed = config.readDoubleProperty("frankenarm.motor.backwards.speed");

        fwdStall = config.readDoubleProperty("frankenarm.motor.forwards.speed.stall");
        backwardStall = config.readDoubleProperty("frankenarm.motor.backwards.speed.stall");
  }

  public Command runFoward() {
    return this.runEnd(() -> armMotor.set(fwdSpeed), this::stallfwd);
  }

  public Command runBack() {
    return this.runEnd(() -> armMotor.set(backwardSpeed), this::stallback);
  }

  private void stallfwd() {
    armMotor.set(fwdStall);
  }

  private void stallback() {
    armMotor.set(backwardStall);
  }
}
