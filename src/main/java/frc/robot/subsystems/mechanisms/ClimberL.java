// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class ClimberL extends SubsystemBase {

  private TalonFX objClimberL = new TalonFX(Constants.CANids.iClimbLeft, "canivore");

  /** Creates a new Climb. */
  public ClimberL() {
    objClimberL.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration objClimbLConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs objCurrentLimitsConfigs = objClimbLConfig.CurrentLimits;
    objCurrentLimitsConfigs.SupplyCurrentLimit = 60.0;
    objCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    objClimberL.getConfigurator().apply(objCurrentLimitsConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopClimber(){
    objClimberL.stopMotor();
  }

  public void moveClimber(double dSpeed) {
    objClimberL.set(dSpeed);
  }
}
