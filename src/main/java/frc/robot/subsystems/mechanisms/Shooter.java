// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANids;

// ---- TALONFX ---- \\
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// ---- CANSPARK ---- \\
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkMax;

// --- Smart Dashboard --- \\
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends SubsystemBase {

  // private CANSparkMax objShooter1 = new CANSparkMax(Constants.CANids.iShooter , MotorType.kBrushless);
  // private CANSparkMax objShooter2 = new CANSparkMax(Constants.CANids.iShooter2 , MotorType.kBrushless);
  
  private TalonFX objShooterMotor = new TalonFX(CANids.iShooter, "canivore");
  private StatusSignal objSignal;
  private StatusCode objTalonCode;
  private final double dKrakenMaxRPM = 6000.0;
  private final double dKp = 2.5 / dKrakenMaxRPM;
  private final double dKi = 0.05 / dKrakenMaxRPM;
  private double dIntegral = 0.0;
  private double dErrorPart = 0.0;
  private double dIntegralPart = 0.0;
  private double dTargetDisplay;
  // private final double dKd = 10.0 / dKrakenMaxRPM;
  // private double dSpeedOld;
  // private double dCommandOld;

  /** Creates a new Shooter. */
  public Shooter() {

    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    objTalonCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 1; i < 5; i++) {
      objTalonCode = objShooterMotor.getConfigurator().apply(objTalonFXConfig);
      if (objTalonCode.isOK()) break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", getSpeed());
    // SmartDashboard.putNumber("Shooter Error Part", dErrorPart);
    // SmartDashboard.putNumber("Shooter Integral Part", dIntegralPart);
    SmartDashboard.putNumber("Shooter Target Speed", dTargetDisplay);
  }
  
  public void stopShooter() {
    objShooterMotor.stopMotor();
  }

  public void runShooter(double dSpeed) {
    objShooterMotor.set(dSpeed);
  }

  public double getSpeed() {
    StatusSignal ssVel = objShooterMotor.getVelocity();
    return ssVel.getValueAsDouble() * 60.0;             // Turns rotations per second into per minute
  }

  // public void resetShooterRPM() {
  //   dSpeedOld = 0.0;
  //   dCommandOld = 0.0;
  // }

  public void resetIntegral(){
    dIntegral = 0.0;
  }

  public void runShooterRPM(double dTargetRPM) {
    double dCurrentSpeed = getSpeed();
    double dError;
    double dCommand;
    double dBaseSpeed;

    dError = dTargetRPM - dCurrentSpeed;
    dBaseSpeed = dTargetRPM / dKrakenMaxRPM;
    dIntegral = dIntegral + dError;
    dIntegral = limitDouble(-4000.0, dIntegral, 4000.0);
    dCommand = dBaseSpeed + dIntegral * dKi;
    objShooterMotor.set(limitDouble(-1.0, dCommand, 1.0));
    dTargetDisplay = dTargetRPM;
    // dErrorPart = dError * dKp;
    // dIntegralPart = dIntegral * dKi;
  }

  // -------------UTILITY FUNCTIONS || KEEP AT BOTTOM ------------- \\

  public double limitDouble (double dMinVal, double dValue, double dMaxVal) {
    double dLimit = Math.max(dValue, dMinVal);
    return Math.min(dLimit, dMaxVal);
  }
}