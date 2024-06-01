// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

  // -- TALON -- \\
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.CANids;

  // -- CANSPARK -- \\
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder extends SubsystemBase {
  // -- TALON -- \\
  private TalonFX objFeeder = new TalonFX(CANids.iFeeder, "canivore");
  private StatusSignal objSignal;
  private StatusCode objTalonCode;
  private final double dKrakenMaxRPM = 6000.0;
  private final double dKi = 0.1 / 1000.0;
  private final double dKp = 1.0 / dKrakenMaxRPM;
  private final double dKd = 10.0 / dKrakenMaxRPM;
  private double dSpeedOld;
  private double dCommandOld;
  // private CANSparkMax objFeeder = new CANSparkMax(Constants.CANids.iFeeder , MotorType.kBrushless);
  // private RelativeEncoder objFeederEncoder;

  private double dMaxCurrent;
  private double dCurrent;
  private boolean bMotorStarting;
  private int iCount;
  private boolean bNoteDetected;
  private boolean bNoteDetectedPlusDelay;
  private int iDelayCount;
  private double dTargetDisplay;
  private double dErrorPart = 0.0;
  private final double dIntegralPart = 0.0;
  private double dIntegral = 0.0;

  /** Creates a new Feeder. */
  public Feeder() {

    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    objTalonCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 1; i < 5; i++) {
      objTalonCode = objFeeder.getConfigurator().apply(objTalonFXConfig);
      if (objTalonCode.isOK()) break;
    }
    // objTilterMotor.setPosition(0.0);
    // objAbsEncoder = new DutyCycleEncoder(9);


    // objFeeder.setSmartCurrentLimit(30);
    // objFeeder.setIdleMode(IdleMode.kBrake);
    // objFeeder.setOpenLoopRampRate(0.2);

    // objFeederEncoder = objFeeder.getEncoder();

    // objFeederEncoder.setVelocityConversionFactor(1.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder Speed", getSpeed());
    SmartDashboard.putNumber("Feeder Error Part", dErrorPart);
    SmartDashboard.putNumber("Feeder Integral Part", dIntegralPart);
    SmartDashboard.putNumber("Feeder Target Speed", dTargetDisplay);
  }
  
  public void stopFeeder(){
    objFeeder.stopMotor();
  }

  public double getSpeed() {
    StatusSignal ssVel = objFeeder.getVelocity();
    return ssVel.getValueAsDouble() * 60.0;             // Turns rotations per second into per minute
  }

  public void resetIntegral(){
    dIntegral = 0.0;
  }

  public void runFeederRPM(double dTargetRPM) {
    double dCurrentSpeed = getSpeed();
    double dError = dTargetRPM - dCurrentSpeed;
    double dCommand;

    dIntegral = dIntegral + dError;
    dIntegral = limitDouble(-2000.0, dIntegral, 2000.0);
    dCommand = dError * dKp + dIntegral * dKi;
    objFeeder.set(limitDouble(-1.0, dCommand, 1.0));
    dTargetDisplay = dTargetRPM;
  }

  public void runFeeder (double dSpeed){
     objFeeder.set(dSpeed);
  }
    // This method will be called once per scheduler run

  public void feedNote(){
    objFeeder.set(0.8);
  }
 
      // COMMENTED DUE TO UNWRITTEN CODE AND UNUSED \\
  // public boolean detectNote() {
  //   dCurrent = objFeeder.getOutputCurrent();
  //   SmartDashboard.putNumber("Feeder Current", dCurrent);
  //   if (Math.abs(objFeeder.getAppliedOutput()) > 0.0 & !bMotorStarting) {
  //     bMotorStarting = true;
  //     dMaxCurrent = 0.0;
  //     iCount = 0;
  //   }
  //   if (bMotorStarting & iCount < 10) {
  //     iCount = iCount + 1;
  //   }
  //   else {
  //     if (dCurrent > dMaxCurrent) {
  //       dMaxCurrent = dCurrent;
  //       if (dMaxCurrent > 12.0) {
  //         bNoteDetected = true;
  //       }
  //     }
  //   }
  //   if (bNoteDetected) {
  //     iDelayCount = iDelayCount + 1;
  //     if (iDelayCount > 0) {
  //       bNoteDetectedPlusDelay = true;
  //     }
  //   }
  //   SmartDashboard.putNumber("Feeder Max Current", dMaxCurrent);
  //   return bNoteDetectedPlusDelay;
  // }

  public void resetDetectNote() {
    bMotorStarting = false;
    bNoteDetected = false;
    bNoteDetectedPlusDelay = false;
  }

  // -------------UTILITY FUNCTIONS || KEEP AT BOTTOM ------------- \\
  
  public double limitDouble (double dMinVal, double dValue, double dMaxVal) {
    double dLimit = Math.max(dValue, dMinVal);
    return Math.min(dLimit, dMaxVal);
  }
}