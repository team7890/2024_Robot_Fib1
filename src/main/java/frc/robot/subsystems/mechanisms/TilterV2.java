// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// === KRACKEN IMPORTS ===
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// === END KRAKEN ===

// spark max for testing
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// End spark max

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class TilterV2 extends SubsystemBase {

  private double dTilterOffset = -110.0;     // TODO set from encoder

  private final double dKp = 0.2 / 30.0;
  private final double dKi = 0.03 / 100.0;
  // private final double dKi = 0.0;
  // private final double dKGrav = 2.5 * dKp;

  private DutyCycleEncoder objAbsEncoder;
  private double dJoystickSquishOld;
  private boolean bPresetAngleOld = false;
  private double dTargetHold = 0.0;
  private double dIntegral = 0.0;

  private double dIntegralPart;
  private double dErrorPart;
  private double dTargetDisplay;

  private boolean bInit = true;
 
  // -- TALON -- \\
  private TalonFX objTilterMotor = new TalonFX(Constants.CANids.iShooterTilter, "canivore");
  private StatusSignal objSignal;
  private StatusCode objTalonFXStatusCode;
 
  // --- Spark Max For Testing --- \\
  // private CANSparkMax objTilterMotor = new CANSparkMax (Constants.CANids.iShooterTilter, MotorType.kBrushless);
  // private RelativeEncoder objEncoder;

  /** Creates a new TilterV2. */
  public TilterV2() {

      //---TALON---\\
    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objTilterMotor.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }
   
    //---SPARKMAX---\\
    // objEncoder = objTilterMotor.getEncoder();
    // zeroTilterPosition();

    // objTilterMotor.setPosition(0.0);
    objAbsEncoder = new DutyCycleEncoder(9);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tilt Angle", getTilterPosition());
    // SmartDashboard.putNumber("Tilter Error Part", dErrorPart);
    // SmartDashboard.putNumber("Tilter Integral Part", dIntegralPart);
    SmartDashboard.putNumber("Tilter Target Position", dTargetDisplay);
    SmartDashboard.putNumber("Raw Encoder", objAbsEncoder.get());
  }
 
  public void stopTilter() {
    objTilterMotor.stopMotor();
  }
 
  public double getTilterPosition() {
    double dAngle;
    dAngle = objAbsEncoder.get() * 360.0 - dTilterOffset;
    while (dAngle < 0.0 || dAngle > 360.0) {
      if (dAngle < 0.0) {
        dAngle = dAngle + 360.0;
      }
      if (dAngle > 360.0) {
        dAngle = dAngle -360.0;
      }
    }
    dAngle = dAngle % 360.0;
    // dAngle = dAngle - 360.0;
    return dAngle;
  }
 
  public void runTilter(double dJoystick, double dPosTarget, boolean bPresetAngle) {
    double dSpeed, dPos, dError, dJoystickSquish;
    double dMaxSpeed, dMinSpeed;
    dPos = getTilterPosition();
    if (bPresetAngle) {
      if (!bPresetAngleOld) {
        dIntegral = 0.0;
      }
      dError = dPos - dPosTarget;
      // dSpeed = dError * dKp + dKGrav;
      dIntegral = dIntegral + dError;
      dIntegral = limitDouble(-200.0, dIntegral, 200.0);
      dSpeed = dError * dKp + dIntegral * dKi;
      // dSpeed = dError * dKp;
      dIntegralPart = dIntegral * dKi;
      dErrorPart = dError * dKp;
      dJoystickSquish = 99.0; // set to a value here so that dJoystickSquish isn't 0.0 going into the else below
      dTargetDisplay = dPosTarget;
    }
    else{
      dJoystickSquish = deadbandSquish(dJoystick);
      dSpeed = dJoystickSquish;
      if(dJoystickSquish == 0.0) {
        if(dJoystickSquishOld != 0.0 | bInit) {
          dTargetHold = getTilterPosition();
          dIntegral = 0.0;
          bInit = false;
        }
        dError = dPos - dTargetHold;
        dIntegral = dIntegral + dError;
        dIntegral = limitDouble(-200.0, dIntegral, 200.0);
        dSpeed = dError * dKp + dIntegral * dKi;
        // dSpeed = dError * dKp;
        dIntegralPart = dIntegral * dKi;
        dErrorPart = dError * dKp;
        dTargetDisplay = dTargetHold;
      }
    }
    // set max and min speeds to prevent travel below 1.0 degrees and above 115 degrees
    // ... remember negative speed moves in + angle direction so if > 115 degrees set minimum speed to 0 to not let tilter go up any more
    dMaxSpeed = 0.2;
    dMinSpeed = -0.2;
    if (dPos <= 17.0) dMaxSpeed = 0.0;
    if (dPos >= 130.0) dMinSpeed = 0.0;
    objTilterMotor.set(limitDouble(dMinSpeed, dSpeed, dMaxSpeed));
    dJoystickSquishOld = dJoystickSquish;
    bPresetAngleOld = bPresetAngle;
  }

  // public void zeroTilterPosition() {
  //   objEncoder.setPosition(0.0);
  //   dTargetHold = 0.0;
  // }

  // -------------UTILITY FUNCTIONS || KEEP AT BOTTOM ------------- \\

  public double limitDouble(double dMinLimit, double dValue, double dMaxLimit) {
    double dResult;
    dResult = Math.min(dValue, dMaxLimit);
    dResult = Math.max(dResult, dMinLimit);
    return dResult;
  }



  public double deadbandSquish(double dValue){    
    //   Gives Joystick 0.15 Deadband   \\
   //       Limits Speed at 0.25         \\
    double dDeadband = 0.15;
    double dLimit = 0.25;
    double m = dLimit / (1.0 - dDeadband);
    double b = -m * dDeadband;
    double dReturn;
    if(dValue > dDeadband){
      dReturn = m * dValue + b;
      // System.out.println("Down Speed:  " + dReturn);
    }
    else{
      if(dValue < -dDeadband){
        dReturn = m * dValue - b;
        // System.out.println("Up Speed:  " + dReturn);
      }
      else{
        dReturn = 0.0;
      }
    }
    return dReturn;
  }
}
