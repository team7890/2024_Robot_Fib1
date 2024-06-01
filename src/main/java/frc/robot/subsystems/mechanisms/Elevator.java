// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import com.revrobotics.RelativeEncoder;
import java.lang.Math;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends SubsystemBase {

  // private CANSparkMax objElevator = new CANSparkMax(Constants.CANids.iElevator, MotorType.kBrushless);
  private TalonFX objElevator = new TalonFX(Constants.CANids.iElevator, "canivore");
  // private RelativeEncoder objElevEncoder;
  private StatusSignal<Double> objStatSig;
  private StatusCode objTalonFXStatusCode;

  private final double dKp = 0.3 / 30.0;
  private final double dKi = 0.03 / 100.0;

  private double dSpeedDisplay;
  private double dJoystickSquishOld;
  private boolean bPresetPositionOld = false;
  private double dTargetHold = 0.0;
  private double dIntegral = 0.0;
  private double dTargetDisplay;
  private double dPosDisplay;

  private boolean bInit = true;


  /** Creates a new Elevator. */
  public Elevator() {
    //---TALON---\\
    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objElevator.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }
  }

  @Override
  public void periodic() {
    dPosDisplay = getElevatorPos();
    SmartDashboard.putNumber("Elevator Pos", dPosDisplay);
    SmartDashboard.putNumber("Elevator Speed", dSpeedDisplay);
    SmartDashboard.putNumber("Elevator Target Position", dTargetDisplay);
  }

  public void stopElevator() {
    objElevator.stopMotor();
  }

  public double getElevatorPos() {
    objStatSig = objElevator.getPosition();
    return -objStatSig.getValue();            // invert position so that elevator moving up is positive
  }

  public void zeroElevatorPos() {
    objElevator.setPosition(0.0);
    dTargetHold = 0.0;
  }

  public void runElevator(double dJoystick, double dPosTarget, boolean bPresetPosition) {
    double dSpeed, dPos, dError, dJoystickSquish;
    double dMaxSpeed, dMinSpeed;
    dPos = getElevatorPos();
    if (bPresetPosition) {
      if (!bPresetPositionOld) {
        dIntegral = 0.0;
      }
      dError = dPos - dPosTarget;
      dIntegral = dIntegral + dError;
      dIntegral = limitDouble(-200.0, dIntegral, 200.0);
      dSpeed = dError * dKp + dIntegral * dKi;
      // dSpeed = dError * dKp;
      // dIntegralPart = dIntegral * dKi;
      // dErrorPart = dError * dKp;
      dJoystickSquish = 99.0; // set to a value here so that dJoystickSquish isn't 0.0 going into the else below
      dTargetDisplay = dPosTarget;
    }
    else{
      dJoystickSquish = deadbandSquish(dJoystick);
      dSpeed = dJoystickSquish;
      if(dJoystickSquish == 0.0) {
        if(dJoystickSquishOld != 0.0 | bInit) {
          dTargetHold = getElevatorPos();
          dIntegral = 0.0;
          bInit = false;
        }
        dError = dPos - dTargetHold;
        dIntegral = dIntegral + dError;
        dIntegral = limitDouble(-200.0, dIntegral, 200.0);
        dSpeed = dError * dKp + dIntegral * dKi;
        // dSpeed = dError * dKp;
        // dIntegralPart = dIntegral * dKi;
        // dErrorPart = dError * dKp;
        dTargetDisplay = dTargetHold;
      }
    }
    // set max and min speeds to prevent travel below position 1.0 and above position 40
    // ... remember negative speed moves in + position direction so if > 40 position set min speed to 0 to not let elevator go up any more
    dMaxSpeed = 0.3;
    dMinSpeed = -0.3;
    // if (dPos <= 0.0) dMaxSpeed = 0.0;
    // if (dPos >= 55.0) dMinSpeed = 0.0;
    objElevator.set(limitDouble(dMinSpeed, dSpeed, dMaxSpeed));
    dJoystickSquishOld = dJoystickSquish;
    bPresetPositionOld = bPresetPosition;
  }

  

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
    double dLimit = 0.3;
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