// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Tilter extends SubsystemBase {

  private static final MotorType objMotorType = MotorType.kBrushless;

  private CANSparkMax objTilter = new CANSparkMax (Constants.CANids.iShooterTilter, objMotorType);
  private RelativeEncoder objEncoder;
  private DutyCycleEncoder objAbsEncoder;
  private boolean bHold = false;
  private double dHoldPos;
  private double dPosOld = 0.0;

  private double dTilterOffset = 27.0;

  private double dTest;
  // private double dKp = 0.02;
  // private double dMaxSpeed = 0.5;
  // private double dMinSpeed = 0.025;
//  private CANSparkMax objShootTilter = new CANSparkMax(Constants.CANids.iShooterTilter, MotorType.kBrushless);
  /** Creates a new ShooterTilter. */
  public Tilter() {
    objAbsEncoder = new DutyCycleEncoder(9);
    objTilter.restoreFactoryDefaults ();
    objTilter.setSmartCurrentLimit(25);
    objTilter.setIdleMode(IdleMode.kBrake);
    objTilter.setOpenLoopRampRate(0.5);
    objTilter.setInverted(true);
    
    // objEncoder = objTilter.getEncoder(RelativeEncoder.Type.kQuadrature, 4096);
    objEncoder = objTilter.getEncoder();
    // objEncoder.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Test Target Angle", dTest);
    SmartDashboard.putNumber("Tilt Angle", getTilterPosition());
    SmartDashboard.putBoolean("Hold Tilt", bHold);
    if(bHold) {
      drivePos(dHoldPos);
    }
    

    // This method will be called once per scheduler run
  }

  public void stopTilter() {
    objTilter.stopMotor();
  }

  public void runTilter(double dSpeed) {
    // case where tilter is above minimum angle - run per speed input in variable dSpeed
    if(getTilterPosition() > Constants.Tilter.dMinAngle) {
      objTilter.set(limitDouble(dSpeed, 0.15));
    }
    else{
      // speed > 0.0 means tilting down which we cannot allow if <= minimum angle already
      if(dSpeed > 0.0) {
        // if(!bHold){
        //   startHold();
        // }
        // drivePos(dHoldPos);    // DANGER - this line is a recursive call
        objTilter.stopMotor();    //  ... so just stop the motor
      }
      else{
        if(bHold){
          cancelHold();
        }
        objTilter.set(limitDouble(dSpeed, 0.15));
      }
    }
  }

  public void runTilter2(double dSpeed_in){
    double dSpeed;
    dSpeed = deadbandSquish(dSpeed_in);
    if(dSpeed == 0.0 & bHold) {
      drivePos(dHoldPos);
    }
    if(dSpeed == 0.0 & !bHold) {
      startHold();
      drivePos(dHoldPos);
    }
    if(Math.abs(dSpeed) > 0.0) {
      cancelHold();
      // if trying to move down (which is speed > 0.0) and below or equal to minimum angle, stop the motor
      if(getTilterPosition() <= Constants.Tilter.dMinAngle & dSpeed > 0.0){
        stopTilter();
      }
      else{
        // if trying to move up (which is speed < 0.0) and above or equal to max angle, stop motor
        if(getTilterPosition() >= Constants.Tilter.dMaxAngle & dSpeed < 0.0){
          stopTilter();
        }
        // if between the min and max, then just move up or down per dSpeed input
        else{
          objTilter.set(dSpeed);
        }
      }
    }
  }

  public void cancelHold() {
    bHold = false;
  }

  public void startHold() {
    bHold = true;
    dHoldPos = getTilterPosition();
  }

  // public void runTilterToPosition(double dTargetPosition) {
  //   double dError = getTilterPosition() - dTargetPosition;
  //   double dSign = Math.signum(dError);
  //   if (Math.abs(dError) > 5.0) {
  //     runTilter(dMaxSpeed * dSign);
  //   }
  //   else {
  //     runTilter(dError * dKp + dSign * dMinSpeed);
  //   }
  // }

  public double getTilterPositionOld() {
    return objAbsEncoder.get() * 360.0 - dTilterOffset;
  }

  public void zeroTilterPosition() {
    objEncoder.setPosition(0.0);
  }

  public double getTilterPosition() {
    return -objEncoder.getPosition();
  }

  public void drivePos(double dTarget) {
    double dPos = getTilterPosition();
    double dError = dPos - dTarget;
    runTilter(dError * 0.022);
    dTest = dTarget;
    // System.out.println("drivePos Target:  " + dTarget);
    // System.out.println("drivePos Error:  " + dError);
  }


  public double limitDouble(double dValue, double dLimit) {
    double dResult;
    if (dValue > 0.0) {
      dResult = Math.min(dValue, dLimit);
    }
    else {
      dResult = Math.max(dValue, -dLimit);
    }
    return dResult;
  }

  public double deadbandSquish(double dValue){
    double m = 0.25 / 0.85;
    double b = -m * 0.15;
    double dReturn;
    if(dValue > 0.15){
      dReturn = m * dValue + b;
      // System.out.println("Down Speed:  " + dReturn);
    }
    else{
      if(dValue < -0.15){
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