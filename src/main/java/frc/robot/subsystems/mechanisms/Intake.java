// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;


public class Intake extends SubsystemBase {

  private CANSparkMax objLowRoller = new CANSparkMax(Constants.CANids.iLowRoller, MotorType.kBrushless);
  private CANSparkMax objHighRoller = new CANSparkMax(Constants.CANids.iHighRoller, MotorType.kBrushless);

  private double dMaxCurrent;
  private double dCurrent;
  private boolean bMotorStarting;
  private int iCount;
  private boolean bNoteDetected;
  private boolean bNoteDetectedPlusDelay;
  private int iDelayCount;

  /** Creates a new Intake. */
  public Intake() {
    objLowRoller.setSmartCurrentLimit(60);
    objLowRoller.setIdleMode(IdleMode.kBrake);
    objLowRoller.setOpenLoopRampRate(0.2);

    objHighRoller.setSmartCurrentLimit(60);
    objHighRoller.setIdleMode(IdleMode.kBrake);
    objHighRoller.setOpenLoopRampRate(0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putNumber("Intake Max Current", dMaxCurrent);
     SmartDashboard.putNumber("Intake Current", dCurrent);
  }

  public void stopIntake() {
    objLowRoller.stopMotor();
    objHighRoller.stopMotor();
  }

  public void intakeFwd() {
    objLowRoller.set(0.55);
    objHighRoller.set(-0.35);
  }

  public void intakeRvs() {
    objLowRoller.set(-0.5);
    objHighRoller.set(-0.3);
  }

  public void unjamIntake(boolean bForward) {
    if (bForward) {
      objLowRoller.set(0.8);
      objHighRoller.set(0.8);
    }
    else {
      objLowRoller.set(-0.8);
      objHighRoller.set(0.8);
    }
  }

  public boolean detectNote() {
    dCurrent = objLowRoller.getOutputCurrent();
    // SmartDashboard.putNumber("Intake Current", dCurrent);
    if (Math.abs(objLowRoller.getAppliedOutput()) > 0.0 & !bMotorStarting) {
      bMotorStarting = true;
      dMaxCurrent = 0.0;
      iCount = 0;
      iDelayCount = 0;
    }
    if (bMotorStarting & iCount < 10) {
      iCount = iCount + 1;
    }
    else {
      if (dCurrent > dMaxCurrent) {
        dMaxCurrent = dCurrent;
        if (dMaxCurrent > 30.0) {
          bNoteDetected = true;
        }
      }
    }
    if (bNoteDetected) {
      iDelayCount = iDelayCount + 1;
      if (iDelayCount > 20) {
        bNoteDetectedPlusDelay = true;
        // bNoteDetectedPlusDelay = false;
      }
    }
    // SmartDashboard.putNumber("Intake Max Current", dMaxCurrent);
    return bNoteDetectedPlusDelay;
  }

  public void resetDetectNote() {
    bMotorStarting = false;
    bNoteDetected = false;
    bNoteDetectedPlusDelay = false;
    dMaxCurrent = 0.0;
  }

}
