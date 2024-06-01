// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.mechanisms.TilterV2;
import java.util.function.DoubleSupplier;
import frc.robot.Constants;

public class TiltMoveAnalog extends Command {

  private final TilterV2 objTilter;
  private final DoubleSupplier dsSpeed;
  private boolean bHold;
  private double dHoldPos;

  /** Creates a new TiltMoveAnalog. */
  public TiltMoveAnalog(TilterV2 objTilter_in, DoubleSupplier dsSpeed_in) {
    objTilter = objTilter_in;
    dsSpeed = dsSpeed_in;
    addRequirements(objTilter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // dHoldPos = objTilter.getTilterPosition();
    // bHold = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dSpeed = dsSpeed.getAsDouble();                 // invert because joystick up is negative
    // if (Math.abs(dSpeed) < 0.15) {                          // if speed is smaller than a deadband value, set it to zero because of joystick
    //   if (!bHold) {
    //     bHold = true;
    //     dHoldPos = objTilter.getTilterPosition();
    //     System.out.println("TiltMoveAnalog Position at Execute:  " + dHoldPos);
    //   }
    //   objTilter.drivePos(dHoldPos);
    // }
    // else {
    //   if (dSpeed > 0.0) dSpeed = dSpeed - 0.15;   // subtract deadband value so tilter can move slowly  (move value closer to 0.0)
    //   if (dSpeed < 0.0) dSpeed = dSpeed + 0.15;   // add deadband to move value closer to 0.0
    //   bHold = false;
    //   if (objTilter.getTilterPosition() < Constants.Tilter.dMinPosition & dSpeed < 0.0) dSpeed = 0.0;
    //   objTilter.runTilter(-dSpeed);
    // }
    // objTilter.runTilter2(dSpeed);
    objTilter.runTilter(dSpeed, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // bHold = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
