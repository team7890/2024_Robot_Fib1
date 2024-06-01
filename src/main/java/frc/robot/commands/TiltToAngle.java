// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.TilterV2;

import frc.robot.Constants;

public class TiltToAngle extends Command {
  /** Creates a new TiltToAngle. */

  private final TilterV2 objTilter;
  private final double dTargetAngle;
  private final boolean bHome;

  public TiltToAngle(TilterV2 objTilter_in, double dTargetAngle_in, boolean bHome_in) {
    objTilter = objTilter_in;
    dTargetAngle = dTargetAngle_in;
    bHome = bHome_in;
    addRequirements(objTilter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // objTilter.cancelHold();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objTilter.runTilter(0.0, dTargetAngle, true);
    // objTilter.runTilterToPosition(dTargetAngle);
    // objTilter.drivePos(dTargetAngle);
  }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
  objTilter.stopTilter();
  // objTilter.startHold();
 }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean bReturn;
    if (bHome) {
      // handle case where tilter is going home and position could be below home minus tolerance so need simple less than condition
      bReturn = objTilter.getTilterPosition() < Constants.Tilter.dHomeAngle + Constants.Tilter.dTolerance;
    }
    else {
      // for all other situations, get to within tolerance of angle
      bReturn = Math.abs(objTilter.getTilterPosition() - dTargetAngle) < Constants.Tilter.dTolerance;
    }
    return bReturn;
  }
}
