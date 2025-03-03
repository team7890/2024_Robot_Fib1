// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Tilter;

public class TiltUp extends Command {
  private final Tilter objTilter;

  /** Creates a new TiltUp. */
  public TiltUp(Tilter objTilter_in) {
    // Use addRequirements() here to declare subsystem dependencies.
    objTilter = objTilter_in;
    addRequirements(objTilter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objTilter.cancelHold();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objTilter.runTilter(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objTilter.stopTilter();
    objTilter.startHold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
