// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.mechanisms.Elevator;

import frc.robot.Constants;

public class ElevToPos extends Command {

  private final Elevator objElevator;
  private final double dTargetPos;

  /** Creates a new ElevToPos. */
  public ElevToPos(Elevator objElevator_in, double dTargetPos_in) {

    objElevator = objElevator_in;
    dTargetPos = dTargetPos_in;

    addRequirements(objElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objElevator.runElevator(0.0, dTargetPos, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objElevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(objElevator.getElevatorPos() - dTargetPos) < Constants.Elevator.dTolerance;
  }
}
