// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.ClimberR;

public class ClimbRMove extends Command {

  private final ClimberR objClimber;
  private final double dSpeed;

  /** Creates a new ClimbLMove. */
  public ClimbRMove(ClimberR objClimberR_in, double dSpeed_in) {
    objClimber = objClimberR_in;
    dSpeed = dSpeed_in;
    addRequirements(objClimber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Climb R Move:  " + dSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objClimber.moveClimber(dSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objClimber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
