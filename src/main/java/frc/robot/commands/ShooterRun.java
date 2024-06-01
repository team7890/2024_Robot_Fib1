// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.mechanisms.Shooter;

public class ShooterRun extends Command {
  private final Shooter objShooter;
  private final double dSpeed;

  /** Creates a new ShooterRun. */
  public ShooterRun(Shooter objShooter_in, double dSpeedRPM_in) {
    objShooter = objShooter_in;
    dSpeed = dSpeedRPM_in;
    addRequirements(objShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objShooter.resetIntegral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // objShooter.runShooter(dSpeed);
    objShooter.runShooterRPM(dSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objShooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
