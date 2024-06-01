// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.mechanisms.Feeder;

public class FeederRun extends Command {

  private final Feeder objFeeder;
  private final double dSpeed;

  /** Creates a new FeederRun. */
  public FeederRun(Feeder objFeeder_in, double dSpeed_in) {
    objFeeder = objFeeder_in;
    dSpeed = dSpeed_in;
    addRequirements(objFeeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objFeeder.resetDetectNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objFeeder.runFeeder(dSpeed); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objFeeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return objFeeder.detectNote();
    return false;
  }
}
