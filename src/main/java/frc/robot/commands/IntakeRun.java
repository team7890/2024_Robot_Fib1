// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Intake;

public class IntakeRun extends Command {

  private final Intake objIntake;
  // private final double dSpeed;
  private final boolean bFwd;        //bMode true = Intake while Robot Forward, bMode false = Intake while Robot Reverse

  /** Creates a new RunUpperIntake. */
  // public RunIntake(Intake objIntake_in, double dSpeed_in, boolean bMode_in) {
    public IntakeRun(Intake objIntake_in, boolean bFwd_in) {
    objIntake = objIntake_in;
    // dSpeed = dSpeed_in;
    bFwd = bFwd_in;

    addRequirements(objIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objIntake.resetDetectNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (bFwd) {
      objIntake.intakeFwd();
    } 
      else {
      objIntake.intakeRvs();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return objIntake.detectNote();
    return false;
  }
}
