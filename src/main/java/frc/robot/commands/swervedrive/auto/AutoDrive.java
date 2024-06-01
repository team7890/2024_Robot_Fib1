// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoDrive extends Command {
  
  private final SwerveSubsystem objSwerve;
  private final double dTransX;
  private final double dTransY;
  private final double dRotate;

  /** Creates a new AutoDriveToAprilTag. */
  public AutoDrive(SwerveSubsystem objSwerve_in, double dTransX_in, double dTransY_in, double dRotate_in) {
    objSwerve = objSwerve_in;
    dTransX = dTransX_in;
    dTransY = dTransY_in;
    dRotate = dRotate_in;
    addRequirements(objSwerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objSwerve.drive(new Translation2d(dTransX, dTransY), dRotate, true);
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // objSwerve.driveCommand(() -> 0.0, () -> 0.0, () -> 0.0);
    objSwerve.drive(new Translation2d(0.0, 0.0), 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
