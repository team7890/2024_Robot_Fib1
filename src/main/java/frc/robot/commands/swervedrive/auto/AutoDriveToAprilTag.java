// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoDriveToAprilTag extends Command {
  
  private final SwerveSubsystem objSwerve;
  private final Limelight objLimelight;

  /** Creates a new AutoDriveToAprilTag. */
  public AutoDriveToAprilTag(SwerveSubsystem objSwerve_in, Limelight objLimelight_in) {
    objSwerve = objSwerve_in;
    objLimelight = objLimelight_in;

    addRequirements(objSwerve, objLimelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (objLimelight.getArea() > 0.0){
      // objSwerve.driveCommand(() -> 0.0, () -> 0.0, () -> objLimelight.getDriveAngle());
      objSwerve.drive(new Translation2d(0.0, 0.0), objLimelight.controlDriveRotation(), false);
      // System.out.println("Aim Shoot:  " + objLimelight.getTargetAngle());
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // objSwerve.driveCommand(() -> 0.0, () -> 0.0, () -> 0.0);
    objSwerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(objLimelight.getTargetAngle()) < 1.0;
    // return false;
  }
}
