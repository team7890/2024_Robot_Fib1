// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class TeleOpDriveSlow extends Command {
  
  private final SwerveSubsystem objSwerve;
  private final DoubleSupplier dsTransX;
  private final DoubleSupplier dsTransY;
  private final DoubleSupplier dsRotate;


  /** Creates a new AutoDriveToAprilTag. */
  public TeleOpDriveSlow(SwerveSubsystem objSwerve_in, DoubleSupplier dsTransX_in, DoubleSupplier dsTransY_in, DoubleSupplier dsRotate_in) {
    objSwerve = objSwerve_in;
    dsTransX = dsTransX_in;
    dsTransY = dsTransY_in;
    dsRotate = dsRotate_in;
    addRequirements(objSwerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dTransX = Math.pow(dsTransX.getAsDouble(), 3) * objSwerve.getMaxVelHack();
    double dTransY = Math.pow(dsTransY.getAsDouble(), 3) * objSwerve.getMaxVelHack();
    double dRotate = Math.pow(dsRotate.getAsDouble(), 3) * objSwerve.getMaxAngVelHack();

    objSwerve.drive(new Translation2d(dTransY * 0.25, dTransX * 0.25), dRotate * 0.25, true);
    
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
