// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.DoubleSupplier;


public class AutoDriveAndKeepAimingToAprilTag extends Command {

  private final SwerveSubsystem objSwerve;
  private final Limelight objLimelight;
  private final DoubleSupplier dsTransX;
  private final DoubleSupplier dsTransY;
  private final DoubleSupplier dsAngleRotate;

  /** Creates a new AutoDriveAndKeepAimingToAprilTag. */
  public AutoDriveAndKeepAimingToAprilTag(SwerveSubsystem objSwerve_in, Limelight objLimelight_in, DoubleSupplier dsTransX_in, DoubleSupplier dsTransY_in, DoubleSupplier dsAngleRotate_in) {
    objSwerve = objSwerve_in;
    objLimelight = objLimelight_in;
    dsTransX = dsTransX_in;
    dsTransY = dsTransY_in;
    dsAngleRotate = dsAngleRotate_in;

    addRequirements(objSwerve, objLimelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Max Vel:  " + objSwerve.getMaxVelHack());
    // System.out.println("Max Ang Vel:  " + objSwerve.getMaxAngVelHack());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dTransX = Math.pow(dsTransX.getAsDouble(), 3) * objSwerve.getMaxVelHack();
    double dTransY = Math.pow(dsTransY.getAsDouble(), 3) * objSwerve.getMaxVelHack();
    double dAngleRotate = Math.pow(dsAngleRotate.getAsDouble(), 3) * objSwerve.getMaxAngVelHack();
    if (objLimelight.getArea() > 0.0){
      objSwerve.drive(new Translation2d(dTransX, dTransY), objLimelight.controlDriveRotation(), false);
    }
    else {
      objSwerve.drive(new Translation2d(dTransX, dTransY), dAngleRotate, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objSwerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
