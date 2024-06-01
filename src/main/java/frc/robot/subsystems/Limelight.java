// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {

  NetworkTable table;
  NetworkTableEntry tx, ty, ta;
  double dX, dY, dArea;

  /** Creates a new Limelight. */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  @Override
  public void periodic() {

    //read values periodically
    dX = tx.getDouble(0.0);
    dY = ty.getDouble(0.0);
    dArea = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", dX);
    SmartDashboard.putNumber("LimelightY", dY);
    SmartDashboard.putNumber("LimelightArea", dArea);

    // This method will be called once per scheduler run
  }

  public double controlDriveRotation(){
    double dControl;
    double dDelta = Math.abs(dX);
    if (dDelta > 5.0) {
      dControl = dX * 0.5 / 30.0 + Math.signum(dX) * 0.25;
    }
    else if (dDelta > 0.5) {
      dControl = dX * 0.4 / 30.0 + Math.signum(dX) * 0.1;
    }
    else {
      dControl = 0.0;
    }
    return dControl;
  }

  public double getTargetAngle() {
    return dX;
  }

  public double getArea(){
    return dArea;
  }

}
