// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensor extends SubsystemBase {

  private DigitalInput objSensor = new DigitalInput(1);

  /** Creates a new Sensor. */
  public Sensor() {}

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Note Sensor", objSensor.get());

    // This method will be called once per scheduler run
  }

  public boolean getSensor() {
    return objSensor.get();
  }

}
