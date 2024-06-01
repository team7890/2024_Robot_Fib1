// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.mechanisms.Elevator;
import java.util.function.DoubleSupplier;

public class ElevMoveAnalog extends Command {

  private final Elevator objElevator;
  private final DoubleSupplier dsSpeed;

  /** Creates a new ElevMoveAnalog. */
  public ElevMoveAnalog(Elevator objElevator_in, DoubleSupplier dsSpeed_in) {
    objElevator = objElevator_in;
    dsSpeed = dsSpeed_in;
    addRequirements(objElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dSpeed = dsSpeed.getAsDouble();
    // if (Math.abs(dSpeed) < 0.15) dSpeed = 0.0;              // if speed is smaller than a deadband value, set it to zero because of joystick
    // method in Elevator subsystem limits speed and applies a deadband so don't need it in this command
    // objElevator.moveElevator(dSpeed);
    objElevator.runElevator(dSpeed, 0.0,false);   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objElevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
