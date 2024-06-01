// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CmndGroups;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// import frc.robot.commands.TiltToAngle;
import frc.robot.commands.FeederRun;
import frc.robot.commands.ShooterRun;
// import frc.robot.commands.ShooterAccel;

import frc.robot.subsystems.mechanisms.Shooter;
import frc.robot.subsystems.mechanisms.Feeder;

public class ManualFeedNShoot extends SequentialCommandGroup {
  public ManualFeedNShoot(Feeder objFeeder, Shooter objShooter, double dShootSpeed, double dFeedSpeed, double dDelay){
    addCommands(
      new ParallelCommandGroup(
        new FeederRun(objFeeder, -0.05),
        new ShooterRun(objShooter, -500.0)
      ).withTimeout(0.15),
      new ShooterRun(objShooter, dShootSpeed).withTimeout(dDelay),
      new ParallelCommandGroup(
        new ShooterRun(objShooter, dShootSpeed),
        new FeederRun(objFeeder, dFeedSpeed)
      )
    );
  }
 
  /** Creates a new FeedNShoot. */
  // public FeedNShoot() {
  //   // Use addRequirements() here to declare subsystem dependencies.
  // }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {}

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
