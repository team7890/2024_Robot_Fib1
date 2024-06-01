// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.TilterV2;
import frc.robot.subsystems.mechanisms.Feeder;
import frc.robot.subsystems.mechanisms.Shooter;

import frc.robot.commands.CmndGroups.ManualFeedNShoot;
import frc.robot.commands.CmndGroups.PositionForSpeaker;
import frc.robot.commands.swervedrive.auto.AutoDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.CmndGroups.MoveToHome;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SingleStraightDelay extends SequentialCommandGroup {
  /** Creates a new ShootAndMove. */
  public SingleStraightDelay(Elevator objElevator, TilterV2 objTilter, SwerveSubsystem objSwerve, Feeder objFeeder, Shooter objShooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PositionForSpeaker(objElevator, objTilter).withTimeout(5.0),
      new ManualFeedNShoot(objFeeder, objShooter, 4500.0, 0.3, 0.6).withTimeout(1.0),
      new WaitCommand(8.5),
      new ParallelCommandGroup(
        new AutoDrive(objSwerve, -0.6, 0.0, 0.0),    // negative x move goes away from shoot direction
        new MoveToHome(objElevator, objTilter)
      ).withTimeout(5.0)
    );
  }
}
