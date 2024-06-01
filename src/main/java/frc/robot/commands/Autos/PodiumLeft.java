// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// = = = = = SHOOT MOVE PICKUP MOVE BACK SHOOT = = = = = \\

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.TilterV2;
import frc.robot.subsystems.mechanisms.Feeder;
import frc.robot.subsystems.mechanisms.Shooter;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.commands.FeederRun;
import frc.robot.commands.CmndGroups.IntakeNFeederRun;
import frc.robot.commands.CmndGroups.ManualFeedNShoot;
import frc.robot.commands.CmndGroups.PositionForSpeaker;
import frc.robot.commands.swervedrive.auto.AutoDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.CmndGroups.MoveToHome;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PodiumLeft extends SequentialCommandGroup {
  /** Creates a new AutoTwo. */
  public PodiumLeft(Elevator objElevator, TilterV2 objTilter, SwerveSubsystem objSwerve, Feeder objFeeder, Shooter objShooter, Intake objIntake, boolean bFwd) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PositionForSpeaker(objElevator, objTilter),
      new ManualFeedNShoot(objFeeder, objShooter, 4500.0, 0.3, 0.6).withTimeout(1.0),
      new ParallelCommandGroup(
        new AutoDrive(objSwerve, -0.75, -0.15, 0.35),
        new IntakeNFeederRun(objElevator, objTilter, objFeeder, objIntake, objShooter, bFwd)
      ).withTimeout(3.65),
      new ParallelCommandGroup(
        new IntakeNFeederRun(objElevator, objTilter, objFeeder, objIntake, objShooter, bFwd),
        new AutoDrive(objSwerve, 0.0, 0.0, 0.0)
      ).withTimeout(0.35),
      new ParallelCommandGroup( 
        new AutoDrive(objSwerve, 0.75, 0.0, -0.35),
        new IntakeNFeederRun(objElevator, objTilter, objFeeder, objIntake, objShooter, bFwd).withTimeout(1.0)
      ).withTimeout(3.65),
      new PositionForSpeaker(objElevator, objTilter).withTimeout(3.5),
      new ManualFeedNShoot(objFeeder, objShooter, 4500.0, 0.3, 0.6).withTimeout(1.0),
      new MoveToHome(objElevator, objTilter).withTimeout(1.0)
    );
  }
}
