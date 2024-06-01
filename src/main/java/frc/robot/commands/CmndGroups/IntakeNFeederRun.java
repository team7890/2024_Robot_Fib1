// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CmndGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.TilterV2;
import frc.robot.subsystems.mechanisms.Feeder;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.Shooter;

import frc.robot.commands.ElevToPos;
import frc.robot.commands.TiltToAngle;
import frc.robot.commands.FeederRun;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.ShooterRun;

import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNFeederRun extends SequentialCommandGroup {
  /** Creates a new IntakeNFeederRun. */
  public IntakeNFeederRun(Elevator objElevator, TilterV2 objTilter, Feeder objFeeder, Intake objIntake, Shooter objShooter, boolean bFwd) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ElevToPos(objElevator, Constants.Elevator.dHomePositon),
        new TiltToAngle(objTilter, Constants.Tilter.dHomeAngle, true)
      ),
      new ParallelRaceGroup(
        new IntakeRun(objIntake, bFwd),
        new FeederRun(objFeeder, 0.06),
        new ShooterRun(objShooter, -500.0)
      )
      // new FeederRun(objFeeder, -0.1).withTimeout(0.2)
    );
  }
}
