// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CmndGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevToPos;
import frc.robot.commands.TiltToAngle;

import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.TilterV2;

import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionForDefensePodium extends SequentialCommandGroup {
  /** Creates a new PositionForChainShoot. */
  public PositionForDefensePodium(Elevator objElevator, TilterV2 objTilter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(  
        new ElevToPos(objElevator, Constants.Elevator.dDefensePodiumPosition),
        new TiltToAngle(objTilter, Constants.Tilter.dDefencePodiumAngle, false)
      )
    );
  }
}