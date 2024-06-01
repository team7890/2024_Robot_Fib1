// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int iDriverPort = 0;
    public static final int iCoPilotPort = 1;
    public static final int iButtonPort = 2;
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class CANids{
    public static final int iLowRoller = 21;        // Neo
    public static final int iHighRoller = 22;       // Neo
    public static final int iElevator = 23;         // Kraken
    public static final int iFeeder = 24;           // Neo
    public static final int iShooter = 25;          // Neo
    public static final int iShooterTilter = 26;    // Neo
    public static final int iClimbLeft = 27;        // Kraken
    public static final int iClimbRight = 28;       // Kraken

    // TODO: Add Drivabase CANids
    // Front Left Drive CANID 12
    // Front Left Steer CANID 17
    // Front Right Drive CANDID 11
    // Front Right Steer CANID 10
    // Back Left Drive CANID 14
    // Back Left Steer CANID 16
    // Back Right Drive CANID 13
    // Back Right Steer CANID 15
    // Pigeon CANID 20
  }
  public static class Elevator {
    public static final double dTolerance = 0.3;
    public static final double dSpeedMax = 0.3;
    public static final double dHomePositon = 0.0;
    public static final double dAmpPosition = 50.0;
    public static final double dSpeakerPosition = 0.0;
    public static final double dChainPosition = 0.0;
    public static final double dPodiumPosition = 0.0;
    public static final double dOverStagePosition = 0.0;
    public static final double dDefensePodiumPosition = 83.0;
    public static final double dSourcePositon = 50.0;
  }

  public static class Tilter{
    public static final double dTolerance = 0.3;
    public static final double dMinAngle = 1.5;    // don't let tilter tilt below this position
    public static final double dMaxAngle = 123.0;
    public static final double dHomeAngle = 20.0;
    // public static final double dAmpAngle = 100.0;
    public static final double dAmpAngle = 140.0;
    public static final double dSpeakerAngle = 40.0;
    public static final double dChainAngle = 65.0;
    public static final double dPodiumAngle = 59.0;
    public static final double dOverStageAngle = 50.0;
    public static final double dDefencePodiumAngle = 65.0;
    public static final double dSourceAngle = 30.0;;
  }

  public static class ManualFeedNShoot{
    public static final double dSpeed = 0.5;

  }

}