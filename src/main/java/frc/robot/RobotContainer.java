// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// === SUBSYSTEMS ===
import frc.robot.subsystems.mechanisms.Shooter;
import frc.robot.subsystems.mechanisms.ClimberL;
import frc.robot.subsystems.mechanisms.ClimberR;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.Sensor;
// import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.mechanisms.TilterV2;
import frc.robot.subsystems.mechanisms.Feeder;
import frc.robot.subsystems.mechanisms.Intake;

// === COMMANDS ===
// import frc.robot.commands.Shoot;
import frc.robot.commands.ElevToPos;
import frc.robot.commands.swervedrive.auto.AutoBalanceCommand;
import frc.robot.commands.swervedrive.auto.AutoDriveToAprilTag;
import frc.robot.commands.swervedrive.auto.AutoDriveAndKeepAimingToAprilTag;
import frc.robot.commands.FeederRun;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ClimbLMove;
import frc.robot.commands.ClimbRMove;
import frc.robot.commands.ElevMoveAnalog;
import frc.robot.commands.TiltMoveAnalog;
import frc.robot.commands.TiltToAngle;
import frc.robot.commands.IntakeUnjam;
import frc.robot.commands.ShooterRun;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.swervedrive.drivebase.TeleOpDriveSlow;

// === COMMAND GROUPS ===
import frc.robot.commands.CmndGroups.ManualFeedNShoot;
import frc.robot.commands.CmndGroups.MoveToHome;
import frc.robot.commands.CmndGroups.IntakeNFeederRun;
import frc.robot.commands.CmndGroups.PositionForAmp;
import frc.robot.commands.CmndGroups.PositionForChainShoot;
import frc.robot.commands.CmndGroups.PositionForDefensePodium;
import frc.robot.commands.CmndGroups.PositionForPodium;
import frc.robot.commands.CmndGroups.PositionForSpeaker;
import frc.robot.commands.CmndGroups.SourceFeedSpeaker;
import frc.robot.commands.CmndGroups.PositionForOverStage;
// === AUTO COMMANDS ===
import frc.robot.commands.Autos.SingleStraight;
import frc.robot.commands.Autos.SingleStraightDelay;
import frc.robot.commands.Autos.DoubleAmpLeft;
import frc.robot.commands.Autos.DoubleAmpRight;
import frc.robot.commands.Autos.DoubleCenter;
import frc.robot.commands.Autos.PodiumLeft;
import frc.robot.commands.Autos.SingleLeft;
import frc.robot.commands.Autos.SingleRight;
import frc.robot.commands.Autos.PodiumRight;
import frc.robot.commands.Autos.ShootStay;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


  // === SUBSYSTEMS ===
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  // private final Limelight objLimeLight = new Limelight();
  private final Shooter objShooter = new Shooter();
  // private final Sensor objSensor = new Sensor();
  private final Elevator objElevator = new Elevator();
  private final TilterV2 objTilter = new TilterV2();
  private final Feeder objFeeder = new Feeder();
  private final Intake objIntake = new Intake();
  // private final ClimberL objClimberL = new ClimberL();
  // private final ClimberR objClimberR = new ClimberR();


  // === JOYSTICK STUFF ===
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController objDriverXbox =
      new CommandXboxController(OperatorConstants.iDriverPort);

  private final CommandXboxController objCoPilotXbox =
    new CommandXboxController(OperatorConstants.iCoPilotPort);
 
  private final Joystick objButtonBox = new Joystick(OperatorConstants.iButtonPort);

  JoystickButton Button1 = new JoystickButton(objButtonBox, 1);
  JoystickButton Button2 = new JoystickButton(objButtonBox, 2);
  JoystickButton Button3 = new JoystickButton(objButtonBox,3);
  JoystickButton Button4 = new JoystickButton(objButtonBox,4);
  

    // === AUTO STUFF ===
    SendableChooser<Command> objChooser = new SendableChooser<>();

    private final SequentialCommandGroup objSingleStraight = new SingleStraight(objElevator, objTilter, drivebase, objFeeder, objShooter);
    private final SequentialCommandGroup objDualCenter = new DoubleCenter(objElevator, objTilter, drivebase, objFeeder, objShooter, objIntake, true);
    private final SequentialCommandGroup objSingleRight = new SingleRight(objElevator, objTilter, drivebase, objFeeder, objShooter);
    private final SequentialCommandGroup objSingleLeft = new SingleLeft(objElevator, objTilter, drivebase, objFeeder, objShooter);
    private final SequentialCommandGroup objDoublePodiumRight = new PodiumRight(objElevator, objTilter, drivebase, objFeeder, objShooter, objIntake, false);
    private final SequentialCommandGroup objDoublePodiumLeft = new PodiumLeft(objElevator, objTilter, drivebase, objFeeder, objShooter, objIntake, false);
    private final SequentialCommandGroup objDoubleAmpBlue = new DoubleAmpLeft(objElevator, objTilter, drivebase, objFeeder, objShooter, objIntake, false);
    private final SequentialCommandGroup objDoubleAmpRed = new DoubleAmpRight(objElevator, objTilter, drivebase, objFeeder, objShooter, objIntake, false);
    private final SequentialCommandGroup objDelaySingleStraight = new SingleStraightDelay(objElevator, objTilter, drivebase, objFeeder, objShooter);
    private final SequentialCommandGroup objShootStay = new ShootStay(objElevator, objTilter, drivebase, objFeeder, objShooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
     AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(objDriverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(objDriverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(objDriverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   objDriverXbox.y(),
                                                                   objDriverXbox.a(),
                                                                   objDriverXbox.x(),
                                                                   objDriverXbox.b());
     

// m_driverController::getYButtonPressed,
// m_driverController::getAButtonPressed,
// m_driverController::getXButtonPressed,
// m_driverController::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(objDriverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(objDriverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> objDriverXbox.getRightX(),
        () -> objDriverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(objDriverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(objDriverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> objDriverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(objDriverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(objDriverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> objDriverXbox.getRawAxis(2));


    // drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    objElevator.setDefaultCommand(new ElevMoveAnalog(objElevator, () -> objCoPilotXbox.getLeftY()));
    objTilter.setDefaultCommand(new TiltMoveAnalog(objTilter, () -> objCoPilotXbox.getRightY()));

    // === SENDABLE CHOOSER FOR AUTOS ===
    objChooser.setDefaultOption("Single Straight", objSingleStraight);
    objChooser.addOption("Single Stay", objShootStay);
    objChooser.addOption("Single Straight Delay", objDelaySingleStraight);
    objChooser.addOption("Single Left", objSingleLeft);
    objChooser.addOption("Single Right", objSingleRight);
    objChooser.addOption("Double Podium Red", objDoublePodiumLeft);
    objChooser.addOption("Double Podium Blue", objDoublePodiumRight);
    objChooser.addOption("Dual Center", objDualCenter);
    objChooser.addOption("Double Amp Blue", objDoubleAmpBlue);
    objChooser.addOption("Double Amp Red", objDoubleAmpRed);
    
    // objChooser.addOption("New Option", objShootAndMove);
    Shuffleboard.getTab("7890 Autos").add(objChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // === DRIVER BUTTONS ===
    // objDriverXbox.a().whileTrue(new ManualFeedNShoot(objFeeder, objShooter, 1.0, 0.5, 0.7));
    objDriverXbox.axisGreaterThan(3, 0.5).whileTrue(new ManualFeedNShoot(objFeeder, objShooter, 4500.0, 0.3, 0.6)); // axis 3 = right trigger
    objDriverXbox.axisGreaterThan(2, 0.5).whileTrue(new ManualFeedNShoot(objFeeder, objShooter, 1000.0, 0.3, 0.6)); // axis 2 = left trigger
    // objDriverXbox.b().whileTrue(new ManualFeedNShoot(objFeeder, objShooter, 0.25, 0.25, 0.5));
    objDriverXbox.y().whileTrue(new ManualFeedNShoot(objFeeder, objShooter, 6000.0, 0.3, 0.6));
    objDriverXbox.leftBumper().whileTrue(new TeleOpDriveSlow(drivebase, 
      () -> MathUtil.applyDeadband(objDriverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(objDriverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
      () -> objDriverXbox.getRightX()));


    objDriverXbox.start().onTrue((new InstantCommand(drivebase::zeroGyro)));
    // objDriverXbox.x().whileTrue(new AutoDriveToAprilTag(drivebase, objLimeLight));
    // objDriverXbox.rightBumper().whileTrue(new AutoDriveAndKeepAimingToAprilTag(drivebase, objLimeLight,
    //     () -> MathUtil.applyDeadband(objDriverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> MathUtil.applyDeadband(objDriverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> objDriverXbox.getRightX()));

         // right trigger
    // objDriverXbox.axisGreaterThan(2, 0.5).whileTrue(new IntakeUnjam(objIntake, false));     // left trigger
    // objDriverXbox.y().whileTrue(new ElevToPos(objElevator, 67.0));

    // === COPILOT BUTTONS ===

    // objCoPilotXbox.y().whileTrue(new IntakeRun(objIntake, true));
    // objCoPilotXbox.back().whileTrue(new IntakeNFeederRun(objElevator, objTilter, objFeeder, objIntake, objShooter, false));       //rvs
    
    objCoPilotXbox.axisGreaterThan(3, 0.5).whileTrue(new MoveToHome(objElevator, objTilter));     // Axis 3 = Right Trigger
    objCoPilotXbox.rightBumper().whileTrue(new FeederRun(objFeeder, -0.05));    // use to nudge note back
    objCoPilotXbox.leftBumper().whileFalse(new ZeroElevator(objElevator));
    objCoPilotXbox.axisGreaterThan(2, 0.5).whileTrue(new IntakeNFeederRun(objElevator, objTilter, objFeeder, objIntake, objShooter, true));

    objCoPilotXbox.a().whileTrue(new PositionForSpeaker(objElevator, objTilter));
    objCoPilotXbox.b().whileTrue(new PositionForAmp(objElevator, objTilter));
    objCoPilotXbox.x().whileTrue(new IntakeUnjam(objIntake, true));
    objCoPilotXbox.y().whileTrue(new PositionForOverStage(objElevator, objTilter));

    objCoPilotXbox.start().whileTrue(new SourceFeedSpeaker(objElevator, objTilter, objFeeder, objIntake, objShooter, false));
    // objCoPilotXbox.y().whileTrue(new PositionForPodium(objElevator, objTilter));
    // objCoPilotXbox.x().whileTrue(new PositionForChainShoot(objElevator, objTilter));

    // objCoPilotXbox.a().whileTrue(new TiltToAngle(objTilter, 21.0, false));  // for testing
    // objCoPilotXbox.b().whileTrue(new TiltToAngle(objTilter, 55.0, false));  // for testing
    // objCoPilotXbox.y().whileTrue(new FeederRun(objFeeder, 0.25));                                 // for testing
    // objCoPilotXbox.y().whileTrue(new ShooterRun(objShooter, 2000.0));
    // objCoPilotXbox.x().whileTrue(new ElevToPos(objElevator, 20.0));

    

    // objCoPilotXbox.axisGreaterThan(2, 0.5).whileTrue(new IntakeNFeederRun(objElevator, objTilter, objFeeder, objIntake, objShooter, true));     // Axis 2 = Left Trigger


    // === BUTTON BOX ===

    // Button1.whileTrue(new PositionForAmp(objElevator, objTilter));
    // Button3.whileTrue(new PositionForSpeaker(objElevator, objTilter));
    // Button2.whileTrue(new PositionForOverStage(objElevator, objTilter));
    // Button4.whileTrue(new PositionForDefensePodium(objElevator, objTilter));



    // m_driverController.y().whileTrue(new FeederRun(objFeeder, 0.75));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`)
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return objChooser.getSelected();
  }
}
