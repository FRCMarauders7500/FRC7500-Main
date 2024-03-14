// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManualLift;
import frc.robot.commands.ManualWrist;
import frc.robot.commands.WristToAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Wrist wrist;
  private final Shooter shooter;
  private final Lift lift;

  private Trigger intakeTrig, scoreTrig; // Take out last trigger later
  private InstantCommand suck, spit;
  private InstantCommand shoot;
  private InstantCommand shootStop;
  private InstantCommand intakeStop1, intakeStop2;
  private Command manualWrist, wristIntake, wristScore;
  private Command manualLift;

  // Controller
  private final CommandXboxController dController, mController;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    dController = new CommandXboxController(0);
    mController = new CommandXboxController(1);

    lift = new Lift();
    lift.register();
    wrist = new Wrist();
    wrist.register();
    intake = new Intake();
    intake.register();
    shooter = new Shooter();
    shooter.register();

    NamedCommands.registerCommand("spit", new RunCommand(() -> intake.setPower(-1), intake));
    NamedCommands.registerCommand(
        "shoot",
        new RunCommand(shooter::shoot, shooter)
            .withTimeout(3)
            .andThen(new InstantCommand(shooter::stop)));

    suck = new InstantCommand(() -> intake.setPower(-.8), intake);
    spit = new InstantCommand(() -> intake.setPower(-1), intake);
    shoot = new InstantCommand(shooter::shoot, shooter);
    intakeStop1 = new InstantCommand(intake::stop);
    intakeStop2 = new InstantCommand(intake::stop);
    shootStop = new InstantCommand(shooter::stop);
    manualWrist = new ManualWrist(wrist, mController::getLeftY);
    wristIntake = new WristToAngle(wrist, -152);
    wristScore = new WristToAngle(wrist, -20);
    manualLift = new ManualLift(lift, mController::getRightY);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));

        // drive = new Drive(
        // new GyroIOPigeon2(true),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        break;
    }

    // Set up auto routines

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("shoot", testAuto());
    // Configure the button bindings
    configureButtonBindings();

    intakeTrig
        .whileTrue(
            new ParallelCommandGroup(
                wristIntake.withInterruptBehavior(InterruptionBehavior.kCancelSelf), suck))
        .onFalse(
            new ParallelCommandGroup(
                wristScore.withInterruptBehavior(InterruptionBehavior.kCancelSelf), intakeStop2));
    scoreTrig
        .whileTrue(
            new ParallelCommandGroup(shoot, new SequentialCommandGroup(new WaitCommand(3), spit)))
        .onFalse(new ParallelCommandGroup(shootStop, intakeStop1));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    intakeTrig = mController.x();
    scoreTrig = mController.b();

    wrist.setDefaultCommand(manualWrist);
    lift.setDefaultCommand(manualLift);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> dController.getLeftY(),
            () -> dController.getLeftX(),
            () -> dController.getRightX()));
    dController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    dController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    dController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                new Rotation2d(Math.toRadians(180)))),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private SequentialCommandGroup testAuto() {
    return new SequentialCommandGroup(new InstantCommand(shooter::shoot));
  }
}
