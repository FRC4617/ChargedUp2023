// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.elevator.DriveElevator;
import frc.robot.commands.elevator.DriveElevatorPosition;
import frc.robot.commands.elevator.DriveIntake;
import frc.robot.commands.elevator.DriveIntakeSpeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain k_drivetrain;
  private final Elevator k_elevator;
  private final Intake k_intake;

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static XboxController k_driver = new XboxController(0);
  public static XboxController k_operator = new XboxController(1);

  private final JoystickButton b_resetDriveEncoders;
  private final JoystickButton b_zeroElevator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    k_drivetrain = new Drivetrain();
    k_elevator = new Elevator();
    k_intake = new Intake();

    b_resetDriveEncoders = new JoystickButton(k_driver, XboxController.Button.kB.value);
    b_zeroElevator = new JoystickButton(k_driver, XboxController.Button.kY.value);

    k_drivetrain.setDefaultCommand(new ArcadeDrive(k_drivetrain, () -> k_driver.getYButtonPressed()));
    k_elevator.setDefaultCommand(new DriveElevator(
        k_elevator, () -> {
          return MathUtil.applyDeadband(k_driver.getRightTriggerAxis() - k_driver.getLeftTriggerAxis(), 0.1);
        }, () -> k_operator.getAButton(), () -> k_operator.getBButton()));

    k_intake.setDefaultCommand(new DriveIntake(
        k_intake, () -> {
          return MathUtil.applyDeadband(k_operator.getRightTriggerAxis() - k_operator.getLeftTriggerAxis(), 0.1);
        }, () -> k_operator.getXButton()));

    configureButtonBindings();
    configureAuto();
  }

  private void configureButtonBindings() {
    b_resetDriveEncoders.onTrue(new InstantCommand(() -> k_drivetrain.resetEncoders()));
    b_zeroElevator.onTrue(new InstantCommand(() -> k_elevator.zeroElevator()));
  }

  private void configureAuto() {
    PathPlannerTrajectory topBlue = PathPlanner.loadPath("Blue 1",
        new PathConstraints(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared),
        true);
    PathPlannerTrajectory middleBlue = PathPlanner.loadPath("Blue 2",
        new PathConstraints(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared),
        true);
    PathPlannerTrajectory bottomBlue = PathPlanner.loadPath("Blue 1",
        new PathConstraints(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared),
        true);

    PathPlannerTrajectory topRed = PathPlanner.loadPath("Red 1",
        new PathConstraints(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared),
        true);
    PathPlannerTrajectory middleRed = PathPlanner.loadPath("Red 2",
        new PathConstraints(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared),
        true);
    PathPlannerTrajectory bottomRed = PathPlanner.loadPath("Red 3",
        new PathConstraints(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared),
        true);

    Command driveBottomBlue = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, Constants.DriveConstants.kElevatorHighPosition)).withTimeout(3),
        new DriveIntakeSpeed(k_intake, 1).withTimeout(1),
        new DriveIntakeSpeed(k_intake, 0).withTimeout(0.1),
        new DriveElevatorPosition(k_elevator, 0));
    /*
     * new ParallelCommandGroup(
     * new DriveElevatorPosition(k_elevator, 0),
     * k_drivetrain.followTrajectoryCommand(bottomBlue, true)));
     */
    Command driveMiddleBlue = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, Constants.DriveConstants.kElevatorHighPosition)).withTimeout(3),
        new DriveIntakeSpeed(k_intake, 0.75).withTimeout(1),
        new DriveIntakeSpeed(k_intake, 0).withTimeout(0.1),
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, 0),
            k_drivetrain.followTrajectoryCommand(middleBlue, true)));
    Command driveTopBlue = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, Constants.DriveConstants.kElevatorHighPosition)).withTimeout(3),
        new DriveIntakeSpeed(k_intake, 0.75).withTimeout(1),
        new DriveIntakeSpeed(k_intake, 0).withTimeout(0.1),
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, 0),
            k_drivetrain.followTrajectoryCommand(topBlue, true)));

    Command driveBottomRed = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, Constants.DriveConstants.kElevatorHighPosition)).withTimeout(3),
        new DriveIntakeSpeed(k_intake, 0.75).withTimeout(1),
        new DriveIntakeSpeed(k_intake, 0).withTimeout(0.1),
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, 0),
            k_drivetrain.followTrajectoryCommand(bottomRed, true)));
    Command driveMiddleRed = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, Constants.DriveConstants.kElevatorHighPosition)).withTimeout(3),
        new DriveIntakeSpeed(k_intake, 0.75).withTimeout(1),
        new DriveIntakeSpeed(k_intake, 0).withTimeout(0.1),
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, 0),
            k_drivetrain.followTrajectoryCommand(middleRed, true)));
    Command driveTopRed = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, Constants.DriveConstants.kElevatorHighPosition)).withTimeout(3),
        new DriveIntakeSpeed(k_intake, 0.75).withTimeout(1),
        new DriveIntakeSpeed(k_intake, 0).withTimeout(0.1),
        new ParallelCommandGroup(
            new DriveElevatorPosition(k_elevator, 0),
            k_drivetrain.followTrajectoryCommand(topRed, true)));

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("None", null);
    m_chooser.addOption("Top Blue", driveTopBlue);
    m_chooser.addOption("Middle Blue", driveMiddleBlue);
    m_chooser.addOption("Bottom Blue", driveBottomBlue);
    m_chooser.addOption("Top Red", driveTopRed);
    m_chooser.addOption("Middle Red", driveMiddleRed);
    m_chooser.addOption("Bottom Red", driveBottomRed);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
