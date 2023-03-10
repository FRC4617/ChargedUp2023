// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.DriveTime;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static XboxController k_driver = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    k_drivetrain = new Drivetrain();
    k_elevator = new Elevator();

    k_drivetrain.setDefaultCommand(new ArcadeDrive(k_drivetrain));

    Command basic = new DriveTime(k_drivetrain, 0.5, 0).withTimeout(1.5);
    Command advancedMiddle = new SequentialCommandGroup(
        new DriveTime(k_drivetrain, 0.5, 0).withTimeout(1.5), // exit community
        new DriveTime(k_drivetrain, 0.5, 0).withTimeout(1.5) // drive onto docking station
    );
    Command advancedRight = new SequentialCommandGroup(
        new DriveTime(k_drivetrain, 0.5, 0).withTimeout(1.5), // exit community
        new DriveTime(k_drivetrain, 0.5, 0).withTimeout(1.5) // drive onto docking station
    );
    // Command advancedLeft = new SequentialCommandGroup(
    // new DriveTime(k_drivetrain, 0.5, 0).withTimeout(1.5), // exit community
    // new DriveTime(k_drivetrain, 0.5, 0).withTimeout(1.5) // drive onto docking
    // station
    // );
    Command drivePath = new SequentialCommandGroup(
        new InstantCommand(() -> k_drivetrain.zeroAngle()),
        k_drivetrain.getAutonomousCommand());

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Simple Auto", basic);
    m_chooser.addOption("Middle Auto", advancedMiddle);
    m_chooser.addOption("Right Auto", advancedRight);
    m_chooser.addOption("Path", drivePath);

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
