// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */

  private final Drivetrain drivetrain;
  private final boolean isReversed;
  private int state;
  private int debounceCount;
  private double robotSpeedSlow;
  private double robotSpeedFast;
  private double robotSpeedBalance;
  private double onChargeStationDegree;
  private double levelDegree;
  private double debounceTime;
  private boolean finished;

  public AutoBalance(Drivetrain subsystem, boolean isReversed) {
    this.drivetrain = subsystem;
    this.isReversed = isReversed;

    addRequirements(subsystem);
  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    debounceCount = 0;
    finished = false;

    /**********
     * CONFIG *
     **********/
    // Speed the robot drived while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.3;

    // Speed the robot drives while balancing itself on the charge station.
    // Should be roughly half the fast speed, to make the robot more accurate,
    // default = 0.2
    robotSpeedSlow = 0.08;

    robotSpeedBalance = 0.055;

    // Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 13.0;

    // Angle where the robot can assume it is level on the charging station
    // Used for exiting the drive forward sequence as well as for auto balancing,
    // default = 6.0
    levelDegree = 10.5;

    // Amount of time a sensor condition needs to be met before changing states in
    // seconds
    // Reduces the impact of sensor noice, but too high can make the auto run
    // slower, default = 0.2
    debounceTime = 0.2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Current State", state);
    SmartDashboard.putNumber("Debounce", debounceCount);
    SmartDashboard.putNumber("Tilt", drivetrain.getTilt());
    switch (state) {
      // drive forwards to approach station, exit when tilt is detected
      case 0:
        if (drivetrain.getTilt() > onChargeStationDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 1;
          debounceCount = 0;
          drivetrain.arcadeDrive(robotSpeedSlow * (isReversed ? -1 : 1), 0, true);
          break;
        }
        drivetrain.arcadeDrive(robotSpeedFast * (isReversed ? -1 : 1), 0, true);
        break;
      // driving up charge station, drive slower, stopping when level
      case 1:
        if (drivetrain.getTilt() < levelDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 2;
          debounceCount = 0;
          drivetrain.arcadeDrive(0, 0, true);
          break;
        }
        drivetrain.arcadeDrive(robotSpeedSlow * (isReversed ? -1 : 1), 0, true);
        break;
      // on charge station, stop motors and wait for end of auto
      case 2:
        if (Math.abs(drivetrain.getTilt()) <= levelDegree / 2) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 4;
          finished = true;
          debounceCount = 0;
          drivetrain.arcadeDrive(0, 0, true);
          break;
        }
        if (drivetrain.getTilt() >= levelDegree) {
          drivetrain.arcadeDrive(robotSpeedBalance, 0, true);
          break;
        } else if (drivetrain.getTilt() <= -levelDegree) {
          drivetrain.arcadeDrive(-robotSpeedBalance, 0, true);
          break;
        }
      case 3:
        drivetrain.arcadeDrive(0, 0, true);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
