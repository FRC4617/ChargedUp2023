// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class DriveIntake extends CommandBase {

  private final Intake intake;
  private final DoubleSupplier intakeSpeed;
  private final BooleanSupplier outputButton;

  /** Creates a new DriveIntake. */
  public DriveIntake(Intake intake, DoubleSupplier intakeSpeed, BooleanSupplier outputButton) {

    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    this.outputButton = outputButton;

    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tempSpeed = intakeSpeed.getAsDouble();
    if (Math.abs(tempSpeed) < 0.15)
      tempSpeed = -0.15;
    if (outputButton.getAsBoolean())
      tempSpeed = 0.35;
    intake.setIntakeSpeed(tempSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
