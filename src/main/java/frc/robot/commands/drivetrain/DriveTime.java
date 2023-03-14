package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class DriveTime extends CommandBase{
    private final Drivetrain drivetrain;
  private final double x, y;


public DriveTime(Drivetrain subsystem, double y, double x) {
    addRequirements(subsystem);
    this.drivetrain = subsystem;
    this.y = y;
    this.x = x;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.drivetrain.arcadeDrive(y, x, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


