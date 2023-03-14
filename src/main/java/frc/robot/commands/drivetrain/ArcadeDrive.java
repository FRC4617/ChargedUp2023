package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {

  public final Drivetrain drivetrain;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.75);
  private final SlewRateLimiter zLimiter = new SlewRateLimiter(0.75);

  public ArcadeDrive(Drivetrain subsystem) {
    drivetrain = subsystem;

    addRequirements(subsystem);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (RobotContainer.k_driver.getBButtonPressed()) {
      drivetrain.resetEncoders();
    }

    double x = -RobotContainer.k_driver.getRawAxis(4);
    double z = -RobotContainer.k_driver.getRawAxis(1);

    x = MathUtil.applyDeadband(x, Constants.kDriverDeadband);
    z = MathUtil.applyDeadband(z, Constants.kDriverDeadband);

    x = xLimiter.calculate(x);
    z = zLimiter.calculate(z);

    drivetrain.arcadeDrive(z, x, true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
