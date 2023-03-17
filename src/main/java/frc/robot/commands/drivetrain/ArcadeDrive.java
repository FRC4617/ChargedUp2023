package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {

  public final Drivetrain drivetrain;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.3);
  private final SlewRateLimiter zLimiter = new SlewRateLimiter(0.8);

  private final BooleanSupplier endgame;

  private boolean endgameActive = false;
  private final double kMaxSpeedEndgame = 0.5;

  public ArcadeDrive(Drivetrain subsystem,
      BooleanSupplier endgame) {
    this.drivetrain = subsystem;
    this.endgame = endgame;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double maxSpeed = 1;

    // If the endgame button is pressed, limit the speed of the robot
    if (endgame.getAsBoolean()) {
      if (!endgameActive) {
        maxSpeed = kMaxSpeedEndgame;
        endgameActive = true;
      }
    } else {
      if (endgameActive) {
        endgameActive = false;
      }
    }

    double x = -RobotContainer.k_driver.getRawAxis(4);
    double z = -RobotContainer.k_driver.getRawAxis(1);

    x = MathUtil.applyDeadband(x, Constants.kDriverDeadband);
    z = MathUtil.applyDeadband(z, Constants.kDriverDeadband);

    // Apply the max speed
    x *= maxSpeed;
    z *= maxSpeed;

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
