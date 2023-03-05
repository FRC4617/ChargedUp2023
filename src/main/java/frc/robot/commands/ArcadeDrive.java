package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {

    public final Drivetrain drivetrain;

    public ArcadeDrive(Drivetrain subsystem) {
        drivetrain = subsystem;

        addRequirements(subsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
      if(RobotContainer.k_driver.getBButtonPressed()){
        drivetrain.resetEncoders();
      }


        double x = RobotContainer.k_driver.getRawAxis(4);
        double y = RobotContainer.k_driver.getRawAxis(1);
    
        if (Math.abs(x) <= 0.05) {
          x = 0;
        }
    
        if (Math.abs(y) <= 0.05) {
          y = 0;
        }
    
        drivetrain.arcadeDrive(y, x, true);

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
