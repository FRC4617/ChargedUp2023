package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class ElevatorTest extends CommandBase {
    private final Elevator elevator;
    //private final DoubleSupplier elevatorSpeed;
    //private final DoubleSupplier intakeSpeed;
  
    public ElevatorTest(Elevator subsystem) {
        elevator = subsystem;
    
        addRequirements(subsystem);
    
      }

 @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (RobotContainer.k_operator.getAButtonPressed()) {
        elevator.setIntakeSpeed(-0.5);
    }
    if (RobotContainer.k_operator.getYButtonPressed()) {
        elevator.setIntakeSpeed(0.5);
    }

    elevator.setElevatorSpeed(RobotContainer.k_operator.getRawAxis(1));

}

@Override
public void end(boolean interrupted) {
    elevator.driveElevator(0, 0);
}

@Override
public boolean isFinished() {
  return false;
}

}


