package frc.robot.commands.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class DriveElevator extends CommandBase {

    private final Elevator elevator;
    private final DoubleSupplier elevatorSpeed;
    private BooleanSupplier elevatorUp;
    private BooleanSupplier elevatorDown;
    DigitalInput limitSwitch = new DigitalInput(0);

    public DriveElevator(Elevator elevator, DoubleSupplier elevatorSpeed, BooleanSupplier elevatorUp,
            BooleanSupplier elevatorDown) {
        this.elevator = elevator;
        this.elevatorSpeed = elevatorSpeed;
        this.elevatorUp = elevatorUp;
        this.elevatorDown = elevatorDown;
        addRequirements(this.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double tempSpeed = elevatorSpeed.getAsDouble();
        if (limitSwitch.get() == true) // switch will be on
            tempSpeed = 0;
        else if (elevatorDown.getAsBoolean() && limitSwitch.get() == false)// switch is NOT on
            tempSpeed = 0.80;
        else if (elevatorUp.getAsBoolean())
            tempSpeed = -0.80;

        elevator.setElevatorSpeed(tempSpeed);

        // this.elevator.driveElevator(elevatorSpeed.getAsDouble());

        // if (elevatorSpeed.getAsDouble() > 0)
        // this.elevator.driveElevator(elevatorSpeed.getAsDouble());
        // else {
        // if (scorePosition.getAsBoolean())
        // this.elevator.setElevatorPosition(Constants.DriveConstants.kElevatorHighPosition);
        // else
        // this.elevator.setElevatorPosition(0);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.elevator.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
