package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class DriveElevator extends CommandBase {

    private final Elevator elevator;
    private final DoubleSupplier elevatorSpeed;
    private final DoubleSupplier intakeSpeed;

    public DriveElevator(Elevator subsystem, DoubleSupplier elevatorSpeed, DoubleSupplier intakeSpeed) {
        this.elevator = subsystem;
        this.elevatorSpeed = elevatorSpeed;
        this.intakeSpeed = intakeSpeed;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.elevator.driveElevator(elevatorSpeed.getAsDouble(), intakeSpeed.getAsDouble());
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
