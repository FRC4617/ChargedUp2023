package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private final CANSparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;

    public Elevator() {
        elevatorMotor = new CANSparkMax(Constants.DriveConstants.kElevatorMotor, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(40);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(true);
        elevatorEncoder.setPosition(0);
    }

    public void driveElevator(double elevatorSpeed) {
        setElevatorSpeed(elevatorSpeed);
    }

    public void setElevatorSpeed(double speed) {
        elevatorMotor.set(speed * 0.5);
    }

    public void stop() {
        setElevatorSpeed(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity());

    }

}
