package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private final CANSparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final SparkMaxPIDController pidController;
    private final DigitalInput limitSwitch;

    public Elevator() {
        elevatorMotor = new CANSparkMax(Constants.DriveConstants.kElevatorMotor, MotorType.kBrushless);
        limitSwitch = new DigitalInput(0);

        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(40);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(true);

        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPosition(0);

        pidController = elevatorMotor.getPIDController();
        pidController.setP(Constants.DriveConstants.kElevatorP);
        pidController.setI(Constants.DriveConstants.kElevatorI);
        pidController.setD(Constants.DriveConstants.kElevatorD);
        pidController.setIZone(Constants.DriveConstants.kElevatorIZone);
        pidController.setFF(Constants.DriveConstants.kElevatorFF);
        pidController.setOutputRange(-1, 1);
        pidController.setSmartMotionMaxVelocity(Constants.DriveConstants.kElevatorVelocity, 0);
        pidController.setSmartMotionMaxAccel(Constants.DriveConstants.kElevatorAcceleration, 0);
    }

    public void zeroElevator() {
        elevatorEncoder.setPosition(0);

    }

    public void driveElevator(double elevatorSpeed) {
        setElevatorSpeed(elevatorSpeed);
    }

    public void setElevatorSpeed(double speed) {
        if (speed < 0) {
            if (limitSwitch.get()) {
                elevatorMotor.set(0);
                return;
            }
        }
        elevatorMotor.set(speed * 0.5);
    }

    public void setElevatorPosition(double position) {
        pidController.setReference(position, ControlType.kSmartMotion);
    }

    public void stop() {
        setElevatorSpeed(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    
    }
}
