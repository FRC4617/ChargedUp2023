package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private final CANSparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final VictorSP intakeMotor;
    private final ColorSensorV3 colorSensor;

    public Elevator() {
        elevatorMotor = new CANSparkMax(Constants.DriveConstants.kElevatorMotor, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(40);
        elevatorEncoder.setPosition(0);

        intakeMotor = new VictorSP(Constants.DriveConstants.kIntakeMotor);
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }

    public void driveElevator(double elevatorSpeed, double intakeSpeed) {
        setElevatorSpeed(elevatorSpeed);
        setIntakeSpeed(intakeSpeed);
    }

    public void setElevatorSpeed(double speed) {
        elevatorMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public Color getColor() {
        return colorSensor.getColor();
    }

    public void stop() {
        setElevatorSpeed(0);
        setIntakeSpeed(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Position", elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Velocity", elevatorEncoder.getVelocity());
        SmartDashboard.putString("Color", getColor().toHexString());
    }

}
