package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;

public class Elevator {
    private final CANSparkMax elevatorMotor = new CANSparkMax(Constants.DriveConstants.kelevatorMotor,
            MotorType.kBrushless);
    private final VictorSP intakeMotor = new VictorSP(Constants.DriveConstants.kintakeMotor);

    private final static I2C.Port i2cPort = I2C.Port.kOnboard;

    private final static ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public static double getRed() {
        return colorSensor.getRed();
    }

    public static double getBlue() {
        return colorSensor.getBlue();
    }

    public static double getGreen() {
        return colorSensor.getGreen();
    }

}
