package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

        private final byte MPU6050_ADRESS = 0x68;
        private final int REGISTER_PWR_MGMT_1 = 0x6B;
        private final byte REGISTER_GYRO = 0x43;

        private I2C accelerometer = new I2C(I2C.Port.kOnboard, MPU6050_ADRESS);
        private byte[] buffer = new byte[6];

        private final DifferentialDrive drive;

        private final CANSparkMax leftMainMotor = new CANSparkMax(Constants.DriveConstants.kLeftMainMotor,
                        MotorType.kBrushless);
        private final CANSparkMax rightMainMotor = new CANSparkMax(Constants.DriveConstants.kRightMainMotor,
                        MotorType.kBrushless);

        private final CANSparkMax leftFollowerMotor1 = new CANSparkMax(Constants.DriveConstants.kLeftFollowerMotor1,
                        MotorType.kBrushless);
        private final CANSparkMax leftFollowerMotor2 = new CANSparkMax(Constants.DriveConstants.kLeftFollowerMotor2,
                        MotorType.kBrushless);

        private final CANSparkMax rightFollowerMotor1 = new CANSparkMax(Constants.DriveConstants.kRightFollowerMotor1,
                        MotorType.kBrushless);
        private final CANSparkMax rightFollowerMotor2 = new CANSparkMax(Constants.DriveConstants.kRightFollowerMotor2,
                        MotorType.kBrushless);

        private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.75);
        private final SlewRateLimiter zLimiter = new SlewRateLimiter(0.75);

        private static Drivetrain instance;

        private Drivetrain() {
                leftMainMotor.restoreFactoryDefaults();
                rightMainMotor.restoreFactoryDefaults();
                leftFollowerMotor1.restoreFactoryDefaults();
                leftFollowerMotor2.restoreFactoryDefaults();
                rightFollowerMotor1.restoreFactoryDefaults();
                rightFollowerMotor2.restoreFactoryDefaults();

                leftFollowerMotor1.follow(leftMainMotor);
                leftFollowerMotor2.follow(leftMainMotor);

                rightFollowerMotor1.follow(rightMainMotor);
                rightFollowerMotor2.follow(rightMainMotor);

                leftMainMotor.setInverted(true);
                leftFollowerMotor1.setInverted(true);
                leftFollowerMotor2.setInverted(true);

                leftMainMotor.setIdleMode(IdleMode.kBrake);
                rightMainMotor.setIdleMode(IdleMode.kBrake);

                leftFollowerMotor1.setIdleMode(IdleMode.kCoast);
                leftFollowerMotor2.setIdleMode(IdleMode.kCoast);

                rightFollowerMotor1.setIdleMode(IdleMode.kCoast);
                rightFollowerMotor2.setIdleMode(IdleMode.kCoast);

                drive = new DifferentialDrive(leftMainMotor, rightMainMotor);

                accelerometer.write(REGISTER_PWR_MGMT_1, 0);
        }

        public void arcadeDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
                drive.curvatureDrive(xLimiter.calculate(xSpeed), zLimiter.calculate(zRotation), isQuickTurn);
                drive.feed();
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Left Speed", leftMainMotor.getAppliedOutput());
                SmartDashboard.putNumber("Right Speed", rightMainMotor.getAppliedOutput());

                accelerometer.read(REGISTER_GYRO, 6, buffer);

                // System.out.print("Data: ");
                // for (int i = 0; i < buffer.length; i++) {
                //         System.out.print(buffer[i]);
                //         System.out.print(", ");
                // }
                // System.out.println();

                int x = (buffer[0] << 8) | buffer[1];
                int y = (buffer[2] << 8) | buffer[3];
                int z = (buffer[4] << 8) | buffer[5];

                SmartDashboard.putNumber("X", x);
                SmartDashboard.putNumber("Y", y);
                SmartDashboard.putNumber("Z", z);
        }

        public static Drivetrain getInstance() {
                if (instance == null)
                        instance = new Drivetrain();
                return instance;
        }
}
