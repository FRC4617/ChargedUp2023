package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

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

        public Drivetrain() {
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
        }

        public void arcadeDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
                drive.curvatureDrive(xLimiter.calculate(xSpeed), zLimiter.calculate(zRotation), isQuickTurn);
                drive.feed();
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Left Speed", leftMainMotor.getAppliedOutput());
                SmartDashboard.putNumber("Right Speed", rightMainMotor.getAppliedOutput());
        }
}
