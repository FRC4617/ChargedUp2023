package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.sensors.MPU6050;
import frc.robot.commands.ArcadeDrive;


public class Drivetrain extends SubsystemBase {

        public double[] xData = {0,0,0,0,0};

        private final DifferentialDrive drive;

        private final byte MPU6050_ADDRESS = 0x68;
        private final MPU6050 gyro = new MPU6050(MPU6050_ADDRESS);

        private final CANSparkMax leftMainMotor = new CANSparkMax(Constants.DriveConstants.kLeftMainMotor,
                        MotorType.kBrushless);
        private final CANSparkMax rightMainMotor = new CANSparkMax(Constants.DriveConstants.kRightMainMotor,
                        MotorType.kBrushless);

        public final RelativeEncoder leftEncoder = leftMainMotor.getEncoder();
        public final RelativeEncoder rightEncoder = rightMainMotor.getEncoder();

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
        }

        public void arcadeDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
                drive.curvatureDrive(xLimiter.calculate(xSpeed*0.25), zLimiter.calculate(zRotation*0.25), isQuickTurn);
                drive.feed();
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Left Speed", leftMainMotor.getAppliedOutput());
                SmartDashboard.putNumber("Right Speed", rightMainMotor.getAppliedOutput());

                Rotation2d x = gyro.getRotationX();
                Rotation2d y = gyro.getRotationY();
                Rotation2d z = gyro.getRotationZ();

                gyro.printAllData();

                SmartDashboard.putNumber("X", x.getDegrees());
                SmartDashboard.putNumber("Y", y.getDegrees());
                SmartDashboard.putNumber("Z", z.getDegrees());

                
                SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
                SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());

                SmartDashboard.putNumber("Distance", rightEncoder.getPositionConversionFactor()+leftEncoder.getPositionConversionFactor());

        }


        public static double avgX(){
           

                return 0;
        }

        public void resetEncoders(){
               rightEncoder.setPosition(0);
               leftEncoder.setPosition(0);

        }


        public static Drivetrain getInstance() {
                if (instance == null)
                        instance = new Drivetrain();
                return instance;
        }

        public void stop() {
                drive.stopMotor();
        }

}
