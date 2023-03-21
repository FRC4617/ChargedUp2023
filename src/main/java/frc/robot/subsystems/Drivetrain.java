package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

        private final DifferentialDrive drive;

        // Odometry class for tracking robot pose
        private final DifferentialDriveOdometry odometry;
        private final Field2d field;

        private final AHRS gyro;

        private final CANSparkMax leftMainMotor = new CANSparkMax(Constants.DriveConstants.kLeftMainMotor,
                        MotorType.kBrushless);
        private final CANSparkMax rightMainMotor = new CANSparkMax(Constants.DriveConstants.kRightMainMotor,
                        MotorType.kBrushless);

        private final RelativeEncoder leftEncoder;
        private final RelativeEncoder rightEncoder;

        private final CANSparkMax leftFollowerMotor1 = new CANSparkMax(Constants.DriveConstants.kLeftFollowerMotor1,
                        MotorType.kBrushless);
        private final CANSparkMax leftFollowerMotor2 = new CANSparkMax(Constants.DriveConstants.kLeftFollowerMotor2,
                        MotorType.kBrushless);

        private final CANSparkMax rightFollowerMotor1 = new CANSparkMax(Constants.DriveConstants.kRightFollowerMotor1,
                        MotorType.kBrushless);
        private final CANSparkMax rightFollowerMotor2 = new CANSparkMax(Constants.DriveConstants.kRightFollowerMotor2,
                        MotorType.kBrushless);

        public Drivetrain() {
                gyro = new AHRS(SPI.Port.kMXP);
                gyro.calibrate();

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

                rightMainMotor.setInverted(true);
                rightFollowerMotor1.setInverted(true);
                rightFollowerMotor2.setInverted(true);

                leftMainMotor.setIdleMode(IdleMode.kBrake);
                rightMainMotor.setIdleMode(IdleMode.kBrake);

                leftFollowerMotor1.setIdleMode(IdleMode.kCoast);
                leftFollowerMotor2.setIdleMode(IdleMode.kCoast);

                rightFollowerMotor1.setIdleMode(IdleMode.kCoast);
                rightFollowerMotor2.setIdleMode(IdleMode.kCoast);

                leftMainMotor.setSmartCurrentLimit(20, 15);
                rightMainMotor.setSmartCurrentLimit(20, 15);
                leftFollowerMotor1.setSmartCurrentLimit(20, 15);
                leftFollowerMotor2.setSmartCurrentLimit(20, 15);
                rightFollowerMotor1.setSmartCurrentLimit(20, 15);
                rightFollowerMotor2.setSmartCurrentLimit(20, 15);

                leftEncoder = leftMainMotor.getEncoder();
                rightEncoder = rightMainMotor.getEncoder();

                leftEncoder.setPosition(0);
                rightEncoder.setPosition(0);

                // Sleep to configure the motors
                Timer.delay(1);

                if (!gyro.isCalibrating()) {
                        gyro.reset();
                }

                drive = new DifferentialDrive(leftMainMotor, rightMainMotor);
                odometry = new DifferentialDriveOdometry(getAngle(), leftDistance(), rightDistance());

                field = new Field2d();
                SmartDashboard.putData(field);
        }

        public Rotation2d getAngle() {
                return Rotation2d.fromDegrees(-gyro.getYaw());
        }

        public void zeroAngle() {
                gyro.reset();
        }

        public double leftDistance() {
                return leftEncoder.getPosition() * (Constants.DriveConstants.kWheelCircumference
                                / Constants.DriveConstants.kGearingRatio);
        }

        public double rightDistance() {
                return rightEncoder.getPosition() * (Constants.DriveConstants.kWheelCircumference
                                / Constants.DriveConstants.kGearingRatio);
        }

        public void arcadeDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
                drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
                drive.feed();
        }

        public void resetEncoders() {
                leftEncoder.setPosition(0);
                rightEncoder.setPosition(0);
        }

        /**
         * Returns the currently-estimated pose of the robot.
         *
         * @return The pose.
         */
        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        /**
         * Controls the left and right sides of the drive directly with voltages.
         *
         * @param leftVolts  the commanded left output
         * @param rightVolts the commanded right output
         */
        public void tankDriveVolts(double leftVolts, double rightVolts) {
                leftMainMotor.setVoltage(leftVolts);
                rightMainMotor.setVoltage(rightVolts);
                drive.feed();
        }

        @Override
        public void periodic() {

                // Update the odometry in the periodic block
                odometry.update(
                                getAngle(), leftDistance(), rightDistance());

                field.setRobotPose(getPose());

                SmartDashboard.putNumber("Left Speed", leftMainMotor.getAppliedOutput());
                SmartDashboard.putNumber("Right Speed", rightMainMotor.getAppliedOutput());

                SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
                SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());

                SmartDashboard.putNumber("Gyro", getAngle().getDegrees());

                SmartDashboard.putNumber("Left Distance", leftDistance());
                SmartDashboard.putNumber("Right Distance", rightDistance());
        }

        /**
         * Returns the current wheel speeds of the robot.
         *
         * @return The current wheel speeds.
         */
        public DifferentialDriveWheelSpeeds getWheelSpeeds() {
                return new DifferentialDriveWheelSpeeds(
                                leftEncoder.getVelocity() * (Constants.DriveConstants.kWheelCircumference
                                                / Constants.DriveConstants.kGearingRatio),
                                rightEncoder.getVelocity() * (Constants.DriveConstants.kWheelCircumference
                                                / Constants.DriveConstants.kGearingRatio));
        }

        /**
         * Resets the odometry to the specified pose.
         *
         * @param pose The pose to which to set the odometry.
         */
        public void resetOdometry(Pose2d pose) {
                resetEncoders();
                odometry.resetPosition(
                                getAngle(), leftDistance(), rightDistance(),
                                pose);
        }

        /**
         * 
         * @param traj
         * @param isFirstPath
         * @return
         */
        
        public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        // Reset odometry for the first path you run during auto
                                        if (isFirstPath) {
                                                this.resetOdometry(traj.getInitialPose());
                                        }
                                }),
                                new PPRamseteCommand(
                                                traj,
                                                this::getPose, // Pose supplier
                                                new RamseteController(Constants.DriveConstants.kRamseteB,
                                                                Constants.DriveConstants.kRamseteZeta),
                                                new SimpleMotorFeedforward(
                                                                DriveConstants.ksVolts,
                                                                DriveConstants.kvVoltSecondsPerMeter,
                                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                                Constants.DriveConstants.kinematics,
                                                this::getWheelSpeeds,
                                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                                this::tankDriveVolts,
                                                false,
                                                this));
        }

        public void stop() {
                drive.stopMotor();
        }
}
