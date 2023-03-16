package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

                leftEncoder = leftMainMotor.getEncoder();
                rightEncoder = rightMainMotor.getEncoder();

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
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Create a voltage constraint to ensure we don't accelerate too fast
                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(
                                                DriveConstants.ksVolts,
                                                DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                Constants.DriveConstants.kinematics,
                                10);

                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                                Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(Constants.DriveConstants.kinematics)
                                // Apply the voltage constraint
                                .addConstraint(autoVoltageConstraint);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 1, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1.5, 0)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 0, new Rotation2d(0)),
                                // Pass config
                                config);

                odometry.resetPosition(getAngle(), leftDistance(), rightDistance(), exampleTrajectory.getInitialPose());

                RamseteCommand ramseteCommand = new RamseteCommand(
                                exampleTrajectory,
                                this::getPose,
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
                                // RamseteCommand passes volts to the callback
                                this::tankDriveVolts,
                                this);

                // Reset odometry to the starting pose of the trajectory.
                this.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
        }

        public void stop() {
                drive.stopMotor();
        }

}
