// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kDriverDeadband = 0.15;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {

    public static final int kLeftMainMotor = 5;
    public static final int kRightMainMotor = 2;
    public static final int kLeftFollowerMotor1 = 4;
    public static final int kLeftFollowerMotor2 = 6;
    public static final int kRightFollowerMotor1 = 1;
    public static final int kRightFollowerMotor2 = 3;

    public static final int kElevatorMotor = 7;
    public static final int kIntakeMotor = 8;

    public static final double kElevatorP = 0.0005;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorIZone = 0;
    public static final double kElevatorFF = 0;
    public static final double kElevatorVelocity = 5000;
    public static final double kElevatorAcceleration = 3000;

    public static final double kElevatorHighPosition = 80;

    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
        Units.inchesToMeters(2*Math.PI*3));

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot.
    public static final double ksVolts = -0.85744;
    public static final double kvVoltSecondsPerMeter = 7.6439;
    public static final double kaVoltSecondsSquaredPerMeter = 1.7086;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.004;


    //2,1
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kGearingRatio = 12.75;

    public static final double kWheelCircumference = Units.inchesToMeters(6) * Math.PI;

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        Constants.DriveConstants.kinematics,
        10);
  }
}
