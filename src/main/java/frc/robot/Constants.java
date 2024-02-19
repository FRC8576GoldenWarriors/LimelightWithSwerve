// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int PDH_ID = 0;

  public static class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class SwerveConstants{
    //Drivetrain motor/encoder IDs
    public static final int LEFT_FRONT_DRIVE_ID = 7;
    public static final int RIGHT_FRONT_DRIVE_ID = 1;
    public static final int LEFT_BACK_DRIVE_ID = 4;
    public static final int RIGHT_BACK_DRIVE_ID = 3;
    
    public static final int LEFT_FRONT_TURN_ID = 6;
    public static final int RIGHT_FRONT_TURN_ID = 8;
    public static final int LEFT_BACK_TURN_ID = 5;
    public static final int RIGHT_BACK_TURN_ID = 2;
    
    public static final int LEFT_FRONT_CANCODER_ID = 2;
    public static final int RIGHT_FRONT_CANCODER_ID = 4;
    public static final int LEFT_BACK_CANCODER_ID = 1;
    public static final int RIGHT_BACK_CANCODER_ID = 3;

    public static final int PIGEON_ID = 0;

    //Drivetrain characteristics

    public static final double LEFT_FRONT_OFFSET = 0.500488;//329.941;//293.467; //1;//293.467;//327;//abs 329.941 Id: 2
    public static final double RIGHT_FRONT_OFFSET = 0.906982;//230.625;//160.576;//249;//160.576;// 55;//230.625 Id: 4
    public static final double LEFT_BACK_OFFSET = 0.540283;//274.685;//357.848;//357.848;//186.152;//272;//abs 274.685 Id: 1
    public static final double RIGHT_BACK_OFFSET = 0.737305;//30.3468;//258.926;//66.357;//101.643;//66.357;//264;//258.926 Id: 3

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.5); //originally 4 in template
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;//2 * Math.PI
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.575;

    public static final double DRIVETRAIN_MAX_SPEED = 1.75; //4.0
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 1.25 * Math.PI; //3.5

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 2; //3
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 12; //3

    //Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.875);
    public static final double WHEEL_BASE = Units.inchesToMeters(23.875);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );
  }
}