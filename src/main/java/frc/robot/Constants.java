
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
  // boolean isPractice = false;

  // public Constants(){
  //   checkPracticeMode(isPractice);
  // }


  public static class VisionConstants {
    public static class limeLightDistanceConstants{
      public static final double OPTIMAL_SHOOTING_DISTANCE = 2.0;
      public static final double ALLOWED_ANGLE_ERROR = 2.0;
      public static final double ALLOWED_DISTANCE_ERROR = 0.1;
    }

    // In meters and degrees
    // change later once we get true mesurements
    public static class limeLightDimensionConstants{
      public static final double CAMERA_HEIGHT = 0.5;  
      public static final double TARGET_HEIGHT = 2.0; // hight of the speaker
      public static final double CAMERA_PITCH = 30.0;
    }

    public static class aprilTagIDConstants{
      public static final int RED_SPEAKER_TAG_ID = 7;
      public static final int BLUE_SPEAKER_TAG_ID = 4;
    }

   public static class limelightNetworkTableKey{
    public static final String LIMELIGHT_NETWORKTABLE_KEY = "limelight-miracle";
   }


    public static class cameraTranslationConstants {
      //translation of camera in meters (change when camera has been mounted on robot)
      public static final double tX = -32 * 0.01;
      public static final double tY = 0.0 * 0.01;
      public static final double tZ = 32 * 0.01;
    }
    public static class cameraRotationConstants {
      //rotation of camera (change when camera has been mounted on robot)
      public static final double rRoll = 0.0;
      public static final double rPitch = 0.0;
      public static final double rYaw = 0.0;
    }

    public static class distanceConstants {
      public static final double goalMeterDistance = 3.0;
      public static final double visionAngleDegrees = 0.0;
      public static final List<Integer> useableIDs = Arrays.asList(4,7);
    }

    public static class nameConstants{
      public static final String cameraName = "Arducam_OV9281_USB_Camera (1)";
      public static final String tabName = "Vision";
      public static final String publishName = "VisionPose";
    }

    public static class VisionPIDConstants {
      public static final double kPVisionTurning = 0.01;
      public static final double kPVisionMoving = 0.5;
    }
 }



  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }



  public static final class SwerveConstants{
    //Drivetrain motor/encoder IDs
    public static class PoseConfig {
      // Increase these numbers to trust your model's state estimates less.
      public static final double kPositionStdDevX = 0.1;
      public static final double kPositionStdDevY = 0.1;
      public static final double kPositionStdDevTheta = 10;

      // Increase these numbers to trust global measurements from vision less.
      public static final double kVisionStdDevX = 2;
      public static final double kVisionStdDevY = 2;
      public static final double kVisionStdDevTheta = 1;
    }
    //PRACTICE IDS
    // public static final int LEFT_FRONT_DRIVE_ID = 7;
    // public static final int RIGHT_FRONT_DRIVE_ID = 1;
    // public static  final int LEFT_BACK_DRIVE_ID = 4;
    // public static final int RIGHT_BACK_DRIVE_ID = 3;
    
    // public static final int LEFT_FRONT_TURN_ID = 6;
    // public static final int RIGHT_FRONT_TURN_ID = 8;
    // public static  final int LEFT_BACK_TURN_ID = 5; 
    // public static final int RIGHT_BACK_TURN_ID = 2;

    // public static final int LEFT_FRONT_CANCODER_ID = 2;
    // public static final int RIGHT_FRONT_CANCODER_ID = 4;
    // public static final int LEFT_BACK_CANCODER_ID = 1;
    // public static final int RIGHT_BACK_CANCODER_ID = 3;

    //  public static final int PIGEON_ID = 0;


    //COMPETITION IDS

     public static final int LEFT_FRONT_DRIVE_ID = 7;
     public static final int RIGHT_FRONT_DRIVE_ID = 1;
     public static final int LEFT_BACK_DRIVE_ID = 5; // 5
     public static final int RIGHT_BACK_DRIVE_ID = 3; // 3
    
     public static final int LEFT_FRONT_TURN_ID = 6;
     public static final int RIGHT_FRONT_TURN_ID = 8;
     public static  final int LEFT_BACK_TURN_ID = 4; // 4 
     public static final int RIGHT_BACK_TURN_ID = 2; // 2
    
    public static final int LEFT_FRONT_CANCODER_ID = 3;
    public static final int RIGHT_FRONT_CANCODER_ID = 4;
    public static final int LEFT_BACK_CANCODER_ID = 0; // 2
    public static final int RIGHT_BACK_CANCODER_ID = 1;

    public static final int PIGEON_ID = 0;

    //Drivetrain characteristics

    //PRACTICE OFFSETS
    // public static  double LEFT_FRONT_OFFSET = 0.004150;
    // public static  double RIGHT_FRONT_OFFSET = 0.907227;
    // public static  double LEFT_BACK_OFFSET = 0.571533;
    // public static  double RIGHT_BACK_OFFSET = 0.731201;

    //COMP OFFSETS
    public static  double LEFT_FRONT_OFFSET =0.205811;//0.206055; //0.222412;//0.213135;//0.216064;//0.721680;//0.732666;//0.292969;//0.723145;//0.717041;//0.717041;//0.710938 ;//0.725098;//0.719971;	
    public static  double RIGHT_FRONT_OFFSET = 0.798828;//0.301514;//0.799072;//0.302246;//0.306885;//0.792725;//0.712891;//0.817383;
    public static  double LEFT_BACK_OFFSET = 0.5217290;//0.524902;//0.017334;//0.794189;//0.818115;//0.300781;//0.303711;//0.847412;//0.448730;//0.455322;//0.418945;//0.490967;//0.519043;
    public static  double RIGHT_BACK_OFFSET = 0.666748;//0.514160;//0.512451;//0.011475 ;//0.012451;//0.014404;//0.510498;//0.504883;//0.013916; 

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.00); //originally 4 in template
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;//2 * Math.PI
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.575;

    public static final double DRIVETRAIN_MAX_SPEED = 5.3;//4.0, 5.5;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 5 * Math.PI; //3.5, 4.25, 5

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 0.85;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 7.5; //3
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 15; //

    public static final double AUTO_KP_TTANSLATION = 1.35; //1.15
    public static final double AUTO_KP_ROTATIONAL = 0.1; //0.1

    public static final double TRACK_WIDTH = Units.inchesToMeters(23.875);
    public static final double WHEEL_BASE = Units.inchesToMeters(23.875);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt((Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2))) / 2.0;

    public static final HolonomicPathFollowerConfig AUTO_CONFIG = new HolonomicPathFollowerConfig(
      new PIDConstants(AUTO_KP_TTANSLATION, 0, 0),
      new PIDConstants(AUTO_KP_ROTATIONAL, 0, 0),
      DRIVETRAIN_MAX_SPEED,
      DRIVE_BASE_RADIUS,
      new ReplanningConfig()
    );


    //Swerve Kinematics
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

  }


  public static class IntakeConstants{
    public static final int IntakeSensorID = 5;

    public static final int rollerCANSparkID = 15;
    public static final int pivotCANSparkID = 13;

    public static final int intakeEncoderPort = 1;

    public static final int intakeEncoderA = 7;
    public static final int intakeEncoderB = 8;
    public static final int intakeEncoderI = 9;

    //COMPETITION ROLLERS AND ARM PIVOT
    public static final double kRollerInSpeed = 0.8;
    public static final double kRollerOutSpeed = -0.5;

    public static final double kArmUpSpeed = -0.65; 
    public static final double kArmDownSpeed = 0.5; 

    //PRACTICE ROLLER
    // public static final double kRollerInSpeed = -0.50;
    // public static final double kRollerOutSpeed = 0.65;
    
    // public static final double kArmUpSpeed = 0.5;
    // public static final double kArmDownSpeed = -0.45;

    public static final double kArmDownPosition = 1205;//0.6002;//0.602; //encoder resoltuion value
    public static final double kArmUpPosition = 10;//0.00 //encoder resolution value


    //IN ROTATIONS
    public static final double kMaxArmAcceleration = 2.75;//1.05;
    public static final double kMaxArmVelocity = 3.55;//1.5;

    public static final double kP_armUp = 1.5;
    public static final double kP_armDown = 1.5;

    public static final double kI_arm = 0.0;
    public static final double kD_arm = 0.2;

  }


  public static class ShooterConstants{
    public static final int leftCANSparkID = 26;
    public static final int rightCANSparkID = 25;

    public static final double kShooterSpeed = 0.525;
    public static final double kShintakeSpeed = -0.1;

    public static final double kAmpSpeed = 0.2;



    public static final int pivotCANSparkID = 11;

    public static final int shooterEncoderID = 0;

    public static final double shooterEncoderOffset = 0.534;

    //PRACTICE SHOOTER PIVOT
    // public static final double kPivotUpSpeed = 0.2;
    // public static final double kPivotDownSpeed = -0.1;

    //COMP SHOOTER PVIOT
    public static final double kPivotUpSpeed = -0.25;
     public static final double kPivotDownSpeed = 0.25;

     //AutoAim Constants
    public static final double kShooterAutoAngle = Math.tan(211/(VisionConstants.distanceConstants.goalMeterDistance*100.0));
  }


  public static class ClimberConstants{
    public static final int leftCANSparkID = 12;
    public static final int rightCANSparkID = 14;
    public static final double kClimberSpeed = 0.95;
  }


  public static class LEDConstants{
    public static final int LED_PORT1 = 9;
    public static final int LedLength1 = 58;
  }

}