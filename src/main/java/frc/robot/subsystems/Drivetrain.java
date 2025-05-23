// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  //PRACTICE MODULES
  // private SwerveModule leftFront = new SwerveModule(
  //   SwerveConstants.LEFT_FRONT_DRIVE_ID, 
  //   SwerveConstants.LEFT_FRONT_TURN_ID, 
  //   false, 
  //   true, 
  //   SwerveConstants.LEFT_FRONT_CANCODER_ID, 
  //   SwerveConstants.LEFT_FRONT_OFFSET);

  // private SwerveModule rightFront = new SwerveModule(
  //   SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
  //   SwerveConstants.RIGHT_FRONT_TURN_ID, 
  //   true, //used to be true, might have to change back - Om: 2/14/24
  //   true, 
  //   SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
  //   SwerveConstants.RIGHT_FRONT_OFFSET);

  // private SwerveModule leftBack = new SwerveModule(
  //   SwerveConstants.LEFT_BACK_DRIVE_ID, 
  //   SwerveConstants.LEFT_BACK_TURN_ID, 
  //   true, 
  //   true, 
  //   SwerveConstants.LEFT_BACK_CANCODER_ID, 
  //   SwerveConstants.LEFT_BACK_OFFSET);

  //   private SwerveModule rightBack = new SwerveModule(
  //   SwerveConstants.RIGHT_BACK_DRIVE_ID, 
  //   SwerveConstants.RIGHT_BACK_TURN_ID, 
  //   true, 
  //   true, 
  //   SwerveConstants.RIGHT_BACK_CANCODER_ID, 
  //   SwerveConstants.RIGHT_BACK_OFFSET);

  //#@PN 11/28  Display robot pose on 2D and 3D layout in AdvantageScope
  
  private final StructArrayPublisher<SwerveModuleState> m_ModuleStatePublisherIn;
  private final StructArrayPublisher<SwerveModuleState> m_ModuleStatePublisherActual;

  //COMPETITIOM MODULES
   private SwerveModule leftFront = new SwerveModule(
    SwerveConstants.LEFT_FRONT_DRIVE_ID, 
    SwerveConstants.LEFT_FRONT_TURN_ID, 
    false, 
    true, 
    SwerveConstants.LEFT_FRONT_CANCODER_ID, 
    SwerveConstants.LEFT_FRONT_OFFSET);

  private SwerveModule rightFront = new SwerveModule(
    SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
    SwerveConstants.RIGHT_FRONT_TURN_ID, 
    false, //used to be true, might have to change back - Om: 2/14/24
    true, 
    SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
    SwerveConstants.RIGHT_FRONT_OFFSET);

  private SwerveModule leftBack = new SwerveModule(
    SwerveConstants.LEFT_BACK_DRIVE_ID, 
    SwerveConstants.LEFT_BACK_TURN_ID, 
    true, 
    true, 
    SwerveConstants.LEFT_BACK_CANCODER_ID, 
    SwerveConstants.LEFT_BACK_OFFSET);

    private SwerveModule rightBack = new SwerveModule(
    SwerveConstants.RIGHT_BACK_DRIVE_ID, 
    SwerveConstants.RIGHT_BACK_TURN_ID, 
    false, 
    true, 
    SwerveConstants.RIGHT_BACK_CANCODER_ID, 
    SwerveConstants.RIGHT_BACK_OFFSET);

  private SlewRateLimiter frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  private Pigeon2 gyro = new Pigeon2(SwerveConstants.PIGEON_ID);

  private static final Drivetrain drivetrain = new Drivetrain();

  public SwerveDriveOdometry odometry = new SwerveDriveOdometry(SwerveConstants.DRIVE_KINEMATICS, getHeadingRotation2d(), getModulePositions(), new Pose2d());

  public static Drivetrain getInstance(){
    return drivetrain;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {

      //#@PN 11/28
        
    m_ModuleStatePublisherIn = NetworkTableInstance.getDefault().getTable("24Karat").getStructArrayTopic("SwerveStates/In", SwerveModuleState.struct).publish();
    m_ModuleStatePublisherActual = NetworkTableInstance.getDefault().getTable("24Karat").getStructArrayTopic("SwerveStates/Actual", SwerveModuleState.struct).publish();

    System.out.println("--- Created GWRServeStates Topic --- ");

    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();

    AutoBuilder.configureHolonomic(
      this::getPose2d,
      this::resetPose2d,
      this::getRobotRelativeSpeeds,
      this::driveRobotRelative,
      SwerveConstants.AUTO_CONFIG,
      () -> isRedAlliance(),
      this
    );


    


    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> leftFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> leftFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> rightFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> rightFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> leftBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> leftBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> rightBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> rightBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> (getHeading() / 180 * Math.PI), null);
      }
    });
  }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //RobotContainer.poseEstimator.updateOdometry(getHeadingRotation2d(), getModulePositions());
    
    double yaw = gyro.getYaw().getValue();
    SmartDashboard.putNumber("Robot Angle", getHeading());

    //#@PN 11/28
    Pose2d curPose2d = odometry.update(getHeadingRotation2d(), getModulePositions());

    m_ModuleStatePublisherActual.set(getModuleStates());

    //rates 2 is yaw (XYZ in order )
    /*SmartDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((yaw/ 180)) + "pi rad/s");
    // Logger.recordOutput("Robot Angle", getHeading());
    // Logger.recordOutput("Robot Pitch", getPitch());
    // Logger.recordOutput("Robot Roll", getRoll());
    // Logger.recordOutput("Pose", getPose().toString());
    // Logger.recordOutput("Angular Speed", new DecimalFormat("#.00").format((yaw / 180)) + "pi rad/s" );

    SmartDashboard.putString("Pose", getPose2d().toString());

    //new values
    SmartDashboard.putNumber("Left Front Module Velocity", leftFront.getDriveMotorVelocity());
    SmartDashboard.putNumber("Right Front Module Velocity", rightFront.getDriveMotorVelocity());
    SmartDashboard.putNumber("Left Back Module Velocity", leftBack.getDriveMotorVelocity());
    SmartDashboard.putNumber("Right Back Module Velocity", rightBack.getDriveMotorVelocity());
    SmartDashboard.putNumber("Left Front Module abs angle", leftFront.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Front Module abs angle", rightFront.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Left Back Module abs angle", leftBack.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Back Module abs angle", rightBack.getAbsoluteEncoderAngle());*/

    /*SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> leftFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> leftFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> rightFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> rightFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> leftBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> leftBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> rightBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> rightBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> getHeading(), null);
      }
    });// */
  

    // Logger.recordOutput("Drivetrain/Robot Angle", getHeadingRotation2d().getRadians());
    // Logger.recordOutput("Drivetrain/Pose", getPose());
    // Logger.recordOutput("Drivetrain/Angular Speed", yaw / 180);
    // Logger.recordOutput("Drivetrain/Module States", getModuleStates());

    

     
    
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadband){ 
      //Drive with rotational speed control w/ joystick
    if(deadband){
      frontSpeed = Math.abs(frontSpeed) > 0.0825 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.0825 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.0825 ? turnSpeed : 0;
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);

    m_ModuleStatePublisherIn.set(getModuleStates());

  }


  public void drive (Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLooop){
    ChassisSpeeds chassisSpeeds;
    if(fieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroscopeRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED);

    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);

    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValue());
  }


  public void setAllIdleMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    System.out.println("resetAllEncoders()");
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void setHeading(double heading){
    gyro.setYaw(heading);
  }

  public double getHeading(){
    return (Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360)); //clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  // public void setModuleZero(){ Not Called Anywhere
  //   leftFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   rightFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   leftBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   rightBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  // }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();
    return states;
  } 

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  } 

  public Pose2d getPose2d(){
    return odometry.getPoseMeters();
  }

  public void resetPose2d(Pose2d pose){
    odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }
 public void visionDrive(AprilTagStats april){
     try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = april.robotPath();

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }

  }
  public boolean isRedAlliance(){
    if (DriverStation.getAlliance().isPresent()){
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}