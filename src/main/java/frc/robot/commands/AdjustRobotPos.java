// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagStats;
import frc.robot.subsystems.Drivetrain;

public class AdjustRobotPos extends Command {
   private Drivetrain drivetrain;
   private AprilTagStats apriltag;


  public AdjustRobotPos(Drivetrain drive, AprilTagStats apriltag) {
    drivetrain = drive;
    this.apriltag = apriltag;
    addRequirements(drivetrain);
    addRequirements(apriltag);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean targetIsFound = apriltag.hasTarget();
    apriltag.setTagView(targetIsFound);
    if (targetIsFound) {
      apriltag.updateData();
      drivetrain.visionDrive(apriltag);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.swerveDrive(
      0, 
      0, 
      0, 
      true, 
      new Translation2d(), 
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
