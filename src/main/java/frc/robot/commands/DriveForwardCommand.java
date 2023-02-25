// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.DrivetrainConstants;

public class DriveForwardCommand extends CommandBase {
  /** Creates a new DriveForwardCommand. */

  DrivetrainSubsystem drivetrainSub;

  int negative = 1;
  
  public DriveForwardCommand(DrivetrainSubsystem drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrainSub = drivetrainSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveForwardCommand Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftTopMotor = negative * DrivetrainConstants.AUTO_LEFT_DRIVE_FORWARD_SPEED;
    double rightTopMotor = negative * DrivetrainConstants.AUTO_RIGHT_DRIVE_FORWARD_SPEED;
    double leftBottomMotor = negative * DrivetrainConstants.AUTO_LEFT_DRIVE_FORWARD_SPEED;
    double rightBottomMotor = negative * DrivetrainConstants.AUTO_LEFT_DRIVE_FORWARD_SPEED;
    
    drivetrainSub.setMotors(leftTopMotor, rightTopMotor, leftBottomMotor, rightBottomMotor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrainSub.setMotors(0, 0, 0, 0);
    System.out.println("DriveForwardTimedCmd Finished!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
