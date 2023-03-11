// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class DriveBackwardCommand extends CommandBase {
  /** Creates a new DriveBackwardCommand. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navxSub;

  public DriveBackwardCommand(DrivetrainSubsystem drivetrainSub, NavxSubsystem navxSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrainSub = drivetrainSub;
    this.navxSub = navxSub;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrainSub.resetDrivetrainEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSub.driveForward(Constants.DrivetrainConstants.DRIVE_BACKWARD_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((drivetrainSub.roundedMotorPos / Constants.DrivetrainConstants.HIGH_GEAR_ENCODER_PER_INCH) >= Constants.DrivetrainConstants.DRIVE_BACKWARD_SETPOINT){
      return true;
    }else{
      return false;
    }
  }
}
