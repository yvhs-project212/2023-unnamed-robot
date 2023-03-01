// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class DriveUpTheRampCommand extends CommandBase {
  /** Creates a new DriveUpTheRampCommand. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navxSub;

  public DriveUpTheRampCommand(DrivetrainSubsystem drivetrainSub, NavxSubsystem navxSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.drivetrainSub = drivetrainSub;
    this.navxSub = navxSub;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Driving Up The Ramp!");
    drivetrainSub.lastTimestamp = 0;
    drivetrainSub.lastError = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrainSub.driveForward(Constants.DrivetrainConstants.DRIVE_UP_THE_RAMP_kP * ((navxSub.getPitch()) * 0.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Drive Up The Ramp Ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(navxSub.getPitch() == 0 ){
      return true;
    } else{
    return false;
    }
  }
}
