// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;
import frc.robot.Constants.ChargeStation;
import frc.robot.Constants.DrivetrainConstants;

public class DriveForwardCommand extends CommandBase {
  /** Creates a new DriveForwardCommand. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navX;

  int negative = 1;
  
  public DriveForwardCommand(DrivetrainSubsystem drivetrainSub, NavxSubsystem navX) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrainSub = drivetrainSub;
    this.navX = navX;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveForwardCommand Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    drivetrainSub.driveForward(DrivetrainConstants.AUTO_FORWARD_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrainSub.driveForward(0);
    System.out.println("DriveForwardTimedCmd Finished!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if (ChargeStation.RAMP_SLOPE <= navX.getYaw()){
      return true;
    } else {
      return false;
    }
  }
}
