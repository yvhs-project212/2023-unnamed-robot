// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavxSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutonomousInitializeCommand extends CommandBase {
  /** Creates a new AutonomousInitializeCommand. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navxSub;
  ArmSubsystem armSub;
  ElevatorSubsystem elevatorSub;
  
  public AutonomousInitializeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrainSub.resetDrivetrainEncoders();
    navxSub.resetNavX();
    armSub.resetArmEncoder();
    elevatorSub.resetElevatorEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(elevatorSub.elevatorMotorPos == 0){
    return false;
    }
  }
}
