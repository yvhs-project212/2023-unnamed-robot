// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommands extends CommandBase {
  /** Creates a new ArmCommands. */
    public ArmSubsystem arm;

    public ArmCommands(ArmSubsystem arm) {
      this.arm = arm;
      addRequirements(arm);
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArmWithDPadsCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.operatorController.getPOV() >= 0) {
      arm.armWithPOV(RobotContainer.operatorController); 
    } else {
      arm.stopMotors();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ArmWithDPadsCmd ended!");}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
