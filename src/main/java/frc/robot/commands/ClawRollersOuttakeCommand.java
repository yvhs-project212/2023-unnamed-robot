// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawRollersOuttakeCommand extends CommandBase {
  /** Creates a new ClawOuttakeCommand. */
  
  ClawSubsystem clawSub;
  double clawOuttakeSpeed;

  public ClawRollersOuttakeCommand(ClawSubsystem clawSub, double clawOuttakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.clawSub = clawSub;
    this.clawOuttakeSpeed = clawOuttakeSpeed;
    addRequirements(clawSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawSub.clawRollersOuttake(clawOuttakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSub.clawRollersStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
