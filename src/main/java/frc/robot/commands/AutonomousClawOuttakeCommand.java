// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class AutonomousClawOuttakeCommand extends CommandBase {
  /** Creates a new AutonomousClawOuttakeCommand. */

  ClawSubsystem clawSub;
  double initialTimestamp;

  public AutonomousClawOuttakeCommand(ClawSubsystem clawSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.clawSub = clawSub;
    addRequirements(clawSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTimestamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawSub.clawRollersOuttake(Constants.ClawConstants.CLAW_AUTO_OUTTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - initialTimestamp == 8){
      return true;
    } else{
    return false;
    }
  }
}
