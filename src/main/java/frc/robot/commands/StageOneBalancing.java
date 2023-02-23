// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class StageOneBalancing extends CommandBase {
  /** Creates a new StageOneBalancing. */
  
  public StageOneBalancing(DrivetrainSubsystem driveTrain, NavxSubsystem navX) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
      new PrintCommand("StageOneBalancing Started")
    );
  }



  private void addCommands(PrintCommand printCommand) {
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
