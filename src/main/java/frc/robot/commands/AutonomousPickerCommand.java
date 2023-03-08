// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonomousPickerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class AutonomousPickerCommand extends CommandBase {
  /** Creates a new AutoPickerCommand. */

  private final AutonomousPickerSubsystem autonomousSub;
  private final DrivetrainSubsystem driveTrain;
  private final NavxSubsystem m_NavxSubsystem;


  public AutonomousPickerCommand(AutonomousPickerSubsystem autonomousSub, NavxSubsystem m_NavxSubsystem, DrivetrainSubsystem driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.autonomousSub = autonomousSub;
    this.driveTrain = driveTrain;
    this.m_NavxSubsystem = m_NavxSubsystem;

    addRequirements(autonomousSub, m_NavxSubsystem, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public Command runAutonomous() {
    Command autoCommands = new SequentialCommandGroup();

    switch(autonomousSub.getAutonomous()) {
      case NONE: 
        System.out.println("No Autonomous");
        autoCommands = new SequentialCommandGroup(
        );
        System.out.println("No Autonomous Chosen");
        break;
        
      case ONE:
        System.out.println("Stage One Autonomous Selecetd");
        autoCommands = new SequentialCommandGroup(
          new DriveForwardCommand(driveTrain, m_NavxSubsystem)
        );
        break;
        
      default:
      System.out.println("Default Chosen");
        break;
    }
    return autoCommands;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Autonomous ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
