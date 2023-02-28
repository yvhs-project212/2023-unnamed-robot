// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.AutonomousPickerSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class AutonomousPickerCommand extends SubsystemBase {
  /** Creates a new AutonomousPickerCommand. */

  private final AutonomousPickerSubsystem autonomousSub;
  private final DrivetrainSubsystem driveTrain;
  private final NavxSubsystem m_NavxSubsystem;



  public AutonomousPickerCommand(AutonomousPickerSubsystem autonomousSub, NavxSubsystem m_NavxSubsystem, ClawSubsystem clawSub, 
  ArmSubsystem arm, DrivetrainSubsystem driveTrain) {

    this.autonomousSub = autonomousSub;
    this.driveTrain = driveTrain;
    this.m_NavxSubsystem = m_NavxSubsystem;

    addRequirements(autonomousSub, driveTrain, m_NavxSubsystem);
  }

  
  private void addRequirements(AutonomousPickerSubsystem autonomousSub2, DrivetrainSubsystem driveTrain2,
      NavxSubsystem m_NavxSubsystem2) {
  }




  public Command runAutonomous() {
    Command autoCommands = new SequentialCommandGroup();

    switch(autonomousSub.getAutonomousMode()) {
      case NONE: 
        System.out.println("No Autonomous");
        autoCommands = new SequentialCommandGroup(
        );
        break;
        
      case ONE:
        System.out.println("First Autonomous");
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

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
