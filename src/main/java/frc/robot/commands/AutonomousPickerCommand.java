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
import frc.robot.subsystems.AutonomousPickerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class AutonomousPickerCommand extends SubsystemBase {
  /** Creates a new AutonomousPickerCommand. */

  private final AutonomousPickerSubsystem autonomousPicker;
  private final DrivetrainSubsystem driveTrain;
  private final NavxSubsystem navX;

  public AutonomousPickerCommand(AutonomousPickerSubsystem autonomousPicker, DrivetrainSubsystem driveTrain, 
  NavxSubsystem navX) {

    this.autonomousPicker = autonomousPicker;
    this.driveTrain = driveTrain;
    this.navX = navX;

    addRequirements(autonomousPicker, driveTrain, navX);
  }

  private void addRequirements(AutonomousPickerSubsystem autonomousPicker2, DrivetrainSubsystem driveTrain2,
      NavxSubsystem navX2) {
  }

  
  public Command runAutonomous() {
    Command autoCommands = new SequentialCommandGroup();

    switch(autonomousPicker.getAutonomousMode()) {
      case NONE: 
        System.out.println("No Autonomous");
        autoCommands = new SequentialCommandGroup( new StageOneBalancing (driveTrain, navX)
        );
        break;
        
      case TWO:
        System.out.println("First Autonomous");
        autoCommands = new SequentialCommandGroup();
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
