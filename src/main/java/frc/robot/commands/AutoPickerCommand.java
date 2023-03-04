// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.AutonomousPickerSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class AutoPickerCommand extends CommandBase {
  /** Creates a new AutoPickerCommand. */

AutonomousPickerSubsystem autonomousPickerSub;
DrivetrainSubsystem drivetrainSub;
NavxSubsystem navxSub;
ClawSubsystem clawSub;
ElevatorSubsystem elevatorSub;
ArmSubsystem armSub;

  public AutoPickerCommand(AutonomousPickerSubsystem autonomousPickerSub, 
  NavxSubsystem navxSub, 
  DrivetrainSubsystem drivetrainSub,
  ClawSubsystem clawSub,
  ElevatorSubsystem elevatorSub,
  ArmSubsystem armSub) {

    // Use addRequirements() here to declare subsystem dependencies.

    this.autonomousPickerSub = autonomousPickerSub;
    this.navxSub = navxSub;
    this.drivetrainSub = drivetrainSub;
    this.clawSub = clawSub;
    this.elevatorSub = elevatorSub;
    this.armSub = armSub;

    addRequirements(autonomousPickerSub, navxSub, drivetrainSub, clawSub, elevatorSub, armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Autonomous started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  
  public Command runAutonomous() {
    Command autoCommands = new SequentialCommandGroup();

    switch(autonomousPickerSub.getAutonomous()) {
      case NONE: 
        System.out.println("No Autonomous");
        autoCommands = new SequentialCommandGroup(
        );
        break;
        
      case CHARGINGSTATION:
        System.out.println("Charging Station Balance Autonomous");
        autoCommands = new SequentialCommandGroup(
          new DriveForwardCommand(driveTrain, m_NavxSubsystem)
        );
        break;

      case
        
      default:
      System.out.println("Default Chosen");
        break;
    }
    return autoCommands;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
