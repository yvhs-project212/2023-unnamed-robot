// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLiftWithjoystickCommand extends CommandBase {
  /** Creates a new ElevatorCommand. */

  ElevatorSubsystem elevatorSub;

  public ElevatorLiftWithjoystickCommand(ElevatorSubsystem elevatorSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSub = elevatorSub;
    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSub.elevatorLiftWithJoystick(RobotContainer.operatorController.getRawAxis(Constants.OperatorConstants.OperationBinds.L_Y_AXIS));
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
