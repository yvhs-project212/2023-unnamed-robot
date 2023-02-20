// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonomousPickerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;
import frc.robot.subsystems.TurretSubsytem;

public class AutonomousCommand extends CommandBase {
  /** Creates a new AutonomousCommand. */

  private final AutonomousPickerSubsystem autonomousPicker;
  private final DrivetrainSubsystem driveTrain;
  private final NavxSubsystem navX;
  private final TurretSubsytem turret;


  public AutonomousCommand(AutonomousPickerSubsystem autonomousPicker, DrivetrainSubsystem driveTrain, 
  NavxSubsystem navX, TurretSubsytem turret) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.autonomousPicker = autonomousPicker;
    this.driveTrain = driveTrain;
    this.navX = navX;
    this.turret = turret;

    addRequirements(autonomousPicker, driveTrain, navX, turret);
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
    Command AutoCommands = new SequentialCommandGroup();

    switch(autonomousPicker.getAutonomousMode()) { //switch (Statement) ==> Switch what you do based on what you input/enter 
    case NONE: //This is the input of the user and the if of an if statement
      System.out.println("No autonomous");
      AutoCommands = new SequentialCommandGroup(
      );
      break;

    case ONE:
      System.out.println("Forward autonomous");
      AutoCommands = new SequentialCommandGroup(
        new DriveForwardCommand(navX, turret, driveTrain, autonomousPicker)
      );
      break;
      default:
        break;
    }
    return AutoCommands;
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
