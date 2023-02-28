// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutonomousPickerSubsystem extends SubsystemBase {
  /** Creates a new AutonomousPickerSubsystem. */

  public enum AutonomousMode {
    NONE, ONE, TWO
  }

  public AutonomousMode startPosition = AutonomousMode.NONE;

  private SendableChooser<AutonomousMode> autonomousModeChooser;

  public AutonomousPickerSubsystem() {
    autonomousModeChooser = new SendableChooser<>();
    autonomousModeChooser.setDefaultOption("None", AutonomousMode.NONE);
    autonomousModeChooser.addOption("One", AutonomousMode.ONE);
    autonomousModeChooser.addOption("Two", AutonomousMode.TWO);
    //Line 23 - Makes autonomous chooser a sendable chooser
    //line 24 - sets the Autonomous chooser as "None" by default
    //lines 25 to 26 adds two options to the autonomous choose named "One" and "Two" 

    SmartDashboard.putData("Autonomous Mode Picker", autonomousModeChooser);
    //
  }

  public AutonomousMode getAutonomousMode() {
    return autonomousModeChooser.getSelected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}