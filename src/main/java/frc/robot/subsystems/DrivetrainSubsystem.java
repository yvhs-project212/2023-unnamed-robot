// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DrivetrainSubsystem extends SubsystemBase {
  private WPI_TalonFX leftTop;
  private WPI_TalonFX rightTop;
  private WPI_TalonFX leftBottom;
  private WPI_TalonFX rightBottom;
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    leftTop = new WPI_TalonFX(Constants.DriveTrainConstants.Left_Top_Motor);
    leftTop = new WPI_TalonFX(Constants.DriveTrainConstants.Right_Top_Motor);
    leftTop = new WPI_TalonFX(Constants.DriveTrainConstants.Left_Bottom_Motor);
    leftTop = new WPI_TalonFX(Constants.DriveTrainConstants.Right_Bottom_Motor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
