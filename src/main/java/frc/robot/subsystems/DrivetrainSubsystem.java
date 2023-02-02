// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */

  public WPI_TalonFX leftTopMotor;
  public WPI_TalonFX leftBottomMotor;
  public WPI_TalonFX rightTopMotor;
  public WPI_TalonFX rightBottomMotor;

  public double averageMotorPos;

  public DrivetrainSubsystem() {

    leftTopMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_TOP_MOTOR);
    leftBottomMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_BOTTOM_MOTOR);
    rightTopMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_TOP_MOTOR);
    rightBottomMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_BOTTOM_MOTOR);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double leftTopMotorPos = leftTopMotor.getSelectedSensorPosition();
    double leftBottomMotorPos = leftBottomMotor.getSelectedSensorPosition();
    double rightTopMotorPos = rightTopMotor.getSelectedSensorPosition();
    double rightBottomMotorPos = rightTopMotor.getSelectedSensorPosition();

    averageMotorPos = (leftTopMotorPos + leftBottomMotorPos + rightTopMotorPos + rightBottomMotorPos) / 4;

    
    SmartDashboard.putNumber("AverageMotorEncoderPositionValue", averageMotorPos);
  }

  public void brakeMode() {
    leftTopMotor.setNeutralMode(NeutralMode.Brake);
    leftBottomMotor.setNeutralMode(NeutralMode.Brake);
    rightBottomMotor.setNeutralMode(NeutralMode.Brake);
    rightTopMotor.setNeutralMode(NeutralMode.Brake);
  } 

  public void coastMode() {
    leftTopMotor.setNeutralMode(NeutralMode.Coast);
    leftBottomMotor.setNeutralMode(NeutralMode.Coast);
    rightBottomMotor.setNeutralMode(NeutralMode.Coast);
    rightTopMotor.setNeutralMode(NeutralMode.Coast);
  }
}
