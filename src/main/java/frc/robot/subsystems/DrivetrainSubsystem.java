// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */

  public WPI_TalonFX leftTopMotor;
  public WPI_TalonFX leftBottomMotor;
  public WPI_TalonFX rightTopMotor;
  public WPI_TalonFX rightBottomMotor;

  public MotorControllerGroup leftMotorGroup;
  public MotorControllerGroup rightMotorGroup;
  public DifferentialDrive diffDrive;

  public double leftTopMotorPos;
  public double leftBottomMotorPos;
  public double rightTopMotorPos;
  public double rightBottomMotorPos;
  public double averageMotorPos;

  public DrivetrainSubsystem() {

    leftTopMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_TOP_MOTOR);
    leftBottomMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_BOTTOM_MOTOR);
    rightTopMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_TOP_MOTOR);
    rightBottomMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_BOTTOM_MOTOR);

    leftMotorGroup = new MotorControllerGroup(leftTopMotor, leftBottomMotor);
    leftMotorGroup.setInverted(false);
    rightMotorGroup = new MotorControllerGroup(rightTopMotor, rightBottomMotor);
    rightMotorGroup.setInverted(true);

    diffDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftTopMotorPos = leftTopMotor.getSelectedSensorPosition();
    leftBottomMotorPos = leftBottomMotor.getSelectedSensorPosition();
    rightTopMotorPos = rightTopMotor.getSelectedSensorPosition();
    rightBottomMotorPos = rightTopMotor.getSelectedSensorPosition();

    averageMotorPos = (leftTopMotorPos + leftBottomMotorPos + rightTopMotorPos + rightBottomMotorPos) / 4;

    SmartDashboard.putNumber("AverageMotorEncoderPositionValue", averageMotorPos);
  }

  public void driveWithJoysticks(double leftThrottle, double rightThrottle, double turn){
    double throttle = rightThrottle - leftThrottle;
    double forwardSpeed = throttle * 0.8;
    double turnSpeed = turn * 0.8;
    diffDrive.arcadeDrive(forwardSpeed, turnSpeed);
  }
}
