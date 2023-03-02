// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  public double roundedMotorPos;

  public double lastTimestamp = 0;
  public double lastError = 0;

  public DrivetrainSubsystem() {

    leftTopMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_TOP_MOTOR);
    leftBottomMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_BOTTOM_MOTOR);
    rightTopMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_TOP_MOTOR);
    rightBottomMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_BOTTOM_MOTOR);

    //Group two left motors together and set their neutral mode as brake mode.
    leftTopMotor.setInverted(true);
    leftBottomMotor.setInverted(true);
    leftMotorGroup = new MotorControllerGroup(leftTopMotor, leftBottomMotor);
    leftTopMotor.setNeutralMode(NeutralMode.Brake);
    leftBottomMotor.setNeutralMode(NeutralMode.Brake);

    //Group two right motors together and set their neutral mode as brake mode.
    rightMotorGroup = new MotorControllerGroup(rightTopMotor, rightBottomMotor);
    rightMotorGroup.setInverted(false);
    rightBottomMotor.setNeutralMode(NeutralMode.Brake);
    rightTopMotor.setNeutralMode(NeutralMode.Brake);

    //Created differential drive by using left motors and right motors.
    diffDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    leftTopMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_TOP_MOTOR);
    leftBottomMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_BOTTOM_MOTOR);
    rightTopMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_TOP_MOTOR);
    rightBottomMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_BOTTOM_MOTOR);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Get the average motor encoder value.
    leftTopMotorPos = leftTopMotor.getSelectedSensorPosition();
    leftBottomMotorPos = leftBottomMotor.getSelectedSensorPosition();
    rightTopMotorPos = rightTopMotor.getSelectedSensorPosition();
    rightBottomMotorPos = rightTopMotor.getSelectedSensorPosition();

    averageMotorPos = (leftTopMotorPos + leftBottomMotorPos + rightTopMotorPos + rightBottomMotorPos) / 4;
    roundedMotorPos = Math.floor(averageMotorPos + 0.5);

    SmartDashboard.putNumber("dtPos", roundedMotorPos);
  }

  public void driveWithJoysticks(double leftThrottle, double rightThrottle, double turn){
    double throttle = rightThrottle - leftThrottle;
    double forwardSpeed = throttle * 0.8;
    double turnSpeed = turn * 0.8;
    diffDrive.arcadeDrive(forwardSpeed, turnSpeed);
  }

  public void driveForward(double driveForwardSpeed){
    leftMotorGroup.set(driveForwardSpeed);
    rightMotorGroup.set(driveForwardSpeed);
  }

  public void chargingStationBalancingWithPID(double kP, double kD, double pitchError){
    double timeChanges = Timer.getFPGATimestamp() - lastTimestamp;
    double errorRate = (pitchError - lastError) / timeChanges;
    leftMotorGroup.set(kP * pitchError * kD * errorRate);
    rightMotorGroup.set(kP * pitchError * kD * errorRate);
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = pitchError;
  }

  public void updateTimestamp(){
    lastTimestamp = Timer.getFPGATimestamp();
  }

}
