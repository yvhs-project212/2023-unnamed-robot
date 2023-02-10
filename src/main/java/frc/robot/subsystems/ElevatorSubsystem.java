// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  public WPI_TalonFX elevatorMotor;

  public double elevatorMotorPos;

  public ElevatorSubsystem() {
    elevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR);
    elevatorMotor.setInverted(true);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorMotorPos = elevatorMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("ElevatorMotorPosition", elevatorMotorPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void liftWithJoystick(double elevatorSpeed){
    elevatorMotor.set(elevatorSpeed * 0.4);
  }
}
