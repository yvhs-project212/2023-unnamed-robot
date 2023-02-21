// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  public WPI_TalonFX elevatorMotor;

  public DigitalInput upperLimitSwitch;
  public boolean upperLimitSwitchEnable;
  public DigitalInput bottomLimitSwitch;
  public boolean bottomLimitSwitchEnable;

  public double elevatorMotorPos;

  public ElevatorSubsystem() {
    //Setting up elevator motor.
    elevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR);
    elevatorMotor.setInverted(true);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    //Setting up the top and the bottom limit switches.
    upperLimitSwitch = new DigitalInput(Constants.ElevatorConstants.UPPER_ELEVATOR_LIMIT_SWITCH);
    if(upperLimitSwitch.get()){
      upperLimitSwitchEnable = true;
    }
    bottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.BOTTOM_ELEVATOR_LIMIT_SWITCH);
    if(bottomLimitSwitch.get()){
      bottomLimitSwitchEnable = true;
    }

    //Read the elevator motor encoder position value.
    elevatorMotorPos = elevatorMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("ElevatorMotorPosition", elevatorMotorPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Move elevator by using joystick methods.
  public void elevatorLiftWithJoystick(double elevatorSpeed){
    elevatorMotor.set(0);
    if(upperLimitSwitchEnable != true){
      if (elevatorSpeed > 0){
        elevatorMotor.set(elevatorSpeed * 0.4);
      }
    }

    if(bottomLimitSwitchEnable != true){
      if(elevatorSpeed < 0){
        elevatorMotor.set(elevatorSpeed * 0.4);
      }
    }
  }
}
