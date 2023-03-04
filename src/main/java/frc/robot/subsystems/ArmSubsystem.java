// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new Arm. */

  public WPI_TalonFX armMotor;
  public double armMotorPos;
  public double inPlaceArmMotorPos;
  public double armDown;
  public double armUp;
  public boolean armMoving;
  
  

  public ArmSubsystem() {
    armMotor = new WPI_TalonFX(Constants.ArmConstants.ARM_MOTOR);
    armMotor.setNeutralMode(NeutralMode.Brake);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armMotorPos = armMotor.getSelectedSensorPosition();
    if (inPlaceArmMotorPos > armMotorPos) {
      armMotor.set(-0.3);
    } else if (inPlaceArmMotorPos <= armMotorPos) {
      armMotor.set(0);
    }
  
  }
  
  public void armWithJoystick(double armSpeed) {
    armMotor.set(armSpeed * 0.5);
    if (armSpeed > 0.01 || armSpeed < -0.01){
      inPlaceArmMotorPos = armMotorPos; 
    }
  }

  public void stopMotors() {
    armMotor.set(0);
  }
}