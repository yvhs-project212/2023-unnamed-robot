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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new Arm. */

  public WPI_TalonFX armMotor;
  public double armMotorPos;
  public double armDown;
  public double armUp;
  public double armError;
  
  

  public ArmSubsystem() {
    armMotor = new WPI_TalonFX(Constants.ArmConstants.ARM_MOTOR);
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armMotorPos = armMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("ArmPosition", armMotorPos);
    SmartDashboard.putNumber("ArmDegrees", armMotorPos / Constants.ArmConstants.ENCODER_PER_DEGREE);
    armError = Constants.ArmConstants.AUTONOMOUS_ARM_SETPOINT + armMotorPos / Constants.ArmConstants.ENCODER_PER_DEGREE;
    SmartDashboard.putNumber("ArmError", armError);
  
  }
  
  public void armWithJoystick(double armSpeed) {
    armMotor.set(armSpeed * 0.5);
  }

  public void setArmAngle(double armAngleSetPoint){
    armMotor.set(Constants.ArmConstants.ARM_kP * (armAngleSetPoint - armMotorPos / Constants.ArmConstants.ENCODER_PER_DEGREE));
  }

  public void resetArmEncoder(){
    armMotor.setSelectedSensorPosition(0);
  }



  public void stopMotors() {
    armMotor.set(0);
  }
}


