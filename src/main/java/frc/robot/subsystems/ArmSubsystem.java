// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
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
  
  }
  
  public void armWithJoystick(double armSpeed) {
    armMotor.set(armSpeed * 0.5);
  }

  public void stopMotors() {
    armMotor.set(0);
  }
}


