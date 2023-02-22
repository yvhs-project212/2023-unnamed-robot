// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;


public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsytem. */

  public WPI_TalonFX turretMotor;
  public double turretMotorPos;
  public DigitalInput leftLimitSwitch;
  public DigitalInput rightLimitSwitch;

  public TurretSubsystem() {
    turretMotor = new WPI_TalonFX(Constants.TurretConstants.TURRET_MOTOR);
    turretMotor.setInverted(true);
    turretMotor.setNeutralMode(NeutralMode.Brake);   
    leftLimitSwitch = new DigitalInput(Constants.TurretConstants.LEFT_LIMITSWITCH);
    rightLimitSwitch = new DigitalInput(Constants.TurretConstants.RIGHT_LIMITSWITCH);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    turretMotorPos = turretMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("turretMotorPosition", Math.floor(100*turretMotorPos+0.5)/100.0);
  }

  public void rotate(double turretSpeed) {
    if (turretSpeed < 0){
      if (leftLimitSwitch.get()){
        turretMotor.set(turretSpeed * -0.2);
      } else {
        turretMotor.set(0);
      }
    } else {
      if (rightLimitSwitch.get()){
        turretMotor.set(turretSpeed * -0.2);
      } else {
        turretMotor.set(0);
      }
    }
  }


}