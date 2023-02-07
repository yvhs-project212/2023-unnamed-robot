// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */

  public WPI_TalonSRX leftRollerMotor;
  public WPI_TalonSRX rightRollerMotor;
  public Solenoid leftClawSolenoid;
  public Solenoid rightClawSolenoid;
  public MotorControllerGroup clawMotorControllerGroup;

  public ClawSubsystem() {

    leftRollerMotor = new WPI_TalonSRX(Constants.ClawConstants.LEFT_CLAW_ROLLER_MOTOR);
    leftRollerMotor.setInverted(false);
    rightRollerMotor = new WPI_TalonSRX(Constants.ClawConstants.RIGHT_CLAW_ROLLER_MOTRO);
    rightRollerMotor.setInverted(true);
    clawMotorControllerGroup = new MotorControllerGroup(leftRollerMotor, rightRollerMotor);

    leftClawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClawConstants.LEFT_CLAW_SOLENOID);
    rightClawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,Constants.ClawConstants.RIGHT_CLAW_SOLENOID);


  }


  @Override
  public void periodic() {
    leftClawSolenoid.set(true);
    rightClawSolenoid.set(true);
  }

  public void clawIntake(boolean intakeEnable){
    if(intakeEnable == true){
      leftClawSolenoid.set(false);
      rightClawSolenoid.set(false);
      clawMotorControllerGroup.set(0.2);
    }
  }

  public void clawOuttake(boolean outtakeEnable){
    if(outtakeEnable == true){
      leftClawSolenoid.set(false);
      rightClawSolenoid.set(false);
      clawMotorControllerGroup.set(-0.2);
    }
  }
}
