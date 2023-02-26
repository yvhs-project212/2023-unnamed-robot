// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */

  //Initializing Motors And Solenoids
  public WPI_TalonSRX leftRollerMotor;
  public WPI_TalonSRX rightRollerMotor;
  public Solenoid clawSolenoid;
  public MotorControllerGroup clawMotorControllerGroup;

  public DigitalInput clawLimitSwitch;
  public boolean clawLimitEnable;
  
  public ClawSubsystem() {

    leftRollerMotor = new WPI_TalonSRX(Constants.ClawConstants.LEFT_CLAW_ROLLER_MOTOR);
    leftRollerMotor.setInverted(false);
    rightRollerMotor = new WPI_TalonSRX(Constants.ClawConstants.RIGHT_CLAW_ROLLER_MOTOR);
    rightRollerMotor.setInverted(true);
    clawMotorControllerGroup = new MotorControllerGroup(leftRollerMotor, rightRollerMotor);

    clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClawConstants.CLAW_SOLENOID);
    clawSolenoid.set(true);

    clawLimitSwitch = new DigitalInput(Constants.ClawConstants.CLAW_LIMIT_SWITCH);
 
  }


  @Override
  public void periodic() {
    if (clawLimitSwitch.get()){
      clawLimitEnable = false;
    } else{
      clawLimitEnable = true;
    }
    SmartDashboard.putBoolean("clawLimitSwitch", clawLimitEnable);
  }

  public void clawIntake(){
    if(clawLimitSwitch.get()){
        clawSolenoid.set(false);
        clawMotorControllerGroup.set(-0.5);
    } else {
        clawSolenoid.set(true);
        clawMotorControllerGroup.set(0);
    }
  }

  public void clawOpen(){
    clawSolenoid.set(false);
  }

  public void clawRollersOuttake(){
    clawMotorControllerGroup.set(0.5);
  }

  public void clawClose(){
    clawSolenoid.set(false);
  }

  public void clawRollersStop(){
    clawMotorControllerGroup.set(0);
  }
}
