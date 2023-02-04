// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavxSubsystem extends SubsystemBase {
  /** Creates a new Navx. */
  private AHRS gyroScope;
  private double yaw;
  private double pitch;
  private double roll;
  private double worldLinearAccelX;
  private double worldLinearAccelY;
  private double worldLinearAccelZ;
   
  public NavxSubsystem() {
    gyroScope = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public void periodic() {
    worldLinearAccelX = gyroScope.getWorldLinearAccelX();
    worldLinearAccelX = gyroScope.getWorldLinearAccelY();
    worldLinearAccelX = gyroScope.getWorldLinearAccelZ();
    yaw = gyroScope.getYaw();
    pitch = gyroScope.getPitch();
    roll = gyroScope.getRoll();

    SmartDashboard.putNumber("YawValue", yaw);
    SmartDashboard.putNumber("PitchValue", pitch);
    SmartDashboard.putNumber("RollValue", roll);
    SmartDashboard.putNumber("getWorldLinearAccelX", worldLinearAccelX);
    SmartDashboard.putNumber("getWorldLinearAccelY", worldLinearAccelY);
    SmartDashboard.putNumber("getWorldLinearAccelZ", worldLinearAccelZ);
          
    // This method will be called once per scheduler run
  }
}
