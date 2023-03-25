// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Class: ClawSubsytem
  * Creates a new ClawSubsystem.
  */

public class ClawSubsystem extends SubsystemBase {
  public final CANSparkMax claw;

    /*Method: ClawSubsystem
   * Parameters: None
   * Variables used: claw
   * What it does: Assigns the claw variables their output port
   */

  public ClawSubsystem() {
    // initalize the CAN Motor
    claw = new CANSparkMax(8, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
