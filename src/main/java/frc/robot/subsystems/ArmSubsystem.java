// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/* Class: ArmSubsytem
  * Creates a new ArmSubsystem.
  */

public class ArmSubsystem extends SubsystemBase {
  //Creates new CANSparkMax Motors
  public final CANSparkMax bend;
  public final CANSparkMax extend;

  
  /*Method: ArmSubsystem
   * Parameters: None
   * Variables used: bend and extend
   * What it does: Assigns the CANSparkMax variables their output ports
   */

  public ArmSubsystem() {
    // initalize the CAN Motors
    extend = new CANSparkMax(6, MotorType.kBrushless);
    bend = new CANSparkMax(7, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
