// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Class: DriveTrain
   * Creates a new DriveTrain.
   *  */

public class DriveTrain extends SubsystemBase {

// create new CAN Motor objects
private final CANSparkMax lF;
private final CANSparkMax lB;
private final CANSparkMax rF;
private final CANSparkMax rB;

// create new Mechanum Drive variable named mecDrive
public DifferentialDrive arcadeDrive;

//public final static Pigeon2 gyro = new Pigeon2(5);  


/**Method: DriveTrain
   * Parameters: None
   * Variables used: leftBack, leftFront, rightBack, rightFront, and mecDrive
   * What it does: Assigns the CANSparkMax variables their output ports
   *               Assigns the Mecanum variable its Spark outputs
   *  */

  public  DriveTrain() {

 // initalize the CAN Motors
 lF = new CANSparkMax(1,MotorType.kBrushless);
 lB = new CANSparkMax(3,MotorType.kBrushless);
 rF = new CANSparkMax(2,MotorType.kBrushless);
 rB = new CANSparkMax(4,MotorType.kBrushless);

// invert left side Motor
 lF.setInverted(true);
 lB.setInverted(true);

 //groups lF and lB together and groups rF and rB together
 MotorControllerGroup leftSide = new MotorControllerGroup(lB, lF);
 MotorControllerGroup rightSide = new MotorControllerGroup(rB, rF);



 
 // use CAN Motors in new MechanumDrive 
 arcadeDrive = new DifferentialDrive(leftSide,rightSide);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
