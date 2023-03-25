// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
public final MecanumDrive mecDrive;

public final static Pigeon2 gyro = new Pigeon2(5);  

/**Method: DriveTrain
   * Parameters: None
   * Variables used: leftBack, leftFront, rightBack, rightFront, and mecDrive
   * What it does: Assigns the CANSparkMax variables their output ports
   *               Assigns the Mecanum variable its Spark outputs
   *  */

  public DriveTrain() {

 // initalize the CAN Motors
 lF = new CANSparkMax(1,MotorType.kBrushless);
 lB = new CANSparkMax(3,MotorType.kBrushless);
 rF = new CANSparkMax(2,MotorType.kBrushless);
 rB = new CANSparkMax(4,MotorType.kBrushless);

 

 // invert left side Motor
 lF.setInverted(true);
 lB.setInverted(true);


 // use CAN Motors in new MechanumDrive 
 mecDrive = new MecanumDrive(lF, lB, rF, rB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //Outputs the Yaw value onto the driver station
  //With a value between 0 and 360
  public static void updateYaw(){
   double value = gyro.getYaw();
    if(value > 360){
      value = value % 360;
    }
    else if(value < 0 && value > -360){
      value = value + 360;
    }
    else if(value < 0 && value < -360){
      value = (value % 360) + 360;
    }

    SmartDashboard.putNumber("Gyro Yaw", value);
  }
  //Outputs the Pitch value onto the driver station
  //With a value between 0 and 360
  public static void updatePitch(){
    double value = gyro.getPitch();
     if(value > 360){
       value = value % 360;
     }
     else if(value < 0 && value > -360){
       value = value + 360;
     }
     else if(value < 0 && value < -360){
       value = (value % 360) + 360;
     }
 
     SmartDashboard.putNumber("Gyro Pitch", value);
   }
}
