// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.PIDTurn;


public class SquareAutonLeft extends CommandBase {

  // Creates drive train object
  private DriveTrain driveTrain;

  /** Creates a new SquareAuton. */
  public SquareAutonLeft(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //safety is false
    driveTrain.mecDrive.setSafetyEnabled(false);

    //goes straight for 1 sec turns 90 stops at 89.99
    driveTrain.mecDrive.driveCartesian(-PIDTurn.getSpeed(driveTrain, 0), 0, 0);    
    Timer.delay(2);
    while(true){
      driveTrain.mecDrive.driveCartesian(0, 0, PIDTurn.getSpeed(driveTrain, -90));
      if(DriveTrain.gyro.getYaw() < -89.99){
        break;
      }
    }

    //goes straight for 1 sec turns 180 stops at 179.99
    driveTrain.mecDrive.driveCartesian(-PIDTurn.getSpeed(driveTrain, 0), 0, 0);
    Timer.delay(2);
    while(true){
      driveTrain.mecDrive.driveCartesian(0, 0, PIDTurn.getSpeed(driveTrain, -180));
      if(DriveTrain.gyro.getYaw() < -179.99){
        break;
      }
    }

    //goes straight for 1 sec turns 270 stops at 269.99
    driveTrain.mecDrive.driveCartesian(-PIDTurn.getSpeed(driveTrain, 0), 0, 0);
    Timer.delay(2);
    while(true){
      driveTrain.mecDrive.driveCartesian(0, 0, PIDTurn.getSpeed(driveTrain, -270));
      if(DriveTrain.gyro.getYaw() < -269.99){
        break;
      }
    }

    //goes straight for 1 sec turns 360 stops a 359.99
    driveTrain.mecDrive.driveCartesian(-PIDTurn.getSpeed(driveTrain, 0), 0, 0);
    Timer.delay(2);
    while(true){
      driveTrain.mecDrive.driveCartesian(0, 0, PIDTurn.getSpeed(driveTrain, -360));
      if(DriveTrain.gyro.getYaw() < -359.99){
        break;
      }
    }
  while(true){
    if(DriveTrain.gyro.getYaw() < 0.5){
      driveTrain.mecDrive.driveCartesian(0, 0, PIDTurn.getSpeed(driveTrain, -360));
    }
    else if(DriveTrain.gyro.getYaw() > -0.5){
      driveTrain.mecDrive.driveCartesian(0, 0, PIDTurn.getSpeed(driveTrain, -360));
    }
    else if(DriveTrain.gyro.getYaw() > -0.25 && DriveTrain.gyro.getYaw() < 0.25){
      break;
    }
  }
  isFinished();
}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
