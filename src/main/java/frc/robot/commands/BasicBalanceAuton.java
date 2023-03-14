// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDBalance;
import frc.robot.subsystems.DriveTrain;

public class BasicBalanceAuton extends CommandBase {

  private DriveTrain driveTrain;

  private Pigeon2 gyro;
  /** Creates a new BasicBalanceAuton. */
  public BasicBalanceAuton(DriveTrain driveTrain, Pigeon2 gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.gyro =  gyro;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveTrain.mecDrive.setSafetyEnabled(false);

    //Go forward to throw cube
    //driveTrain.mecDrive.driveCartesian(-0.25, 0, 0);
    //Timer.delay(0.5); //2.75 + 14"
    //driveTrain.mecDrive.driveCartesian(0, 0, 0);

    //Go backwards to get to charge station/out of the way
    //driveTrain.mecDrive.driveCartesian(0.4, 0, 0);
    //Timer.delay(2);

    //Balance?
    /*while(Timer.getFPGATimestamp() < 14.5){
      if(gyro.getPitch() >= 1.5){
        driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
      }
      else if(gyro.getPitch() <= -1.5){
        driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
      }
    }*/
    while(true){
      if(gyro.getPitch() <= -1.5){
        driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
      }
      else if(gyro.getPitch() >= 1.5){
        driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
      }
      else if (gyro.getUpTime() > 14.5){
        isFinished();
      }
    }
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
