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
  //Creates new DriveTrain Object named driveTrain
  private DriveTrain driveTrain;
  //Creates new Pigeon2 Object named gyro
  private Pigeon2 gyro;

   /**Method: BasicBalanceAuton
   * Parameters: DriveTrain and Pigeon2
   * Variables used: driveTrain and gyro
   * What it does: Assigns the parameter DriveTrain to driveTrain
   *               Assigns the parameter Pigeon2 to gyro
   *               Uses addRequirements to tie DriveTrain to BasicBalanceAuton
   *  */
  public BasicBalanceAuton(DriveTrain driveTrain, Pigeon2 gyro) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveTrain.mecDrive.setSafetyEnabled(false);

    //Go backwards to get to charge station/out of the way
    driveTrain.mecDrive.driveCartesian(0.4, 0, 0);
    Timer.delay(0.2);
    driveTrain.mecDrive.driveCartesian(-0.5, 0, 0);
    Timer.delay(1);

    //Sets a start value of Time
    int Time = 25000; 

    //As time pass, increase i value until equal to Time
    for(int i = 0 ; i < Time ;i++){
      //while i is less than Time, balance the bot
      if(gyro.getPitch() <= -1.5){
        driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
      }
      else if(gyro.getPitch() >= 1.5){
        driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
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
