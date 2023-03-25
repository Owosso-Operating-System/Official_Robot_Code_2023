// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDBalance;
import frc.robot.PIDTurn;
import frc.robot.subsystems.DriveTrain;

public class DockAuton extends CommandBase {
  private DriveTrain driveTrain;
  /** Creates a new DockAuton. */

  private Pigeon2 gyro;
  public DockAuton(DriveTrain driveTrain, Pigeon2 gyro) {
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
    //Disables safety
    driveTrain.mecDrive.setSafetyEnabled(false);
    //Moves the robot forewards fast to drop a cube off the back
    driveTrain.mecDrive.driveCartesian(1, 0, 0);
    Timer.delay(0.3);
    //Moves robot onto station
    driveTrain.mecDrive.driveCartesian(.25, 0, PIDTurn.getSpeed(driveTrain, 0));
    Timer.delay(2);
    //Stops movement of robot
    driveTrain.mecDrive.driveCartesian(0, 0, 0);
    //Starts to balance robot
    int Time = 30000; 

    for(int i = 0 ; i < Time ;i++){
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
