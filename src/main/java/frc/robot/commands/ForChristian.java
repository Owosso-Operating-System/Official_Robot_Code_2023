// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDBalance;
import frc.robot.PIDTurn;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrain;

public class ForChristian extends CommandBase {
  /** Creates a new ForChristian. */
  private DriveTrain driveTrain;
  private ArmSubsystem armSubsystem;
  private ClawSubsystem clawSubsystem;
  private Pigeon2 gyro;

  public ForChristian(DriveTrain driveTrain, Pigeon2 gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Back for 1 second
    driveTrain.mecDrive.driveCartesian(-.25, 0, PIDTurn.getSpeed(driveTrain, 0));
    Timer.delay(1);
    //Forwards toward Cube
    driveTrain.mecDrive.driveCartesian(.25, 0, PIDTurn.getSpeed(driveTrain, 0));
    Timer.delay(2);
    //Pick-Up and Raise arm
    driveTrain.mecDrive.driveCartesian(0, 0, 0);
    clawSubsystem.claw.set(.1);
    Timer.delay(.5);
    clawSubsystem.claw.set(0);
    armSubsystem.bend.set(.1);
    Timer.delay(.5);
    //Rotate 180°
    driveTrain.mecDrive.driveCartesian(0, 0, PIDTurn.getSpeed(driveTrain, 180)); // I thought that the third number was how much it turned so right now it goes in a curved  arc shaped 
    Timer.delay(1);
    //Forward to links
    driveTrain.mecDrive.driveCartesian(.25, 0, PIDTurn.getSpeed(driveTrain, 180));
    Timer.delay(2);
    //Stop and Drop cube
    driveTrain.mecDrive.driveCartesian(0, 0, 0);
    armSubsystem.extend.set(.1);
    Timer.delay(.5);
    armSubsystem.extend.set(0);
    clawSubsystem.claw.set(-.1);
    Timer.delay(.5);
    //Back up a little
    driveTrain.mecDrive.driveCartesian(-.1, 0, 0);
    Timer.delay(.5);
    //Strafe to balance
    driveTrain.mecDrive.driveCartesian(0, 0.25, PIDTurn.getSpeed(driveTrain, 180));
    Timer.delay(1);
    //Back Up onto Balance Beam
    driveTrain.mecDrive.driveCartesian(0, 0, 0);
    Timer.delay(.1);
    driveTrain.mecDrive.driveCartesian(-.33, 0, 0);
    Timer.delay(.9);
    //PIDBalance
    int Time = 15000; 

    for(int i = 0 ; i < Time ;i++){
      if(gyro.getPitch() <= -1.5){
        driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
      }
      else if(gyro.getPitch() >= 1.5){
        driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
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
