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
  //Creates new DriveTrain Object named driveTrain
  private DriveTrain driveTrain;
  //Creates new ArmSubsystem Object named armSubsystem
  private ArmSubsystem armSubsystem;
  //Creates new ClawSubsystem Object named clawSubsystem
  private ClawSubsystem clawSubsystem;
  //Creates new Pigeon2 Object named gyro
  private Pigeon2 gyro;

    /**Method: ForChristian
   * Parameters: DriveTrain, ArmSubsystem, ClawSubsystem, and Pigeon2
   * Variables used: driveTrain, armSubsystem, clawSubsystem, and gyro
   * What it does: Assigns the parameter DriveTrain to driveTrain
   *               Assigns the parameter ClawSubsystem to clawSubsystem
   *               Assigns the parameter ArmSubsystem to armSubsystem
   *               Assigns the parameter Pigeon2 to gyro
   *               Uses addRequirements to tie DriveTrain to ForChristian
   *  */
  public ForChristian(DriveTrain driveTrain, Pigeon2 gyro, ClawSubsystem clawSubsystem, ArmSubsystem armSubsystem) {
    this.driveTrain = driveTrain;
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    this.gyro = gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, clawSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Back for 1 second
    driveTrain.mecDrive.driveCartesian(-1, 0, PIDTurn.getSpeed(driveTrain, 0));
    Timer.delay(.1);

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
    clawSubsystem.claw.set(0);
    driveTrain.mecDrive.driveCartesian(0, 0, PIDTurn.getSpeed(driveTrain, 180)); 
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
    int Time = 13000; 

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
    //Auton stops, Finishing
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
