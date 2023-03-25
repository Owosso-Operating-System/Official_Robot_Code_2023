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

public class DockDropAuton extends CommandBase {
  //Creates new DriveTrain Object named driveTrain 
  private DriveTrain driveTrain;
  //Creates new ArmSubsystem Object named armSubsystem
  private ArmSubsystem armSubsystem;
  //Creates new ClawSubsystem Object named clawSubsystem 
  private ClawSubsystem clawSubsystem;
  //Creates new Pigeon2 Object named gyro
  private Pigeon2 gyro;

  /**Method: DockDropAuton
   * Parameters: DriveTrain and Pigeon2
   * Variables used: driveTrain and gyro
   * What it does: Assigns the parameter DriveTrain to driveTrain
   *               Assigns the parameter Pigeon2 to gyro
   *               Assigns the parameter ArmSubsystem to armSubsystem
   *               Assigns the parameter ClawSubsystem to clawSubsystem
   *               Uses addRequirements to tie DriveTrain to BalanceButton
   *  */
  public DockDropAuton(DriveTrain driveTrain, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, Pigeon2 gyro) {
    this.driveTrain = driveTrain;
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    this.gyro = gyro;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, armSubsystem, clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Disables Safety
    driveTrain.mecDrive.setSafetyEnabled(false);
    
    //Closes claw on cone
    clawSubsystem.claw.set(.25);
    Timer.delay(.2);

    //stops claw
    clawSubsystem.claw.set(0);

    //moves arm up
    armSubsystem.bend.set(.1);

    //moves robot forward in a straight line
    driveTrain.mecDrive.driveCartesian(.25, 0, PIDTurn.getSpeed(driveTrain, 0));
    Timer.delay(2.5);

    //stops bending the arm
    armSubsystem.bend.set(0);

    //extends the arm out
    armSubsystem.extend.set(.1);

    //stops the arm
    driveTrain.mecDrive.driveCartesian(0, 0, 0);
    Timer.delay(1);

    //opens the claw
    clawSubsystem.claw.set(-.1);
    Timer.delay(.2);
    
    //makes the bot move backwards
    driveTrain.mecDrive.driveCartesian(-.25, 0, PIDTurn.getSpeed(driveTrain, 0));

    //retracks the arm
    armSubsystem.extend.set(-.05);
    Timer.delay(2);

    //Balances robot
    int Time = 17000; 

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
    //stops the robot, ends auton
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
