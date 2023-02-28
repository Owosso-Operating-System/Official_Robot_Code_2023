// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.PIDBalance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrain;

public class BasicDropOff extends CommandBase {

  private DriveTrain driveTrain;

  private ArmSubsystem armSubsystem;

  private ClawSubsystem clawSubsystem;

  /** Creates a new BasicDropOff. */
  public BasicDropOff(DriveTrain driveTrain, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    this.driveTrain = driveTrain;
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    // add requierments to call drivetrain
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

    //Clamps the claw slightly
    clawSubsystem.claw.set(-0.1);
    Timer.delay(0.1);
    clawSubsystem.claw.set(0);

    //Moves the bot foreward and raises the arm
    driveTrain.mecDrive.driveCartesian(0.25, 0, 0);
    armSubsystem.bend.set(0.5);
    Timer.delay(2);


    //Stops the bot and arm and aligns the robot with the pole
    driveTrain.mecDrive.driveCartesian(0, 0, 0);
    armSubsystem.bend.set(0);
    while(LimeLight.getX() !=0){
      new LineUp(driveTrain);
    }

    //Extends the arm
    armSubsystem.extend.set(0.5);
    Timer.delay(1);

    //Stops extending the arm and lowers the arm while opening the claw
    armSubsystem.extend.set(0);
    armSubsystem.bend.set(-0.1);
    clawSubsystem.claw.set(-0.1);
    Timer.delay(1);

    //Stops the arm and claw and drives backwards
    armSubsystem.bend.set(0);
    clawSubsystem.claw.set(0);
    driveTrain.mecDrive.driveCartesian(-0.25, 0, 0);
    Timer.delay(2);

    //Stops everything
    driveTrain.mecDrive.driveCartesian(0, 0, 0);
    //Balances robot
    while(Timer.getMatchTime() < 15){
      driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
    }
    Timer.delay(5.25);

    //Ends the auton 
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
