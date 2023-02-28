// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDBalance;
import frc.robot.subsystems.DriveTrain;

/** Class: BalanceButton
   * Creates a new BalanceButton Command.
   *  */

public class BalanceButton extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // Creates new DriveTrain Object named driveTrain 
  public final DriveTrain driveTrain;

  // Creates new Pigeon2 Object
  public final Pigeon2 gyro;

  /**Method: BalanceButton
   * Parameters: DriveTrain, XboxController, and Pigeon2
   * Variables used: driveTrain, controller, and gyro
   * What it does: Assigns the parameter DriveTrain to driveTrain
   *               Assigns the parameter XboxController to controller
   *               Assigns the parameter Pigeon2 to gyro
   *               Uses addRequirements to tie DriveTrain to BalanceButton
   *  */

  public BalanceButton(DriveTrain driveTrain, Pigeon2 gyro) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    // add requirments to call drivetrain
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //If the robot is more than ±1.5° it will balance itself.
    if(gyro.getPitch() >= 1.5){
        driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
    }
    else if(gyro.getPitch() <= -1.5){
      driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
