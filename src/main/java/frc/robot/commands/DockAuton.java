// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDBalance;
import frc.robot.PIDTurn;
import frc.robot.subsystems.DriveTrain;

public class DockAuton extends CommandBase {
  // Creates new DriveTrain Object named driveTrain 
  private DriveTrain driveTrain;

        /**Method: Claw
   * Parameters: ClawSubsystem and XboxController
   * Variables used: clawSub and controller
   * What it does: Assigns the parameter clawSub to ClawSubsystem
   *               Assigns the parameter XboxController to controller
   *               Uses addRequirements to tie ClawSubsystem to Claw
   *  */

  public DockAuton(DriveTrain driveTrain) {
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
    //Disables safety
    driveTrain.mecDrive.setSafetyEnabled(false);
    //Moves robot onto station
    driveTrain.mecDrive.driveCartesian(.25, 0, PIDTurn.getSpeed(driveTrain, 0));
    Timer.delay(2);
    //Stops movement of robot
    driveTrain.mecDrive.driveCartesian(0, 0, 0);
    //Starts to balance robot
    while(Timer.getMatchTime() < 15){
      driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
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
