// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class Claw extends CommandBase {
  //Creates new ClawSubsystem Object
  public final ClawSubsystem clawSub;

  // Creates new XboxController Object
  public final XboxController controller;

      /**Method: Claw
   * Parameters: ClawSubsystem and XboxController
   * Variables used: clawSub and controller
   * What it does: Assigns the parameter clawSub to ClawSubsystem
   *               Assigns the parameter XboxController to controller
   *               Uses addRequirements to tie ClawSubsystem to Claw
   *  */

  public Claw(ClawSubsystem clawSub, XboxController controller) {
    this.clawSub = clawSub;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clawSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(controller.getLeftTriggerAxis() > .1){
      //If left trigger is pressed then it closes the claw at 1/50 of the speed
      clawSub.claw.set(-controller.getLeftTriggerAxis());
    }else if(controller.getRightTriggerAxis() > .1){
      //If right trigger is pressed then it opens the claw at 1/50 the speed
      clawSub.claw.set(controller.getRightTriggerAxis());
    }else if(controller.getRightTriggerAxis() < .1 || controller.getLeftTriggerAxis() < .1){
      //If no trigger is pulled then no movement
      clawSub.claw.set(0);
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
