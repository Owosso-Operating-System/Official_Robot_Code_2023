// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class Arm extends CommandBase {
  // crates new armSubsystem object
  public final ArmSubsystem armSubsystem;
  // Creates new XboxController Object
  public final XboxController controller1;

    /**Method: Arm
   * Parameters: armSubsystems and XboxController
   * Variables used: armSubsystems and controller
   * What it does: Assigns the parameter SubSystem to Subsystem
   *               Assigns the parameter XboxController to controller
   *               Uses addRequirements to tie ArmSubsystem to Arm
   *  */

  public Arm(ArmSubsystem armSubsystem, XboxController controller1) {
    this.armSubsystem = armSubsystem;
    this.controller1 = controller1;
    // add requirments to call arm subsystem 
    addRequirements(this.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // sets speed of the bend
    if(controller1.getRawAxis(1) > 0.1){
      armSubsystem.bend.set(controller1.getRawAxis(1)/2);
    }
    // sets speed of the extend
    else if(controller1.getRawAxis(5) > 0.1){
      armSubsystem.extend.set(controller1.getRawAxis(5)/2);
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
