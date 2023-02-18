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
  public final XboxController controller;

    /**Method: Arm
   * Parameters: armSubsystems and XboxController
   * Variables used: armSubsystems and controller
   * What it does: Assigns the parameter SubSystem to Subsystem
   *               Assigns the parameter XboxController to controller
   *               Uses addRequirements to tie ArmSubsystem to Arm
   *  */

  public Arm(ArmSubsystem armSubsystem, XboxController controller) {
    this.armSubsystem = armSubsystem;
    this.controller = controller;
    addRequirements(this.armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // sets speed of the motor
    armSubsystem.bend.set(controller.getRawAxis(1));
    // sets speed of the claw 
    armSubsystem.claw.set(controller.getRawAxis(4));
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