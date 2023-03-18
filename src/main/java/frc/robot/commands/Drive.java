// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/** Class: Drive
   * Creates a new Drive Command.
   *  */

public class Drive extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // Creates new DriveTrain Object named driveTrain 
  public final DriveTrain driveTrain;
  
  // Creates new XboxController Object
  public final XboxController controller;

  /**Method: Drive
   * Parameters: DriveTrain and XboxController
   * Variables used: driveTrain and controller
   * What it does: Assigns the parameter DriveTrain to driveTrain
   *               Assigns the parameter XboxController to controller
   *               Uses addRequirements to tie DriveTrain to Drive
   *  */

  public Drive(DriveTrain driveTrain, XboxController controller) {
    this.driveTrain = driveTrain;
    this.controller = controller;
    addRequirements(this.driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /**Method: Drive
   * Parameters: None
   * Variables used: driveTrain.mecDrive and controller
   * What it does: Takes the controller outputs, passes the values to mecDrive
   *  */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //Left Stick 0 left/right  1 up/down    Right Stick  4 left/right
    if(controller.getRawAxis(1) > .2 || controller.getRawAxis(0) > .2 || controller.getRawAxis(4) > .2 || controller.getRawAxis(0) < -.2 || controller.getRawAxis(1) < -.2 || controller.getRawAxis(4) < -.2){
      driveTrain.mecDrive.driveCartesian(controller.getRawAxis(1)/3, -controller.getRawAxis(0)/3, -controller.getRawAxis(4)/3);
    }else{
      driveTrain.mecDrive.driveCartesian(0, 0, 0);
    }
    //driveTrain.mecDrive.driveCartesian(controller.getRawAxis(1)/3, -controller.getRawAxis(0)/3, -controller.getRawAxis(4)/3);
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
