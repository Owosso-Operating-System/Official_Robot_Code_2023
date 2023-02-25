// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Create new driveTrain Object
  private final DriveTrain driveTrain;
  //Create new controller Object
  private final XboxController controller0;
  //Create new controller Object
  private final XboxController controller1;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // add in new driveTrain
    driveTrain = new DriveTrain();
    // add in new controller
    controller0 = new XboxController(0);
    // add in new controller
    controller1 = new XboxController(1);

    // Assign the BalanceButton command to the "A" button on controller0
    // Assign the LineUp command to the "B" button on controller0
    configureButtonBindings();
    new JoystickButton(controller0, XboxController.Button.kA.value).whileTrue(new BalanceButton(driveTrain, DriveTrain.gyro));

    new JoystickButton(controller0, XboxController.Button.kB.value).whileTrue(new LineUp(driveTrain));
    // set Defualt Command for driveTrain passing in the driveTrain and controller
    driveTrain.setDefaultCommand(new Drive( driveTrain, controller0));
    // Configure the button bindings  
  }

  private void configureButtonBindings() {
  }

   /**Method: GetAutonomousCommand
   * Parameters: N/A
   * Variables used: autonName
   * What it does: Gets the string from the dashboard 
   *               so the correct auton is run.
   *  */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    String autoName = SmartDashboard.getString("Auto Selector", "OneSecondAuton");
    DriveTrain.gyro.setYaw(0);

    switch(autoName){
      case "OneSecondAuton":
        return new OneSecondAuton(driveTrain);
    }
    return null;
  }
}
