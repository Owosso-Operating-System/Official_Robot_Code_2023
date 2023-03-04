// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Arm;
import frc.robot.commands.BalanceButton;
import frc.robot.commands.BasicBalanceAuton;
import frc.robot.commands.Claw;
import frc.robot.commands.DockAuton;
import frc.robot.commands.DockDropAuton;
import frc.robot.commands.Drive;
import frc.robot.commands.LineUp;
import frc.robot.commands.BasicDropOff;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrain;

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
  //Create new armSubsystem Object
  private final ArmSubsystem armSubsystem;
  //Create new clawSubsystem Object
  private final ClawSubsystem clawSubsystem;
  //Create new controller Object
  private final XboxController controller0;
  //Create new controller Object
  private final XboxController controller1;

  private final Pigeon2 gyro;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // add in new driveTrain
    driveTrain = new DriveTrain();
    //add in new armSubsystem
    armSubsystem = new ArmSubsystem();
    //add in new clawSubsystem
    clawSubsystem = new ClawSubsystem();
    // add in new controller
    controller0 = new XboxController(0);
    // add in new controller
    controller1 = new XboxController(1);
    //add in new gyro
    gyro = new Pigeon2(5);

    // set Defualt Command for driveTrain passing in the driveTrain and controller0
    driveTrain.setDefaultCommand(new Drive(driveTrain, controller0));
    // set Defualt Command for armSubsystem passing in the armSubsystem and controller1
    armSubsystem.setDefaultCommand(new Arm(armSubsystem, controller1));
    // set Defualt Command for clawSubsystem passing in the clawSubsystem and controller1
    clawSubsystem.setDefaultCommand(new Claw(clawSubsystem, controller1));
    // Configure the button bindings  
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Assign the BalanceButton command to the "A" button on controller0
    new JoystickButton(controller0, XboxController.Button.kA.value).whileTrue(new BalanceButton(driveTrain, DriveTrain.gyro));
    // Assign the LineUp command to the "B" button on controller0
    new JoystickButton(controller0, XboxController.Button.kB.value).whileTrue(new LineUp(driveTrain));
  }

   /**Method: GetAutonomousCommand
   * Parameters: N/A
   * Variables used: autonName
   * What it does: Gets the string from the dashboard 
   *               so the correct auton is run.
   *  */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    String autoName = SmartDashboard.getString("Auto Selector", "BasicBalanceAuton");

    switch(autoName){
      case "BasicDropOff":
        return new BasicDropOff(driveTrain, armSubsystem, clawSubsystem);
      case "DockDropAuton":
        return new DockDropAuton(driveTrain, armSubsystem, clawSubsystem);
      case "DockAuton":
        return new DockAuton(driveTrain);
      case "BasicBalanceAuton":
        return new BasicBalanceAuton(driveTrain, gyro);
    }
    return null;
  }
}
