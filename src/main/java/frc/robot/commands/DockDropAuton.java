// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDBalance;
import frc.robot.PIDTurn;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrain;

public class DockDropAuton extends CommandBase {
  /** Creates a new BasicDockAuton. */
  private DriveTrain driveTrain;
  private ArmSubsystem armSubsystem;
  private ClawSubsystem clawSubsystem;

  /** Creates a new OneSecondAuton. */
  public DockDropAuton(DriveTrain driveTrain, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    this.driveTrain = driveTrain;
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;

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
    while(Timer.getMatchTime() < 15){
      driveTrain.mecDrive.driveCartesian(PIDBalance.getSpeed(driveTrain, 0), 0, 0);
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
