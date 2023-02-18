// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Class: ClawSubsytem
  * Creates a new ClawSubsystem.
  */

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystems. */
  public final Compressor compi;
  public final Solenoid soli;

    /*Method: ClawSubsystem
   * Parameters: None
   * Variables used: compi and soli
   * What it does: Assigns the compressor and solenoid numerical values
   *               and true or false values
   */

  public ClawSubsystem() {
    compi = new Compressor(0, PneumaticsModuleType.CTREPCM);
    soli = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    compi.enableDigital();
    compi.disable();

    boolean enabled = compi.enabled();
    boolean pressureSwitch = compi.getPressureSwitchValue();
    double current = compi.getCurrent();

    soli.set(true);
    soli.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
