// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  private final DoubleSolenoid clawPneumatic;

  
  public ClawSubsystem() {
    clawPneumatic = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 1, 0);
  }

  public Value isClawOpen() {
    return clawPneumatic.get();
  }

  public void toggleClaw() {
    clawPneumatic.toggle();
  }

  public void openClaw () {
    clawPneumatic.set(DoubleSolenoid.Value.kForward);
  }

  public void closeClaw () {
    clawPneumatic.set(DoubleSolenoid.Value.kReverse);
  }

  public void releaseClaw () {
    clawPneumatic.set(DoubleSolenoid.Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
