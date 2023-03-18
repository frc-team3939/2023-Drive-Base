// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class SpinUntilLimitClaw extends CommandBase {
  /** Creates a new SpinUntilLimitClaw. */
  private final ClawSubsystem clawSubsystem;
  public SpinUntilLimitClaw(ClawSubsystem clawSubsystem) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawSubsystem.spinClaw(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSubsystem.spinClaw(0.07);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return clawSubsystem.isClawLimitSwitchTripped() == 1 ? true : false;
  }
}