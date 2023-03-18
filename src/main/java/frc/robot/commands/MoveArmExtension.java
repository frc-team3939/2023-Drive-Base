// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class MoveArmExtension extends CommandBase {
  /** Creates a new MoveArmExtension. */
  private final ExtensionSubsystem extendSubsystem;
  double encoderPosition;
  public MoveArmExtension(int encoderPosition, ExtensionSubsystem subsystem) {
    extendSubsystem = subsystem;
    this.encoderPosition = encoderPosition;
    addRequirements(extendSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extendSubsystem.extendArmPosition(encoderPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
