// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class HomeExtensionSystem extends CommandBase {
  /** Creates a new HomeExtensionSystem. */
  private final ExtensionSubsystem extendSubsystem;
  public HomeExtensionSystem(ExtensionSubsystem subsystem) {
    extendSubsystem = subsystem;
    addRequirements(extendSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extendSubsystem.extendArmSpeed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extendSubsystem.extendArmSpeed(0);
    extendSubsystem.resetExtensionEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (extendSubsystem.isLimitSwitchTripped() == 1) 
      return true;
    else
      return false;
  }
}
