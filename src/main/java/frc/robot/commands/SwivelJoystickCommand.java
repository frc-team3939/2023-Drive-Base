// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwivleSubsystem;

public class SwivelJoystickCommand extends CommandBase {
  /** Creates a new SwivelJoystickCommand. */
  
  private final SwivleSubsystem swivleSubsystem;
  private final Supplier<Double> yAxisFunction;
  public SwivelJoystickCommand(SwivleSubsystem subsystem, Supplier<Double> yAxisTilt) {
    swivleSubsystem = subsystem;
    yAxisFunction = yAxisTilt;

    addRequirements(swivleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yAxis = yAxisFunction.get();
    if (Math.abs(yAxis) > OIConstants.kSwivelDeadband) { 
      SmartDashboard.putBoolean("Is Deadband on?", true);
      SmartDashboard.putNumber("swivelyAxisOutput", yAxis);
      swivleSubsystem.moveArm(yAxis * OIConstants.kSwivelDeadband);
    } else {
      SmartDashboard.putBoolean("Is Deadband on?", false);
      SmartDashboard.putNumber("swivelyAxisOutput", 0);
      swivleSubsystem.moveArmPosition(swivleSubsystem.getArmPosition());
    }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swivleSubsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
