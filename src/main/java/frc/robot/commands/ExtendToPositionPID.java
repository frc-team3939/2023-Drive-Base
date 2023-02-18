// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ExtensionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendToPositionPID extends PIDCommand {
  /** Creates a new ExtendToPositionPID. */
  public ExtendToPositionPID(double targetPosition, ExtensionSubsystem subsystem) {
    super(
        // The controller that the command will use
        new PIDController(0.007, 0.0000000001, 0),
        // This should return the measurement
        subsystem::getExtensionEncoder,
        // This should return the setpoint (can also be a constant)
        targetPosition,
        // This uses the output
        output -> {
          subsystem.extendArmSpeed(output);
        });
    addRequirements(subsystem);
    getController().disableContinuousInput();
    getController().setTolerance(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
