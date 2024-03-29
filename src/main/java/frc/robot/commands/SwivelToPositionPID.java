// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwivleSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwivelToPositionPID extends PIDCommand {
  /** Creates a new SwivelToPositionPID. */
  public SwivelToPositionPID(SwivleSubsystem swivleSubsystem, double targetPosition) {
    super(
        // The controller that the command will use
        new PIDController(0.1, 0.0025, 0.0015),
        // This should return the measurement
        swivleSubsystem::getArmPosition,
        // This should return the setpoint (can also be a constant)
        targetPosition,
        // This uses the output
        output -> {
          swivleSubsystem.moveArm(MathUtil.clamp(output, -0.25, 0.25));
        });
    addRequirements(swivleSubsystem);
    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
