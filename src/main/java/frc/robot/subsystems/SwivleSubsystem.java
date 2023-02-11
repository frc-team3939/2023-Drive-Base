// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwivleSubsystem extends SubsystemBase {
  /** Creates a new SwivleSubsystem. */
  private final CANSparkMax swivlemotor;
  private final TalonSRX extendMotor;
  private final RelativeEncoder encoder;
  public SwivleSubsystem() {
    swivlemotor = new CANSparkMax(99, null);
    extendMotor = new TalonSRX(33);
    encoder = swivlemotor.getEncoder();
  }

  public void moveArmPosition (double movePosition) {
    encoder.setPosition(movePosition); 
  }

  public void stopArm () {
    swivlemotor.set(0);
  }

  public void moveArm (double moveSpeed) {
    swivlemotor.set(moveSpeed);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
