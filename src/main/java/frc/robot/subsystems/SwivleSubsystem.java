// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwivleSubsystem extends SubsystemBase {
  /** Creates a new SwivleSubsystem. */
  private final CANSparkMax swivlemotor;
  
  private final RelativeEncoder encoder;
  private final RelativeEncoder followencoder;
  private final CANSparkMax swivelFollower;
  public SwivleSubsystem() {
    swivlemotor = new CANSparkMax(36, MotorType.kBrushless);
    swivlemotor.setIdleMode(IdleMode.kBrake);
    swivelFollower = new CANSparkMax(37, MotorType.kBrushless);
    swivelFollower.follow(swivlemotor, true);
    swivelFollower.setIdleMode(IdleMode.kBrake);
    
    encoder = swivlemotor.getEncoder();
    followencoder = swivelFollower.getEncoder();
    SmartDashboard.putNumber("Swivel Target", 0);
  }


  public void zeroSwivelEncoders() {
    encoder.setPosition(0);
    followencoder.setPosition(0);
  }

  public void moveArmPosition (double movePosition) {
    encoder.setPosition(movePosition);  
  }

  

  public void stopArm () {
    swivlemotor.set(0);
  }

  public double getArmPosition() {
    return encoder.getPosition();
  }

  public double getFollowerPosition() {
    return followencoder.getPosition();
  }
  public void moveArm (double moveSpeed) {
    swivlemotor.set(moveSpeed);
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Swivel Encoder Position", getArmPosition());
    SmartDashboard.putNumber("Swivel Follow Encoder Position", getFollowerPosition());
  }
}
