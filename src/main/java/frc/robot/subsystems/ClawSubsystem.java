// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  private final DoubleSolenoid clawPneumatic;

  private final TalonSRX clawMotor;
  public ClawSubsystem() {
    clawMotor = new TalonSRX(21);
    clawMotor.setNeutralMode(NeutralMode.Brake);
    clawPneumatic = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 1, 0);
    clawPneumatic.set(Value.kReverse);
  }

  public void spinClaw(double speed) {
    clawMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopClaw() {
    clawMotor.set(ControlMode.PercentOutput, 0);
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

  public int isClawLimitSwitchTripped() {
    return clawMotor.isFwdLimitSwitchClosed();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Limit Switch", isClawLimitSwitchTripped());
  }
}
