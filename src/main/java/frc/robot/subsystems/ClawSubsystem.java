// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  private final DoubleSolenoid clawPneumatic;

  private final CANSparkMax clawMotor;
  private final DigitalInput clawLimitSwitch;

  public ClawSubsystem() {
    clawMotor = new CANSparkMax(39, MotorType.kBrushless);
    clawMotor.setIdleMode(IdleMode.kBrake);
    clawLimitSwitch = new DigitalInput(9);
    clawPneumatic = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 1, 0);
    clawPneumatic.set(Value.kReverse);
  }

  public void spinClaw(double speed) {
    clawMotor.set(speed);
  }

  public void stopClaw() {
    clawMotor.set(0);
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

  public boolean isClawLimitSwitchTripped() {
    return clawLimitSwitch.get();
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Claw Limit Switch", isClawLimitSwitchTripped());
  }
}
