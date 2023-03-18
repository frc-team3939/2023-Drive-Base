// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase {
  /** Creates a new ExtensionSubsystem. */
  private final TalonSRX extendMotor;
  private final DigitalInput extensionLimitSwitch;
  public ExtensionSubsystem() {
    extendMotor = new TalonSRX(33);
    extendMotor.setNeutralMode(NeutralMode.Brake);
    extensionLimitSwitch = new DigitalInput(8);
  }

  public void extendArmPosition (double movePosition) {
    extendMotor.set(ControlMode.Position, movePosition);
  }

  public void resetExtensionEncoder(double resetPos) {
    extendMotor.setSelectedSensorPosition(resetPos);
  }

  public double getExtensionEncoder() {
    return extendMotor.getSelectedSensorPosition();
  }

  public boolean isLimitSwitchTripped() {
    return extensionLimitSwitch.get();
  }

  public void extendArmSpeed(double inputSpeed) {
    extendMotor.set(ControlMode.PercentOutput, inputSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Extenson Motor Position", getExtensionEncoder());
    SmartDashboard.putBoolean("limit switch trip status", isLimitSwitchTripped());
  }
}
