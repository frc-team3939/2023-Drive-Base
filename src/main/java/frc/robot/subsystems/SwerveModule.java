package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final TalonSRX turningMotor;

    private final RelativeEncoder driveEncoder;
    //private final Encoder turningEncoder;

    private final PIDController turningPidController;


    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int[] angleEncoderIds, boolean isFL) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new TalonSRX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        turningMotor.setNeutralMode(NeutralMode.Coast);
        turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        driveEncoder = driveMotor.getEncoder();
        //turningEncoder = new Encoder(angleEncoderIds[0], angleEncoderIds[1]);
        
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderPPRad);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        if (isFL = false) {
            turningPidController = new PIDController(ModuleConstants.kPTurning, 0.001, 0);
        } else {
            turningPidController = new PIDController(ModuleConstants.kPTurningFL, 0.001, 0);
        }

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition() * ModuleConstants.kTurningEncoderPPRad * 0.25;
    }

    public double getRawTurningEncoder() {
        return turningMotor.getSelectedSensorPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity() * (10) * ModuleConstants.kTurningEncoderPPRad * 0.25;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getRawAbsoluteEncoderVoltage() {
        return absoluteEncoder.getVoltage();
    }
    
    public PIDController getPIDController() {
        return turningPidController;
    }
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //turningEncoder.setPosition(/*getAbsoluteEncoderRad()*/);
    }

    /**
     * This will set the encoder positions to based apon an angle input
     * @param angle This is expecting an angle in radians -2pi - 2pi
     */
    public void setEncoderAngle(double angle){
        angle = angle * (1 / (ModuleConstants.kTurningEncoderPPRad * 0.25));
        turningMotor.setSelectedSensorPosition(angle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }
    public void testTurnMotor() {
        turningMotor.set(ControlMode.PercentOutput, 0.5);
    }
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        double out =  turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        turningMotor.set(ControlMode.PercentOutput, out);
        SmartDashboard.putNumber("PID turn output" + absoluteEncoder.getChannel(), out);
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
