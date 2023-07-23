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
    /** Swerve module class that encompasses one full swerve module and the motors/encoders that go with it. 
     * Think of this as the template that SwerveSubsystem uses to make things simpler. It would be pretty messy to declare
     * all 4 modules (8 motors) in the same place when you need control over each pair of two individually for swerve.
     * Instead, functions that are needed for each module are placed here, and more general ones that apply to all in subsystem.
    */
    private final CANSparkMax driveMotor; 
    private final TalonSRX turningMotor;

    // This is the object for NEO encoders.
    private final RelativeEncoder driveEncoder;
    //private final Encoder turningEncoder;

    /** See PIDLoops.txt for info on PID loops and how to tune them. This creates an object which we'll declare values for in the
     * constructor below. Then, we can call .calculate(input, setpoint) on this object which returns the output (most likely routed to your
     * motor movement function) to make it do work.
    */
    private final PIDController turningPidController;

    /** Absolute encoders can go to an AnalogInput instead of a DigitalInput. 
     * These output a voltage between 0-5V based on the rotation of the wheel. 
     * However, due to the nature of absolute encoders, they will always be the same even after you turn the robot off - no way to
     * set it to be zero at a certain point. An offset may be needed to tell the robot how far the number should be from zero to be 
     * "zeroed".
     * There are some things done here that I would not repeat.
     * 
     * This code uses a literal "magic number" that was achived by spinning the wheel 360 5 times and seeing the encoder counts move
     * to calculate a conversion factor for pulses to radians. Reason being that the gear ratio just made no sense in comparison to
     * the numbers we were getting. Hopefully not a problem with the mk4i's.
     * 
     * This code actually just doesn't use the absolute encoders. Anthony came in to try and help with that one day and we just didn't
     * get anywhere with it. The offsets all looked right, and we reset the encoder counts to the positions they should have been in.
     * The wheels ended up crooked every time. Code for autohoming may be available online for mk4i's so that this isn't so painful for you.
    */
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    // See comment above for this value.
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int[] angleEncoderIds, boolean isFL) {
        // The this keyword allows for less confusion when passing params into variables. You can name them the same thing, but only
        // when assigning like this.variable = variable.
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

        /** I used a different PID controller for the front left motor because it was harder to move. 
         * Assigns PIDControllers for each swerve module. 
         */
        if (isFL = false) {
            turningPidController = new PIDController(ModuleConstants.kPTurning, 0.001, 0);
        } else {
            turningPidController = new PIDController(ModuleConstants.kPTurningFL, 0.001, 0);
        }

        // 
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
