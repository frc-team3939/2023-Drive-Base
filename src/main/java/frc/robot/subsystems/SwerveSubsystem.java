package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.io.PrintStream;

//test
public class SwerveSubsystem extends SubsystemBase {
    private double testValueBR;
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderPorts,
            true);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderPorts,
            false);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderPorts,
            false);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            DriveConstants.kBackRightTurningEncoderPorts,
            false);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
              }); 

    public SwerveSubsystem() {
        testValueBR = 0;
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public double[] getWheelAbsReadings() {
        return null;

    }
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    } 
    
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(
            getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose);
    }  

    @Override
    public void periodic() {
        odometer.update(
            getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        
            }
        );
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("RobotPose", odometer.getPoseMeters().toString());
        SmartDashboard.putNumber("FL ABS Voltage", frontLeft.getRawAbsoluteEncoderVoltage());
        SmartDashboard.putNumber("FR ABS Voltage", frontRight.getRawAbsoluteEncoderVoltage());
        SmartDashboard.putNumber("BL ABS Voltage", backLeft.getRawAbsoluteEncoderVoltage());
        SmartDashboard.putNumber("BR ABS Voltage", backRight.getRawAbsoluteEncoderVoltage());
        SmartDashboard.putNumber("FL ABS Calculated Angle" , frontLeft.getAbsoluteEncoderRad() * 180/Math.PI);
        SmartDashboard.putNumber("FR ABS Calculated Angle" , frontRight.getAbsoluteEncoderRad() * 180/Math.PI);
        SmartDashboard.putNumber("BL ABS Calculated Angle" , backLeft.getAbsoluteEncoderRad() * 180/Math.PI);
        SmartDashboard.putNumber("BR ABS Calculated Angle" , backRight.getAbsoluteEncoderRad() * 180/Math.PI);
        SmartDashboard.putNumber("FL ABS Calculated Radians" , frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FR ABS Calculated Radians" , frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BL ABS Calculated Radians" , backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BR ABS Calculated Radians" , backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("5V Rail Voltage", RobotController.getVoltage5V());
        SmartDashboard.putString("Swerve frontLeft current state", frontLeft.getState().toString());
        SmartDashboard.putString("Swerve frontRight current state", frontRight.getState().toString());
        SmartDashboard.putString("Swerve backLeft current state", backLeft.getState().toString());
        SmartDashboard.putString("Swerve backRight current state", backRight.getState().toString());
        SmartDashboard.putNumber("Angle Encoder Left Front", frontLeft.getRawTurningEncoder());
        SmartDashboard.putNumber("Angle Encoder Right Front", frontRight.getRawTurningEncoder());
        SmartDashboard.putNumber("Angle Encoder Left Back ", backLeft.getRawTurningEncoder());
        SmartDashboard.putNumber("Angle Encoder Right Back", backRight.getRawTurningEncoder());
        //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    

    public double testBR() {
        
        testValueBR = testValueBR + Math.PI/4;
        SmartDashboard.putNumber("most reasonable code print", testValueBR);
        System.out.print(testValueBR);
        return testValueBR;
    }
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    public boolean getBRTurningSetpoint() {
        return backRight.getPIDController().atSetpoint();
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
