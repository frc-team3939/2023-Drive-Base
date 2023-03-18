package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveToVision extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<PhotonPipelineResult> visionInfo;
    private final PIDController xSpdController, ySpdController, turningSpdController;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    /**
     * This command writes swerveModuleStates to a photonvision target.
     * @param swerveSubsystem Requirement parameter
     * @param visionInfo PhotonPipelineResult class, input the result
     */
    public SwerveToVision(SwerveSubsystem swerveSubsystem, Supplier<PhotonPipelineResult> visionInfo) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionInfo = visionInfo;
        xSpdController = new PIDController(0.01, 0.0000001, 0.0001);
        xSpdController.setTolerance(.3);
        ySpdController = new PIDController(0.01, 0.0000001, 0.0001);
        ySpdController.setTolerance(.3);
        turningSpdController = new PIDController(0.01, 0.0000001, 0.0001);
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // get vision info
        PhotonPipelineResult visionResult = visionInfo.get();
        double yaw;
        double pitch;
        double skew;
        if (visionResult.hasTargets() == true) {
                PhotonTrackedTarget target = visionResult.getBestTarget();
                yaw = target.getYaw();
                pitch = target.getPitch();
                skew = target.getSkew();
        } else {
            yaw = 21.7;
            pitch = 3;
            skew = 0;
        }

        // 1. Get real-time joystick inputs
        double xSpeed = -xSpdController.calculate(yaw, 21.7);
        double ySpeed = ySpdController.calculate(pitch, 3);
        double turningSpeed = 0;
        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SmartDashboard.putString("testBRD", moduleStates[3].toString());
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return xSpdController.atSetpoint() && ySpdController.atSetpoint();
    }
}
