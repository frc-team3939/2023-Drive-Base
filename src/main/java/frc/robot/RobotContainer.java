package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.ExtendToPositionPID;
import frc.robot.commands.HomeExtensionSystem;
import frc.robot.commands.MoveArmExtension;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwivelJoystickCommand;
import frc.robot.commands.SwivelToPositionIncrement;
import frc.robot.commands.SwivelToPositionPID;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwivleSubsystem;
import frc.robot.commands.Turn360;
import frc.robot.commands.ZeroExtensionSystem;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.ZeroSwivelEncoders;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final SwivleSubsystem swivleSubsystem = new SwivleSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ExtensionSubsystem extendSubsystem = new ExtensionSubsystem();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick swivelJoystick = new Joystick((OIConstants.kSwivelControllerPort));
    private final Joystick topHalfButtonBoard = new Joystick((OIConstants.kTopHalfButtonBoardPort));
    private final Joystick bottomHalfButtonBoard = new Joystick((OIConstants.kBottomHalfButtonBoardPort));

    Trigger button1 = new JoystickButton(driverJoytick, 1);
    Trigger button2 = new JoystickButton(driverJoytick, 2);
    Trigger button3 = new JoystickButton(driverJoytick, 3);
    Trigger button4 = new JoystickButton(driverJoytick, 4);
    Trigger button5 = new JoystickButton(driverJoytick, 5);
    Trigger button6 = new JoystickButton(driverJoytick, 6);
    Trigger button7 = new JoystickButton(driverJoytick, 7);
    Trigger button8 = new JoystickButton(driverJoytick, 8);
    Trigger button9 = new JoystickButton(driverJoytick, 9);
    Trigger button10 = new JoystickButton(driverJoytick, 10);
    Trigger button11 = new JoystickButton(driverJoytick, 11);
    Trigger button12 = new JoystickButton(driverJoytick, 12);
    
    Trigger button21 = new JoystickButton(swivelJoystick, 1);
    Trigger button22 = new JoystickButton(swivelJoystick, 2);
    Trigger button25 = new JoystickButton(swivelJoystick, 5);
    Trigger button26 = new JoystickButton(swivelJoystick, 6);
    Trigger button28 = new JoystickButton(swivelJoystick, 8);

    Trigger button29 = new JoystickButton(swivelJoystick, 9);
    Trigger button210 = new JoystickButton(swivelJoystick, 10);
    Trigger button211 = new JoystickButton(swivelJoystick, 11);
    Trigger button212 = new JoystickButton(swivelJoystick, 12);

    Trigger buttonT1 = new JoystickButton(topHalfButtonBoard, 1);
    Trigger buttonT2 = new JoystickButton(topHalfButtonBoard, 2);
    Trigger buttonT3 = new JoystickButton(topHalfButtonBoard, 3);
    Trigger buttonT4 = new JoystickButton(topHalfButtonBoard, 4);
    Trigger buttonT5 = new JoystickButton(topHalfButtonBoard, 5);
    Trigger buttonT6 = new JoystickButton(topHalfButtonBoard, 6);
    Trigger buttonT7 = new JoystickButton(topHalfButtonBoard, 7);
    Trigger buttonT8 = new JoystickButton(topHalfButtonBoard, 8);
    Trigger buttonT9 = new JoystickButton(topHalfButtonBoard, 9);
    Trigger buttonT10 = new JoystickButton(topHalfButtonBoard, 10);

    Trigger buttonB1 = new JoystickButton(bottomHalfButtonBoard, 1);
    Trigger buttonB2 = new JoystickButton(bottomHalfButtonBoard, 2);
    Trigger buttonB3 = new JoystickButton(bottomHalfButtonBoard, 3);
    Trigger buttonB4 = new JoystickButton(bottomHalfButtonBoard, 4);
    Trigger buttonB5 = new JoystickButton(bottomHalfButtonBoard, 5);
    Trigger buttonB6 = new JoystickButton(bottomHalfButtonBoard, 6);
    Trigger buttonB7 = new JoystickButton(bottomHalfButtonBoard, 7);
    Trigger buttonB8 = new JoystickButton(bottomHalfButtonBoard, 8);
    Trigger buttonB9 = new JoystickButton(bottomHalfButtonBoard, 9);
    Trigger buttonB10 = new JoystickButton(bottomHalfButtonBoard, 10);


    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getZ(),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
        
        swivleSubsystem.setDefaultCommand(new SwivelJoystickCommand(
                swivleSubsystem,
                () -> swivelJoystick.getRawAxis(OIConstants.kSwivelYAxis)));
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        //new JoystickButton(driverJoytick, 10).whenPressed(() -> new Turn360(swerveSubsystem));
        button1.onTrue(new ToggleClaw(clawSubsystem));
        button2.onTrue(new ZeroHeading(swerveSubsystem));
        //button2.onTrue(new Turn360(swerveSubsystem, new SwerveModuleState(0.03, new Rotation2d(swerveSubsystem.testBR()))));
        button7.onTrue(new SwivelToPositionPID(swivleSubsystem, -6.5));
        button8.onTrue(new MoveArmExtension(-408, extendSubsystem));
        button10.onTrue(new MoveArmExtension(0, extendSubsystem));
        button11.onTrue(new OpenClaw(clawSubsystem));
        button12.onTrue(new CloseClaw(clawSubsystem));
        
        button21.onTrue(new SwivelToPositionPID(swivleSubsystem, SmartDashboard.getNumber("Swivel Target", 0)));
        button22.onTrue(new ZeroSwivelEncoders(swivleSubsystem));
        button25.onTrue(new SwivelToPositionPID(swivleSubsystem, -13));
        button26.onTrue(new SwivelToPositionPID(swivleSubsystem, 13));
        

        
        buttonT1.onTrue(new SwivelToPositionPID(swivleSubsystem, -6.5)); //low
        buttonT2.onTrue(new SwivelToPositionPID(swivleSubsystem, -13.4)); //middle 
        buttonT3.onTrue(new SwivelToPositionPID(swivleSubsystem, -15.6)); //high
        buttonT4.onTrue(new SwivelToPositionPID(swivleSubsystem, -14.3));
        buttonT5.onTrue(new SwivelToPositionPID(swivleSubsystem, 0));

        buttonT6.onTrue(new MoveArmExtension(-408, extendSubsystem)); //pickup
        buttonT7.onTrue(new MoveArmExtension(-175, extendSubsystem)); //middle
        buttonT8.onTrue(new MoveArmExtension(-510, extendSubsystem)); // middle cube and cone
        //buttonT9.onTrue(new MoveArmExtension(-510, extendSubsystem)); // max extension
        buttonT10.onTrue(new MoveArmExtension(0, extendSubsystem));

        buttonB1.onTrue(new SwivelToPositionPID(swivleSubsystem, 6.5));
        buttonB2.onTrue(new SwivelToPositionPID(swivleSubsystem, 13.4));
        buttonB3.onTrue(new SwivelToPositionPID(swivleSubsystem, 15.6));
        buttonB4.onTrue(new SwivelToPositionPID(swivleSubsystem, 14.3));
        buttonB5.onTrue(new SwivelToPositionPID(swivleSubsystem, 0));

        buttonB6.onTrue(new ZeroExtensionSystem(extendSubsystem));
        buttonB7.onTrue(new ZeroSwivelEncoders(swivleSubsystem));
        buttonB8.onTrue(new OpenClaw(clawSubsystem));
        buttonB9.onTrue(new CloseClaw(clawSubsystem));
        buttonB10.onTrue(new HomeExtensionSystem(extendSubsystem));
        
        

}

    public Command getAutonomousCommand() {
        /* 
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
                */
                
        return null;
    }
}
