package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.SwivelToPositionPIDCutoff;
import frc.robot.commands.SwivelToPositionPIDLowPower;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwivleSubsystem;
import frc.robot.commands.ZeroExtensionSystem;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.ZeroSwivelEncoders;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final SwivleSubsystem swivleSubsystem = new SwivleSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ExtensionSubsystem extendSubsystem = new ExtensionSubsystem();

    private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
    private String selectedAuto;
    private static final String kDropOnlyAuto = "Drop Cube Only";
    private static final String kRightCubeCenter = "Right Cube to Center";
    private static final String kRightCubetoCS = "Right Cube to Charge Station";
    private static final String kRightCubeBack = "Right Cube to Back";
    private static final String kLeftCubeCenter = "Left Cube to Center";
    private static final String kLeftCubeBack = "Left Cube to Back";
    private static final String kCenterCubeCS = "Center Cube to Charge Station";
    

    NetworkTable fmsInfoNetworkTable = NetworkTableInstance.getDefault().getTable("FMSInfo");

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
        
        //SmartDashboard.putBoolean("Drop Auto Only?", false);

        m_autoChooser.setDefaultOption("Drop Only", kDropOnlyAuto);
        m_autoChooser.addOption("Right Cube to Center", kRightCubeCenter);
        m_autoChooser.addOption("Right Cube to Charge Station", kRightCubetoCS);
        m_autoChooser.addOption("Right Cube Back", kRightCubeBack);
        m_autoChooser.addOption("Left Cube to Center", kLeftCubeCenter);
        m_autoChooser.addOption("Left Cube Back", kLeftCubeBack);
        m_autoChooser.addOption("Center Cube to Charge Station", kCenterCubeCS);
        SmartDashboard.putData("Autonomous Selection", m_autoChooser);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getZ(),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> driverJoytick.getRawButton(OIConstants.kDriverSlowMode),
                () -> swivleSubsystem.isRedSide()));
               
        
        swivleSubsystem.setDefaultCommand(new SwivelJoystickCommand(
                swivleSubsystem,
                () -> swivelJoystick.getRawAxis(OIConstants.kSwivelYAxis)));
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        //new JoystickButton(driverJoytick, 10).whenPressed(() -> new Turn360(swerveSubsystem));
        button1.onTrue(new ToggleClaw(clawSubsystem));
        //button2.onTrue(new ZeroHeading(swerveSubsystem));
        //button2.onTrue(new Turn360(swerveSubsystem, new SwerveModuleState(0.03, new Rotation2d(swerveSubsystem.testBR()))));
        button3.onTrue(new MoveArmExtension(-450, extendSubsystem));
        button4.onTrue(new MoveArmExtension(0, extendSubsystem));
        button5.onTrue(new SwivelToPositionPID(swivleSubsystem, 6.5));
        button6.onTrue(new SwivelToPositionPID(swivleSubsystem, -6.5));
        
        button7.onTrue(new ZeroSwivelEncoders(swivleSubsystem));
        button9.onTrue(new MoveArmExtension(-450, extendSubsystem));
        //button10.onTrue(new MoveArmExtension(0, extendSubsystem));
        button11.onTrue(new OpenClaw(clawSubsystem));
        button12.onTrue(new ZeroHeading(swerveSubsystem));
        
        button21.onTrue(new SwivelToPositionPID(swivleSubsystem, SmartDashboard.getNumber("Swivel Target", 0)));
        button22.onTrue(new ZeroSwivelEncoders(swivleSubsystem));
        button25.onTrue(new SwivelToPositionPID(swivleSubsystem, -13));
        button26.onTrue(new SwivelToPositionPID(swivleSubsystem, 13));
        

        
        buttonT1.onTrue(new SwivelToPositionPIDLowPower(swivleSubsystem, -7)); //low
        buttonT2.onTrue(new SwivelToPositionPID(swivleSubsystem, -13.4)); //middle 
        buttonT3.onTrue(new SwivelToPositionPID(swivleSubsystem, -15.6)); //high
        buttonT4.onTrue(new SwivelToPositionPID(swivleSubsystem, -14.3));
        buttonT5.onTrue(new SwivelToPositionPID(swivleSubsystem, 0));

        buttonT6.onTrue(new MoveArmExtension(-438, extendSubsystem)); //pickup
        buttonT7.onTrue(new MoveArmExtension(-175, extendSubsystem)); //middle
        buttonT8.onTrue(new MoveArmExtension(-505, extendSubsystem)); // middle cube and cone
        //buttonT9.onTrue(new MoveArmExtension(-510, extendSubsystem)); // max extension
        buttonT10.onTrue(new MoveArmExtension(0, extendSubsystem));

        buttonB1.onTrue(new SwivelToPositionPIDLowPower(swivleSubsystem, 7));
        buttonB2.onTrue(new SwivelToPositionPID(swivleSubsystem, 13.4));
        buttonB3.onTrue(new SwivelToPositionPID(swivleSubsystem, 15.6));
        buttonB4.onTrue(new SwivelToPositionPID(swivleSubsystem, 14.3));
        buttonB5.onTrue(new SwivelToPositionPID(swivleSubsystem, 0));

        buttonB6.onTrue(new ZeroExtensionSystem(extendSubsystem));
        buttonB7.onTrue(new HomeExtensionSystem(extendSubsystem));

        buttonB9.onTrue(new OpenClaw(clawSubsystem));
        buttonB10.onTrue(new CloseClaw(clawSubsystem));
        
        
        

}

    public Command getAutonomousCommand() {
           
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);
        
        // Left Cube
        Trajectory leftCubetoCenter = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(-0.1, -0.7),
                        new Translation2d(-0.2, -3.75),
                        new Translation2d(1, -3.75)
                ),
                new Pose2d(2, -3.75, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        Trajectory leftCubeBack = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(-0.1, -0.7),
                        new Translation2d(-0.2, -3.25)
                ),
                new Pose2d(-0.2, -4.25, Rotation2d.fromDegrees(0)),
                trajectoryConfig);
        // Straight Back to player station
        Trajectory centerCubeChargeStation = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0, -0.5),
                        new Translation2d(0, -2.24)
                ),
                new Pose2d(0, -2.4, Rotation2d.fromDegrees(0)),
                trajectoryConfig);
        // Right cube below
        Trajectory rightCubetoCenterTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.1, -0.7),
                        new Translation2d(0.2, -3.75),
                        new Translation2d(-1, -3.75)
                ),
                new Pose2d(-2, -3.75, Rotation2d.fromDegrees(0)),
                trajectoryConfig);
        Trajectory rightCubetoCS = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.1, -0.7),
                        new Translation2d(0.2, -3.75),
                        new Translation2d(-1, -3.75),
                        new Translation2d(-2, -3.75)
                ),
                new Pose2d(-2, -2.25, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        Trajectory rightCubeBack = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.1, -0.7),
                        new Translation2d(0.2, -3.25)
                ),
                new Pose2d(0.2, -4.25, Rotation2d.fromDegrees(0)),
                trajectoryConfig);
        // DEFAULT
        Trajectory moveBackSlightly = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0, -0.7),
                        new Translation2d(0, -0.5)
                ),
                new Pose2d(0, -0.3, Rotation2d.fromDegrees(0)),
                trajectoryConfig);
        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0.0005, 0.0003);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0.0005, 0.0003);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory trajectoryToFollow;
        selectedAuto = m_autoChooser.getSelected();
        SmartDashboard.putString("Autochoice", selectedAuto);
        switch (selectedAuto) {
                case kRightCubeBack:
                        trajectoryToFollow = rightCubeBack;
                        break;
                case kRightCubeCenter:
                        trajectoryToFollow = rightCubetoCenterTrajectory;
                        break;
                case kRightCubetoCS:
                        trajectoryToFollow = rightCubetoCS;
                        break;
                case kDropOnlyAuto:
                        trajectoryToFollow = moveBackSlightly;
                        break;        
                case kLeftCubeBack:
                        trajectoryToFollow = leftCubeBack;
                        break;
                case kLeftCubeCenter:
                        trajectoryToFollow = leftCubetoCenter;
                        break;
                case kCenterCubeCS:
                        trajectoryToFollow = centerCubeChargeStation;
                        break;
                default:
                        trajectoryToFollow = moveBackSlightly;
                        break;
        }

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
                trajectoryToFollow,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        
        //if fmsInfoNetworkTable.getIntegerTopic("null")
        SequentialCommandGroup autoStation3 = new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose())),
                //swerveControllerCommand1, 
                //new InstantCommand(() -> swerveSubsystem.stopModules()),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.75), 
                        new HomeExtensionSystem(extendSubsystem)),                
                new ZeroExtensionSystem(extendSubsystem),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.8), 
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),               
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new MoveArmExtension(-505, extendSubsystem),
                                new WaitCommand(0.7),
                                new OpenClaw(clawSubsystem),
                                new WaitCommand(0.3),
                                new CloseClaw(clawSubsystem),  
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(0.85)),      
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new SwivelToPositionPID(swivleSubsystem, 0)),
                swerveControllerCommand1,
                new InstantCommand(() -> swerveSubsystem.stopModules())
                );
                
        SequentialCommandGroup autoDropOnly = new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose())),
                //swerveControllerCommand1, 
                //new InstantCommand(() -> swerveSubsystem.stopModules()),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.75), 
                        new HomeExtensionSystem(extendSubsystem)),                
                new ZeroExtensionSystem(extendSubsystem),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.8), 
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),               
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new MoveArmExtension(-505, extendSubsystem),
                                new WaitCommand(0.7),
                                new OpenClaw(clawSubsystem),
                                new WaitCommand(0.3),
                                new CloseClaw(clawSubsystem),  
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(0.85)),      
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new SwivelToPositionPID(swivleSubsystem, 0)),
                new InstantCommand(() -> swerveSubsystem.stopModules())
                ); 
        
        SequentialCommandGroup autonomousToFollow;
        switch (selectedAuto) {
                case kDropOnlyAuto:
                        autonomousToFollow = autoDropOnly;
                        break;
                default:
                        autonomousToFollow = autoStation3;
                        break;
        } 
        return autonomousToFollow;
    }
}
