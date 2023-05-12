package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.AxisCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.HoldIfNoTarget;
import frc.robot.commands.HomeExtensionSystem;
import frc.robot.commands.MoveArmExtension;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.ResetSwerveEncodersCommand;
import frc.robot.commands.SpinClaw;
import frc.robot.commands.SpinUntilLimitClaw;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveToVision;
import frc.robot.commands.SwivelJoystickCommand;
import frc.robot.commands.SwivelToPositionPID;
import frc.robot.commands.SwivelToPositionPIDLowPower;
import frc.robot.commands.SwivelToZeroPIDSafe;
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

    private final PhotonCamera visionCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");

    private final AxisCamera axisCam = CameraServer.addAxisCamera("10.39.39.18");

    private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
    private String selectedAuto;
    private static final String kDropOnlyAuto = "Drop Cube Only";
    private static final String kRightCubeCenter = "Right Cube to Center";
    private static final String kRightCubetoCS = "Right Cube to Charge Station";
    private static final String kRightCubeBack = "Right Cube to Back";
    private static final String kLeftCubeCenter = "Left Cube to Center";
    private static final String kLeftCubeBack = "Left Cube to Back";
    private static final String kCenterCubeCS = "Center Cube to Charge Station";
    private static final String kTwoCubeRight = "Two Cube Right";
    private static final String kTwoCubeLeft = "Two Cube Left";
    

    NetworkTable fmsInfoNetworkTable = NetworkTableInstance.getDefault().getTable("FMSInfo");

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    //private final Joystick swivelJoystick = new Joystick((OIConstants.kSwivelControllerPort));
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

    Trigger driverPOVNorth = new POVButton(driverJoytick, 0);
    Trigger driverPOVSouth = new POVButton(driverJoytick, 180);
    Trigger driverPOVWest = new POVButton(driverJoytick, 270);
    
    /*
    Trigger button21 = new JoystickButton(swivelJoystick, 1);
    Trigger button22 = new JoystickButton(swivelJoystick, 2);
    Trigger button25 = new JoystickButton(swivelJoystick, 5);
    Trigger button26 = new JoystickButton(swivelJoystick, 6);
    Trigger button28 = new JoystickButton(swivelJoystick, 8);

    Trigger button29 = new JoystickButton(swivelJoystick, 9);
    Trigger button210 = new JoystickButton(swivelJoystick, 10);
    Trigger button211 = new JoystickButton(swivelJoystick, 11);
    Trigger button212 = new JoystickButton(swivelJoystick, 12);
         */

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
        
        SmartDashboard.putString("Claw Camera", "/CameraPublisher/ClawCamera/streams=mjpg:http://10.39.39.18/mjpg/video.mjpg");

        m_autoChooser.setDefaultOption("Drop Only", kDropOnlyAuto);
        m_autoChooser.addOption("Right Cube to Center", kRightCubeCenter);
        m_autoChooser.addOption("Right Cube to Charge Station", kRightCubetoCS);
        m_autoChooser.addOption("Right Cube Back", kRightCubeBack);
        m_autoChooser.addOption("Left Cube to Center", kLeftCubeCenter);
        m_autoChooser.addOption("Left Cube Back", kLeftCubeBack);
        m_autoChooser.addOption("Center Cube to Charge Station", kCenterCubeCS);
        m_autoChooser.addOption("Right Two Cubes", kTwoCubeRight);
        m_autoChooser.addOption("Left Two Cubes", kTwoCubeLeft);
        SmartDashboard.putData("Autonomous Selection", m_autoChooser);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getZ(),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> driverJoytick.getRawButton(OIConstants.kDriverSlowMode),
                () -> swivleSubsystem.isRedSide()));
               
        /*
        swivleSubsystem.setDefaultCommand(new SwivelJoystickCommand(
                swivleSubsystem,
                () -> swivelJoystick.getRawAxis(OIConstants.kSwivelYAxis)));
        */
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        //new JoystickButton(driverJoytick, 10).whenPressed(() -> new Turn360(swerveSubsystem));
        driverPOVNorth.whileTrue(new SpinClaw(clawSubsystem, -0.3));
        driverPOVNorth.onFalse(new SpinClaw(clawSubsystem, 0));
        driverPOVSouth.whileTrue(new SpinClaw(clawSubsystem, -0.15));
        driverPOVSouth.onFalse(new SpinClaw(clawSubsystem, 0));
        driverPOVWest.whileTrue(new SpinClaw(clawSubsystem, -1));
        driverPOVWest.onFalse(new SpinClaw(clawSubsystem, 0));
        button1.whileTrue(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new SpinUntilLimitClaw(clawSubsystem),
                                new MoveArmExtension(-470, extendSubsystem)
                        ),
                        new MoveArmExtension(0, extendSubsystem)
                )
        );
        button1.onFalse(new MoveArmExtension(0, extendSubsystem));
        //button2.onTrue(new ZeroHeading(swerveSubsystem));
        //button2.onTrue(new Turn360(swerveSubsystem, new SwerveModuleState(0.03, new Rotation2d(swerveSubsystem.testBR()))));
        button3.onTrue(new SwivelToZeroPIDSafe(extendSubsystem, swivleSubsystem, 0));
        button4.onTrue(new SwivelToZeroPIDSafe(extendSubsystem, swivleSubsystem, 0));
        button5.onTrue(new SwivelToPositionPID(swivleSubsystem, 6.5));
        button6.onTrue(new SwivelToPositionPID(swivleSubsystem, -6.5));
        
        button7.onTrue(new ZeroSwivelEncoders(swivleSubsystem));
        button8.whileTrue(new SwerveToVision(swerveSubsystem, () -> visionCamera.getLatestResult(), true));
        button9.onTrue(new MoveArmExtension(-450, extendSubsystem));
        button10.onTrue(new ResetSwerveEncodersCommand(swerveSubsystem));
        button11.onTrue(new OpenClaw(clawSubsystem));
        button12.onTrue(new ZeroHeading(swerveSubsystem));
        
        /*
        button21.onTrue(new SwivelToPositionPID(swivleSubsystem, SmartDashboard.getNumber("Swivel Target", 0)));
        button22.onTrue(new ZeroSwivelEncoders(swivleSubsystem));
        button25.onTrue(new SwivelToPositionPID(swivleSubsystem, -13));
        button26.onTrue(new SwivelToPositionPID(swivleSubsystem, 13));
        */

        
        buttonT1.onTrue(new SwivelToPositionPIDLowPower(swivleSubsystem, -7)); //low
        buttonT2.onTrue(new SwivelToPositionPID(swivleSubsystem, -13.4)); //middle 
        buttonT3.onTrue(new SwivelToPositionPID(swivleSubsystem, -15.6)); //high
        //buttonT4.onTrue(new SwivelToPositionPID(swivleSubsystem, -14.3));
        buttonT4.onTrue(new MoveArmExtension(-60, extendSubsystem));
        buttonT5.onTrue(new SwivelToZeroPIDSafe(extendSubsystem, swivleSubsystem, 0));

        buttonT6.onTrue(new MoveArmExtension(-438, extendSubsystem)); //pickup
        buttonT7.onTrue(new MoveArmExtension(-175, extendSubsystem)); //middle
        buttonT8.onTrue(new MoveArmExtension(-505, extendSubsystem)); // middle cube and cone
        //buttonT9.onTrue(new MoveArmExtension(-510, extendSubsystem)); // max extension
        buttonT10.onTrue(new MoveArmExtension(0, extendSubsystem));

        buttonB1.onTrue(new SwivelToPositionPIDLowPower(swivleSubsystem, 7));
        buttonB2.onTrue(new SwivelToPositionPID(swivleSubsystem, 13.4));
        buttonB3.onTrue(new SwivelToPositionPID(swivleSubsystem, 16.2));
        buttonB4.onTrue(new SwivelToPositionPID(swivleSubsystem, 14.3));
        buttonB5.onTrue(new SwivelToPositionPID(swivleSubsystem, 0));

        buttonB6.onTrue(new ZeroExtensionSystem(extendSubsystem));
        buttonB7.onTrue(new HomeExtensionSystem(extendSubsystem));

        buttonB8.onTrue(new SpinClaw(clawSubsystem, 0.07));
        buttonB9.onTrue(new SpinUntilLimitClaw(clawSubsystem));
        buttonB10.onTrue(new SpinClaw(clawSubsystem, -0.3));
        

        
        

}

    public Command getAutonomousCommand() {
           
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);
        trajectoryConfig.setStartVelocity(0);
        trajectoryConfig.setEndVelocity(0);

        TrajectoryConfig trajectoryConfigR = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);
        trajectoryConfig.setStartVelocity(0);
        trajectoryConfig.setEndVelocity(0);
        
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
                        new Translation2d(-0.25, -3.25)
                ),
                new Pose2d(-0.25, -4.4, Rotation2d.fromDegrees(0)),
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
                new Pose2d(-0.08, -4.4, Rotation2d.fromDegrees(0)),
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
        
        Trajectory rightCubeReturn = TrajectoryGenerator.generateTrajectory(
                //swerveSubsystem.getPose(),
                new Pose2d(-.16, -4.66, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.2, -3.75),
                        new Translation2d(0.1, -0.65)
                ),
                new Pose2d(-0.15, -0.05, Rotation2d.fromDegrees(0)),
                trajectoryConfigR);

        Trajectory rightCubeReturnMid = TrajectoryGenerator.generateTrajectory(
                //swerveSubsystem.getPose(),
                new Pose2d(-.16, -4.66, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.2, -3.75),
                        new Translation2d(0.1, -0.45)
                ),
                new Pose2d(-0.15, -0.175, Rotation2d.fromDegrees(0)),
                trajectoryConfigR);
        
        Trajectory leftCubeReturn = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-.17, -4.66, new Rotation2d(0)),
                List.of(
                        new Translation2d(-0.2, -3.75),
                        new Translation2d(-0.1, -0.45)
                ),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfigR);
        
        Trajectory leftCubeReturnMid = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-.16, -4.66, new Rotation2d(0)),
                List.of(
                        new Translation2d(-0.2, -3.75),
                        new Translation2d(-0.1, -0.45)
                ),
                new Pose2d(-.15, -0.175, Rotation2d.fromDegrees(0)),
                trajectoryConfigR);
        
        
                // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0.0005, 0.0003);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0.0005, 0.0003);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory trajectoryToFollow;
        Trajectory trajectoryToReturn;
        selectedAuto = m_autoChooser.getSelected();
        SmartDashboard.putString("Autochoice", selectedAuto);
        switch (selectedAuto) {
                case kRightCubeBack:
                case kTwoCubeRight:
                        trajectoryToFollow = rightCubeBack;
                        trajectoryToReturn = rightCubeReturnMid;
                        break;
                case kRightCubeCenter:
                        trajectoryToFollow = rightCubetoCenterTrajectory;
                        trajectoryToReturn = rightCubeReturn;
                        break;
                case kRightCubetoCS:
                        trajectoryToFollow = rightCubetoCS;
                        trajectoryToReturn = rightCubeReturn;
                        break;
                case kDropOnlyAuto:
                        trajectoryToFollow = moveBackSlightly;
                        trajectoryToReturn = rightCubeReturn;
                        break;        
                case kLeftCubeBack:
                case kTwoCubeLeft:
                        trajectoryToFollow = leftCubeBack;
                        trajectoryToReturn = leftCubeReturnMid;
                        break;
                case kLeftCubeCenter:
                        trajectoryToFollow = leftCubetoCenter;
                        trajectoryToReturn = leftCubeReturn;
                        break;
                case kCenterCubeCS:
                        trajectoryToFollow = centerCubeChargeStation;
                        trajectoryToReturn = rightCubeReturn;
                        break;
                default:
                        trajectoryToFollow = moveBackSlightly;
                        trajectoryToReturn = leftCubeReturn;
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
        
        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                trajectoryToFollow,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
        
        SwerveControllerCommand swerveControllerCommand3= new SwerveControllerCommand(
                trajectoryToFollow,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
        SwerveControllerCommand swerveControllerCommand4= new SwerveControllerCommand(
                trajectoryToFollow,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        SwerveControllerCommand swerveReturn = new SwerveControllerCommand(
                trajectoryToReturn,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
        SwerveControllerCommand swerveReturn2 = new SwerveControllerCommand(
                trajectoryToReturn,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
        SwerveControllerCommand swerveReturn3 = new SwerveControllerCommand(
                trajectoryToReturn,
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
                new HomeExtensionSystem(extendSubsystem),                
                new ParallelDeadlineGroup(
                        new WaitCommand(1.8), 
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),               
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new MoveArmExtension(-505, extendSubsystem),
                                new WaitCommand(0.7),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(0.5),
                                        new SpinClaw(clawSubsystem, -0.15)),
                                new SpinClaw(clawSubsystem, 0),  
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(0.85)),      
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new SwivelToPositionPID(swivleSubsystem, 0)),
                swerveControllerCommand1,
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                new SwerveToVision(swerveSubsystem, () -> visionCamera.getLatestResult(), false),
                new InstantCommand(() -> swerveSubsystem.stopModules())
                );
        
        SequentialCommandGroup twoCubeGroup = new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose())),
                //swerveControllerCommand1, 
                //new InstantCommand(() -> swerveSubsystem.stopModules()),               
                new ParallelDeadlineGroup(
                        new WaitCommand(1.4), 
                        new HomeExtensionSystem(extendSubsystem),
                        new SequentialCommandGroup(
                                new WaitCommand(0.2),
                                new SwivelToPositionPID(swivleSubsystem, -14.5))),     
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new MoveArmExtension(-505, extendSubsystem),
                                new WaitCommand(0.7),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(0.35),
                                        new SpinClaw(clawSubsystem, -0.15)),
                                new SpinClaw(clawSubsystem, 0),  
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(0.85)),      
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.8), 
                        new SwivelToPositionPID(swivleSubsystem, 0)),
                swerveControllerCommand2,
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(0.6),
                                new SwerveToVision(swerveSubsystem, () -> visionCamera.getLatestResult(), false)), 
                        new SwivelToPositionPID(swivleSubsystem, 6.5)),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                //new HoldIfNoTarget(visionCamera.getLatestResult()),
                                new MoveArmExtension(-470, extendSubsystem),
                                new WaitCommand(0.15),
                                new ParallelRaceGroup(
                                        new SpinUntilLimitClaw(clawSubsystem),
                                        new WaitCommand(1.3)
                                ),
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(.7)), 
                        new SwivelToPositionPID(swivleSubsystem, 6.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new SwivelToPositionPID(swivleSubsystem, 0)),
                new ParallelRaceGroup(
                        swerveReturn,
                        new SwivelToPositionPID(swivleSubsystem, 0)),
                new InstantCommand(() -> swerveSubsystem.stopModules())//,
                /* 
                new ParallelDeadlineGroup(
                        new WaitCommand(1.8), 
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),               
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new MoveArmExtension(-505, extendSubsystem),
                                new WaitCommand(0.7),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(0.35),
                                        new SpinClaw(clawSubsystem, 0.15)),
                                new SpinClaw(clawSubsystem, 0),  
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(0.8)),      
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new SwivelToPositionPID(swivleSubsystem, 0)) 
                        */
                );
        
        SequentialCommandGroup noFrontDeliver = new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(13.7), new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose())),
                //swerveControllerCommand1, 
                //new InstantCommand(() -> swerveSubsystem.stopModules()),               
                new ParallelDeadlineGroup(
                        new WaitCommand(1.2), 
                        new HomeExtensionSystem(extendSubsystem),
                        new SequentialCommandGroup(
                                new SwivelToPositionPID(swivleSubsystem, -14.5))),     
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new WaitCommand(0.35),
                                        new SpinClaw(clawSubsystem, -0.4)),
                                new SpinClaw(clawSubsystem, 0)),    
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),
                new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.8), 
                                new SwivelToPositionPID(swivleSubsystem, 0)),
                        swerveControllerCommand3),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new SwerveToVision(swerveSubsystem, () -> visionCamera.getLatestResult(), false)), 
                        new SwivelToPositionPID(swivleSubsystem, 6.5)),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                //new HoldIfNoTarget(visionCamera.getLatestResult()),
                                new MoveArmExtension(-470, extendSubsystem),
                                new WaitCommand(0.15),
                                new ParallelRaceGroup(
                                        new SpinUntilLimitClaw(clawSubsystem),
                                        new WaitCommand(1.3)
                                ),
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(.7)), 
                        new SwivelToPositionPID(swivleSubsystem, 6.5)),
                new ParallelDeadlineGroup(
                        swerveReturn2,
                        new SpinUntilLimitClaw(clawSubsystem),
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2.25), 
                                        new SwivelToPositionPID(swivleSubsystem, 0)),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(3), 
                                        new SwivelToPositionPID(swivleSubsystem, -14)))),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new WaitCommand(0.35),
                                        new SpinClaw(clawSubsystem, -0.4)),
                                new SpinClaw(clawSubsystem, 0)),  
                        new SwivelToPositionPID(swivleSubsystem, -14)),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.8), 
                        new SwivelToPositionPID(swivleSubsystem, 0)))),
                new SpinClaw(clawSubsystem, -0.4)
                /* 
                new ParallelDeadlineGroup(
                        new WaitCommand(1.8), 
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),               
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new MoveArmExtension(-505, extendSubsystem),
                                new WaitCommand(0.7),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(0.35),
                                        new SpinClaw(clawSubsystem, 0.15)),
                                new SpinClaw(clawSubsystem, 0),  
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(0.8)),      
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new SwivelToPositionPID(swivleSubsystem, 0)) 
                        */
                );
        
        SequentialCommandGroup kickCube = new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose())),
                //swerveControllerCommand1, 
                //new InstantCommand(() -> swerveSubsystem.stopModules()),               
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new HomeExtensionSystem(extendSubsystem),
                                        new SwivelToPositionPID(swivleSubsystem, -4.5)),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(1), 
                                        new SwivelToPositionPID(swivleSubsystem, 0))),
                        swerveControllerCommand4),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new SwerveToVision(swerveSubsystem, () -> visionCamera.getLatestResult(), false)), 
                        new SwivelToPositionPID(swivleSubsystem, 6.5)),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                //new HoldIfNoTarget(visionCamera.getLatestResult()),
                                new MoveArmExtension(-470, extendSubsystem),
                                new WaitCommand(0.15),
                                new ParallelRaceGroup(
                                        new SpinUntilLimitClaw(clawSubsystem),
                                        new WaitCommand(1.3)
                                ),
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(.7)), 
                        new SwivelToPositionPID(swivleSubsystem, 6.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.1), 
                        new SwivelToPositionPID(swivleSubsystem, 0)),
                new ParallelRaceGroup(
                        swerveReturn3,
                        new SwivelToPositionPID(swivleSubsystem, 0)),
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.3), 
                        new SwivelToPositionPID(swivleSubsystem, -15.6)),     
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new WaitCommand(0.25),
                                        new SpinClaw(clawSubsystem, -0.3)),
                                new SpinClaw(clawSubsystem, 0)),  
                        new SwivelToPositionPID(swivleSubsystem, -15.6)),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.8), 
                        new SwivelToPositionPID(swivleSubsystem, 0))
                /* 
                new ParallelDeadlineGroup(
                        new WaitCommand(1.8), 
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),               
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new MoveArmExtension(-505, extendSubsystem),
                                new WaitCommand(0.7),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(0.35),
                                        new SpinClaw(clawSubsystem, 0.15)),
                                new SpinClaw(clawSubsystem, 0),  
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(0.8)),      
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new SwivelToPositionPID(swivleSubsystem, 0)) 
                        */
                );

        SequentialCommandGroup autoDropOnly = new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose())),
                //swerveControllerCommand1, 
                //new InstantCommand(() -> swerveSubsystem.stopModules()),
                new HomeExtensionSystem(extendSubsystem),                
                new ParallelDeadlineGroup(
                        new WaitCommand(1.8), 
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),               
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new MoveArmExtension(-505, extendSubsystem),
                                new WaitCommand(0.7),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(0.5),
                                        new SpinClaw(clawSubsystem, -0.15)),
                                new SpinClaw(clawSubsystem, 0),  
                                new MoveArmExtension(0, extendSubsystem),
                                new WaitCommand(0.85)),      
                        new SwivelToPositionPID(swivleSubsystem, -14.5)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new SwivelToPositionPID(swivleSubsystem, 0)),
                new InstantCommand(() -> swerveSubsystem.stopModules())
                ); 

        
        SequentialCommandGroup lowCubeDrop = new SequentialCommandGroup(

        );

        SequentialCommandGroup autonomousToFollow;
        switch (selectedAuto) {
                case kDropOnlyAuto:
                        autonomousToFollow = autoDropOnly;
                        break;
                case kTwoCubeLeft:
                case kTwoCubeRight:
                        autonomousToFollow = noFrontDeliver;
                        break;
                default:
                        autonomousToFollow = autoStation3;
                        break;
        } 
        return autonomousToFollow;
    }
}
