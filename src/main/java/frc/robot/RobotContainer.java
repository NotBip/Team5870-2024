package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos.DriveStraight;
import frc.robot.commands.Autos.RedDriveStraight;
import frc.robot.commands.Autos.blueAmp1;
import frc.robot.commands.Autos.blueAmp2;
import frc.robot.commands.Autos.blueAmp3;
import frc.robot.commands.Autos.BlueDriveStraight;
import frc.robot.commands.Climber.ClimberDown;
import frc.robot.commands.Climber.ClimberManualPosition;
import frc.robot.commands.Climber.ClimberStop;
import frc.robot.commands.Climber.ClimberUp;
import frc.robot.commands.Climber.zeroClimber;
import frc.robot.commands.Intake.IntakeFullPower;
import frc.robot.commands.Intake.IntakeSpinBack;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.LeftIntakeJoystick;
import frc.robot.commands.Intake.RightIntakeJoystick;
import frc.robot.commands.Pneumatics.FullDetract;
import frc.robot.commands.Pneumatics.FullExtend;
// import frc.robot.commands.S>erve.AutoAlign;
import frc.robot.commands.Swerve.Reposition;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.PhotonLL;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
        // Autonomous Chooser
        SendableChooser<Command> autoChooser = new SendableChooser<Command>();

        // Field Generator
        Field2d field = new Field2d(); 
        
        // Initializing Robot's Subsystems
        public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final Intake intake = new Intake();
        private final Climber climber = new Climber(); 
        private final Pneumatics pneumatics = new Pneumatics(); 
        // private final PhotonLL limelight = new PhotonLL(); 

        // Initializing Controllers and Joysticks
        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort); 
        private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort); 
        private final XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort); 

        // Initializing Auto Commands 
        //private final blueAmp1 blueAmpAuto1 = new blueAmp1(intake, swerveSubsystem); 
        //private final blueAmp2 blueAmpAuto2 = new blueAmp2(intake, swerveSubsystem); 
        //private final blueAmp3 blueAmpAuto3 = new blueAmp3(intake, swerveSubsystem, zeroGyro); 
        private final BlueDriveStraight blueDriveStraight = new BlueDriveStraight(); 
        private final RedDriveStraight redDriveStraight = new RedDriveStraight(); 

        // Initializing Commands
        // Intake
        private final IntakeSpinBack intakeSpinBack = new IntakeSpinBack(intake); 
        private final IntakeSpinForward intakeSpinForward = new IntakeSpinForward(intake); 
        private final IntakeStop intakeStop = new IntakeStop(intake);
        private final IntakeFullPower intakeFullPower = new IntakeFullPower(intake); 
        
        // Climber
        private final ClimberDown climberDown = new ClimberDown(climber); 
        private final ClimberUp climberUp = new ClimberUp(climber); 
        private final ClimberStop climberStop = new ClimberStop(climber); 
        private final zeroClimber zClimber = new zeroClimber(climber); 

        // Pneumatics
        private final FullExtend fullExtend = new FullExtend(pneumatics); 
        private final FullDetract fullDetract = new FullDetract(pneumatics); 

        // Swerve
        private final ZeroGyro zeroGyro = new ZeroGyro(swerveSubsystem); 
        // private final AutoAlign autoAlign = new AutoAlign(swerveSubsystem, limelight); // Currently in testing. 
        private final Reposition reposition = new Reposition(swerveSubsystem); // Currently in testing. 
        private final DriveStraight driveStraight = new DriveStraight(intake, swerveSubsystem, zeroGyro); 

        // Game Controllers
        public JoystickButton drBtnA, drBtnB, drBtnX, drBtnY, drBtnLB, drBtnRB, drBtnStrt, drBtnSelect;
        public JoystickButton opBtnA, opBtnB, opBtnX, opBtnY, opBtnLB, opBtnRB, opBtnStrt, opBtnSelect; 

        // Trajectory for Autonomous
        Trajectory finalTrajectory = new Trajectory();
        
        //Get X and Y axis from the joystick to control the robot
        public RobotContainer() {
        
        // Adding options to Auto Chooser
        autoChooser.setDefaultOption("Nothing", null); // Default auto will be `Commands.none()`
        autoChooser.addOption("Blue Drive Straight", blueDriveStraight.getAutonomousCommand(swerveSubsystem));
        autoChooser.addOption("Red Drive Straight", redDriveStraight.getAutonomousCommand(swerveSubsystem));
        // autoChooser.addOption("BA2", blueAmpAuto2.blueAmp2AutoCommand());
        // autoChooser.addOption("BA3", blueAmpAuto3.blueAmp3AutoCommand());
        // Shuffleboard.getTab("Autonomous").add("Select Auto", autoChooser).withSize(2, 1);


        // set default commands for each Subsystem
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(6)));
        intake.setDefaultCommand(intakeStop);
        climber.setDefaultCommand(climberStop);
        
        // Xbox Driver Controller Buttons
        drBtnA = new JoystickButton(driverJoystick, OIConstants.KXboxButtonA);
        drBtnB = new JoystickButton(driverJoystick, OIConstants.KXboxButtonB);
        drBtnX = new JoystickButton(driverJoystick, OIConstants.KXboxButtonX);
        drBtnY = new JoystickButton(driverJoystick, OIConstants.KXboxButtonY);
        drBtnLB = new JoystickButton(driverJoystick, OIConstants.KXboxLeftBumper);
        drBtnRB = new JoystickButton(driverJoystick, OIConstants.KXboxRightBumper);
        drBtnSelect = new JoystickButton(driverJoystick, OIConstants.KXboxSelectButton);
        drBtnStrt = new JoystickButton(driverJoystick, OIConstants.KXboxStartButton);

        // Xbox Operator Controller Buttons. 
        opBtnA = new JoystickButton(operatorJoystick, OIConstants.KXboxButtonA);
        opBtnB = new JoystickButton(operatorJoystick, OIConstants.KXboxButtonB);
        opBtnX = new JoystickButton(operatorJoystick, OIConstants.KXboxButtonX);
        opBtnY = new JoystickButton(operatorJoystick, OIConstants.KXboxButtonY);
        opBtnLB = new JoystickButton(operatorJoystick, OIConstants.KXboxLeftBumper);
        opBtnRB = new JoystickButton(operatorJoystick, OIConstants.KXboxRightBumper);
        opBtnSelect = new JoystickButton(operatorJoystick, OIConstants.KXboxSelectButton);
        opBtnStrt = new JoystickButton(operatorJoystick, OIConstants.KXboxStartButton);
        
        configureButtonBindings(); 
        }

        private void configureButtonBindings() {
        // QOL Swerve Controls
        drBtnStrt.onTrue(zeroGyro);
        // drBtnY.whileTrue(autoAlign); // Currently Testing this auto align feature so yeahhh.    

        // Climber Controls
        new Trigger(()-> operatorController.getRightTriggerAxis() > 0.3).whileTrue(climberUp); 
        new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.3).whileTrue(climberDown); 
        drBtnSelect.onTrue(zClimber); 
        opBtnA.whileTrue(new ClimberManualPosition(climber, -26.7));

        // Intake Controls
        new Trigger(() -> Math.abs(operatorJoystick.getRawAxis(1)) > 0.3).whileTrue(
                new LeftIntakeJoystick(() -> operatorJoystick.getRawAxis(1), intake));
        new Trigger(() -> Math.abs(operatorJoystick.getRawAxis(5)) > 0.3).whileTrue(
                new RightIntakeJoystick(() -> operatorJoystick.getRawAxis(5), intake));

        // Pneumatics Controls 
        new POVButton(operatorJoystick, 0).onTrue(fullExtend); 
        new POVButton(operatorJoystick, 180).onTrue(fullDetract); 
        

}


        public Command getAutonomousCommand() {
                // return null; 
                // return driveStraight.driveStraight();  
        
             // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(3, 0)),
                new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
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
                new InstantCommand(() -> swerveSubsystem.zeroHeading()), 
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
        }
} 