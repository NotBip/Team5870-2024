package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.Climber.ClimberDown;
import frc.robot.commands.Climber.ClimberManualPosition;
import frc.robot.commands.Climber.ClimberStop;
import frc.robot.commands.Climber.ClimberUp;
import frc.robot.commands.Climber.zeroClimber;
import frc.robot.commands.Intake.IntakeSpinBack;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.LeftIntakeJoystick;
import frc.robot.commands.Intake.RightIntakeJoystick;
import frc.robot.commands.Pneumatics.FullDetract;
import frc.robot.commands.Pneumatics.FullExtend;
import frc.robot.commands.Swerve.NudgeBack;
import frc.robot.commands.Swerve.NudgeFront;
import frc.robot.commands.Swerve.NudgeLeft;
import frc.robot.commands.Swerve.NudgeRight;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

        // Initalizing a sendable chooser which will hold all the different autonomous modes to be later added onto smartdashboard
        SendableChooser<Command> autoChooser = new SendableChooser<Command>();

        // Field Generator for smartDashboard
        Field2d field = new Field2d(); 
        
        // Initializing Robot's Subsystems
        public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final Intake intake = new Intake();
        private final Climber climber = new Climber(); 
        private final Pneumatics pneumatics = new Pneumatics(); 

        // Initializing Controllers
        private final CommandXboxController driverController = new CommandXboxController(IOConstants.kDriverControllerPort); 
        private final CommandXboxController operatorController = new CommandXboxController(IOConstants.kOperatorControllerPort); 

        // Initializing Commands
        // Intake
        private final IntakeSpinBack intakeSpinBack = new IntakeSpinBack(intake);  // Motors on full speed to intake the note
        private final IntakeSpinForward intakeSpinForward = new IntakeSpinForward(intake);  // Motors on full speed to shoot the note
        private final IntakeStop intakeStop = new IntakeStop(intake);  // Stops all Intake Motors.   
        
        // Climber
        private final ClimberDown climberDown = new ClimberDown(climber);  // Moves the climber down at 20% of it's max speed. 
        private final ClimberUp climberUp = new ClimberUp(climber);  // Moves the climber up at 20% of it's max speed.
        private final ClimberStop climberStop = new ClimberStop(climber); // Completely stops both the climber motors and enable their brakes.
        private final zeroClimber zClimber = new zeroClimber(climber);  // Resets the climber encoder value. 

        // Pneumatics
        private final FullExtend fullExtend = new FullExtend(pneumatics);  // Fully extends the pneumatic piston out
        private final FullDetract fullDetract = new FullDetract(pneumatics); // Fully detracts the pneumatic piston out. 

        // Swerve
        private final ZeroGyro zeroGyro = new ZeroGyro(swerveSubsystem);  // Resets the gyro on the robot to reconfigure the absolute front of the robot. 
        private final NudgeLeft nudgeLeft = new NudgeLeft(swerveSubsystem);  // Nudges the robot slightly to the left. 
        private final NudgeRight nudgeRight = new NudgeRight(swerveSubsystem);  // Nudges the robot slightly to the right.
        private final NudgeFront nudgeFront = new NudgeFront(swerveSubsystem);  // Nudges the robot slightly to the front.
        private final NudgeBack nudgeBack = new NudgeBack(swerveSubsystem);  // Nudges the robot slightly to the back.

        
        public RobotContainer() {
                
                // Initializing all the command names made in pathPlanner and assigning them actual commands. 
                configureNamedCommands();

                // Adding options to Auto Chooser 
                autoChooser.setDefaultOption("Drive Straight", new SequentialCommandGroup(
                        new InstantCommand(() -> swerveSubsystem.zeroHeading()), 
                        AutoBuilder.buildAuto("DriveStraight")));
                
                autoChooser.addOption("Amp", new SequentialCommandGroup(
                        new InstantCommand(() -> swerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Amp1"))), 
                        new InstantCommand(() -> swerveSubsystem.zeroHeading()),
                        new WaitCommand(7),
                        new PathPlannerAuto("Amp1")));

                autoChooser.addOption("Drive To Mid Line", new SequentialCommandGroup(
                        new InstantCommand(() -> swerveSubsystem.zeroHeading()), 
                        new WaitCommand(10), 
                        AutoBuilder.buildAuto("DriveToMid")));

                autoChooser.addOption("Amp + Mobility", new SequentialCommandGroup(
                        new InstantCommand(() -> swerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Amp1"))), 
                        new InstantCommand(() -> swerveSubsystem.zeroHeading()),
                        new PathPlannerAuto("amp+mob")));
                
                // Create a autonomous tab and add the auto chooser
                Shuffleboard.getTab("Autonomous").add("Select Auto", autoChooser).withSize(2, 1);

                // set default commands for each Subsystem

                // Default command for this swerve drive so it always takes in controller input by default and is on standby. 
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                        swerveSubsystem,
                        () -> driverController.getRawAxis(IOConstants.kDriverYAxis),
                        () -> driverController.getRawAxis(IOConstants.kDriverXAxis),
                        () -> -driverController.getRawAxis(IOConstants.kDriverRotAxis),
                        () -> !driverController.rightBumper().getAsBoolean(),          
                        () -> driverController.getRightTriggerAxis() > 0.5 ? true : false));     

                intake.setDefaultCommand(intakeStop);  // Intake is always stopped by default. 
                climber.setDefaultCommand(climberStop);  // Climbers are set to brake mode and stopped by default. 

                configureButtonBindings(); // Bind all the controller buttons to commands. 
        }

        /**
         * This method should always be called at the very end of the constructor in the RobotContainer Class. This method is used to assign driver/operator controller buttons 
         * to commands. 
         */
        private void configureButtonBindings() {
                
                // ======================================================== DRIVER COMMANDS =========================================================================================
                // QOL Swerve Commands
                driverController.start().onTrue(zeroGyro);  // .onTrue only runs the Command once when it's pressed.
                driverController.button(7).onTrue(zClimber); // .onTrue only runs the Command once when it's pressed.

                driverController.povUp().whileTrue(nudgeFront); 
                driverController.povRight().whileTrue(nudgeRight); 
                driverController.povLeft().whileTrue(nudgeLeft); 
                driverController.povDown().whileTrue(nudgeBack); 

                // Reset the climber to it's inital position (Hold button). 
                driverController.b().whileTrue(new ClimberManualPosition(climber, 0)); 

                // ======================================================== OPERATOR COMMANDS =========================================================================================
                // Intake Commands
                operatorController.x().whileTrue(intakeSpinBack);

                // Can control the intake speed using Joysticks after it passes the threshold of 0.3
                operatorController.axisGreaterThan(1, 0.3).whileTrue(new LeftIntakeJoystick(() -> operatorController.getRawAxis(1), intake)); 
                operatorController.axisGreaterThan(5, 0.3).whileTrue(new RightIntakeJoystick(() -> operatorController.getRawAxis(5), intake));

                // Climber Commands
                operatorController.axisGreaterThan(3, 0.3).whileTrue(climberUp);  // Can control the climber speed using right trigger after the threshold of 0.3 has passed.
                operatorController.axisGreaterThan(2, 0.3).whileTrue(climberDown); // Can control the climber speed using left trigger after the threshold of 0.3 has passed.
                operatorController.a().whileTrue(new ClimberManualPosition(climber, -59.072261810302734));  // Moves the climber to a set position (HAS TO BE CONFIGURED PROPERLY AT THE START)
                operatorController.y().whileTrue(new ClimberManualPosition(climber, 79.21708679199219));  // Moves the climber to a set position (HAS TO BE CONFIGURED PROPERLY AT THE START)
                operatorController.povUp().onTrue(fullExtend); // .onTrue only runs the Command once when it's pressed.
                operatorController.povDown().onTrue(fullDetract); // .onTrue only runs the Command once when it's pressed.
        }

        /**
         * This method should be called at the very start of the constructor in the RobotContainer Class. This method initializes all the autonomous command names from Pathplanner
         * and assigns them a command. 
         */
        public void configureNamedCommands() { 
                NamedCommands.registerCommand("ShootIntake", intakeSpinForward.withTimeout(.75));  
                NamedCommands.registerCommand("ZeroGyro", zeroGyro);
                NamedCommands.registerCommand("ArmDown", new ClimberManualPosition(climber, 0).withTimeout(2));
                NamedCommands.registerCommand("ArmUp", new ClimberManualPosition(climber, 79.21708679199219).withTimeout(2));
        }

        /**
         * Method used to return whichever autonomous routine/command you are running. 
         * @return Returns a autonmous routine chosen from smartDashboard whenever auto mode is enabled. 
         */
        public Command getAutonomousCommand() {                
                return autoChooser.getSelected(); 
        }        
}