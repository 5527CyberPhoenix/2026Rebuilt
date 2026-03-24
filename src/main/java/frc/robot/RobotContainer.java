// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Controls.DriveControls;
import frc.robot.Controls.OperatorControls;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();
    private final HopperSubsystem hopper = new HopperSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final Superstructure superstructure = new Superstructure(shooter, turret, intake, kicker, hopper, hood, () -> drivetrain.getState().Pose);
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

        private final ShooterSubsystem shooters = new ShooterSubsystem(drivetrain, Constants.LimelightConstants.LIMELIGHT_NAME);


    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
          LimelightHelpers.setCameraPose_RobotSpace(
                                Constants.LimelightConstants.LIMELIGHT_NAME,
                                0,
                                0,
                                0,
                                0,
                                0.0,
                                0.0);
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Chosen Auto", autoChooser);

        // NamedCommands.registerCommand("ShootOnTheMove", new ShootOnTheMoveCommand(drivetrain, superstructure, () -> superstructure.getAimPoint()));
        // NamedCommands.registerCommand("FEED", superstructure.feedAllCommand());
        // NamedCommands.registerCommand("Intake", superstructure.setIntakeDeployAndRoll());
               configureBindings();
            }
        
            private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * .5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * .5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftTrigger().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        DriveControls.configure(0,drivetrain, superstructure);
        OperatorControls.configure(1, drivetrain, superstructure, shooters);
    }

    public Command getAutonomousCommand() {
       // Run selected auton from auto chooser
       return Commands.none();
    }
}
