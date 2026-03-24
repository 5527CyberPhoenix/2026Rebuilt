package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.util.AllianceUtil;
import edu.wpi.first.epilogue.Logged;

/**
 * A complex command that combines automatic alignment to the hub with a
 * distance-based shooting sequence.
 * This command first aligns the robot to the hub and spins up the shooters to
 * the calculated speed in parallel.
 * Once both conditions (alignment and shooter at speed) are met, it activates
 * the feeder and washer subsystems
 * to initiate shooting. All mechanisms stop when the command is interrupted.
 */
@Logged
public class AutoAlignAndShoot extends Command {
    private final Command fullCommand;
    private final AutoAlignCommand autoAlignCommand;
    private final ShooterSubsystem shooter;

    /**
     * Creates a new AutoAlignAndShoot command.
     *
     * @param drivetrain     The {@link CommandSwerveDrivetrain} subsystem for robot
     *                       movement and alignment.
     * @param shooter        The {@link ShooterSubsystem} for controlling shooter
     *                       motors.
     * @param feeder         The {@link FeederSubsystem} for controlling the feeder
     *                       mechanism.
     * @param washer         The {@link WasherSubsystem} for controlling the washer
     *                       mechanism.
     * @param superstructure The {@link Superstructure} for alliance zone checks.
     * @param limelight      The {@link LimelightSubsystem} for vision data used in
     *                       alignment.
     */
    public AutoAlignAndShoot(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            Superstructure superstructure,
            LimelightSubsystem limelight
            ) {

        autoAlignCommand = new AutoAlignCommand(drivetrain,
                () -> Rotation2d.fromDegrees(
                        frc.robot.util.AllianceUtil.getTargetHeadingToHub(drivetrain, Constants.LimelightConstants.LIMELIGHT_NAME)),
                () -> 0.0, () -> 0.0);
        this.shooter = shooter;

        addRequirements(drivetrain, shooter, superstructure, limelight);

        fullCommand = Commands.parallel(
                autoAlignCommand,
                shooter.runInterpolatedShot(shooter::getHubDistance),
                Commands.sequence(
                        Commands.waitUntil(shooter::areShootersAtSpeed)
                        ))
               
                .withName("AutoAlignAndShoot");
    }
    
    @Override
    public void initialize() {
        fullCommand.initialize();
    }

    @Override
    public void execute() {
        fullCommand.execute();

        SmartDashboard.putBoolean("AutoAlignAndShoot/ShootersAtSpeed", shooter.areShootersAtSpeed());
        SmartDashboard.putNumber("AutoAlignAndShoot/HubDistance", shooter.getHubDistance());
    }

    @Override
    public boolean isFinished() {
        return fullCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        fullCommand.end(interrupted);
    }
}
