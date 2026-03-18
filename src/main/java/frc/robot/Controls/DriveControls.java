// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Controls;

import static edu.wpi.first.units.Units.Meter;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;


/** Add your docs here. */
public class DriveControls {
    static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
      private static Pose2d getTargetPose() {
        Pose2d hubPose = new Pose2d(
            Meter.of(11.902),
            Meter.of(4.031),
            Rotation2d.kZero);
    
        //Logger.recordOutput("DriverControls/TargetHubPose", hubPose);
    
        return hubPose;
      }
    
        public static void configure(int port,CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
        CommandXboxController controller = new CommandXboxController(port);
            controller.leftBumper().whileTrue(
              superstructure.feedAllCommand()
                  .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));
    
          controller.rightBumper()
              .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));
    
        //   controller.leftTrigger().whileTrue(Commands.repeatingSequence(
        //     fireFuel(drivetrain, superstructure),
        // Commands.waitSeconds(.1)));
        


}

 public static Command fireFuel( CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      SimulatedArena arena = SimulatedArena.getInstance();

      RebuiltFuelOnFly fuel = new RebuiltFuelOnFly(
          drivetrain.getPose().getTranslation(),
          new Translation2d(
              superstructure.turret.turretPose.getX() * -1,
              superstructure.turret.turretPose.getY()),
          drivetrain.getSpeed(), drivetrain.getPose().getRotation().rotateBy(superstructure.getAimRotation3d().toRotation2d()),
          superstructure.turret.turretPose.getMeasureZ(),

          // 0.5 times because we're applying spin to the fuel as we shoot it
          superstructure.getTangentialVelocity().times(0.5),
          superstructure.getHoodAngle());

      // // Configure callbacks to visualize the flight trajectory of the projectile
      // fuel.withProjectileTrajectoryDisplayCallBack(
      //     // Callback for when the note will eventually hit the target (if configured)
      //     (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
      //         pose3ds.toArray(Pose3d[]::new)),
      //     // Callback for when the note will eventually miss the target, or if no target
      //     // is configured
      //     (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot",
      //         pose3ds.toArray(Pose3d[]::new)));

      arena.addGamePieceProjectile(fuel);
    });
  }












}