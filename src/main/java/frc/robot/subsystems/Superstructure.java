// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Superstructure extends SubsystemBase {
  
   public final Shooter shooter;
   public final Turret turret;
   public final IntakeSubsystem intake;
   public final KickerSubsystem kicker;  
   public final HopperSubsystem hopper; 
   public final HoodSubsystem hood;

   //Tolerance for setpoint checks
   private static final AngularVelocity shooterTolerance = RPM.of(100);
   private static final Angle turretTolerance = Degrees.of(1);
    private static final Angle HOOD_TOLERANCE = Degrees.of(2);

   
   //Triggers for readiness
   private final Trigger isShooterOnTarget;
   private final Trigger turretOnTarget;
   private final Trigger readyToShoot;

   private AngularVelocity targetShooterSpeed = RPM.of(0);
   private Angle turretTarget = Degrees.of(0);
   private Angle targetHoodAngle = Degrees.of(0);

   //Default aim point red hub 
   private Translation3d aimPoint = Constants.POIs.RED_HUB.value;

  /** Creates a new Superstructure. */
  public Superstructure(Shooter shooter, Turret turret, IntakeSubsystem intake, KickerSubsystem kicker,
  HopperSubsystem hopper, HoodSubsystem hood) {    
    this.turret = turret;
    this.kicker = kicker;
    this.intake = intake;
    this.shooter = shooter;
    this.hopper = hopper;
    this.hood = hood;

    //Create triggers to check if mechs are on target
    this.isShooterOnTarget = new Trigger(
      () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < shooterTolerance.in(RPM));
    
     this.turretOnTarget = new Trigger(
      () -> Math.abs(turret.getRawAngle().in(Degrees) - turretTarget.in(Degrees)) < turretTolerance
      .in(Degrees));
    
     this.readyToShoot = isShooterOnTarget.and(turretOnTarget);

    }

     public Command stopAllCommand() {
    return Commands.parallel(
        shooter.stop().asProxy(),
        turret.set(0).asProxy(),
        hood.set(0).asProxy()).withName("Superstructure.stopAll");
  }

   public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return Commands.runOnce(() -> {
      targetShooterSpeed = shooterSpeed;
      turretTarget = turretAngle;
      targetHoodAngle = hoodAngle;
    }).andThen(
        Commands.parallel(
            shooter.setSpeed(shooterSpeed).asProxy(),
            turret.setAngle(turretAngle).asProxy(),
            hood.setAngle(hoodAngle).asProxy()))
        .withName("Superstructure.aim");
  }
    public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    targetShooterSpeed = shooterSpeed;
    turretTarget = turretAngle;
    targetHoodAngle = hoodAngle;
  }

   public Command aimDynamicCommand(
      Supplier<AngularVelocity> shooterSpeedSupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier) {
    return Commands.parallel(
        shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
        turret.setAngleDynamic(turretAngleSupplier).asProxy(),
        hood.setAngleDynamic(hoodAngleSupplier).asProxy())
        .withName("Superstructure.aimDynamic");
  }
 public Command waitUntilReadyCommand() {
    return Commands.waitUntil(readyToShoot).withName("Superstructure.waitUntilReady");
  }
 
  public Command aimAndWaitCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return aimDynamicCommand(() -> shooterSpeed, () -> turretAngle, () -> hoodAngle)
        .andThen(waitUntilReadyCommand())
        .withName("Superstructure.aimAndWait");
  }

   public Command setTurretForward() {
    return turret.setAngle(Degrees.of(0)).withName("Superstructure.setTurretForward");
  }

  public Command setTurretLeft() {
    return turret.setAngle(Degrees.of(45)).withName("Superstructure.setTurretLeft");
  }

  public Command setTurretRight() {
    return turret.setAngle(Degrees.of(-45)).withName("Superstructure.setTurretRight");
  }

  // Getters for current state
  public AngularVelocity getShooterSpeed() {
    return shooter.getSpeed();
  }

  public Angle getTurretAngle() {
    return turret.getRawAngle();
  }

  public Angle getHoodAngle() {
    return hood.getAngle();
  }

  public AngularVelocity getTargetShooterSpeed() {
    return targetShooterSpeed;
  }

  public Angle getTargetTurretAngle() {
    return turretTarget;
  }

  public Angle getTargetHoodAngle() {
    return targetHoodAngle;
  }

  public Translation3d getAimPoint() {
    return aimPoint;
  }

  public void setAimPoint(Translation3d newAimPoint) {
    this.aimPoint = newAimPoint;
  }

  public Rotation3d getAimRotation3d() {
    // See
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    return new Rotation3d(
        Degrees.of(0), // no roll 🤞
        hood.getAngle().unaryMinus(), // pitch is negative hood angle
        turret.getRobotAdjustedAngle());
  }
  public Command intakeCommand() {
    return intake.intakeCommand().withName("Superstructure.intake");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.ejectCommand().withName("Superstructure.eject");
  }

  /**
   * Command to run the hopper forward while held.
   */
  public Command hopperFeedCommand() {
    return hopper.feedCommand().withName("Superstructure.feed");
  }

  /**
   * Command to run the hopper in reverse while held.
   */
  public Command hopperReverseCommand() {
    return hopper.reverseCommand().withName("Superstructure.hopperReverse");
  }

  /**
   * Command to run the kicker forward while held, stops when released.
   */
  public Command kickerFeedCommand() {
    return kicker.feedCommand().withName("Superstructure.kickerFeed");
  }

  /**
   * Command to run the kicker stop while held, stops when released.
   */
  public Command kickerStopCommand() {
    return kicker.stopCommand().withName("Superstructure.kickerStop");
  }

  public Command feedAllCommand() {
    return Commands.parallel(
        kicker.feedCommand().asProxy(),
        hopper.feedCommand().asProxy(),
        kicker.feedCommand().asProxy()).withName("Superstructure.feedAll");
    // intake.setPivotAngle(Degrees.of(46)).asProxy()).withName("Superstructure.feedAll");
  }

  public Command backFeedAllCommand() {
    return Commands.parallel(
        hopper.backFeedCommand().asProxy(),
        intake.backFeedAndRollCommand().asProxy()).withName("Superstructure.backFeedAll");
  }
   public Command stopFeedingAllCommand() {
    return Commands.parallel(
        hopper.stopCommand().asProxy(),
        kicker.stopCommand().asProxy(),
        intake.deployAndRollCommand().asProxy()).withName("Superstructure.stopFeedingAll");
  }

  /**
   * Command to set the intake pivot angle.
   */
  public Command setIntakePivotAngle(Angle angle) {
    return intake.setPivotAngle(angle).withName("Superstructure.setIntakePivotAngle");
  }

  public Command setIntakeDeployAndRoll() {
    return intake.deployAndRollCommand().withName("Superstructure.setIntakeDeployAndRoll");
  }

  /**
   * Command to shoot - spins up shooter.
   */
  public Command shootCommand() {
    // return shooter.sysId().withName("Superstructure.shoot");
    return shooter.spinUp().withName("Superstructure.shoot");
  }

  /**
   * Command to stop shooting - stops shooter.
   */
  public Command stopShootingCommand() {
    return shooter.stop().withName("Superstructure.stopShooting");
  }

  // Re-zero intake pivot if needed
  public Command rezeroIntakePivotAndTurretCommand() {
    return Commands.parallel(
        turret.rezero().withName("Superstructure.rezeroTurret"),
        intake.rezero().withName("Superstructure.rezeroIntakePivot"))
        .withName("Superstructure.rezeroIntakePivotAndTurret");
  }


  @Override
  public void periodic() {
     // Superstructure doesn't need periodic updates - subsystems handle their own

    String shooterOut = "S:" + isShooterOnTarget.getAsBoolean() + "(" + Math.round(shooter.getSpeed().in(RPM)) + "/"
        + Math.round(targetShooterSpeed.in(RPM)) + ")";

    String turretOut = "T:" + turretOnTarget.getAsBoolean() + "(" + Math.round(turret.getRawAngle().in(Degrees)) + "/"
        + Math.round(turretTarget.in(Degrees)) + ")";

    String readyOut = "R:" + readyToShoot.getAsBoolean();

    // System.out.println(shooterOut + " " + turretOut + " " + hoodOut + " " +
    // readyOut);
  }
public Command useRequirement() {
    return runOnce(() -> {
    });
  }

  public Pose3d getShooterPose() {
    // Position of the shooter relative to the "front" of the robot. Rotation
    // element is based on hood and turret angles
    return new Pose3d(new Translation3d(
        Meter.of(-0.3),
        Meter.of(0),
        Meter.of(0.6)),
        getAimRotation3d());
  }

  public LinearVelocity getTangentialVelocity() {
    return shooter.getTangentialVelocity();
  }

}
