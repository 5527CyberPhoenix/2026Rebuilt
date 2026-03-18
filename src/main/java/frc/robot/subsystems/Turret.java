// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TalonIDs;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends SubsystemBase {
   private static TalonFXConfiguration config = new TalonFXConfiguration();
  public final Translation3d turretPose = new Translation3d(-0.225, 0, .4); // Turret Position
  private TalonFX motor = new TalonFX(TalonIDs.turretmotor, "CAN");  
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(15, 0, 0, DegreesPerSecond.of(2440), DegreesPerSecondPerSecond.of(2440))
    .withFeedforward(new SimpleMotorFeedforward(0,7.5,0))
    .withTelemetry("Motor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(4,10)))
    .withMotorInverted(true)
    .withIdleMode(MotorMode.COAST)
    .withSoftLimit(Degrees.of(-90), Degrees.of(90))
    .withStatorCurrentLimit(Amps.of(10))
    .withClosedLoopRampRate(Seconds.of(.1))
    .withOpenLoopRampRate(Seconds.of(.1));

    private SmartMotorController smc = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1), smcConfig);
    
    private final PivotConfig turretConfig = new PivotConfig(smc)
    .withHardLimit(Degrees.of(-95), Degrees.of(95))
    .withStartingPosition(Degrees.of(0))
    .withMOI(0.05)
    .withTelemetry("Turret", TelemetryVerbosity.HIGH)
    .withMechanismPositionConfig(
      new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(turretPose));

    private Pivot turret = new Pivot(turretConfig);
  
   public Turret() {
      // config
      // .Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
      
      //   motor.getConfigurator().apply(config);
   }

  public Command setAngle(Angle angle){
    return turret.setAngle(angle);
  }
  
   public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return turret.setAngle(turretAngleSupplier);
  }

    public Command center() {
    return turret.setAngle(Degrees.of(0));
  }

   public Angle getRobotAdjustedAngle() {
    // Returns the turret angle in the robot's coordinate frame
    // since the turret is mounted backwards, we need to add 180 degrees
    return turret.getAngle();
  }

    public Angle getRawAngle() {
    return turret.getAngle();
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command rezero() {
    return Commands.runOnce(() -> motor.setPosition(0), this).withName("Turret.Rezero");
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }
 
  public Command turretEstimate(){
    var ll = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-cyber");

    if(ll != null && ll.tagCount > 0){
      return turret.setAngle(Degrees.of(LimelightHelpers.getTX("limelight-cyber")));
    }
    else{
      return null;
    }
  }
  @Override
  public void periodic() {
    turret.updateTelemetry();

    // Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
    //     new Pose3d(
    //         turretPose,
    //         new Rotation3d(0, 0, turret.getAngle().in(Radians)))
    // });
  }
    @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}

