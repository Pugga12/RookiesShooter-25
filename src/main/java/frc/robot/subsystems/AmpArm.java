// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AmpForkCommand;
import frc.robot.commands.ArmHandoff;

public class AmpArm extends SubsystemBase {
  private final TalonFX pivotMotor = new TalonFX(24, "canivore");
  private final TalonFX shootMotor = new TalonFX(23, "canivore");
  private final CANcoder encoder = new CANcoder(25, "canivore");

  private final ProfiledPIDController pivotController =
    new ProfiledPIDController(
      Constants.AmpArm.pivotkP, 
      Constants.AmpArm.pivotkI,
      Constants.AmpArm.pivotkD,
      new Constraints(Constants.AmpArm.kMaxVelocityRadPerSecond, Constants.AmpArm.kMaxAccelerationRadPerSecSquared)
  );

  private final ArmFeedforward pivotFeedforward = 
    new ArmFeedforward(
      Constants.AmpArm.pivotkS,
      Constants.AmpArm.pivotkG,
      Constants.AmpArm.pivotkV
    );

  /** Creates a new AmpArm. */
  public AmpArm() {
    pivotController.enableContinuousInput(0, Math.PI * 2);
    pivotController.setTolerance(0.01);
    pivotController.setIZone(.1);

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    shootMotor.setNeutralMode(NeutralModeValue.Coast);
    pivotMotor.setPosition(-90 / 14.7);
    setGoal(Constants.AmpArm.ampShootPosition);
  }

  private double convert360To180(double angle) {
    return (angle + 180) % 360 - 180;
  }

  public double getMeasurementRadians() {
      double preConversion = convert360To180(((getCANCoderPositionDegrees())) % 360) * Math.PI / 180;
      return preConversion + 3.425 - Math.PI;
  }

  public double getCANCoderPositionDegrees() {
      return ((encoder.getAbsolutePosition().getValueAsDouble() * 360 + 360) - 63) % 360;
  }
  
  public void setGoal(double goal) {
    pivotController.setGoal(goal);
  }

  public Command getHomeCommand() {
    return new InstantCommand(() -> {
      setGoal(Constants.AmpArm.homePosition);
    }, this);
  }

  public Command getAmpShootCommand() {
    return new InstantCommand(() -> {
      setGoal(Constants.AmpArm.ampShootPosition);
    }, this);
  }

  public void shoot() {
    shootMotor.set(Constants.AmpArm.shootSpeed);
  }

  public void armHandoff() {
    shootMotor.set(Constants.AmpArm.handoffSpeed);
  }

  public void stopShooter() {
    shootMotor.stopMotor();
  }

  public boolean pivotAtGoal() {
    return pivotController.atGoal();
  } 

   public Command handoffArmToIntake() {
    return new ParallelDeadlineGroup(
      new ArmHandoff(),
      new AmpForkCommand()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidOutput = pivotController.calculate(getMeasurementRadians());
    double feedForward = pivotFeedforward.calculate(pivotController.getSetpoint().position, pivotController.getSetpoint().velocity);
    double outputVoltage = pidOutput + feedForward;
    pivotMotor.setVoltage(outputVoltage);

    SmartDashboard.putNumber("arm/cancoderRadians", getMeasurementRadians());
    SmartDashboard.putNumber("arm/motorVoltage", outputVoltage);
    SmartDashboard.putNumber("arm/goal", pivotController.getGoal().position);
  }
}
