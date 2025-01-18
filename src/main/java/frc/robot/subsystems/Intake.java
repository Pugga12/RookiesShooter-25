// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor1 = new TalonFX(13, "canivore");
  private final TalonFX intakeMotor2 = new TalonFX(26, "canivore");
  private final TalonFX diverterMotor = new TalonFX(14, "canivore");

  //Creates BeamBreaker
  DigitalInput beamBreaker = new DigitalInput(8);
  DigitalInput armBeamBreaker = new DigitalInput(9);

  private boolean previousIntakeBeamBreaker = false;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor1.setInverted(true);
    intakeMotor2.setInverted(false);
    diverterMotor.setInverted(true);
  }

  public void setMotorPower(double motorPower, double diverterPower) {
    intakeMotor1.set(motorPower);
    intakeMotor2.set(motorPower);
    diverterMotor.set(diverterPower);
  }

  public void stopMotor(){
    intakeMotor1.stopMotor();
    intakeMotor2.stopMotor();
    diverterMotor.stopMotor();
  }

  public boolean intakeBeamBroken(){
    return !beamBreaker.get();
  }

  public boolean armBeamBroken() {
    return !armBeamBreaker.get();
  }

  public void forkToShooter() {
    diverterMotor.set(1);
  }

  public void forkToAmp() {
    diverterMotor.set(0.8);
    intakeMotor1.set(0.8);
    intakeMotor2.set(0.8);
  }

  public void runForkToIntake() {
    intakeMotor1.set(-0.5);
    intakeMotor2.set(-0.5);
    diverterMotor.set(-0.5);
  }

  public boolean intakeChangeFromBrokenToUnbroken() {
    if (intakeBeamBroken()) {
      previousIntakeBeamBreaker = true;
      return false;
    } else {
      if (previousIntakeBeamBreaker) {
        previousIntakeBeamBreaker = false;
        return true;
      } else {
        return false;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake/intake beam", intakeBeamBroken());
    SmartDashboard.putBoolean("intake/arm beam", armBeamBroken());
  }
}
