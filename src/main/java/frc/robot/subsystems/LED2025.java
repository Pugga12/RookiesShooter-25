// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;;

public class LED2025 extends SubsystemBase {
  private static AddressableLED strip;
  private static AddressableLEDBuffer buffer;
  private static LEDPattern currentPattern = LEDPattern.kOff;
  private static boolean isAnimated = false;
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);
  

  /** Creates a new LED2025. */
  public LED2025(int length) {
    strip = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(length);
    strip.setLength(length);
    strip.start(); 
    currentPattern.applyTo(buffer);
    strip.setData(buffer);
  }

  @Override
  public void periodic() {
    // only apply the buffer if an animation is applied (saves cpu)
    if (isAnimated) {
      currentPattern.applyTo(buffer);
      strip.setData(buffer);
    }
  }

  public Command setColor(Color color) {
    return new InstantCommand(() -> {
      currentPattern = LEDPattern.solid(color);
      currentPattern.applyTo(buffer);
      strip.setData(buffer);
      isAnimated = false;
    }, this);
  }

  public Command applyRainbow() {
    return new InstantCommand(() -> {
      LEDPattern rainbow = LEDPattern.rainbow(255, 128);
      currentPattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
      isAnimated = true;
    }, this);
  }
}
