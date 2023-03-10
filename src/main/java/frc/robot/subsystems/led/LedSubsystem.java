// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.led.LedConfig.*;

public class LedSubsystem extends SubsystemBase {

  PWMSparkMax motorController = new PWMSparkMax(BLINKIN_PWM_PORT);

  enum LedState {
    Confetti,
    Green,
    OceanRainbow,
    Purple,
    Red,
    Yellow
  }

  private LedState currentState;

  public LedSubsystem() {
    setLedToConfetti();
    initDashBoard();
  }

  @Override
  public void periodic() {
  }

  public void setLedToConfetti() {
    currentState = LedState.Confetti;
    motorController.set(-0.87);
  }

  public void setLedToGreen() {
    currentState = LedState.Green;
    motorController.set(0.77);
  }

  public void setLedToRed() {
    currentState = LedState.Red;
    motorController.set(0.61);
  }

  public void setLedToYellow() {
    currentState = LedState.Yellow;
    motorController.set(0.69);
  }

  public void setLedToPurple() {
    currentState = LedState.Purple;
    motorController.set(0.91);
  }

  public void setLedToOceanRainbow() {
    currentState = LedState.OceanRainbow;
    motorController.set(-0.95);
  }

  private void initDashBoard() {
    var tab = Shuffleboard.getTab("LED");
    tab.addString("State", () -> currentState.toString());
  }
}
