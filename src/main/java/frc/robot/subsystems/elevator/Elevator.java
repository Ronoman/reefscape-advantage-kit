// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.AngularAcceleration;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.EnumMap;
import java.util.Map;

public class Elevator extends SubsystemBase {
  public static class Constants {
    // public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(60.0);
    // public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(115.0);

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(10.0);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(115.0);

    public static final Angle POSITION_TOLERANCE = Rotations.of(0.25);
    public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(1.0);
  
    public static enum ElevatorPosition {
      ALGAE_GROUND,
      ALGAE_STOW,
      ALGAE_PROCESSOR,
      ALGAE_L2,
      ALGAE_L3,
      ALGAE_BARGE,
  
      CORAL_GROUND,
      CORAL_STOW,
      CORAL_L1,
      CORAL_L2,
      CORAL_L3,
      CORAL_L4,
    }
  
    public static final EnumMap<ElevatorPosition, Angle> ElevatorPositionToRotations = new EnumMap<>(
      Map.ofEntries(
        Map.entry(ElevatorPosition.ALGAE_GROUND, Rotations.of(3.0)),
        Map.entry(ElevatorPosition.ALGAE_STOW, Rotations.of(0.0)),
        Map.entry(ElevatorPosition.ALGAE_PROCESSOR, Rotations.of(8.8)),
        Map.entry(ElevatorPosition.ALGAE_L2, Rotations.of(24.018)),
        Map.entry(ElevatorPosition.ALGAE_L3, Rotations.of(34.77)),
        Map.entry(ElevatorPosition.ALGAE_BARGE, Rotations.of(34.29)),
  
        Map.entry(ElevatorPosition.CORAL_GROUND, Rotations.of(0.0)),
        Map.entry(ElevatorPosition.CORAL_STOW, Rotations.of(0.0)),
        Map.entry(ElevatorPosition.CORAL_L1, Rotations.of(12.0)),
        Map.entry(ElevatorPosition.CORAL_L2, Rotations.of(13.256)),
        Map.entry(ElevatorPosition.CORAL_L3, Rotations.of(23.914 + 0.75)),
        Map.entry(ElevatorPosition.CORAL_L4, Rotations.of(39.0))
      )
    );
  }

  private final ElevatorIO io;
  // private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private AngularVelocity maxVelocity = Constants.MAX_VELOCITY;
  private AngularAcceleration maxAcceleration = Constants.MAX_ACCELERATION;

  private TrapezoidProfile profile = new TrapezoidProfile(
    new Constraints(
      Constants.MAX_VELOCITY.in(RotationsPerSecond),
      Constants.MAX_ACCELERATION.in(RotationsPerSecondPerSecond)
    )
  );

  private State startState = new State();
  private State targetState = new State();
  private final Timer timeSinceLastGoalSet = new Timer();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    State desiredState = this.profile.calculate(
      this.timeSinceLastGoalSet.get(),
      this.startState,
      this.targetState
    );
    this.io.setDesiredState(new PositionVoltage(desiredState.position).withVelocity(desiredState.velocity));

    Logger.recordOutput("Elevator/profile/maxVelocity", this.maxVelocity);
    Logger.recordOutput("Elevator/profile/maxAcceleration", this.maxAcceleration);
    Logger.recordOutput("Elevator/desiredState/position", Rotations.of(desiredState.position));
    Logger.recordOutput("Elevator/desiredState/velocity", RotationsPerSecond.of(desiredState.velocity));
  }

  public void setElevatorGoal(
    Constants.ElevatorPosition position,
    AngularVelocity maxVelocity,
    AngularAcceleration maxAcceleration
  ) {
    this.profile = new TrapezoidProfile(
      new Constraints(maxVelocity.in(RotationsPerSecond), maxAcceleration.in(RotationsPerSecondPerSecond))
    );

    // Only used to log in periodic() later.
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;

    // Right motor is the leader, so it's the source of truth for current state
    this.startState = new State(this.inputs.rightPosition.in(Rotations), this.inputs.rightVelocity.in(RotationsPerSecond));

    // Assume the desired end velocity is zero
    this.targetState = new State(Constants.ElevatorPositionToRotations.get(position).in(Rotations), 0.0);
    this.timeSinceLastGoalSet.reset();
  }

  public boolean isAtSetpoint() {
    return this.inputs.rightPosition.minus(Rotations.of(this.targetState.position)).magnitude() < Constants.POSITION_TOLERANCE.magnitude() &&
           this.inputs.rightVelocity.minus(RotationsPerSecond.of(this.targetState.velocity)).magnitude() < Constants.VELOCITY_TOLERANCE.magnitude();
  }
}
