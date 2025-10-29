// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
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

/**
 * All business logic for the Elevator subsystem. This class has a subclass for subsystem-specific constants like IDs,
 * position enumerations, max velocities/accelerations for trapezoid profiles, and velocity/acceleration tolerances.
 * 
 * This subsystems owns its own TrapezoidProfile. In periodic, it will continuously sample over the TrapezoidProfile,
 * passing the desired position and velocity along to the inputs class.
 */
public class Elevator extends SubsystemBase {
  public static class Constants {
    // Actual parameters from the 2025 codebase.
    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(60.0);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(115.0);

    // Position and velocity tolernaces, used by isAtGoal.
    public static final Angle POSITION_TOLERANCE = Rotations.of(0.25);
    public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(1.0);
  
    // Different enumerated positions that the elevator has positions for.
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

    // Mappings for named positions to the number of motor rotations needed to get there, measured from the bottom
    // resting position. These were found empirically during testing, and tweaked during competitions.
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

  // The IO class to actually manage hardware interaction with the subsystem.
  private final ElevatorIO io;

  // The AdvantageKit-generated AutoLogged input class. When used in periodic(), this will result in all input fields
  // being logged automatically to NetworkTables (accessible by SmartDashboard, Elastic, AdvantageScope, etc).
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // The current max velocity that the TrapezoidProfile uses.
  @AutoLogOutput
  private AngularVelocity maxVelocity = Constants.MAX_VELOCITY;

  // The current max acceleration that the TrapezoidProfile uses.
  @AutoLogOutput
  private AngularAcceleration maxAcceleration = Constants.MAX_ACCELERATION;

  // The subsystem samples over this trapezoid profile with the below start and target states to determine the current
  // desired position and velocity of the subsystem.
  private TrapezoidProfile profile = new TrapezoidProfile(
    new Constraints(
      Constants.MAX_VELOCITY.in(RotationsPerSecond),
      Constants.MAX_ACCELERATION.in(RotationsPerSecondPerSecond)
    )
  );

  // Start and target states used by the TrapezoidProfile.
  private State startState = new State();
  private State targetState = new State();

  // A Timer to keep track of the duration since the last time a goal was set on the TrapezoidProfile. This is used to
  // sample into the TrapezoidProfile.
  private final Timer timeSinceLastGoalSet = new Timer();

  // This Elevator will receive some concrete implementation of ElevatorIO. For RobotContainer.Constants.REAL, this will
  // be an instance of ElevatorIOTalonFX.
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update our IO inputs, and log them under the Elevator/ prefix.
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Sample over the TrapezoidProfile, then pass the desired state off to the IO implementation for handoff to
    // the motors.
    State desiredState = this.profile.calculate(
      this.timeSinceLastGoalSet.get(),
      this.startState,
      this.targetState
    );
    this.io.setDesiredState(new PositionVoltage(desiredState.position).withVelocity(desiredState.velocity));

    // Record various parameters that we care about in addition to auto logged inputs.
    Logger.recordOutput("Elevator/profile/position", Rotations.of(desiredState.position));
    Logger.recordOutput("Elevator/profile/velocity", RotationsPerSecond.of(desiredState.velocity));
    Logger.recordOutput("Elevator/profile/timeSinceLastGoalSet", this.timeSinceLastGoalSet.get());
    Logger.recordOutput("Elevator/isAtGoal", this.isAtGoal());
  }

  /**
   * The primary way for Commands to interact with this subsystem. They provide a named ElevatorPosition to travel to,
   * as well as the max velocity and acceleration that should be used for the TrapezoidProfile.
   * 
   * @param position The position that the elevator should travel to. This is one of the enumerated pre-determined positions.
   * @param maxVelocity The maximum velocity that the TrapezoidProfile is allowed to reach.
   * @param maxAcceleration The maximum velocity that the TrapezoidProfile is allowed to reach.
   * 
   * @return Nothing, but will update the goal for the TrapezoidProfile, which is likely to induce subsystem movement.
   */
  public void setElevatorGoal(
    Constants.ElevatorPosition position,
    AngularVelocity maxVelocity,
    AngularAcceleration maxAcceleration
  ) {
    System.out.println("Setting new goal (position: " + Constants.ElevatorPositionToRotations.get(position).in(Rotations) + ")");
    this.profile = new TrapezoidProfile(
      new Constraints(maxVelocity.in(RotationsPerSecond), maxAcceleration.in(RotationsPerSecondPerSecond))
    );

    // Only used to log in periodic() later.
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;

    // Right motor is the leader, so it's the source of truth for current state
    this.startState = new State(this.inputs.rightInputs.position.in(Rotations), this.inputs.rightInputs.velocity.in(RotationsPerSecond));

    // Assume the desired end velocity is zero
    this.targetState = new State(Constants.ElevatorPositionToRotations.get(position).in(Rotations), 0.0);
    this.timeSinceLastGoalSet.reset();
    this.timeSinceLastGoalSet.start();
  }

  /**
   * Whether the Elevator is currently within tolerance of the most recent goal passed in to setElevatorGoal.
   * @return True if the position and velocity are both within the Constant-specified tolerance.
   */
  public boolean isAtGoal() {
    return this.inputs.rightInputs.position.isNear(Rotations.of(this.targetState.position), Constants.POSITION_TOLERANCE) &&
           this.inputs.rightInputs.velocity.isNear(RotationsPerSecond.of(this.targetState.velocity), Constants.VELOCITY_TOLERANCE);
  }

  public void resetOnInit() {
    this.setElevatorGoal(Constants.ElevatorPosition.CORAL_STOW, Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION);
  }
}
