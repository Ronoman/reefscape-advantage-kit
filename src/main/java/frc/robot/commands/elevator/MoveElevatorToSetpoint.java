// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Constants.ElevatorPosition;

// This command moves to the elevator from its current position to the argument
// targetHeight (meters). This command will end when the elevator is within some
// tolorence of the desired position. This command uses the trapazoid motion profile.
// This command is long and requires a fair amount of state so it is not defined within the
// Elevator subsystem.
public class MoveElevatorToSetpoint extends Command {
  Elevator elevator;
  ElevatorPosition targetPosition;

  AngularVelocity velocityOverride = null;
  AngularAcceleration accelerationOverride = null;

  public MoveElevatorToSetpoint(Elevator elevator, ElevatorPosition targetPosition) {
    this.targetPosition = targetPosition;
    this.elevator = elevator;

    addRequirements(elevator);
  }

  public MoveElevatorToSetpoint(Elevator elevator, ElevatorPosition targetPosition, AngularVelocity velocityOverride, AngularAcceleration accelerationOverride) {
    this(elevator, targetPosition);

    this.velocityOverride = velocityOverride;
    this.accelerationOverride = accelerationOverride;
  }

  @Override
  public void initialize() {
    System.out.println("Move elevator to Setpoint Command Initialize");

    if(this.velocityOverride == null && this.accelerationOverride == null) {
      this.elevator.setElevatorGoal(this.targetPosition, Elevator.Constants.MAX_VELOCITY, Elevator.Constants.MAX_ACCELERATION);
    } else {
      this.elevator.setElevatorGoal(this.targetPosition, this.velocityOverride, this.accelerationOverride);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("Move elevator to Setpoint Command End");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
