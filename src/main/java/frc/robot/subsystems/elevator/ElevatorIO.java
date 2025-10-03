package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;


public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        // TODO: Convert to linear measurement?

        public Angle leftPosition = Rotations.of(0.0);
        public Angle leftError = Rotations.of(0.0);
        public AngularVelocity leftVelocity = RotationsPerSecond.of(0.0);
        public Voltage leftVoltage = Volts.of(0.0);
        public Current leftCurrent = Amps.of(0.0);

        public Angle rightPosition = Rotations.of(0.0);
        public Angle rightError = Rotations.of(0.0);
        public AngularVelocity rightVelocity = RotationsPerSecond.of(0.0);
        public Voltage rightVoltage = Volts.of(0.0);
        public Current rightCurrent = Amps.of(0.0);

        public boolean bottomLimit = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setDesiredState(PositionVoltage desiredState) {}
}
