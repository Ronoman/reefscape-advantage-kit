package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;


public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public Distance height = Inches.of(0.0);
        public LinearVelocity velocity = InchesPerSecond.of(0.0);

        public boolean bottomLimit = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setDesiredState(PositionVoltage desiredState) {}
}
