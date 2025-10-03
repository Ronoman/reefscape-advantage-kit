package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.PositionVoltage;
import frc.robot.util.LoggedTalonFX.TalonFXInputs;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public TalonFXInputs leftInputs = new TalonFXInputs();
        public TalonFXInputs rightInputs = new TalonFXInputs();

        public boolean bottomLimit = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setDesiredState(PositionVoltage desiredState) {}
}
