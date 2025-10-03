package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

public class LoggedTalonFX extends TalonFX {
    private final StatusSignal<Angle> position;
    private final StatusSignal<Double> error;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;

    @AutoLog
    public static class TalonFXInputs {
        public Angle position = Rotations.of(0.0);
        public Double error = 0.0;
        public AngularVelocity velocity = RotationsPerSecond.of(0.0);
        public Voltage voltage = Volts.of(0.0);
        public Current current = Amps.of(0.0);
    }

    public LoggedTalonFX(int id, String canbus) {
        super(id, canbus);

        position = getPosition();
        error = getClosedLoopError();
        velocity = getVelocity();
        voltage = getMotorVoltage();
        current = getSupplyCurrent();

        getConfigurator().apply(new TalonFXConfiguration());

        BaseStatusSignal.setUpdateFrequencyForAll(50, position, error, velocity, voltage, current);
        super.optimizeBusUtilization();
    }

    public LoggedTalonFX(int id) {
        this(id, "rio");
    }

    public void updateInputs(TalonFXInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, error, voltage, current);

        inputs.position = position.getValue();
        inputs.error = error.getValue();
        inputs.velocity = velocity.getValue();
        inputs.voltage = voltage.getValue();
        inputs.current = current.getValue();
    }
}
