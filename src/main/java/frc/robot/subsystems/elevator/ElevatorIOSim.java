package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;

public class ElevatorIOSim implements ElevatorIO {
    PositionVoltage desiredState = new PositionVoltage(0.0);

    public ElevatorIOSim() {}

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftInputs.position = desiredState.getPositionMeasure();
        inputs.leftInputs.velocity = desiredState.getVelocityMeasure();

        inputs.rightInputs.position = desiredState.getPositionMeasure();
        inputs.rightInputs.velocity = desiredState.getVelocityMeasure();

        // This will give a nonsense value, but will provide a number.
        inputs.height = Meters.of(desiredState.getPositionMeasure().baseUnitMagnitude());
        inputs.velocity = MetersPerSecond.of(desiredState.getVelocityMeasure().baseUnitMagnitude());

        inputs.bottomLimit = false;
    }

    public void setDesiredState(PositionVoltage desiredState) {}
}
