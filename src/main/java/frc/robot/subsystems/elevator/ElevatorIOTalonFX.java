package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;


public class ElevatorIOTalonFX implements ElevatorIO {
    public static class Constants {
        public static String CAN_BUS = "rio";

        public static int LEFT_MOTOR_ID = 28;
        public static int RIGHT_MOTOR_ID = 27;
        public static int BOTTOM_LIMIT_ID = 5;

        public static Current CURRENT_LIMIT = Amps.of(80);
        
        public static final double P = 1.6;
        public static final double I = 0.125;
        public static final double D = 0.0;
        public static final double F = 0.58;
        public static final double V = 0.0;
        public static final double A = 0.0;
        public static final double S = 0.0;

        public static double UPDATE_FREQUENCY = 50.0;
    }

    private DigitalInput bottomLimitSwitch = new DigitalInput(Constants.BOTTOM_LIMIT_ID);

    private final TalonFX leftMotor = new TalonFX(Constants.LEFT_MOTOR_ID, Constants.CAN_BUS);
    private final TalonFX rightMotor = new TalonFX(Constants.RIGHT_MOTOR_ID, Constants.CAN_BUS);

    private final StatusSignal<Angle> leftPosition = leftMotor.getPosition();
    private final StatusSignal<Double> leftError = leftMotor.getClosedLoopError();
    private final StatusSignal<AngularVelocity> leftVelocity = leftMotor.getVelocity();
    private final StatusSignal<Voltage> leftVoltage = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> leftCurrent = leftMotor.getSupplyCurrent();

    private final StatusSignal<Angle> rightPosition = rightMotor.getPosition();
    private final StatusSignal<Double> rightError = rightMotor.getClosedLoopError();
    private final StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();
    private final StatusSignal<Voltage> rightVoltage  = rightMotor.getMotorVoltage();
    private final StatusSignal<Current> rightCurrent  = rightMotor.getSupplyCurrent();

    private final PositionVoltage desiredState = new PositionVoltage(0).withVelocity(0);

    public ElevatorIOTalonFX() {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.CURRENT_LIMIT.in(Amps);
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rightConfig.CurrentLimits.SupplyCurrentLimit = Constants.CURRENT_LIMIT.in(Amps);
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rightConfig.Slot0.kP = Constants.P;
        rightConfig.Slot0.kI = Constants.I;
        rightConfig.Slot0.kD = Constants.D;
        rightConfig.Slot0.kG = Constants.F;
        rightConfig.Slot0.kV = Constants.V;
        rightConfig.Slot0.kA = Constants.A;
        rightConfig.Slot0.kS = Constants.S;

        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig, 0.25));
        tryUntilOk(5, () -> rightMotor.getConfigurator().apply(rightConfig, 0.25));

        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.UPDATE_FREQUENCY,
            leftPosition, leftError, leftVelocity, leftVoltage, leftCurrent,
            rightPosition, rightError, rightVelocity, rightVoltage, rightCurrent
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        leftMotor.setControl(new Follower(Constants.RIGHT_MOTOR_ID, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftPosition, leftVelocity, leftVoltage, leftCurrent,
            rightPosition, rightVelocity, rightVoltage, rightCurrent
        );

        inputs.leftInputs.updateInputs();

        inputs.rightPosition = rightPosition.getValue();
        inputs.rightError = Rotations.of(rightError.getValue());
        inputs.rightVelocity = rightVelocity.getValue();
        inputs.rightVoltage = rightVoltage.getValue();
        inputs.rightCurrent = rightCurrent.getValue();

        inputs.bottomLimit = bottomLimitSwitch.get();
    }

    @Override
    public void setDesiredState(PositionVoltage desiredState) {
        this.desiredState = desiredState;
        rightMotor.setControl(desiredState.withEnableFOC(true));
    }
}
