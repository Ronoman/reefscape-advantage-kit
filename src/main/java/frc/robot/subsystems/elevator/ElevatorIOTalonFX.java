package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.CustomUnits.MetersPerRotation;
import static frc.robot.util.CustomUnits.MetersPerSecondPerRotationPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.LoggedTalonFX;
import frc.robot.util.TalonFXInputsAutoLogged;


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

        private static final double _METERS_PER_ROTATION = 1.0;
        public static final Per<DistanceUnit, AngleUnit> METERS_PER_ROTATION = MetersPerRotation.ofNative(_METERS_PER_ROTATION);
        public static final Per<LinearVelocityUnit, AngularVelocityUnit> METERS_PER_SECOND_PER_ROTATIONS_PER_SECOND = MetersPerSecondPerRotationPerSecond.ofNative(_METERS_PER_ROTATION);
    }

    private DigitalInput bottomLimitSwitch = new DigitalInput(Constants.BOTTOM_LIMIT_ID);

    private final LoggedTalonFX leftMotor = new LoggedTalonFX(Constants.LEFT_MOTOR_ID, Constants.CAN_BUS);
    private final LoggedTalonFX rightMotor = new LoggedTalonFX(Constants.RIGHT_MOTOR_ID, Constants.CAN_BUS);

    private final TalonFXInputsAutoLogged leftInputs = new TalonFXInputsAutoLogged();
    private final TalonFXInputsAutoLogged rightInputs = new TalonFXInputsAutoLogged();

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

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        leftMotor.setControl(new Follower(Constants.RIGHT_MOTOR_ID, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        leftMotor.updateInputs(leftInputs);
        rightMotor.updateInputs(rightInputs);
    
        Logger.processInputs("Elevator/leftMotor", leftInputs);
        Logger.processInputs("Elevator/rightMotor", rightInputs);

        inputs.height = (Distance) Constants.METERS_PER_ROTATION.timesDivisor(leftInputs.position);
        inputs.velocity = (LinearVelocity) Constants.METERS_PER_SECOND_PER_ROTATIONS_PER_SECOND.timesDivisor(leftInputs.velocity);

        inputs.bottomLimit = bottomLimitSwitch.get();
    }

    @Override
    public void setDesiredState(PositionVoltage desiredState) {
        rightMotor.setControl(desiredState.withEnableFOC(true));
    }
}
