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


/**
 * The concrete implementation of an ElevatorIO, built to run on TalonFXs (Krakens in the case of our 2025 robot).
 * This class owns the instances to the left and right LoggedTalonFX and the DigitalInput for the magnetic limit switch.
 * 
 * When constructed, this class resets the configuration on the left and right motors, then applies configuration based
 * on the values in the Constants subclass.
 */
public class ElevatorIOTalonFX implements ElevatorIO {
    /**
     * Various hardware constants and conversion factors for the real robot's Elevator.
     */
    public static class Constants {
        // All Elevator CAN devices are connected to the Rio, not one of the CANivores.
        public static String CAN_BUS = "rio";

        // Pulled from the 2025 code base.
        public static int LEFT_MOTOR_ID = 28;
        public static int RIGHT_MOTOR_ID = 27;
        public static int BOTTOM_LIMIT_ID = 5;

        // Supply current limits for both motors.
        public static Current CURRENT_LIMIT = Amps.of(80);
        
        // Elevator PID constants, pulled from the 2025 code base.
        public static final double P = 1.6;
        public static final double I = 0.125;
        public static final double D = 0.0;
        public static final double F = 0.58;
        public static final double V = 0.0;
        public static final double A = 0.0;
        public static final double S = 0.0;

        // Conversion factors that will transform Motor Rotations into Elevator Position and Velocity. The second one
        // feels very clunky, and I have a thread out to try to improve (reduce) it's verbosity:
        // https://www.chiefdelphi.com/t/wpilib-units-part-2-unit-conversions-and-cancellations/506892
        private static final double _METERS_PER_ROTATION = 1.0;
        public static final Per<DistanceUnit, AngleUnit> METERS_PER_ROTATION = MetersPerRotation.ofNative(_METERS_PER_ROTATION);
        public static final Per<LinearVelocityUnit, AngularVelocityUnit> METERS_PER_SECOND_PER_ROTATIONS_PER_SECOND = MetersPerSecondPerRotationPerSecond.ofNative(_METERS_PER_ROTATION);
    }

    // The hardware attached to the Elevator subsystem.
    private final DigitalInput bottomLimitSwitch;
    private final LoggedTalonFX leftMotor;
    private final LoggedTalonFX rightMotor;

    /**
     * Constructing an instance of this will construct hardware instances, and configure the motors. It will also
     * set the left motor as a follower for the right to link both sides of the elevator together on the motor controllers.
     */
    public ElevatorIOTalonFX() {
        bottomLimitSwitch = new DigitalInput(Constants.BOTTOM_LIMIT_ID);

        leftMotor = new LoggedTalonFX(Constants.LEFT_MOTOR_ID, Constants.CAN_BUS);
        rightMotor = new LoggedTalonFX(Constants.RIGHT_MOTOR_ID, Constants.CAN_BUS);
    
        // Construct new configuration objects, which will have all config parameters default to factory settings.
        // Doing this ensures that both the left and right motor's config will be fully reset.
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        // Current and brake mode configuration
        leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.CURRENT_LIMIT.in(Amps);
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rightConfig.CurrentLimits.SupplyCurrentLimit = Constants.CURRENT_LIMIT.in(Amps);
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID configuration parameters
        rightConfig.Slot0.kP = Constants.P;
        rightConfig.Slot0.kI = Constants.I;
        rightConfig.Slot0.kD = Constants.D;
        rightConfig.Slot0.kG = Constants.F;
        rightConfig.Slot0.kV = Constants.V;
        rightConfig.Slot0.kA = Constants.A;
        rightConfig.Slot0.kS = Constants.S;

        // Attempt to apply both configurations. Sometimes the Phoenix library will return an error on applying on the
        // first attempt, so we try up to 5 times.
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig, 0.25));
        tryUntilOk(5, () -> rightMotor.getConfigurator().apply(rightConfig, 0.25));

        // Disable all status signals that we did not explicitly request. Note that most (currently all) status signals
        // are configured in the constructor of the left and right motors, instead of explicitly in this subsystem.
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        // Set the left motor as an inverted follower to the right motor.
        leftMotor.setControl(new Follower(Constants.RIGHT_MOTOR_ID, true));
    }

    /**
     * Update the inputs of this implementation. This will update the inputs for both of the TalonFXs (which have their
     * own inputs implementation), then derive the current height and velocity of the elevator as a function of the left
     * motor's position.
     * 
     * @param inputs The inputs object to populate with updated values.
     * @return Nothing, but the passed in inputs object will be modified in place.
     */
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        leftMotor.updateInputs(inputs.leftInputs);
        rightMotor.updateInputs(inputs.rightInputs);

        // Derive the height and velocity of the elevator as a function of the left motor's inputs. This is where the
        // conversion from rotations to meters happens. The type casts are safe, as Distance extends Measure<DistanceUnit>,
        // and LinearVelocity extends Measure<LinearVelocityUnit>, which are the return types of each timesDivisor respectively.
        inputs.height = (Distance) Constants.METERS_PER_ROTATION.timesDivisor(inputs.leftInputs.position);
        inputs.velocity = (LinearVelocity) Constants.METERS_PER_SECOND_PER_ROTATIONS_PER_SECOND.timesDivisor(inputs.leftInputs.velocity);

        inputs.bottomLimit = bottomLimitSwitch.get();
    }

    /**
     * The hardware implementation of changing the desired state of the elevator. Field oriented control will be
     * enabled even if the passed-in desiredState did not request it.
     * 
     * @param desiredState The desired state of the subsystem. This should be constructed with a Position and Velocity.
     * @return Nothing, but passes the desiredState off to the motor controller to follow.
     */
    @Override
    public void setDesiredState(PositionVoltage desiredState) {
        rightMotor.setControl(desiredState.withEnableFOC(true));
    }
}
