package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.Elevator.Constants.ElevatorPosition;

public class RobotContainer {
    public class Constants {
        public enum Mode {
            REAL,
            SIM
        }

        public static final int OI_BUTTONS_ID = 0;
        public static final int LEFT_JOYSTICK_ID = 1;
        public static final int RIGHT_JOYSTICK_ID = 2;

        public static final int GROUND_ID = 1;
        public static final int L1_ID = 2;
        public static final int L2_ID = 3;
        public static final int L3_ID = 4;
        public static final int L4_ID = 5;
    }
    public final Elevator elevator;
    private final Joystick oiButtons = new Joystick(Constants.OI_BUTTONS_ID);

    public RobotContainer(Constants.Mode mode) {
        switch(mode) {
            case REAL:
                elevator = new Elevator(new ElevatorIOTalonFX());
                break;
            case SIM:
                elevator = new Elevator(new ElevatorIOSim());
                break;
            default:
                elevator = new Elevator(new ElevatorIO() {});
        }

        new JoystickButton(oiButtons, Constants.GROUND_ID).onTrue(
            new MoveElevatorToSetpoint(elevator, ElevatorPosition.CORAL_GROUND)
        );
        new JoystickButton(oiButtons, Constants.L1_ID).onTrue(
            new MoveElevatorToSetpoint(elevator, ElevatorPosition.CORAL_L1)
        );
        new JoystickButton(oiButtons, Constants.L2_ID).onTrue(
            new MoveElevatorToSetpoint(elevator, ElevatorPosition.CORAL_L2)
        );
        new JoystickButton(oiButtons, Constants.L3_ID).onTrue(
            new MoveElevatorToSetpoint(elevator, ElevatorPosition.CORAL_L3)
        );
        new JoystickButton(oiButtons, Constants.L4_ID).onTrue(
            new MoveElevatorToSetpoint(elevator, ElevatorPosition.CORAL_L4)
        );
    }
}
