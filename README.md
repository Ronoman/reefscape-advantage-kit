# Bringup plan

## Elevator
1. Comment out ElevatorIOTalonFX setGoal implementation so that motor isn't actually commanded to do anything
1. Comment out the resetOnInit behavior in Robot.java so that the Elevator isn't commanded to do anything on init
1. Deploy
1. Command the robot to a position like L2. Observe AdvantageScope and demonstrate that the trapezoid is commanding it to the correct desired position (will display as Meters, but will be Rotations due to 1.0 conversion factor)
1. Verify motor configurations were properly applied through PhoenixTuner
1. Uncomment ElevatorIOTalonFX implementation. Verify that the small max velocity and acceleration constants are in use
1. Rerun the above test with motor output enabled. Verify behavior visually and through AdvantageScope
1. If confident, increase parameters to competition values
1. Test out reset behavior
1. Measure the Rotations to Meters ratio. Update the parameter in ElevatorIOTalonFX.
1. Verify that the logged height of the elevator is valid as the Elevator moves through various positions.