package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.PerUnit;

public class CustomUnits {
    public static final PerUnit<DistanceUnit, AngleUnit> MetersPerRotation = Meters.per(Rotation);
    public static final PerUnit<LinearVelocityUnit, AngularVelocityUnit> MetersPerSecondPerRotationPerSecond = MetersPerSecond.per(RotationsPerSecond);
}
