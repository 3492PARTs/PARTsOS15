// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.PARTs.Classes;

import java.util.function.Function;

/** PARTsUnit - 
 * A unit container to store and convert values.
 */
public class PARTsUnit {

    public enum PARTsUnitType {
        Angle, // Degrees. todo: change to degree?
        Radian,
        Meter,
        Inch,
        Foot,
        Percent,
        Rotations,
        MetersPerSecond
    }

    private double value;
    private PARTsUnitType unitType;

    /**
     * Create a new PARTsUnit.
     * @param value The initial value of the unit.
     * @param unitType The type that the initial value is currently in.
     */
    public PARTsUnit(double value, PARTsUnitType unitType) {
        this.value = value;
        this.unitType = unitType;
    }

    /**
     * Gets the raw value as a double.
     * @return The value as a double.
     */
    public double getValue() {
        return value;
    }

    /**
     * Gets the absolute value as a double.
     * @return The value as a double.
     */
    public double getMagnitude() {
        return Math.abs(value);
    }

    /**
     * Get the current unit type for an instance of {@link frc.robot.util.PARTs.Classes.PARTsUnit PARTsUnit}.
     * @return The unit type as a {@link frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType PARTsUnitType}.
     */
    public PARTsUnitType getUnitType() {
        return unitType;
    }

    /**
     * Converts current unit into the requested unit.
     * @param unitType The target unit.
     * @return Converted unit in double.
     */
    public double to(PARTsUnitType unitType) {
        String message = "No to type for unit.";
        switch (this.unitType) {
            case Angle:
                if (unitType == PARTsUnitType.Radian)
                    return this.value * Math.PI / 180.0;
                else if (unitType == this.unitType)
                    return this.value;
                throw new RuntimeException(message);
            case Radian:
                if (unitType == PARTsUnitType.Angle)
                    return this.value *  180.0 / Math.PI;
                else if (unitType == this.unitType)
                    return this.value;
                throw new RuntimeException(message);
            case Meter:
                switch (unitType) {
                    case Inch:
                        return this.value * 39.3700787;
                    case Foot:
                        return this.value * 3.2808399;
                    case Meter:
                        return this.value;
                    default:
                        throw new RuntimeException(message);
                }
            case Inch:
                switch (unitType) {
                    case Meter:
                        return this.value / 39.3700787;
                    case Foot:
                        return this.value / 12;
                    case Inch:
                        return this.value;
                    default:
                        throw new RuntimeException(message);
                }
            case Foot:
                switch (unitType) {
                    case Meter:
                        return this.value / 3.2808399;
                    case Inch:
                        return this.value * 12;
                    case Foot:
                        return this.value;
                    default:
                        throw new RuntimeException(message);
                }
            case Rotations:
                switch (unitType) {
                    case Angle:
                        return this.value * 360;
                    default:
                        throw new RuntimeException(message);
                }
            case Percent:
                throw new RuntimeException(message + " Percent cannot not be translated to any other type.");
            default:
                throw new RuntimeException(message);
        }
    }

    /**
     * Converts current unit into the requested unit.
     * @param unitType The target {@link frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType PARTsUnitType}.
     * @return The converted {@link frc.robot.util.PARTs.Classes.PARTsUnit PARTsUnit}.
     */
    public PARTsUnit as(PARTsUnitType unitType) {
        return new PARTsUnit(to(unitType), unitType);
    }

    //* DIRECT STATIC CONVERSIONS */
    public static Function<Double, Double> InchesToMeters = inches -> inches / 39.37;
    public static Function<Double, Double> MetersToInches = meters -> meters * 39.37;
    public static Function<Double, Double> DegreesToRadians = degrees -> degrees * (Math.PI/180);
}
