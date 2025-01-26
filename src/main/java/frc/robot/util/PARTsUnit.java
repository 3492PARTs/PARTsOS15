// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import java.util.function.Function;

/** Add your docs here. */
public class PARTsUnit {

    public enum PARTsUnitType {
        Angle,
        Radian,
        Meter,
        Inch,
        Foot
    }

    private double value;
    private PARTsUnitType unitType;

    public PARTsUnit(double value, PARTsUnitType unitType) {
        this.value = value;
        this.unitType = unitType;
    }

    public double getValue() {
        return value;
    }

    public PARTsUnitType getUnitType() {
        return unitType;
    }

    public double to(PARTsUnitType unitType) {
        String message = "No to type for unit.";
        switch (this.unitType) {
            case Angle:
                if (unitType == PARTsUnitType.Radian)
                    return this.value * Math.PI / 180.0;
                    throw new RuntimeException(message);
            case Radian:
                if (unitType == PARTsUnitType.Radian)
                    return this.value *  180.0 / Math.PI;
                    throw new RuntimeException(message);
            case Meter:
                switch (unitType) {
                    case Inch:
                        return this.value * 39.3700787;
                    case Foot:
                        return this.value * 3.2808399;
                    default:
                        throw new RuntimeException(message);
                }
            case Inch:
                switch (unitType) {
                    case Meter:
                        return this.value / 39.3700787;
                    case Foot:
                        return this.value / 12;
                    default:
                        throw new RuntimeException(message);
                }
            case Foot:
                switch (unitType) {
                    case Meter:
                        return this.value / 3.2808399;
                    case Inch:
                        return this.value * 12;
                    default:
                        throw new RuntimeException(message);
                }
            default:
                throw new RuntimeException(message);
        }
    }

    //* DIRECT CONVERSIONS */
    public static Function<Double, Double> InchesToMeters = inches -> inches / 39.37;
    public static Function<Double, Double> MetersToInches = meters -> meters * 39.37;
    public static Function<Double, Double> DegreesToRadians = degrees -> degrees * (Math.PI/180);
}
