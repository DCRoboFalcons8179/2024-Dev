// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ATATPose {

    public double distF;
    public double distB;
    public double angle;

    public ATATPose(double distF, double distB, double angle) {
        this.distF = distF;
        this.distB = distB;
        this.angle = angle;
    }

    public double getDistF() {
        return distF;
    }

    public double getDistB() {
        return distB;
    }

    public double getAngle() {
        return angle;
    }
}
