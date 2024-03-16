// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ATATPose {

    public double distDriver;
    public double distFollower;
    public double atatAngle;

    /**
     * Stores poses used for RequestATATPose for readability.
     * @param distDriver
     * @param distFollower
     * @param atatAngle
     */
    public ATATPose(double distDriver, double distFollower, double atatAngle) {
        this.distDriver = distDriver;
        this.distFollower = distFollower;
        this.atatAngle = atatAngle;
    }

    public double getDistF() {
        return distDriver;
    }

    public double getDistB() {
        return distFollower;
    }

    public double getAngle() {
        return atatAngle;
    }
}
