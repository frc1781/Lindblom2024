// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package tech.team1781.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class SparkMaxWrapper extends CANSparkMax {

    public SparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    public double getRelPosition() {
        return getEncoder().getPosition();
    }

    public void stop() {
        set(0);
    }

    public double getSpeed() {
        return getEncoder().getVelocity();
    }

    public void reverseSpeed() {
        set(get() * -1);
    }

}
