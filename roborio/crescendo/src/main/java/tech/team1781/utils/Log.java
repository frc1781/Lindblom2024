// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package tech.team1781.utils;

/** Add your docs here. */
public class Log {
    private static boolean loggingOn;
    public static void logValues(String key, double... values) {
      if (!loggingOn)
        return;

      System.out.print(key);
      for(double x : values) {
        System.out.printf(",%.2f", x);
      }
      System.out.println("");
    }

    public static void startLogging(boolean on) {
        loggingOn = on;
    }
}
