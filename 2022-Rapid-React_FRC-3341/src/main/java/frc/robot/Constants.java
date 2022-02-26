// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;

 
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class I2CAddresses {
        public static final int MaxbotixUltrasonicSensor = 112;
    }

    public static final class JoystickPorts {
        public static final int JoystickPort1 = 0;
    }

    public static final class DriveTrainPorts {
        public static final int LeftDriveTalonPort = 1;
        public static final int RightDriveTalonPort = 2;
    }

    public static final class MotorPorts {
            public static final int port1 = 14; // Flywheel
            public static final int port2 = 3;
            public static final int port3 = 9; // Pivot
            public static final int port4 = 7;
            public static final int port5 = 10; // Roller
            public static final int port6 = 10;
         } 
    
    public static final class JoystickAxis {
        public static final int  XAxis = 0;
        public static final int YAxis = 1;
    } 

    public static final class USBOrder {
        public static final int Zero = 0;
        public static final int One = 1;
    }
    
    public static final class flywheelPIDConsts { // Tested on last year's chassis, not this year!
        public static double pidP = 0.07;
        public static double pidI = 0;
        public static double pidD = 0;
    }

    public static final class pivotPIDConsts { // Tested on last year's chassis, not this year!
        public static final double pidP = 0.014;
        public static final double pidI = 0.01;
        public static final double pidD = 0;
    }

    public static final class LeftflywheelFF { // Tested on last year's chassis, not this year!
        public static final double kS = 0.41733;
        public static final double kV = 0.4025;
        public static final double kA = 0.046839;
    }
    public static final class charConsts{
        public static final double ks = 0;
        public static final double kg = 0;
        public static final double kv = 0;
    }
}