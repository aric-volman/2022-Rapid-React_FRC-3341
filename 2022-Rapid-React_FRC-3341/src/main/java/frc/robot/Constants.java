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
public final class Constants 
{
    public static final class DriveTrainPorts
    {
        public static final int LeftDriveTalonPort = 2; 
        public static final int RightDriveTalonPort = 3;
        public static final int LeftDriveVictorPort = 4;
        public static final int RightDriveVictorPort = 5;
    }

    public static final class ArmPorts{
        public static final int FrontLeftArmRot = 6;  //encoder
        public static final int FrontRightArmRot = 7; //encoder
        public static final int BackLeftArmRot = 8;  //encoder
        public static final int BackRightArmRot = 9; //encoder

        public static final int FrontLeftArmExt = 10;
        public static final int FrontRightArmExt = 11;
        public static final int BackLeftArmExt = 12;
        public static final int BackRightArmExt = 13;
    }

    public static final class ReflecSensorPorts{
        //THESE NEED TO BE CHECKED!!
        public static final int FrontLeftSens = 0;
        public static final int FrontRightSens = 1;
        public static final int BackLeftSens = 2;
        public static final int BackRightSens = 3;
    }

    public static final class BallHandlerPorts {
        public static final int leftFlywheelPort = 16;
        public static final int rightFlywheelPort = 17;
        public static final int pivotPort = 14;
        public static final int rollerPort = 15;
    }

    public static final class leftFlywheelPIDConsts {
        public static double pidP = 0.07;
        public static double pidI = 0;
        public static double pidD = 0;
    }

    public static final class rightFlywheelPIDConsts {
        public static double pidP = 0.07;
        public static double pidI = 0;
        public static double pidD = 0;
    }

    public static final class pivotPIDConsts {
        public static final double pidP = 0.002;
        public static final double pidI = 0;
        public static final double pidD = 0;
    }

    public static final class leftFlywheelFF { // Tested on last year's chassis, not this year!
        public static final double kS = 0.41733;
        public static final double kV = 0.4025;
        public static final double kA = 0.046839;
    }

    public static final class rightFlywheelFF { // Tested on last year's chassis, not this year!
        public static final double kS = 0.41733;
        public static final double kV = 0.4025;
        public static final double kA = 0.046839;
    }

    public static final class JoystickAxis 
    {
        public static final int YAxis = 1;
        public static final int XAxis = 0;
    }

    public static final class USBOrder {
        public static final int Zero = 0;
        public static final int One = 1;

    }

    public static final double angularOffset = 90.0;
}

