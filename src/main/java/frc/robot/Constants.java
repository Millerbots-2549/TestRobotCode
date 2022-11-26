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

    public static final class DriveConstants {
        // Drive motor controller ports
        public static final int kLeftMotor1Port = 13;
        public static final int kLeftMotor2Port = 12;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 2;
    }

    public static final class IOConstants {
        // XBox controller ports
        public static final int kDriverControllerPort = 0;
        public static final int kManipulatorControllerPort = 0;
    }

    public static final class ArmConstants {
        public static final int[] kArmSolenoidPorts = new int[] {1, 0};
    }

    public static final class ShootConstants {
        // Shooter motor controller and solenoid ports
        public static final int kTopShootMotorPort = 0;
        public static final int kBottomShootMotorPort = 1;
        public static final int[] kShootSolenoid1Ports = new int[] {4, 5};
        public static final int[] kShootSolenoid2Ports = new int[] {7, 6};

        // PID constants
        public static final double kShootMotor_kP = 0.09;
        public static final double kShootMotor_kF = 0.04589027;
        public static final double kShootMotor_kD = 4.2;
        public static final double kShootMotor_kI = 0;

        //Shoot motor speeds, tolerances
        public static final double kShootLowSpeed = 7500;
        public static final double kShootHighSpeed = 15400;
        public static final double kShootMotorTolerance = 250;
    }
}
