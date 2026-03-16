package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Bus
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    

    // Talon FX IDs
    public static final int kIntakePivot = 23;
    public static final int kIntakeRollers = 100;
    public static final int kFloor = 22;
    public static final int kFeeder = 101;
    public static final int kShooterLeft = 99;
    public static final int kShooterMiddle = 98;
    public static final int kShooterRight = 97;
    public static final int kHanger = 20;
    public static final int kHanger2 = 21;
    // PWM Ports
    //public static final int kHoodLeftServo = 20;
    //public static final int kHoodRightServo = 10;
    
}
