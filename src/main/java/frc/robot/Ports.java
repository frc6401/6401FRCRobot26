package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Bus
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");
    
    

    // Talon FX IDs
    public static final int kIntakePivot = 34;
    public static final int kIntakeRollers = 32;
    public static final int kFloor = 60;
    public static final int kFeeder = 61;
    public static final int kShooterLeft = 31;
    public static final int kShooterMiddle = 30;
    public static final int kShooterRight = 29;
    //public static final int kHanger = 20;
    //public static final int kHanger2 = 21;
    
    // PWM Ports
    //public static final int kHoodLeftServo = 120;
    //public static final int kHoodRightServo = 130;
    
}
