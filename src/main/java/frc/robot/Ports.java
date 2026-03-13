package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Bus
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    

    // Talon FX IDs
    public static final int kIntakePivot = 13;
    public static final int kIntakeRollers = 15;
    public static final int kFloor = 14;
    public static final int kFeeder = 16;
    public static final int kShooterLeft = 17;
    public static final int kShooterMiddle = 18;
    public static final int kShooterRight = 19;
    public static final int kHanger = 20;

    // PWM Ports
    //public static final int kHoodLeftServo = 20;
    //public static final int kHoodRightServo = 10;
    
}
