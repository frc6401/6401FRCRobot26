package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class Camera extends SubsystemBase {
    public Camera()
    {
        UsbCamera camera = CameraServer.startAutomaticCapture(0);
    }



}
