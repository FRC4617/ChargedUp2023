package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {

  private final UsbCamera camera1;
  private final UsbCamera camera2;
  private final VideoSink server;

  public Cameras() {
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();
    server.setSource(camera1);
  }

  public void swapCamera() {
    if (server.getSource() == camera1) {
      server.setSource(camera2);
    } else {
      server.setSource(camera1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
