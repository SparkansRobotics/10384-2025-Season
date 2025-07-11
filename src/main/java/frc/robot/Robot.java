package frc.robot;
 
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

 
public class Robot extends TimedRobot {


    public static Drivetrain drivetrain = new Drivetrain();
    public static Intake intake = new Intake();
    public static Elevator elevator = new Elevator();

    public static Joystick driveControl = new Joystick(2);
    public static Joystick operatorControl = new Joystick(3);
 
 
    UsbCamera frontCamera = CameraServer.startAutomaticCapture(0);
    UsbCamera backCamera = CameraServer.startAutomaticCapture(1);
 
 
 
    @Override
    public void robotInit() {

        Robot.drivetrain.robotInit();
        Robot.intake.robotInit();
        Robot.elevator.robotInit();


    }
 
    @Override
    public void robotPeriodic() {
        // Intentionally empty
    }
 
    @Override
    public void disabledPeriodic() {
        // Intentionally empty
    }
 
    @Override
    public void autonomousInit() {
   
    }
 
    @Override
    public void autonomousPeriodic() {
    
    }
 
    @Override
    public void teleopPeriodic() {

    }


}