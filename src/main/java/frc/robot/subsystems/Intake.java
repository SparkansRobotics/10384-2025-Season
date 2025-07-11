package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {


    private SparkFlex intakeMotor;



    public void robotInit() {

        intakeMotor = new SparkFlex(6, MotorType.kBrushless);
    }

    public void robotPeriodic() {

    }

    public void autonomousInit() {

    }

    public void autonomousPeriodic() {

    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        
        //button X: spin out
        boolean spinForwardButton = Robot.operatorControl.getRawButton(3);
        //button B: spin in
        boolean spinBackwardButton = operatorControl.getRawButton(1);


        if (spinForwardButton) {
            // Spin the motor out
            intakeMotor.set(intakeSpeed);
            System.out.println("Forward button pressed. Motor speed: " + intakeSpeed);
        } else if (spinBackwardButton) {
            // Spin the motor in
            intakeMotor.set(-intakeSpeed);
            System.out.println("Backward button pressed. Motor speed: " + -intakeSpeed);
        } else {
            // Stop the motor if no button is pressed
            intakeMotor.set(0);
        }
    }
    
}
