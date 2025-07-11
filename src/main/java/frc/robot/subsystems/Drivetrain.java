package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {


    // Main motors
    private SparkMax frontLeftMotor;
    private SparkMax backLeftMotor;
    private SparkMax frontRightMotor;
    private SparkMax backRightMotor;

    // Config
    private SparkMaxConfig leftConfig;
    private SparkMaxConfig rightConfig;

    // Encoders
    private RelativeEncoder m_rightEncoder;
    private RelativeEncoder m_leftEncoder;


    record AutonomousMove(double displacement, double turnAngle) {};
    private int moveIndex;
    private double leftTarget;



    public void robotInit() {

        // Drive motors
        frontLeftMotor = new SparkMax(1, MotorType.kBrushed);
        backLeftMotor = new SparkMax(2, MotorType.kBrushed);
        frontRightMotor = new SparkMax(3, MotorType.kBrushed);
        backRightMotor = new SparkMax(4, MotorType.kBrushed);

        // Motor config
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        // Encoders
        m_leftEncoder = frontLeftMotor.getEncoder();
        m_rightEncoder = backLeftMotor.getEncoder();


        leftConfig.encoder.countsPerRevolution(8192);
        leftConfig.encoder.positionConversionFactor(Constants.wheelCircumference);
        frontLeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightConfig.encoder.countsPerRevolution(8192);
        rightConfig.encoder.positionConversionFactor(Constants.wheelCircumference);
        frontRightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void robotPeriodic() {

    }

    public void autonomousInit() {

        //reseting index to zero
        //setting initial target
        moveIndex = 0;
        leftTarget = m_leftEncoder.getPosition() + getLeftEncoderTravelDistance(autonomousMoves.get(moveIndex)) ;

    }

    public void autonomousPeriodic() {

        if (moveIndex >= autonomousMoves.size()) {
            setDrivetrainMotors(0, 0);
            return;
        }
        var move = autonomousMoves.get(moveIndex);
         
        double left = 0;
        double right = 0;
        //interpret displacment and turn values for motor direction
        if (move.turnAngle > 0){
            left = -1;
            right = 1;
        } else if (move.turnAngle < 0){
            left = 1;
            right = -1;
        } else if (move.displacement > 0){
            left = 1;
            right = 1;
        } else if (move.displacement < 0){
            left = -1;
            right = -1;
        }
       
       
        final var leftCurrentPosition = m_leftEncoder.getPosition();
         
        //backwards and left
        if (left<0) {
            if (leftCurrentPosition > leftTarget){
                setDrivetrainMotors(left * autoSpeed, right * autoSpeed);
            } else {
                moveIndex++;
                if (moveIndex<autonomousMoves.size()){
                    leftTarget = leftCurrentPosition + getLeftEncoderTravelDistance(autonomousMoves.get(moveIndex)) ;  
                }
            }
        }

        //forwards and right
        if (left > 0) {
            if (leftCurrentPosition < leftTarget){
                setDrivetrainMotors(left * autoSpeed, right * autoSpeed);
            } else {
                moveIndex++;
                if (moveIndex<autonomousMoves.size()){
                    leftTarget = leftCurrentPosition + getLeftEncoderTravelDistance(autonomousMoves.get(moveIndex)) ;  
                }
            }
        }
         
        System.out.println(m_leftEncoder.getPosition());
    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        //joystick deadband for driver controller
        double forwardJoystick = applyDeadband(driveControl.getRawAxis(1) * -1);        // 1
        double turnJoystick = applyDeadband(driveControl.getRawAxis(2));                // 0
 
        double leftWheelsRaw = forwardJoystick + turnJoystick * 0.7;                         // 1
        double rightWheelsRaw = forwardJoystick - turnJoystick * 0.7;                        // 1
    
        //button X: spin out
        boolean spinForwardButton = OperatorControl.getRawButton(3);
        //button B: spin in
        boolean spinBackwardButton = OperatorControl.getRawButton(1);
    
    
        double leftWheels = 0;
        double rightWheels = 0;
    
        // This is not required but good practice
        // Normalize the motor values
        if (leftWheelsRaw > 1) {
            leftWheels = 1;
        } else if (leftWheelsRaw < -1) {
            leftWheels = -1;
        } else {
            leftWheels = leftWheelsRaw;
        }
    
        if (rightWheelsRaw > 1) {
            rightWheels = 1;
        } else if (rightWheelsRaw < -1) {
            rightWheels = -1;
        } else {
            rightWheels = rightWheelsRaw;
        }
    
        if (Robot.driveControl.getRawButton(5)){
            setDrivetrainMotors(leftWheels * lowSpeed, rightWheels * lowSpeed);
        } else {
            setDrivetrainMotors(leftWheels * speed, rightWheels * speed);
        }
    }
    

    private void setDrivetrainMotors(double left, double right) {
        frontLeftMotor.set(-left);
        backLeftMotor.set(left);
        frontRightMotor.set(-right);
        backRightMotor.set(right);
    }


    public double getLeftStartPosition() {
        return m_leftEncoder.getPosition(); // Returns position in inches
    }
 
    public double getRightPosition() {
        return m_rightEncoder.getPosition(); // Same for the right side
    }


}
