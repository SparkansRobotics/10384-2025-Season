package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Subsystems.Drivetrain.AutonomousMove;

public class Elevator {

    enum ElevatorMode {
        JOYSTICK,
        SETPOINT
    }


    private ElevatorMode elevatorMode = ElevatorMode.JOYSTICK;
  private final SparkMax m_elevatorMotor = new SparkMax(5, MotorType.kBrushless);
  private final SparkMaxConfig m_motorConfig = new SparkMaxConfig();
  private final RelativeEncoder m_relativeEncoder = m_elevatorMotor.getAlternateEncoder();
  private final SparkClosedLoopController m_closedLoopController = m_elevatorMotor.getClosedLoopController();

  private static final double SPROCKET_DIAMETER = 1.868; // inches
  private static final double SPROCKET_CIRCUMFERENCE = Math.PI * SPROCKET_DIAMETER;
  private static final double POSITION_CONVERSION_FACTOR = SPROCKET_CIRCUMFERENCE;
  private final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR; // Same factor applies for velocity
  private double kp = 0.1; // Proportional gain
  private double ki = 0; // Integral gain
  private double kd = 0; // Derivative gain
 
  //private boolean motorOn = false;
 
 
  // the top limit switch should be wired to port 0 on the roborio
  private DigitalInput toplimitSwitch = new DigitalInput(0);
  // bottom limit switch wired to port
  private DigitalInput bottomlimitSwitch = new DigitalInput(1);


    public void robotInit() {


        // pid configuration
    m_motorConfig.alternateEncoder
        .positionConversionFactor(POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
 
    m_motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .p(kp)
        .i(ki)
        .d(kd);
    m_elevatorMotor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        

        System.out.println("Relative Encoder Counts: " + m_relativeEncoder.getPosition());

    // button pressed values (true/false)
    var isLevelOneButtonPressed = OperatorControl.getRawButton(4);
    var isProcessorButtonPressed = OperatorControl.getRawButton(7);
    var isBottomButtonPressed = OperatorControl.getRawButton(6);
    var isEndgameButtonPressed = OperatorControl.getRawButton(8);
 
    var setPointButtons = List.of(isLevelOneButtonPressed, isProcessorButtonPressed, isBottomButtonPressed, isEndgameButtonPressed);
    // check if buttons are pressed
    var isSetPointButtonPressed = setPointButtons.stream().anyMatch(isButtonPressed -> isButtonPressed);
    // number of buttons pressed
    var numberOfSetPointButtonsPressed = setPointButtons.stream().filter(isButtonPressed -> isButtonPressed).count();
 
    double ElevatorJoystickValue = getManualElevator();
    var isJoystickPressed = ElevatorJoystickValue != 0;
 
    //go into joystick or button mode depending on which one is used
    if (isJoystickPressed) {
      elevatorMode = ElevatorMode.JOYSTICK;
    } else if (isSetPointButtonPressed) {
      elevatorMode = ElevatorMode.SETPOINT;
    }
 
    // if joystick is being used for elevator
    if (ElevatorMode.JOYSTICK.equals(elevatorMode)) {
      // limit switch
      if (isElevatorAtTop() && ElevatorJoystickValue > 0 || isElevatorAtBottom() && ElevatorJoystickValue < 0) {
        m_elevatorMotor.stopMotor();
      } else {
        // set elevator to joystick values
        m_elevatorMotor.set(ElevatorJoystickValue);
        System.out.println(ElevatorJoystickValue);
      }
    }
 
    //if buttons are being used for elevator
    if (ElevatorMode.SETPOINT.equals(elevatorMode)) {
      if (isElevatorAtTop() || isElevatorAtBottom()) {
        System.out.println("working");
        // limit switch code
        m_elevatorMotor.stopMotor();
      } else {
        // if more than one elevator button is pressed, motor will stop
        if (numberOfSetPointButtonsPressed > 1) {
          m_elevatorMotor.stopMotor();
        } else if (isBottomButtonPressed) {
          //ground
          m_closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else if (isProcessorButtonPressed) {
          //processor
          m_closedLoopController.setReference(13.2, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else if (isLevelOneButtonPressed) {
          //L1
          m_closedLoopController.setReference(36.6, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else if (isEndgameButtonPressed) {
          //endgame
          m_closedLoopController.setReference(50.4, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
      }
    }
    }


    private boolean isElevatorAtTop() {
    return toplimitSwitch.get();
  }
 
  private boolean isElevatorAtBottom() {
    return bottomlimitSwitch.get();
  }
 
  private double getManualElevator() {
    // elevator values
    return applyDeadband(-operatorControl.getRawAxis(1));
 
  }
  private double getLeftEncoderTravelDistance (AutonomousMove move){
    if (move.turnAngle == 0) {
      return move.displacement;
    }
    else {
      return (21.5/2 * -Math.toRadians(move.turnAngle)); //21.5 = distance between wheels, needs to be inversed.
    }
  }
    
}
