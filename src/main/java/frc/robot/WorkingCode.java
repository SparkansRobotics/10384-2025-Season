package frc.robot;
 
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
 
import java.util.List;
 
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

 
public class WorkingCode extends TimedRobot {
  enum ElevatorMode {
    JOYSTICK,
    SETPOINT
  }
  record AutonomousMove(double displacement, double turnAngle){};
 
  private Joystick driveControl;
  private Joystick OperatorControl;
 
  private SparkMax frontLeftMotor; // main motors
  private SparkMax backLeftMotor;
  private SparkMax frontRightMotor;
  private SparkMax backRightMotor;
  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  // private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private double deadBand = 0.05;
 
 
  UsbCamera frontCamera = CameraServer.startAutomaticCapture(0);
  UsbCamera backCamera = CameraServer.startAutomaticCapture(1);
 
 
  PixelFormat pixelFormat = PixelFormat.kGray;
 
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
 
  private SparkFlex intakeMotor;
 
  //autonomous list
  private List<AutonomousMove> autonomousMoves = List.of(new AutonomousMove(30,0), new AutonomousMove(10, 0),new AutonomousMove(0, -24.5));
  private int moveIndex;
  private double leftTarget;
  private RelativeEncoder m_rightEncoder;
  private RelativeEncoder m_leftEncoder;
 
  private double autoSpeed = 0.3; //autonomous speed
  private double speed = 0.60; // drive train speed
  private double lowSpeed = 0.4; //when low speed button is pressed
  private double intakeSpeed = 0.4; // intake speed
  //private boolean motorOn = false;
 
 
  // the top limit switch should be wired to port 0 on the roborio
  private DigitalInput toplimitSwitch = new DigitalInput(0);
  // bottom limit switch wired to port
  private DigitalInput bottomlimitSwitch = new DigitalInput(1);
 
  @Override
  public void robotInit() {
    // drivetrain motors
    frontLeftMotor = new SparkMax(1, MotorType.kBrushed);
    backLeftMotor = new SparkMax(2, MotorType.kBrushed);
    frontRightMotor = new SparkMax(3, MotorType.kBrushed);
    backRightMotor = new SparkMax(4, MotorType.kBrushed);
    //testing conversion factor
 
    //Auto encoders
    m_leftEncoder = frontLeftMotor.getEncoder();
    m_rightEncoder = backLeftMotor.getEncoder();
    // controllers
    driveControl = new Joystick(2);
    OperatorControl = new Joystick(3);
    //intake motor
    intakeMotor = new SparkFlex(6, MotorType.kBrushless);
 
    double wheelCircumference = Math.PI * 6.0; // 6-inch wheel
    leftConfig.encoder.countsPerRevolution(8192);
    leftConfig.encoder.positionConversionFactor(wheelCircumference);
    frontLeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
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
    //reseting index to zero
    //setting initial target
    moveIndex = 0;
    leftTarget = m_leftEncoder.getPosition() + getLeftEncoderTravelDistance(autonomousMoves.get(moveIndex)) ;
   
  }
 
  @Override
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
      if (leftCurrentPosition>leftTarget){
        setDrivetrainMotors(left*autoSpeed, right*autoSpeed);
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
 
  @Override
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
   
 
    //setDrivetrainMotors(0.3, 0.3);
   
 
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
 
    if (driveControl.getRawButton(5)){
      setDrivetrainMotors(leftWheels * lowSpeed, rightWheels * lowSpeed);
    } else {
      setDrivetrainMotors(leftWheels * speed, rightWheels * speed);
    }

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
 
  // System.out.println(m_rightEncoder.getPosition());}
 
  // System.out.println("Left Position (inches): " + leftCurrentPosition + ", Left
  // Encoder Counts: " + m_leftEncoder.getPosition());
  // System.out.println("Right Position (inches): " + rightPosition + ", Right
  // Encoder Counts: " + m_rightEncoder.getPosition());
  // System.out.println("startingposition (inches):" + leftStartPosition);
 
  public double getLeftStartPosition() {
    return m_leftEncoder.getPosition(); // Returns position in inches
  }
 
  public double getRightPosition() {
    return m_rightEncoder.getPosition(); // Same for the right side
  }
 
  private double applyDeadband(double controllerValue) {
    if (Math.abs(controllerValue) < deadBand) {
      return 0;
    }
 
    return Math.copySign((Math.abs(controllerValue) - deadBand) / (1 - deadBand), controllerValue); // 0.07 - 0.05 = 0.02 / 0.95 = 1.05
  }
 
  private void setDrivetrainMotors(double left, double right) {
    frontLeftMotor.set(-left);
    backLeftMotor.set(left);
    frontRightMotor.set(-right);
    backRightMotor.set(right);
  }
 
  private boolean isElevatorAtTop() {
    return toplimitSwitch.get();
  }
 
  private boolean isElevatorAtBottom() {
    return bottomlimitSwitch.get();
  }
 
  private double getManualElevator() {
    // elevator values
    return applyDeadband(-OperatorControl.getRawAxis(1));
 
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