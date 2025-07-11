package frc.robot;

import java.util.List;

import frc.robot.Robot.AutonomousMove;

public class Constants {


    public static final double deadBand = 0.05;
    public static final double wheelCircumference = Math.PI * 6.0; // 6-inch wheel
    public static final List<AutonomousMove> autonomousMoves = List.of(new AutonomousMove(30,0), new AutonomousMove(10, 0),new AutonomousMove(0, -24.5));

    public static final double autoSpeed = 0.3; //autonomous speed
    public static final double speed = 0.60; // drive train speed
    public static final double lowSpeed = 0.4; //when low speed button is pressed
    public static final double intakeSpeed = 0.4; // intake speed
    
}
