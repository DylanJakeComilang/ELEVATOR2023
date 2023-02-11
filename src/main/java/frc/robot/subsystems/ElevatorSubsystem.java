package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SingleChannelEncoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase {

    // objects

    private DigitalInput upperLimit = new DigitalInput(OperatorConstants.upperLimit);
    private DigitalInput lowerLimit = new DigitalInput(OperatorConstants.lowerLimit);
    private PIDController PID = new PIDController(0.007, 0, 0);
    // private WPI_TalonFX elevatorMotor = new WPI_TalonFX(OperatorConstants.motorID);
    // private TalonFXSensorCollection elevatorEncoder = new TalonFXSensorCollection(elevatorMotor);
    //private CANSparkMax elevatorMotor = new CANSparkMax(OperatorConstants.motorID, MotorType.kBrushless);
    private WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(OperatorConstants.motorID); 
    private SingleChannelEncoder SingleChannelEnc = new SingleChannelEncoder(elevatorMotor, lowerLimit);
    // private RelativeEncoder elevatorEncoder;
    
    private double previousErrorPos;


    private int setpoint;

    public ElevatorSubsystem() {
        PID.setTolerance(10); // creates range for set point | ie. positionTolerence = 0.1, setpoint = 1 ->
                                                // setpoint range = 0.9 , 1.1
        setpoint = getEncoder();
    }

    ////////////////////////////////////
    //            Dead Zone           //
    ////////////////////////////////////

    public double deadZone(double speed) {
        if (Math.abs(speed) < 0.1) {
            return 0;
        } else {
            return speed;
        }
    }

    ////////////////////////////////////
    //            Encoders            //
    ////////////////////////////////////

    public void resetEncoder() {
        // elevatorEncoder.setIntegratedSensorPosition(0, 0);
        // elevatorEncoder.setPosition(0);
        SingleChannelEnc.reset();
    }

    public int getEncoder() {
        // return elevatorEncoder.getIntegratedSensorPosition();
        // return elevatorEncoder.getPosition();
        return SingleChannelEnc.get();
    }

    public void setSetpoint(int newSetpoint){
         setpoint = newSetpoint;
    }
    
    ////////////////////////////////////
    //              PID               //
    ////////////////////////////////////

    public double calculate(double setpoint) {
        double calc = PID.calculate(getEncoder(), setpoint); // calculates error
        double limit = 0.2;
        if (PID.atSetpoint()) { // if at setpoint, motor stops
            return 0;
        }
        if (calc > limit) { // if error over limit encoder counts, set motor max "limit"
            return limit;
        } else if (calc < -limit) { // if error under -limit encoder counts, set motor max "-limit"
            return -limit;
        } else { // set motor to error
            return calc;
        }
    }

    public void controlI() { // continues to check for vaule changes | ie. 1 -> -1
        double currentErrorPos = PID.getPositionError();
        if (previousErrorPos > 0 && currentErrorPos < 0) { // if error was negative and now positive, reset I term
            PID.reset();
        } else if (previousErrorPos < 0 && currentErrorPos > 0) { // if error was positive and now negative, reset I
                                                                  // term
            PID.reset();
        }
        previousErrorPos = PID.getPositionError();
        SmartDashboard.getNumber("currentErrorPos", currentErrorPos);
        SmartDashboard.getNumber("previousErrorPos", previousErrorPos);
    }

    public void outputMotor(double setpoint) { // PID used for motors
        double calc = calculate(setpoint);
        SmartDashboard.putNumber("Error", calc);
        SmartDashboard.putNumber("Setpoint", setpoint);
        elevatorMotor.set(calc);
        controlI();
    }

    ////////////////////////////////////
    //             Limits             //
    ////////////////////////////////////

    public boolean upperLimitPressed() { // checks if upperLimit is pressed
        return upperLimit.get();
    }

    public boolean lowerLimitPressed() { // checks if lowerLimit is pressed
        return lowerLimit.get();
    }

    // RANGES

    public boolean inLowRange(double low, double range) { // uses parameters checks if position is low
        double lowMax = low + range;
        return (getEncoder() < lowMax);
    }

    public boolean inMidRange(double mid, double range) { // uses parameters checks if position is in the middle
        double midMax = mid + range;
        double midMin = mid - range;
        return (getEncoder() < midMax && getEncoder() > midMin);
    }

    public boolean inHighRange(double high, double range) { // checks if position is high
        double highMin = high - range;
        return (getEncoder() > highMin);
    }

    /////////////////////////////////////
    //           Set Motors            //
    /////////////////////////////////////

    public void setSpeed(double speed) {
        elevatorMotor.set(deadZone(speed));
    }

    public void setUp() {
        elevatorMotor.set(0.2);
    }

    public void setDown() {
        elevatorMotor.set(-0.2);
    }

    public void setStop() {
        elevatorMotor.stopMotor();
    }

    ///////////////////////////////
    //     TeleOp  |  Manual     //
    ///////////////////////////////

    public void stopSpeedPositive(double elevatorSpeed){ // prevents manual controls from going positive / up ↑
        if (elevatorSpeed > 0) {
            setStop();
        }
        else{
            setSpeed(elevatorSpeed);
        }
    }

    public void stopSpeedNegative(double elevatorSpeed){ // prevents manual controls from going negative / down ↓
        if (elevatorSpeed < 0) {
            setStop();
        }
        else{
            setSpeed(elevatorSpeed);
        }
    }


    public void manualElevator(double elevatorSpeed) { // set the elevator with the xbox joystick, when limits are
                                                        // pressed, they do not move to the coresponding direction
        if (upperLimitPressed()) {
            stopSpeedPositive(elevatorSpeed);
        } else if (lowerLimitPressed()) {
            stopSpeedNegative(elevatorSpeed);
        }
        else{
            setSpeed(elevatorSpeed);
        }
    }

    /////////////////////////////////////
    //  Set Position Through Encoders  //
    /////////////////////////////////////

    public void goLow(double low, double range) {
        if (!inLowRange(low, range)) {
            setDown();
        } else {
            setStop();
        }
    }

    public void goMid(double mid, double range) {
        if (!inMidRange(mid, range)) {
            if (getEncoder() < mid) {
                setUp();
            } else if (getEncoder() > mid) {
                setDown();
            }
        } else {
            setStop();
        }
    }

    public void goHigh(double high, double range) {
        if (!inHighRange(high, range)) {
            setUp();
        } else {
            setStop();
        }
    }

    // ROBOT SETUP

    public void initialize() {
        // elevatorMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void periodic() {
        if(lowerLimitPressed() || upperLimitPressed()){
            setpoint = getEncoder();
        }
        
        double calc = calculate(setpoint);
        SmartDashboard.putNumber("Error", calc);
        SmartDashboard.putNumber("Setpoint", setpoint);
        elevatorMotor.set(calc);
        controlI();

        SmartDashboard.putNumber("Encoder Count", getEncoder());
        SmartDashboard.putBoolean("Upper Limit Pressed", upperLimitPressed());
        SmartDashboard.putBoolean("Lower Limit", lowerLimitPressed());
    }
}