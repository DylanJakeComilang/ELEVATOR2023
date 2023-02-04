package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
   
   // objects
   
    private DigitalInput upperLimit = new DigitalInput(OperatorConstants.upperLimit);
    private DigitalInput lowerLimit = new DigitalInput(OperatorConstants.lowerLimit);
    private double kp;
    private double ki;
    private double kd;
    private PIDController PID = new PIDController(0.000000000000001, ki, kd);
    private WPI_TalonFX elevatorMotor = new WPI_TalonFX(OperatorConstants.motorID);
    private TalonFXSensorCollection elevatorEncoder = new TalonFXSensorCollection(elevatorMotor);
    private double previousErrorPos;

    public ElevatorSubsystem(){
        PID.setTolerance(10); // creates range for set point | ie. positionTolerence = 1, setpoint = 1 -> setpoint range = 0.9 , 1.1
    }

    ////////////////////////////////////
    //            DEADZONE            //
    ////////////////////////////////////

    public double deadZone(double speed){
        if (speed < Math.abs(0.1)){
            return 0;
        } else {
            return speed;
        }
    }
    
    

    ////////////////////////////////////
    //            Encoders            //
    ////////////////////////////////////

    public void resetEncoder() {
        elevatorEncoder.setIntegratedSensorPosition(0, 0);
    }

    public double getEncoder() {
        return elevatorEncoder.getIntegratedSensorPosition();
    }

    ////////////////////////////////////
    //              PID               //
    ////////////////////////////////////

    public double calculate(double setpoint) {
        double calc = PID.calculate(getEncoder(), setpoint); // calculates error
        if (PID.atSetpoint()) { // if at setpoint, motor stops
            return 0;
        }
        if (calc > 0.1) { // if error over 1 encoder, set motor max "1"
            return 0.1;
        } else if (calc < -0.1) { // if error under -1 encoder, set motor max "-1"
            return -0.1;
        } else { // set motor to error
            return calc;
        }
    }

    public void controlI() { // continues to check for vaule changes | ie. 1 -> -1
        double currentErrorPos = PID.getPositionError();
        if (previousErrorPos > 1 && currentErrorPos < 1) { // if error was negative and now positive, reset I term
            PID.reset();
        } else if (previousErrorPos < 1 && currentErrorPos > 1) { // if error was positive and now negative, reset I term
            PID.reset();
        }
        previousErrorPos = PID.getPositionError();
        SmartDashboard.getNumber("currentErrorPos", currentErrorPos);
        SmartDashboard.getNumber("previousErrorPos", previousErrorPos);
    }

    public void outputMotor(double setpoint){ // PID used for motors
        double calc = calculate(setpoint);
        SmartDashboard.putNumber("Error", calc);
        SmartDashboard.putNumber("Setpoint", setpoint);
        elevatorMotor.set(calc);
        controlI();
    }

    ////////////////////////////////////
    //             Limits             //
    ////////////////////////////////////

    public boolean getUpperLimit() {
        return upperLimit.get();
    }

    public boolean getLowerLimit() {
        return lowerLimit.get();
    }

    ////////////////////////////////////
    //           Set Motors           //
    ////////////////////////////////////

    public void setSpeed(double speed){
        elevatorMotor.set(deadZone(speed));
    }

    public void setUp(double speed) {
        elevatorMotor.set(speed);
    }

    public void setDown(double speed) {
        elevatorMotor.set(-speed);
    }

    public void setStop() {
        elevatorMotor.set(0);
    }

    public void setUpperLimit() {
        if (getUpperLimit()) {
            setStop();
        } else {
            setUp(0);
        }
    }

    public void setLowerLimit() {
        if (getLowerLimit()) {
            setStop();
            resetEncoder();
        }
    }

    public void initialize(){
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void periodic(){
        kp = SmartDashboard.getNumber("kp", 0);
        SmartDashboard.putNumber("kp", kp);
        kp = SmartDashboard.getNumber("ki", 0);
        SmartDashboard.putNumber("ki", ki);
        kp = SmartDashboard.getNumber("kd", 0);
        SmartDashboard.putNumber("kd", kd);
        SmartDashboard.putNumber("Encoder Count", getEncoder());
    }
}
