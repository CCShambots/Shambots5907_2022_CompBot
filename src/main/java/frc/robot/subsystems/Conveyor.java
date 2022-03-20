package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.hardware.ColorSensor;
import frc.robot.util.hardware.ProximitySensor;
import frc.robot.util.intake.Ball;
import frc.robot.util.intake.BallTracker;
import frc.robot.util.intake.Ball.BallPosition;
import frc.robot.util.priorityFramework.PrioritizedSubsystem;

import static frc.robot.Constants.Conveyor.*;

import java.util.ArrayList;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Conveyor extends PrioritizedSubsystem{
    private WPI_TalonFX conveyorStage1 = new WPI_TalonFX(CONVEYOR_STAGE1_ID);
    private WPI_TalonFX conveyorStage2 = new WPI_TalonFX(CONVEYOR_STAGE2_ID);

    private ProximitySensor proxStage1 = new ProximitySensor(PROX_STAGE1_PORT);
    private ProximitySensor proxStage2 = new ProximitySensor(PROX_STAGE2_PORT);
    private ProximitySensor proxStage3 = new ProximitySensor(PROX_STAGE3_PORT);

    private ColorSensor colorSensor = new ColorSensor(COLOR_SENSOR_PORT);

    private BallTracker tracker = new BallTracker(proxStage1, proxStage2, proxStage3, colorSensor, this);
    
    boolean running = false;
    Direction direction = Direction.Stopped;

    private boolean ejecting = false;
    
    public Conveyor() {
        setupMotor(conveyorStage1);
        setupMotor(conveyorStage2);
    }

    private void setupMotor(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configSupplyCurrentLimit(CURRENT_LIMIT);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
    }

    public void intakeAll() {setAll(DEFAULT_CONVEYOR_SPEED);}
    public void exhaustAll() {setAll(-DEFAULT_CONVEYOR_SPEED);}
    public void stopAll() {setAll(0);}
    public void intakeStage1() {setStage1(DEFAULT_CONVEYOR_SPEED);}
    public void exhaustStage1() {setStage1(-DEFAULT_CONVEYOR_SPEED);}
    public void stopStage1() {setStage1(0);}
    public void intakeStage2() {setStage2(DEFAULT_CONVEYOR_SPEED);}
    public void exhaustStage2() {setStage2(-DEFAULT_CONVEYOR_SPEED);}
    public void stopStage2() {setStage2(0);}

    private void setAll(double power) {
        setStage1(power);
        setStage2(power);
    }

    private void setStage1(double power) {conveyorStage1.set(ControlMode.PercentOutput, power);}
    private void setStage2(double power) {conveyorStage2.set(ControlMode.PercentOutput, power);}


    //Interactions with the BallTracker util
    public int getNumberOfBalls() {return tracker.getNumberOfBalls();} 
    public BallPosition getBall1Pos() {return tracker.getBall1Pos();} 
    public BallPosition getBall2Pos() {return tracker.getBall2Pos();} 

    public boolean isRunning() {return running;}
    public Direction getDirection() {return direction;}

    /**
     * Sets the state of the tracker to being completely empty (to be used after ejecting balls or shooting)
     */
    public void clearTracker() {
        tracker.setCurrentState(new ArrayList<Ball>());
        tracker.setPrevSensorStates(false, false);
    }

    /**
     * Set there to be one ball in the tracker 
     */
    public void setupTrackerAuto() {
        tracker.resetForAuto();
        tracker.setPrevSensorStates(false, false);
    }

    public void setTrackerDisabled(boolean value) {
        tracker.setDisabled(value);
    }

    public void setEjecting(boolean value) {ejecting = value;}
    public boolean isEjecting() {return ejecting;}

    public boolean isTrackerError() {return tracker.getError();}

    @Override
    public void periodic() {
        tracker.periodic();

        running = !(conveyorStage1.getMotorOutputPercent() == 0 && conveyorStage1.getMotorOutputPercent() == 0);
        
        if(!running) direction = Direction.Stopped;
        else if(conveyorStage1.getMotorOutputPercent() > 0 || conveyorStage2.getMotorOutputPercent() > 0) direction = Direction.Intake;
        else if(conveyorStage1.getMotorOutputPercent() < 0 || conveyorStage2.getMotorOutputPercent() < 0) direction = Direction.Exhaust;

        SmartDashboard.putData("Ball tracker", tracker);
        SmartDashboard.putNumber("Stage 1 speed", conveyorStage1.getMotorOutputPercent());
        SmartDashboard.putNumber("Stage 2 speed", conveyorStage2.getMotorOutputPercent());
        //TOOD: Implement lights for tracker error
        SmartDashboard.putBoolean("Tracker status", !tracker.getError());
    }

    public static enum Direction {
        Intake, Exhaust, Stopped
    }

}
