package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.hardware.ProximitySensor;
import frc.robot.util.intake.BallTracker;
import frc.robot.util.intake.Ball.BallPosition;

import static frc.robot.Constants.Conveyor.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Conveyor extends SubsystemBase{
    private WPI_TalonFX conveyorStage1 = new WPI_TalonFX(CONVEYOR_STAGE1_ID);
    private WPI_TalonFX conveyorStage2 = new WPI_TalonFX(CONVEYOR_STAGE2_ID);

    private ProximitySensor proxStage1 = new ProximitySensor(PROX_STAGE1_ID);
    private ProximitySensor proxStage2 = new ProximitySensor(PROX_STAGE2_ID);

    private BallTracker tracker = new BallTracker(proxStage1, proxStage2, this);
    
    boolean running = false;
    
    public Conveyor() {
        setupMotor(conveyorStage1);
        setupMotor(conveyorStage2);
    }

    private void setupMotor(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configSupplyCurrentLimit(CURRENT_LIMIT);
        motor.setNeutralMode(NeutralMode.Brake);
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


    public int getNumberOfBalls() {return tracker.getNumberOfBalls();} 
    public BallPosition getBall1Pos() {return tracker.getBall1Pos();} 
    public BallPosition getBall2Pos() {return tracker.getBall2Pos();} 

    public boolean isRunning() {return running;}

    @Override
    public void periodic() {
        tracker.periodic();

        running = !(conveyorStage1.getMotorOutputPercent() == 0 && conveyorStage1.getMotorOutputPercent() == 0);

        //TODO: Remove this telemetry later
        SmartDashboard.putData("Ball tracker", tracker);
        SmartDashboard.putNumber("Stage 1 speed", conveyorStage1.getMotorOutputPercent());
        SmartDashboard.putNumber("Stage 2 speed", conveyorStage2.getMotorOutputPercent());
    }

}