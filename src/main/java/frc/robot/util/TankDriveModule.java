package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TankDriveModule {
    private WPI_TalonFX leader;
    private WPI_TalonFX follower;

    private PIDController pidController;
    private SimpleMotorFeedforward feedForwardController;

    private double pidOutput = 0;
    private double feedForwardOutput = 0;

    public TankDriveModule(int leaderID, int followerID, boolean inverted, DrivetrainModuleConstants c) {
        pidController = new PIDController(c.getP(), c.getI(), c.getD());
        feedForwardController = new SimpleMotorFeedforward(c.getKS(), c.getKV());
        
        leader = new WPI_TalonFX(leaderID);
        follower = new WPI_TalonFX(followerID);

        leader.configFactoryDefault();
        follower.configFactoryDefault();

        leader.configSupplyCurrentLimit(CURRENT_LIMIT);
        follower.configSupplyCurrentLimit(CURRENT_LIMIT);

        leader.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);

        follower.follow(leader);

        if(inverted) {
            leader.setInverted(true);
            follower.setInverted(InvertType.FollowMaster); 
        }

        pidController.reset();

    }

    public void setTargetVelocity(double velocity) {
        pidController.setSetpoint(velocity);
    }

    public double getVelocity() {return countsToMeters(leader.getSelectedSensorVelocity()*10);}

    public double getEncoderMeters() {return countsToMeters(leader.getSelectedSensorPosition());}

    public double getVoltage() {return leader.getMotorOutputVoltage();}

    private double countsToMeters(double counts) {return counts / Constants.Drivetrain.TICKS_PER_METER;}

    public void resetEncoder() {leader.setSelectedSensorPosition(0);}

    public void stop() {leader.setVoltage(0);}
    
    public void setNeutralMode(NeutralMode mode) {
        leader.setNeutralMode(mode);
        follower.setNeutralMode(mode);
    }

    public double getPIDOutput() {return pidOutput;}

    public double getFeedForwardOutput() {return feedForwardOutput;}

    public void runControlLoop() {
        pidOutput = pidController.calculate(getVelocity());
        feedForwardOutput = feedForwardController.calculate(pidController.getSetpoint());

        leader.setVoltage(feedForwardOutput + pidOutput);
    }


    //TODO: Remove this after tuning
    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    public PIDController getPIDController() {
        return pidController;
    }
    
}
