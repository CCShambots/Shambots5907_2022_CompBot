package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TankDriveModule {
    private WPI_TalonFX leader;
    private WPI_TalonFX follower;

    private PIDController autoPID;
    private PIDController telePID;
    private PIDController activePID;
    private SimpleMotorFeedforward feedForwardController;

    private double pidOutput = 0;
    private double feedForwardOutput = 0;

    public TankDriveModule(int leaderID, int followerID, boolean inverted, PIDandFFConstants autoC, PIDandFFConstants teleC) {
        autoPID = new PIDController(autoC.getP(), autoC.getI(), autoC.getD());
        telePID = new PIDController(teleC.getP(), teleC.getI(), teleC.getD());
        feedForwardController = new SimpleMotorFeedforward(autoC.getKS(), autoC.getKV());

        activePID = autoPID;
        
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

        autoPID.reset();

    }

    public void setTargetVelocity(double velocity) {
        activePID.setSetpoint(velocity);
    }

    public double getVelocity() {
        return 
            positionToMeters(leader.getSelectedSensorVelocity()) 
            * 10;}

    public double getEncoderMeters() {return positionToMeters(leader.getSelectedSensorPosition());}

    public double getVoltage() {return leader.getMotorOutputVoltage();}

    private double positionToMeters(double counts) {return (counts / COUNTS_PER_REV) * (WHEEL_SIZE * Math.PI) * GEAR_RATIO * 0.0254; }

    public void resetEncoder() {leader.setSelectedSensorPosition(0);}

    public void stop() {leader.setVoltage(0);}
    
    public void setNeutralMode(NeutralMode mode) {
        leader.setNeutralMode(mode);
        follower.setNeutralMode(mode);
    }

    public double getPIDOutput() {return pidOutput;}

    public double getFeedForwardOutput() {return feedForwardOutput;}

    public void resetPID() {
        activePID.reset();
    }

    public void setPID(ControlMode mode) {
        if(mode == ControlMode.Auto) {
            activePID = autoPID;
        } else activePID = telePID;
        activePID.reset();
    }

    public void runControlLoop() {
        pidOutput = activePID.calculate(getVelocity());
        feedForwardOutput = feedForwardController.calculate(activePID.getSetpoint());

        leader.setVoltage(feedForwardOutput + pidOutput);
    }

    public double getSetpoint() {return activePID.getSetpoint();}
    public PIDController getPIDController() {return activePID;}

    public enum ControlMode {
        Auto, TeleOp
    }
}
