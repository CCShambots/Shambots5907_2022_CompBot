package frc.robot.commands.limelight;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Turret;

public class TeleopTrackingCommand extends BasicTrackingCommand{
    // private Conveyor conveyor;
    private BooleanSupplier shootingSupplier;

    //TODO: correct the shooting function to actually end once we can effectively track how many balls are in the bot
    //TODO: Add the conveyor back to the teleop command
    public TeleopTrackingCommand(Turret turret, BooleanSupplier shootingSupplier) {
        super(turret);//conveyor);
        // this.conveyor = conveyor;
        this.shootingSupplier = shootingSupplier;
    }

    @Override
    public boolean isComplete() {
        return false; //We want the function to end only when it's cancelled (it's entered and exited solely in teleop)
    }

    @Override
    public void additionalCodeInInitialize() {
        // turret.setFlywheelTarget(4300);
    }

    @Override
    public void additionalCodeInExecute() {
        SmartDashboard.putBoolean("Shooting indicated", shootingSupplier.getAsBoolean());
        

        //If the button to shoot has been pressed and the shooter is in a valid location, shoot
        if(shootingSupplier.getAsBoolean() && turret.isShootingAllowed()) shoot();
    }

    private void shoot() {
        // conveyor.intakeAll(1);
    }

    @Override
    public void additionalCodeInEnd() {
        // conveyor.intakeAll(0);
        
    }
}
