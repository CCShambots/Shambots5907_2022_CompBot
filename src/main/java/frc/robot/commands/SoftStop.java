package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class SoftStop extends CommandBase{

    private Climber climber;
    private Turret turret;
    private Conveyor conveyor;
    private Intake intake;

    public SoftStop(Intake intake, Conveyor conveyor, Turret turret, Climber climber) {
        this.intake = intake;
        this.conveyor = conveyor;
        this.turret = turret;
        this.climber = climber;
    
        addRequirements(intake, conveyor, turret, climber);
    }

    @Override
    public void initialize() {
        intake.stop();
        conveyor.stopAll();
        turret.setFlywheelTarget(0);
        turret.setLimelightOff();
        turret.setSpinnerTarget(turret.getSpinnerAngle());

        conveyor.setEjecting(false);
        climber.brake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
