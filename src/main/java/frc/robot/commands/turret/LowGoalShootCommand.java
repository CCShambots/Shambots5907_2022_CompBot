package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.turret.shooting.TimedShootCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Turret;

public class LowGoalShootCommand extends SequentialCommandGroup{

    private Turret turret;

    public LowGoalShootCommand(Conveyor conveyor, Turret turret) {
        this.turret = turret;
        addCommands(
            new SpinUpFlywheelCommand(turret, Constants.Turret.FLYWHEEL_LOW_RPM),
            new TimedShootCommand(conveyor),
            new InstantCommand(() -> turret.setFlywheelTarget(0))
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        if(interrupted) {turret.setFlywheelTarget(0);}
    }

    
}
