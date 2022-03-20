package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Turret;

public class DefaultLightCommand extends CommandBase{


    private Lights lights;
    private Drivetrain drivetrain;
    private Intake intake;
    private Conveyor conveyor;
    private Turret turret;
    private Climber climber;

    public DefaultLightCommand(Lights lights, Drivetrain drivetrain, Intake intake, Conveyor conveyor, Turret turret, Climber climber) {
        this.lights = lights;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.conveyor = conveyor;
        this.turret = turret;
        this.climber = climber;
    }

    
}
