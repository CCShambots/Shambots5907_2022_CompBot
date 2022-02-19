package frc.robot.util.auton;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class AllRobotSubsystems {
    private Drivetrain drivetrain;
    private Intake intake;
    private Conveyor conveyor;
    private Turret turret;
    private Climber climber;

    public AllRobotSubsystems(Drivetrain drivetrain, Intake intake, Conveyor conveyor, Turret turret, Climber climber) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.conveyor = conveyor;
        this.turret = turret;
        this.climber = climber;
    }

    public Drivetrain getDrivetrain() { return drivetrain;}
    public Intake getIntake() { return intake;}
    public Conveyor getConveyor() { return conveyor;}
    public Turret getTurret() { return turret;}
    public Climber getClimber() { return climber;}
}
