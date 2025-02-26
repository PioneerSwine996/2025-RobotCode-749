// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
      private final CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();
    private final ElevatorSubSystems elevatorSubSystems = new ElevatorSubSystems();
    private final PivotSubSystem pivotSubSystem = new PivotSubSystem();
    private final AlgaeSubSystem algaeSubSystem = new AlgaeSubSystem();
    private final AlageRollerSubsystem alageRollerSubsystem = new AlageRollerSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();

    // The driver's controller
    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);

    // The operator's controller
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.OPERATOR_CONTROLLER_PORT);

    // The autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        SmartDashboard.putData(autoChooser);
        // Set the options to show up in the Dashboard for selecting auto modes. If you
        // add additional auto modes you can add additional lines here with
        // autoChooser.addOption
        autoChooser.setDefaultOption("AutonomusFoward", new SequentialCommandGroup(
                new AutoForawrd(driveSubsystem, Constants.DriveConstants.distance),
        new ParallelCommandGroup(
                new ElevatorPID(elevatorSubSystems, Constants.ElevatorConstants.encoderSetpoint)),
                new SequentialCommandGroup(
                        new AutoRoller(rollerSubsystem, Constants.RollerConstants.ROLLER_EJECT_VALUE))));

        autoChooser.addOption("AutonomousTurnthenFoward", new SequentialCommandGroup(
                new AutoTurn(driveSubsystem, Constants.DriveConstants.leftTurn, Constants.DriveConstants.rightTurn),
                new AutoForawrd(driveSubsystem, Constants.DriveConstants.distance),
                new ParallelCommandGroup(
                        new ElevatorPID(elevatorSubSystems, Constants.ElevatorConstants.encoderSetpoint)),
                new SequentialCommandGroup(
                        new AutoRoller(rollerSubsystem,Constants.RollerConstants.ROLLER_EJECT_VALUE))));
        autoChooser.addOption("AprilTag Test", new SequentialCommandGroup(new AprilTag(driveSubsystem, visionSubsystem, visionSubsystem.getTargetDistance())));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link CommandPS4Controller
     * PS4} controllers or
     * {@link CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Set the A button to run the "runRoller" command from the factory with a fixed
        // value ejecting the gamepiece while the button is held
//    operatorController.a()
//            .whileTrue(rollerSubsystem.runRoller(rollerSubsystem, () -> Constants.RollerConstants.ROLLER_EJECT_VALUE, () -> 0));
        operatorController.a().whileTrue(new AutoRoller(rollerSubsystem, Constants.RollerConstants.ROLLER_EJECT_VALUE));
        operatorController.b().whileTrue((new AutoRoller(rollerSubsystem, Constants.RollerConstants.ROLLER_SHOOT_VALUE)));


        operatorController.leftTrigger().toggleOnTrue(
                new Pivot(pivotSubSystem, Constants.PivotConstants.intakePosition));
        /*operatorController.leftBumper().toggleOnTrue(
                new Pivot(pivotSubSystem, -8.0)
        );*/

        /*operatorController.y().toggleOnTrue(
                new AlgaePivot(algaeSubSystem, Constants.AlgaeConstants.encoderSetpoint));
       operatorController.x().whileTrue(
                new AlageRoller(alageRollerSubsystem, Constants.AlageRollerConstants.ALAGE_ROLLER_INTAKE));*/
       operatorController.y().toggleOnTrue(Commands.parallel(
               new AlgaePivot(algaeSubSystem, Constants.AlgaeConstants.encoderSetpoint),
               new AlageRoller(alageRollerSubsystem, Constants.AlageRollerConstants.ALAGE_ROLLER_INTAKE)));
        operatorController.x().whileTrue(
                new AlageRoller(alageRollerSubsystem, Constants.AlageRollerConstants.AlAGE_ROLLER_SHOOT)
        );
        operatorController.leftBumper().whileTrue(Commands.parallel(

        ));



//       driverController.x().whileTrue(new AlgaePivot(algaeIntakeSubSystem,Constants.AlgaeIntakeConstants.encoderSetpoint));
//       driverController.rightBumper().toggleOnTrue(new ElevatorPID(elevatorSubSystems, Constants.ElevatorConstants.halfEncoderSetpoint));

        operatorController.rightTrigger().toggleOnTrue(new ElevatorPID(elevatorSubSystems, Constants.ElevatorConstants.encoderSetpoint));
        operatorController.rightBumper().toggleOnTrue(new ElevatorPID(elevatorSubSystems, Constants.ElevatorConstants.halfEncoderSetpoint));



        // Set the default command for the drive subsystem to the command provided by
        // factory with the values provided by the joystick axes on the driver
        // controller. The Y axis of the controller is inverted so that pushing the
        // stick away from you (a negative value) drives the robot forwards (a positive
        // value)
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveArcade(
                        driveSubsystem, () -> Constants.DriveConstants.speedFactor * -operatorController.getLeftY(),
                        () -> Constants.DriveConstants.turningFactor * -operatorController.getRightX()));

        driveSubsystem.setDefaultCommand(
                driveSubsystem.ArcadeDrive(
                        driveSubsystem, ()-> Constants.DriveConstants.speedFactor,
                        () -> Constants.DriveConstants.turningFactor
                )
        );



        elevatorSubSystems.setDefaultCommand(new ElevatorPID(elevatorSubSystems, 0));
        rollerSubsystem.setDefaultCommand(new AutoRoller(rollerSubsystem,0.15));
        algaeSubSystem.setDefaultCommand(new AlgaePivot(algaeSubSystem, Constants.AlgaeConstants.original));//-0.1
        pivotSubSystem.setDefaultCommand(new Pivot(pivotSubSystem, Constants.PivotConstants.lvTwoAndThreeEncoderSetpoint)); //-16 og
        alageRollerSubsystem.setDefaultCommand(new AlageRoller(alageRollerSubsystem, -0.1));

        /*lageRollerSubsystem.setDefaultCommand((
                alageRollerSubsystem.runRoller(
                alageRollerSubsystem,
                () -> -operatorController.getLeftY(),
                () -> -operatorController.getLeftY())
        ));*/


        // Set the default command for the roller subsystem to the command from the
         // factory with the values provided by the triggers on the operator controller
//    rollerSubsystem.setDefaultCommand(
//        rollerSubsystem.runRoller(
//            rollerSubsystem,
//            () -> operatorController.getRightTriggerAxis(),
//            () -> operatorController.getLeftTriggerAxis()));
    }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand () {
            // An example command will be run in autonomous
            return autoChooser.getSelected();

        }
        
    }

