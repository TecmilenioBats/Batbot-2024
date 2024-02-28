package frc.robot.Autonomo;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveAuto extends SubsystemBase {
    private String nombreAutonomo = "Batbot";
    private Command autonomo;
    private Pose2d posicion;
    private PathPlannerPath camino;

    public DriveAuto() {
        posicion = PathPlannerAuto.getStaringPoseFromAutoFile(nombreAutonomo);
        camino = (PathPlannerPath)PathPlannerAuto.getPathGroupFromAutoFile(nombreAutonomo);

        System.out.print(posicion);
        System.out.print(camino);


        autonomo = AutoBuilder.buildAuto(nombreAutonomo);
        SmartDashboard.putData("Prueba Autonomo", autonomo);

    }
}
