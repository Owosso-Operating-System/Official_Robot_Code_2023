package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

    /**Class: Limelight
   * Variables used: table, tx, ty, ta, x, y, area 
   * What it does: Puts the limelights vareables on the dashborad.
   *  */

public class LimeLight {
        static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        static NetworkTableEntry tx = table.getEntry("tx");
        static NetworkTableEntry ty = table.getEntry("ty");
        static NetworkTableEntry ta = table.getEntry("ta");
        
        static double x;
        static double y;
        static double area;

    /**Method: updateTable
   * Parameters: N/A
   * Variables used: tx, ty, ta, x, y, area 
   * What it does: Puts the limelights vareables on the dashborad.
   *  */
  
    public static void updateTable(){
        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    public static double getX(){
        return x;
    }

    public static double getY(){
        return y;
    }

    public static double getArea(){
        return area;
    }
}
