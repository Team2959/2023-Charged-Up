package cwtech.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map.Entry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

public class AprilTagNav {

    HashMap<String, Pose2d> blueField = new HashMap<>();
    HashMap<String, Pose2d> redField = new HashMap<>();
    HashMap<String, Pose2d> genericItems = new HashMap<>();

    public static enum FieldElementType {
        Grid,
        Item,
        Charging,
        Other,
    };

    public static String typeToString(FieldElementType fieldElementType) {
        switch (fieldElementType) {
            case Grid:
                return "Grid";
            case Item:
                return "Item";
            case Charging:
                return "Charging";
            default:
                return "Other";
        }
    }

    public Entry<String, Pose2d> nearestLocationForTeam(Pose2d pose2d, boolean isRed) {
        Entry<String, Pose2d> best = null;
        double bestDistance = Double.MAX_VALUE;
        for(var entry : isRed ? redField.entrySet() : blueField.entrySet()) {
            var pose = entry.getValue();
            double distance = pose.getTranslation().getDistance(pose2d.getTranslation());
            if(distance < bestDistance) {
                bestDistance = distance;
                best = entry;
            }
        }
        return best;
    }

    public static FieldElementType determineType(String name) {
        name = name.toLowerCase();
        if (name.indexOf("grid") > 0) {
            return FieldElementType.Grid;
        } else if (name.indexOf("charging") > 0) {
            return FieldElementType.Charging;
        } else if (name.indexOf("item") > 0) {
            return FieldElementType.Item;
        }
        return FieldElementType.Other;
    }

    public AprilTagNav() throws IOException {
        System.err.println("Started April Tag Loading..");
        File fieldCsv = new File(Filesystem.getDeployDirectory(), "field.csv");
        BufferedReader reader = new BufferedReader(new FileReader(fieldCsv));
        String line;
        boolean firstLine = true;
        while ((line = reader.readLine()) != null) {
            if(firstLine) {
                firstLine = false;
                continue;
            }
            String[] element = line.split(",");
            String fullName = element[0];
            boolean isRed = false;
            boolean isBlue = false;
            if (fullName.toLowerCase().indexOf("red") >= 0) {
                isRed = true;
            } else if (fullName.toLowerCase().indexOf("blue") >= 0) {
                isBlue = true;
            }

            String genericName = fullName.substring(isRed ? 4 : isBlue ? 5 : 0);
            FieldElementType fieldElementType = determineType(genericName);
            double degrees = 0;
            if (fieldElementType == FieldElementType.Grid) {
                degrees = 180;
            } else if (fieldElementType == FieldElementType.Charging) {
                degrees = 0;
            }

            double x = Double.parseDouble(element[1]);
            double y = Double.parseDouble(element[2]);

            Pose2d pose2d = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(degrees));

            System.err.printf(
                    "Registered Item: name: %s, genericName: %s isRed: %b, isBlue: %b, x: %4.2f, y: %4.2f, degrees: %4.2f, fieldElementType: %s\n",
                    fullName, genericName, isRed, isBlue, x, y, degrees, typeToString(fieldElementType));
            if (isRed) {
                redField.put(genericName, pose2d);
            } else if (isBlue) {
                blueField.put(genericName, pose2d);
            } else {
                genericItems.put(genericName, pose2d);
            }
        }

        reader.close();

    }

}
