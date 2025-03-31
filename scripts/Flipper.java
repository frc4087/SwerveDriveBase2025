import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.List;
import java.util.ArrayList;

public class Flipper {
    private static final double Y_MAX = 8.052; // Maximum Y coordinate of the playing field
    
    public static void main(String[] args) {
        String autoDir = "src/main/deploy/pathplanner/autos";
        String pathsDir = "src/main/deploy/pathplanner/paths";
        Set<String> existingPaths = new HashSet<>();
        // Use a list to maintain order
        java.util.List<String> pathNames = new java.util.ArrayList<>();
        String targetAutoName = null;
        
        // Validate command line args - require the auto name
        if (args.length < 1) {
            System.err.println("Error: Auto name is required");
            System.err.println("Usage: java Flipper <auto_name>");
            System.err.println("  where <auto_name> is the name of the auto file to process");
            return;
        }
        
        // Process command line args
        targetAutoName = args[0];
        System.out.println("Looking for paths in auto: " + targetAutoName);
        
        // First check that both directories exist
        File autoDirFile = new File(autoDir);
        File pathsDirFile = new File(pathsDir);
        
        if (!autoDirFile.exists() || !autoDirFile.isDirectory()) {
            System.err.println("Autos directory not found: " + autoDir);
            return;
        }
        
        if (!pathsDirFile.exists() || !pathsDirFile.isDirectory()) {
            System.err.println("Paths directory not found: " + pathsDir);
            return;
        }
        
        // Load existing paths
        File[] pathFiles = pathsDirFile.listFiles((dir, name) -> name.toLowerCase().endsWith(".path"));
        if (pathFiles != null) {
            for (File pathFile : pathFiles) {
                String pathName = pathFile.getName();
                if (pathName.toLowerCase().endsWith(".path")) {
                    pathName = pathName.substring(0, pathName.length() - 5);
                }
                existingPaths.add(pathName);
            }
        }
        
        // Only process the specified auto file
        File autoFile = new File(autoDir, targetAutoName + ".auto");
        if (!autoFile.exists()) {
            System.err.println("Auto file not found: " + autoFile.getPath());
            return;
        }
        
        // Simple regex to extract pathName when type is "path"
        Pattern pattern = Pattern.compile("\"type\"\\s*:\\s*\"path\"[^}]*\"data\"\\s*:\\s*\\{[^}]*\"pathName\"\\s*:\\s*\"([^\"]*)\"");
        
        try {
            String content = new String(Files.readAllBytes(autoFile.toPath()));
            Matcher matcher = pattern.matcher(content);
            
            while (matcher.find()) {
                String pathName = matcher.group(1);
                // Only add if it's not already in the list to keep first occurrence
                if (!pathNames.contains(pathName)) {
                    pathNames.add(pathName);
                }
            }
        } catch (IOException e) {
            System.err.println("Error reading file " + autoFile.getName() + ": " + e.getMessage());
            return;
        }
        
        if (pathNames.isEmpty()) {
            System.out.println("\nNo paths found for auto: " + targetAutoName);
            return;
        }
        
        System.out.println("\nTotal unique paths found: " + pathNames.size());
        System.out.println("All path names (in order of appearance):");
        
        // Use the paths directory directly to write paths
        String outputDir = pathsDir;
        
        // No need to create the directory as it should already exist
        if (!new File(outputDir).exists()) {
            System.err.println("Paths directory doesn't exist: " + outputDir);
            return;
        }
        
        // Process each path from the specific auto
        for (String pathName : pathNames) {
            boolean pathExists = existingPaths.contains(pathName);
            System.out.println("- " + pathName + (pathExists ? "" : " [MISSING]"));
            
            if (pathExists) {
                // Read and process the path file
                File pathFile = new File(pathsDir, pathName + ".path");
                
                try {
                    // Read the path file
                    StringBuilder contentBuilder = new StringBuilder();
                    try (BufferedReader br = new BufferedReader(new FileReader(pathFile))) {
                        String line;
                        while ((line = br.readLine()) != null) {
                            contentBuilder.append(line).append("\n");
                        }
                    }
                    String content = contentBuilder.toString();
                    
                    // Create and write the flipped path - but don't copy the original
                    String flippedContent = flipPath(content);
                    if (flippedContent != null) {
                        String flippedFileName = pathName + " (Flipped).path";
                        try (FileWriter flippedWriter = new FileWriter(outputDir + "/" + flippedFileName)) {
                            flippedWriter.write(flippedContent);
                            System.out.println("  Wrote flipped path: " + flippedFileName);
                        }
                    }
                } catch (IOException e) {
                    System.err.println("  Error processing path file " + pathName + ": " + e.getMessage());
                }
            }
        }
        
        // Create flipped auto if we're targeting a specific auto
        if (targetAutoName != null) {
            createFlippedAuto(autoDir, targetAutoName, pathNames);
        }
    }
    
    /**
     * Creates a flipped version of the auto file
     * 
     * @param autoDir Directory containing auto files
     * @param autoName Name of the auto to flip
     * @param pathNames List of path names found in the auto
     */
    private static void createFlippedAuto(String autoDir, String autoName, List<String> pathNames) {
        File originalAutoFile = new File(autoDir, autoName + ".auto");
        File flippedAutoFile = new File(autoDir, autoName + " (Flipped).auto");
        
        if (!originalAutoFile.exists()) {
            System.err.println("Original auto file not found: " + originalAutoFile.getPath());
            return;
        }
        
        try {
            // Read the original auto file
            String content = new String(Files.readAllBytes(originalAutoFile.toPath()));
            
            // Create regex to only find path commands
            Pattern pathCommandPattern = Pattern.compile(
                "\\{\\s*\"type\"\\s*:\\s*\"path\"[^}]*\"pathName\"\\s*:\\s*\"([^\"]*)\"[^}]*\\}"
            );
            
            Matcher pathCommandMatcher = pathCommandPattern.matcher(content);
            StringBuffer contentBuffer = new StringBuffer();
            
            // Process each path command
            while (pathCommandMatcher.find()) {
                String pathCommand = pathCommandMatcher.group(0);
                String pathName = pathCommandMatcher.group(1);
                
                // Only replace if this path is in our list
                if (pathNames.contains(pathName)) {
                    // Replace only the pathName part
                    String updatedCommand = pathCommand.replaceAll(
                        "\"pathName\"\\s*:\\s*\"" + Pattern.quote(pathName) + "\"",
                        "\"pathName\": \"" + pathName + " (Flipped)\""
                    );
                    pathCommandMatcher.appendReplacement(contentBuffer, 
                        updatedCommand.replace("$", "\\$"));
                }
            }
            
            pathCommandMatcher.appendTail(contentBuffer);
            content = contentBuffer.toString();
            
            // Now update the auto name at the file level (not individual commands)
            // This should just modify the auto name property at the start of the file
            Pattern autoNamePattern = Pattern.compile("\"version\"\\s*:\\s*\"[^\"]*\"[^{]*\"name\"\\s*:\\s*\"([^\"]*)\"");
            Matcher autoNameMatcher = autoNamePattern.matcher(content);
            
            if (autoNameMatcher.find()) {
                // Extract the original name
                String originalAutoName = autoNameMatcher.group(1);
                String flippedAutoName = originalAutoName + " (Flipped)";
                
                // Replace just this instance
                content = content.substring(0, autoNameMatcher.start(1)) +
                          flippedAutoName + 
                          content.substring(autoNameMatcher.end(1));
            }
            
            // Write the flipped auto file
            try (FileWriter writer = new FileWriter(flippedAutoFile)) {
                writer.write(content);
                System.out.println("Created flipped auto: " + flippedAutoFile.getName());
            }
            
        } catch (IOException e) {
            System.err.println("Error creating flipped auto: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Flips a path across the middle of the field by applying these transformations:
     * 1. For each coordinate (x,y): New coordinates are (Y_MAX - x, Y_MAX - y)
     * 2. For each rotation angle: New angle is (angle + 180) % 360
     * 
     * @param pathContent The JSON string containing the path data
     * @return The transformed JSON string with flipped coordinates and rotations
     */
    private static String flipPath(String pathContent) {
        try {
            // ---------- STEP 1: Flip all coordinates across the field center ----------
            
            // For any point (x,y) in the field, the flipped point is (Y_MAX - x, Y_MAX - y)
            // This creates a mirror image across the center of the field
            
            // Apply the coordinate flip to anchor points (the main waypoints of the path)
            pathContent = flipCoordinates(pathContent, "anchor");
            
            // Apply the coordinate flip to control points (these control the curve shape)
            pathContent = flipCoordinates(pathContent, "prevControl");
            pathContent = flipCoordinates(pathContent, "nextControl");
            
            // ---------- STEP 2: Flip all rotation angles ----------
            
            // For any angle in degrees, the flipped angle is (angle + 180) % 360
            // This makes the robot face the opposite direction
            
            // Apply the rotation flip to the end state rotation
            pathContent = flipRotation(pathContent, "goalEndState");
            
            // Apply the rotation flip to the starting state rotation
            pathContent = flipRotation(pathContent, "idealStartingState");
            
            return pathContent;
        } catch (Exception e) {
            System.err.println("Error flipping path: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }
    
    /**
     * Flips the x and y coordinates for a specific point type
     * 
     * @param content The JSON content
     * @param pointType The type of point to flip (anchor, prevControl, nextControl)
     * @return Updated JSON content with flipped coordinates
     */
    private static String flipCoordinates(String content, String pointType) {
        // Find patterns like: "pointType": {"x": 1.2, "y": 3.4}
        Pattern pattern = Pattern.compile("\"" + pointType + "\"\\s*:\\s*\\{\\s*\"x\"\\s*:\\s*([0-9.]+)\\s*,\\s*\"y\"\\s*:\\s*([0-9.]+)\\s*\\}");
        Matcher matcher = pattern.matcher(content);
        StringBuffer sb = new StringBuffer();
        
        while (matcher.find()) {
            // Extract the original x and y values
            double x = Double.parseDouble(matcher.group(1));
            double y = Double.parseDouble(matcher.group(2));
            
            // Apply the transformation: new position = (Y_MAX - old position)
            double newX = Y_MAX - x;
            double newY = Y_MAX - y;
            
            // Replace with the new coordinates
            String replacement = "\"" + pointType + "\": {\"x\": " + newX + ", \"y\": " + newY + "}";
            matcher.appendReplacement(sb, replacement.replace("$", "\\$"));
        }
        
        matcher.appendTail(sb);
        return sb.toString();
    }
    
    /**
     * Flips the rotation angle for a specific state
     * 
     * @param content The JSON content
     * @param stateType The type of state to flip (goalEndState, idealStartingState)
     * @return Updated JSON content with flipped rotation
     */
    private static String flipRotation(String content, String stateType) {
        // Find patterns like: "stateType": {..."rotation": 90.0...}
        Pattern pattern = Pattern.compile("\"" + stateType + "\"\\s*:\\s*\\{([^}]*)\"rotation\"\\s*:\\s*([0-9.]+)([^}]*)\\}");
        Matcher matcher = pattern.matcher(content);
        StringBuffer sb = new StringBuffer();
        
        while (matcher.find()) {
            // Keep the parts before and after the rotation value
            String before = matcher.group(1);
            String after = matcher.group(3);
            
            // Extract and transform the rotation
            double rotation = Double.parseDouble(matcher.group(2));
            
            // Apply the transformation: new rotation = (old rotation + 180) % 360
            // This makes the robot face the opposite direction
            double newRotation = (rotation + 180.0) % 360.0;
            
            // Replace with the new rotation
            String replacement = "\"" + stateType + "\": {" + before + "\"rotation\": " + newRotation + after + "}";
            matcher.appendReplacement(sb, replacement.replace("$", "\\$"));
        }
        
        matcher.appendTail(sb);
        return sb.toString();
    }
}