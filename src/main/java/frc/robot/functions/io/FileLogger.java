package frc.robot.functions.io;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.library.Constants.RobotState;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.io.*;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Arrays;
import java.util.Comparator;

public class FileLogger extends OutputStream {
    public enum EventType {
        Debug ("Debug: "),
        Error ("ERROR: "),
        Warning ("Warning: "),
        Status ("Status: ");

        String name = "";

        EventType(String string) {
            this.name = string;
        }

        public String toString() {
            return this.name;
        }
    }

    private final int debug;
    private static FileWriter writer;
    private final String logName;
    private final String logFileDirectory;
    private final int maxLogFiles = 15;

    private final DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("MMddyyyy_HHmmss_SSS");

    private String tagString = "";

    private static Thread flushThread;

    public FileLogger(int _debug, RobotState robotState){
        this.debug = _debug;

//        flushThread = new Thread(() -> flush());

        this.logName = "log_" + robotState.toString() + "_" + getDateString() + ".txt";

//        this.logFileDirectory = System.getProperty("user.dir") + "\\File Logger\\";
        this.logFileDirectory = "/home/lvuser/File Logger/";

        try {
            writer = new FileWriter(this.logFileDirectory + this.logName);
            writer.write("MMdd_HHmmss_SSS-Type: Event\n");
            flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
        cleanLogs();
    }

    @Override
    public synchronized void flush() {
        try {
            writer.flush();
        } catch (Exception e) {
            System.out.println("Failed to flush FileWriter data");
            DriverStation.reportWarning("Failed to flush FileWriter data", false);
            e.printStackTrace();
        }
    }

    private static synchronized void flushWriter() {
        try {
            writer.flush();
        } catch (Exception e) {
            System.out.println("Failed to flush FileWriter data");
            e.printStackTrace();
        }
    }

    private String getDateString() {
        return dateTimeFormatter.format(LocalDateTime.now());
    }

    /**
     * Writes an event to a log file stored on the directory given
     * Writes only if the given debug level is above the one given in the contructor
     *
     * @param _debug
     * @param Title
     * @param Message
     */
    public synchronized void writeEvent(int _debug, String Title, String Message){
        writeEvent(_debug, Title + "-" + Message);
    }

    /**
     * Writes an event to a log file stored on the directory given
     * Writes only if the given debug level is above the one given in the contructor
     * The event type is used as the Title for the log
     *
     * @param _debug
     * @param event
     * @param Message
     */
    public synchronized void writeEvent(int _debug, EventType event, String Message) {
        writeEvent(_debug, event.toString() + Message);
    }

    /**
     * Writes an event to a glof file stored on the diectory given
     * Writes only if the given debug level is above the one given in the contructor
     * Only writes the message with no Title
     *
     * @param _debug
     * @param Message
     */
    public synchronized void writeEvent(int _debug, String Message) {
        if (_debug <= this.debug) {
            writeLine(Message);
        }
    }

    /**
     * Writes an event to a log file on the directory given
     * Does not start a new line in the file
     * @param Message
     */
    public synchronized void write(String Message) {
        rawWrite(getDateString() + "-" + Message);
    }

    public synchronized void rawWrite(String message) {
        try {
            if (!getTag().equals(""))
                writer.write("~" + getTag() + "~"  + message);
            else
                writer.write(message);
        } catch (IOException e) {
            System.out.println("Error Writing To File Logger!!");
            e.printStackTrace();
        }
        flush();
    }

    /**
     * Internal function that writes a single line to the file and able to handle errors thrown
     * @param toWrite - String to write
     */
    public synchronized void writeLine(String toWrite) {
        try {
            writer.write(getDateString() + "-" + toWrite + "\n");
        } catch (Exception e) {
            System.out.println("Error Writing To File Logger!!");
            e.printStackTrace();
        }
    }

    /**
     * List all the files in the Log directory
     * Then if there is more than the max log amount then delete
     */
    private void cleanLogs(){
        File[] fileList = new File(logFileDirectory).listFiles();

        Arrays.sort(fileList, Comparator.comparing(File::lastModified).reversed());

        if (fileList.length > maxLogFiles) {
            for (int i = maxLogFiles; i < fileList.length; i++) {
                fileList[i].delete();
            }
        }
    }

    public synchronized void logRobotPosition(Translation2d translation2d, Rotation2d rotation2d) {
        writeLine("POSITION (" + translation2d.getX() + ", " + translation2d.getY() + "," + rotation2d.getDegrees() + ")");
    }

    @Override
    public void write(int b) throws IOException {
        writer.write((char) b);
    }

    /**
     * Close the FileLogger writer
     */
    @Override
    public synchronized void close(){
        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String getTag() {
        return tagString;
    }

    public void setTag(String name) {
        tagString = name;
    }
}
