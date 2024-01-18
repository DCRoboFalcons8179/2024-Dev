package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.Supplier;

import javax.naming.event.EventDirContext;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Logger extends SubsystemBase{

    private static Logger logger;

    private static ArrayList<String> names = new ArrayList<>();
    private static ArrayList<Supplier> suppliers = new ArrayList<>();

    private static File dump = new File(System.getProperty("user.dir"), "src\\main\\java\\frc\\logger\\dump.txt");
    
    private static FileWriter writer;
    
    private static final long beginTimeMillis = System.currentTimeMillis();


    public Logger() {
        
        try{

            if (dump.createNewFile()) {
                System.out.println("dump file created (?)");
            } else {
                System.out.println("dump file already exists, erasing (?)");
            }

            dump.setWritable(true);
        } catch (IOException e) {
            System.out.println("dump file could not be created");
            e.printStackTrace();
        }

        try{

            writer = new FileWriter(dump);

        } catch (IOException e) {
            System.out.println("writer could not be created");
        }

        System.out.println("Logger loaded");

    }

    /*
     * Gets the singleton logger instance.
     * 
     * Creates the singleton instance if it has not yet been instantiated.
     */
    public static Logger getInstance() {
        if (logger == null) {
            logger = new Logger();
        }

        return logger;
    }

    @Override
    public void periodic() {

    }

    /*
     * Puts every name and its value into the dump file
     */
    public void dump() {

        String toDump = "";

        write("t = " + (System.currentTimeMillis() - beginTimeMillis)/1000d + "\n");
        
        for (int i = 0; i < names.size(); i++) {
            toDump += names.get(i) + ": " + suppliers.get(i).get() + "\n";
        }

        write(toDump + "\n");

    }

    /*
     * Adds a name and a value to the list that can be logged
     */
    public void addValue(String name, Supplier s) {
        names.add(name);
        suppliers.add(s);
    }

    /*
     * writes the string to the dump file
     */
    public void write(String string) {

        try{
            writer.write(string);
            writer.flush();
        } catch (IOException e) {}
    }

    /*
     * Gets the index of a name to find its supplier.
     */
    private int getIndexFromName(String name) {
        
        for (int i = 0; i < names.size(); i++) {
            if (names.get(i).equals(name)) return i;
        }

        return -1;
    }

    /*
     * dumps a specific value based on its assigned name
     */
    private void dumpValue(String name) {

        int index = getIndexFromName(name);

        if (index != -1) {
            write(suppliers.get(index).get() + "\n");
        } else {
            write("\"" + name + "\" either has not been included in the logger or is mispelled\n");
        }

    }

    /*
     * Puts the included names and their values into the dump file.
     */
    public void dumpValues(String... names) {
        for (String name : names) {
            dumpValue(name);
        }

        write("\n");
    }
}
