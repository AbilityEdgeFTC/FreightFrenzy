package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;

public class FileTesting {

    File configFile = new File("config.properties");

    public void readLine()
    {
        try
        {
            FileReader reader = new FileReader(configFile);
            Properties props = new Properties();
            props.load(reader);

            String p = props.getProperty("P");
            String i = props.getProperty("I");
            String d = props.getProperty("D");

            System.out.print("P is: " + p);
            System.out.print("I is: " + i);
            System.out.print("D is: " + d);
            reader.close();

        } catch (FileNotFoundException ex) {
            // file does not exist
        } catch (IOException ex) {
            // I/O error
        }

    }

}
