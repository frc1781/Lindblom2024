package tech.team1781.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDate;
import java.time.LocalTime;
import java.time.temporal.ChronoField;
import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.DriverStation;

public class Logger {
    public static final String LOG_PATH = System.getProperty("user.home") + "/EELogs";

    private Queue<String> mBatch;
    private String mBuffer = "";
    private int mBatchSize = 10;
    private File mOutput;
    private int mLineCount = 0;

    public Logger(String fileName, int batchSize) {
        LocalTime time = LocalTime.now();
        LocalDate date = LocalDate.now();
        mBatch = new LinkedList<>();
        mOutput = new File(LOG_PATH + "/" + fileName
                + "_" + date.get(ChronoField.YEAR) + "-" + date.get(ChronoField.MONTH_OF_YEAR) + "-"
                + date.get(ChronoField.DAY_OF_MONTH)
                + "_" + time.getHour() + ":" + time.getMinute() + ":" + time.getSecond()
                + "_" + DriverStation.getAlliance()
                + "_" + DriverStation.getEventName()
                + "_" + DriverStation.getMatchType()
                + "_" + DriverStation.getMatchNumber()
                + ".csv");

        mBatchSize = batchSize;

    }

    public void logData(String... data) {
        StringBuilder builder = new StringBuilder();
        for(String s : data) {
            builder.append(s+",");
        }

        String builderResult = builder.toString();
        String recievedData = builderResult.substring(0, builderResult.length() - 1);
        if(mBuffer.equals(recievedData)) {
            return;
        }

        mBuffer = recievedData;
        mBatch.offer(DriverStation.getMatchTime() + "," + recievedData);

        if(mBatch.size() >= mBatchSize) {
            writeBatch();
        }
    }

    public void close() {
        writeBatch();
    }

    private void writeBatch() {
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(mOutput, true));
            for(int i = 0; i < mBatch.size(); i++) {
                writer.write(mBatch.poll());
                writer.newLine();
            }

            writer.flush();
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
