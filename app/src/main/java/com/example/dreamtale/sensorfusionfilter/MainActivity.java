package com.example.dreamtale.sensorfusionfilter;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.widget.TextView;
import android.support.design.widget.FloatingActionButton;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.Toast;

import com.example.dreamtale.sensorfusionfilter.DebugUtils.DebugDataWriter;
import com.example.dreamtale.sensorfusionfilter.Utils.AverageFilter;
import com.example.dreamtale.sensorfusionfilter.Utils.ComplementaryFilter;
import com.example.dreamtale.sensorfusionfilter.Utils.Euler;
import com.example.dreamtale.sensorfusionfilter.Utils.FilterDispatcher;
import com.example.dreamtale.sensorfusionfilter.Utils.MedianFilter;
import com.example.dreamtale.sensorfusionfilter.Motion.DataStructure;
import com.example.dreamtale.sensorfusionfilter.Motion.DataSynchronizer;
import com.example.dreamtale.sensorfusionfilter.Utils.Quaternion;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point3;

import java.io.File;
import java.io.IOException;

public class MainActivity extends AppCompatActivity {
    private Context mContext = this;
    private final long TIME_DURATION = 500;    // Check update every 500 ms
    private enum RecordMode {
        RECORD_START,
        RECORD_STOP,
        RECORD_PROCESSING,
        RECORD_UPDATE
    }
    private RecordMode mRecordMode = RecordMode.RECORD_STOP;

    // <editor-fold desc="Maintain Activity BLOCK">
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initLayouts();
        initAndSetupAllSensors();
        startMotionSensors();
    }

    @Override
    protected void onResume() {
        super.onResume();
        // Check OpenCV weather load successfully
        if(!OpenCVLoader.initDebug()) {
            Toast.makeText(mContext, "Internal OpenCV lib not found",
                    Toast.LENGTH_SHORT).show();
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_2_0, this,
                    new BaseLoaderCallback(mContext) {
                @Override
                public void onManagerConnected(int status) {
                    super.onManagerConnected(status);
                    switch (status) {
                        case BaseLoaderCallback.SUCCESS:
                            Toast.makeText(mContext, "Load module successfully",
                                    Toast.LENGTH_SHORT).show();
                            break;
                        default:
                            onManagerConnected(status);
                            Toast.makeText(mContext, "Load module failed",
                                    Toast.LENGTH_SHORT).show();
                            break;
                    }
                }
            });
        } else {
            Toast.makeText(mContext, "OpenCV lib found inside the package, using it",
                    Toast.LENGTH_SHORT).show();
        }

        mIsFirstIn = true;
        initAndSetupAllSensors();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(mSensorEventListener);
        mIsFirstIn = true;
    }

    // </editor-fold>

    // <editor-fold desc="Motion sensor Block">
    private Sensor mMotionSensor;
    private SensorManager mSensorManager;
    private SensorEventListener mSensorEventListener;
    private double mLastTimestamp;
    private TextView mTvAccX;
    private TextView mTvAccY;
    private TextView mTvAccZ;
    private TextView mTvVelX;
    private TextView mTvVelY;
    private TextView mTvVelZ;
    private TextView mTvPathX;
    private TextView mTvPathY;
    private TextView mTvPathZ;

    private TextView mTvRoll;
    private TextView mTvPitch;
    private TextView mTvYaw;

    private boolean mIsFirstIn = true;

    // structure of motion data:
    // +--------------------+-----------------+------------------+------------------+----------+
    // |   magnitude data   | gyroscope data  |acceleration data |  liner-acc data  |time span |
    // +--------------------+-----------------+------------------+------------------+----------+
    // |   3 * double size  | 3 * double size | 3 * double size  | 3 * double size  |1 * double|
    // +--------------------+-----------------+------------------+------------------+----------+
    private final int SENSOR_NUM = 13;
    private double[] mMotionData = new double[SENSOR_NUM]; // The last one is the time span | ms
    private Handler mSensorHandler;
    private final double UPDATE_UI_FREQUENCY = 5;
    private int mTimeCounter4UpdateUI = 0;
    private DataSynchronizer mMdSynchronizer = new DataSynchronizer();
    private SensorDataProcessor mSensorDataProcessor;
    private File mFileForDebug;
    private DebugDataWriter mDataTimeCmpRecorderMag = new DebugDataWriter(mContext);
    private DebugDataWriter mDataTimeCmpRecorderAcc = new DebugDataWriter(mContext);
    private DebugDataWriter mDataTimeCmpRecorderGyr = new DebugDataWriter(mContext);
    private DebugDataWriter mDataTimeCmpRecorderLin = new DebugDataWriter(mContext);
    private DebugDataWriter mDataSyncCmpRecorderSen = new DebugDataWriter(mContext);

    private void initAndSetupAllSensors() {
        mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);
        // Register accelerometer
        mMotionSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if(mMotionSensor != null) {
            mSensorManager.registerListener(mSensorEventListener, mMotionSensor,
                    SensorManager.SENSOR_DELAY_FASTEST);
        } else {
            Toast.makeText(mContext, "No ACCELEROMETER detected", Toast.LENGTH_SHORT).show();
        }
        // Register magnetic field
        mMotionSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        if(mMotionSensor != null) {
            mSensorManager.registerListener(mSensorEventListener, mMotionSensor,
                    SensorManager.SENSOR_DELAY_FASTEST);
        } else {
            Toast.makeText(mContext, "No MAGNETIC_FIELD detected", Toast.LENGTH_SHORT).show();
        }
        // Register gyroscope
        mMotionSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        if(mSensorManager != null) {
            mSensorManager.registerListener(mSensorEventListener, mMotionSensor,
                    SensorManager.SENSOR_DELAY_FASTEST);
        } else {
            Toast.makeText(mContext, "No GYROSCOPE detected", Toast.LENGTH_SHORT).show();
        }
        // Register linear acceleration for init
        mMotionSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        if(mSensorManager != null) {
            mSensorManager.registerListener(mSensorEventListener, mMotionSensor,
                    SensorManager.SENSOR_DELAY_FASTEST);
        } else {
            Toast.makeText(mContext, "No LINEAR_ACCELERATION detected", Toast.LENGTH_SHORT).show();
        }

        String filePath = String.valueOf(getExternalFilesDir(null));
        mFileForDebug = new File(filePath, "sensorLogForDebug.csv");
        try {
            mFileForDebug.createNewFile();
            System.out.println("filePath = " + filePath);
            Toast.makeText(mContext, "**filePath = " + filePath, Toast.LENGTH_LONG).show();
        } catch (IOException e) {
            e.printStackTrace();
        }

        mDataTimeCmpRecorderMag.setFileName("mDataTimeCmpRecorderMag.csv");
        mDataTimeCmpRecorderMag.writeData("mag,");
        mDataTimeCmpRecorderAcc.setFileName("mDataTimeCmpRecorderAcc.csv");
        mDataTimeCmpRecorderAcc.writeData("acc,");
        mDataTimeCmpRecorderGyr.setFileName("mDataTimeCmpRecorderGyr.csv");
        mDataTimeCmpRecorderGyr.writeData("gyr,");
        mDataTimeCmpRecorderLin.setFileName("mDataTimeCmpRecorderLin.csv");
        mDataTimeCmpRecorderLin.writeData("lin,");
        mDataSyncCmpRecorderSen.setFileName("mDataSyncCmpRecorderSen.csv");
        mDataSyncCmpRecorderSen.writeData("sync,");
    }

    private void startMotionSensors() {

        mSensorDataProcessor = new SensorDataProcessor();
        mSensorDataProcessor.start();

        mSensorEventListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {

                DataStructure mdCurrent = new DataStructure();

                mdCurrent.data[0] = event.values[0];
                mdCurrent.data[1] = event.values[1];
                mdCurrent.data[2] = event.values[2];
                // timestamp Unit:ms
                mdCurrent.timestamp = (long) (event.timestamp / 1e7);
                // If the sensor data is unreliable return
                if(event.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE) {
                    return;
                }
                switch (event.sensor.getType()) {
                    case Sensor.TYPE_GYROSCOPE:
                        mdCurrent.type = DataStructure.DATA_TYPE_GYR;
                        mDataTimeCmpRecorderGyr.writeData("" + mdCurrent.timestamp + ",");
                        break;
                    case Sensor.TYPE_ACCELEROMETER:
                        mdCurrent.type = DataStructure.DATA_TYPE_ACC;
                        mDataTimeCmpRecorderAcc.writeData("" + mdCurrent.timestamp + ",");
                        break;
                    case Sensor.TYPE_MAGNETIC_FIELD:
                        mdCurrent.type = DataStructure.DATA_TYPE_MAG;
                        mDataTimeCmpRecorderMag.writeData("" + mdCurrent.timestamp + ",");
                        break;
                    case Sensor.TYPE_LINEAR_ACCELERATION:
                        mdCurrent.type = DataStructure.DATA_TYPE_LIN;
                        mDataTimeCmpRecorderLin.writeData("" + mdCurrent.timestamp + ",");
                        break;
                    default:
                        break;
                }
                mMdSynchronizer.push(mdCurrent);

                if(mMdSynchronizer.synchronize(mMotionData)) {
                    mDataSyncCmpRecorderSen.writeData("" + (long)mMotionData[12] + ",");

                    if(mIsFirstIn) {
                        mLastTimestamp = mMotionData[SENSOR_NUM - 1];
                        mIsFirstIn = false;
                    } else {
                        double currentTimestamp = mMotionData[12];
                        double timeSpan = currentTimestamp - mLastTimestamp;
                        // At here, the motion data update as time span
//                        System.out.println("timeSpan = " + timeSpan);
                        mMotionData[SENSOR_NUM - 1] = timeSpan;
                        mSensorHandler.obtainMessage(0, mMotionData).sendToTarget();

                        mLastTimestamp = currentTimestamp;
                    }
                }
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {

            }
        };
    }

    class SensorDataProcessor extends Thread {
        @Override
        public void run() {
            Looper.prepare();
            mSensorHandler = new Handler() {
                public void handleMessage(Message msg) {
                    super.handleMessage(msg);
                    switch (msg.what) {
                        case 0:
                            double[] srcData = (double[])msg.obj;
                            final double[] dstData = new double[SENSOR_NUM];
                            // Process sensor data here
                            final boolean ret = procSensorRawData(srcData, dstData);
                            mTimeCounter4UpdateUI++;

                            // Update the UI
                            if (mTimeCounter4UpdateUI > (1000 / UPDATE_UI_FREQUENCY) / 5) {

                                mTimeCounter4UpdateUI = 0;
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        if(ret) {
                                            mTvAccX.setText("" + dstData[0]);
                                            mTvAccY.setText("" + dstData[1]);
                                            mTvAccZ.setText("" + dstData[2]);
                                            mTvVelX.setText("" + dstData[3]);
                                            mTvVelY.setText("" + dstData[4]);
                                            mTvVelZ.setText("" + dstData[5]);
                                            mTvPathX.setText("" + dstData[6]);
                                            mTvPathY.setText("" + dstData[7]);
                                            mTvPathZ.setText("" + dstData[8]);
                                        }
                                        mTvRoll.setText("" + dstData[9]);
                                        mTvPitch.setText("" + dstData[10]);
                                        mTvYaw.setText("" + dstData[11]);
                                    }
                                });
                            }

                            break;
                    }
                }
            };
            Looper.loop();
        }

        // <editor-fold desc="Sensor private parameters BLOCK">
        private double[] mSpeed = {0f, 0f, 0f};
        private double[] mShift = {0f, 0f, 0f};
        private double[] mAngle = {0f, 0f, 0f};
        private volatile Euler mLastAccPose = new Euler();
        private volatile boolean mIsStart = false;
        private volatile boolean mIsNeedCalibrate = false;
        private volatile boolean mIsFirst = true;
        private volatile double[] mAccOffset = {0f, 0f, 0f};
        private volatile double[] mGyrOffset = {0f, 0f, 0f};
        private volatile int mInitCounter = 0;

        private volatile MedianFilter mMfMag = new MedianFilter();
        private volatile MedianFilter mMfGyr = new MedianFilter();
        private volatile MedianFilter mMfAcc = new MedianFilter();
        private volatile MedianFilter mMfLin = new MedianFilter();

        private volatile AverageFilter mAfMag = new AverageFilter();
        private volatile AverageFilter mAfGyr = new AverageFilter();
        private volatile AverageFilter mAfAcc = new AverageFilter();
        private volatile AverageFilter mAfLin = new AverageFilter();

        private volatile FilterDispatcher mFusion = null;

        private volatile ComplementaryFilter mCFilter = new ComplementaryFilter();

        private volatile DebugDataWriter mDdwDstData = new DebugDataWriter(mContext);
        private volatile DebugDataWriter mDdwFusionData = new DebugDataWriter(mContext);
        // </editor-fold>

        /**
         * Deal with the raw sensor data to pose data
         * @param motionData [in] raw sensor data
         * structure of motion data:
         * +--------------------+-----------------+------------------+------------------+----------+
         * |   magnitude data   | gyroscope data  |acceleration data |  liner-acc data  |time span |
         * +--------------------+-----------------+------------------+------------------+----------+
         * |   3 * double size  | 3 * double size | 3 * double size  | 3 * double size  |1 * double|
         * +--------------------+-----------------+------------------+------------------+----------+
         * @param motionData    [out]
         * @param dstData [out] mobile data for debug
         */
        private boolean procSensorRawData(final double[] motionData, double[] dstData) {
            boolean isInDebugMode = true;
            if(isInDebugMode) {

                if (this.mIsFirst) {
                    mDdwFusionData.setFileName("CmpSensorFusion.csv");
                    String hdLine = "mag_x,mag_y,mag_z,gyro_x,gyro_y,"
                            + "gyro_z,acc_x,acc_y," +
                            "acc_z,lin_x,lin_y,lin_z\r\n";
                    mDdwFusionData.setDataSetHeader(hdLine);
                    mIsFirst = false;
                }

                // Write data for future compute  the pose
                for (int i = 0; i < 12; i++) {
                    dstData[i] = motionData[i];
                }

                // Write info for debug
                String tcLine = String.valueOf(dstData[0]) + ","
                        + String.valueOf(dstData[1]) + ","
                        + String.valueOf(dstData[2]) + ","
                        + String.valueOf(dstData[3]) + ","
                        + String.valueOf(dstData[4]) + ","
                        + String.valueOf(dstData[5]) + ","
                        + String.valueOf(dstData[6]) + ","
                        + String.valueOf(dstData[7]) + ","
                        + String.valueOf(dstData[8]) + ","
                        + String.valueOf(dstData[9]) + ","
                        + String.valueOf(dstData[10]) + ","
                        + String.valueOf(dstData[11]) + "\r\n";
                mDdwFusionData.writeData(tcLine);
            } else {


                // <editor-fold desc="Median filter here first to remove the weird data BLOCK">
                mMfMag.addData(new Point3(motionData[0], motionData[1], motionData[2]));
                mMfGyr.addData(new Point3(motionData[3], motionData[4], motionData[5]));
                mMfAcc.addData(new Point3(motionData[6], motionData[7], motionData[8]));
                mMfLin.addData(new Point3(motionData[9], motionData[10], motionData[11]));
                boolean bMag = mMfMag.process();
                boolean bGyr = mMfGyr.process();
                boolean bAcc = mMfAcc.process();
                boolean bLin = mMfLin.process();
                if (bMag && bGyr && bAcc && bLin) {
                    motionData[0] = mMfMag.getResult().x;
                    motionData[1] = mMfMag.getResult().y;
                    motionData[2] = mMfMag.getResult().z;
                    motionData[3] = mMfGyr.getResult().x;
                    motionData[4] = mMfGyr.getResult().y;
                    motionData[5] = mMfGyr.getResult().z;
                    motionData[6] = mMfAcc.getResult().x;
                    motionData[7] = mMfAcc.getResult().y;
                    motionData[8] = mMfAcc.getResult().z;
                    motionData[9] = mMfLin.getResult().x;
                    motionData[10] = mMfLin.getResult().y;
                    motionData[11] = mMfLin.getResult().z;
                } else {
                    return false;
                }
                // </editor-fold>

                // <editor-fold desc="Average filter here for smoothing the data BLOCK">
                mAfMag.addData(new Point3(motionData[0], motionData[1], motionData[2]));
                mAfGyr.addData(new Point3(motionData[3], motionData[4], motionData[5]));
                mAfAcc.addData(new Point3(motionData[6], motionData[7], motionData[8]));
                mAfLin.addData(new Point3(motionData[9], motionData[10], motionData[11]));
                bMag = mAfMag.process();
                bGyr = mAfGyr.process();
                bAcc = mAfAcc.process();
                bLin = mAfLin.process();
                if (bMag && bGyr && bAcc && bLin) {
                    motionData[0] = mAfMag.getResult().x;
                    motionData[1] = mAfMag.getResult().y;
                    motionData[2] = mAfMag.getResult().z;
                    motionData[3] = mAfGyr.getResult().x;
                    motionData[4] = mAfGyr.getResult().y;
                    motionData[5] = mAfGyr.getResult().z;
                    motionData[6] = mAfAcc.getResult().x;
                    motionData[7] = mAfAcc.getResult().y;
                    motionData[8] = mAfAcc.getResult().z;
                    motionData[9] = mAfLin.getResult().x;
                    motionData[10] = mAfLin.getResult().y;
                    motionData[11] = mAfLin.getResult().z;
                } else {
                    return false;
                }
                // </editor-fold>

                if (!this.mIsStart) {
                    // Record the accelerate sensor data in order to get the average for init
                    mAccOffset[0] += motionData[9];
                    mAccOffset[1] += motionData[10];
                    mAccOffset[2] += motionData[11];

                    mGyrOffset[0] += motionData[3];
                    mGyrOffset[1] += motionData[4];
                    mGyrOffset[2] += motionData[5];

                    mInitCounter++;

                    return false;
                } else {
                    if (this.mIsFirst) {
                        if (mInitCounter > 0) {

                            // Compute the offset of accelerate sensor data
                            mAccOffset[0] /= (double) mInitCounter;
                            mAccOffset[1] /= (double) mInitCounter;
                            mAccOffset[2] /= (double) mInitCounter;

                            // Compute the offset of gyroscope sensor data
                            mGyrOffset[0] /= (double) mInitCounter;
                            mGyrOffset[1] /= (double) mInitCounter;
                            mGyrOffset[2] /= (double) mInitCounter;

                            System.out.println("##################################");
                            System.out.println("mAccOffset[0] = " + mAccOffset[0]);
                            System.out.println("mAccOffset[1] = " + mAccOffset[1]);
                            System.out.println("mAccOffset[2] = " + mAccOffset[2]);
                            System.out.println("mGyrOffset[0] = " + mGyrOffset[0]);
                            System.out.println("mGyrOffset[1] = " + mGyrOffset[1]);
                            System.out.println("mGyrOffset[2] = " + mGyrOffset[2]);
                            System.out.println("mInitCounter = " + mInitCounter);
                        }
                    }
                    // The frequency: 100Hz
                    double t = motionData[12] * 0.01f;
                    // For exception to restrict the data for accuracy
                    if (t > 0.011 || t < 0.009)
                        return false;

                    motionData[9] -= mAccOffset[0];
                    motionData[10] -= mAccOffset[1];
                    motionData[11] -= mAccOffset[2];

                    // Step1: use magnitude data & accelerate data to compute the
                    //        absolute pose data with noise-some think as Gaussian
                    // TODO: 2017/3/5 Maybe convert this to c++ code for efficiency

//                System.out.println("t = " + t);
//                System.out.println();

                    double Ax = motionData[6] - mAccOffset[0];
                    double Ay = motionData[7] - mAccOffset[1];
                    double Az = motionData[8] - mAccOffset[2];
                    final double Ex = motionData[0];
                    final double Ey = motionData[1];
                    final double Ez = motionData[2];
                    double Hx = Ey * Az - Ez * Ay;
                    double Hy = Ez * Ax - Ex * Az;
                    double Hz = Ex * Ay - Ey * Ax;
                    final double normH = Math.sqrt(Hx * Hx + Hy * Hy + Hz * Hz);
                    if (normH < 0.1f) {
                        // device is close to free fall (or in space?), or close to
                        // magnetic north pole. Typical values are  > 100.
                        return false;
                    }
                    final double invH = 1.0f / normH;
                    Hx *= invH;
                    Hy *= invH;
                    Hz *= invH;
                    final double invA = 1.0f / (float) Math.sqrt(Ax * Ax + Ay * Ay + Az * Az);
                    Ax *= invA;
                    Ay *= invA;
                    Az *= invA;
                    final double Mx = Ay * Hz - Az * Hy;
                    final double My = Az * Hx - Ax * Hz;
                    final double Mz = Ax * Hy - Ay * Hx;

                    // R is the rotation matrix for understanding
                    //  /  R[0][0]   R[0][1]   R[0][2]  \
                    //  |  R[1][0]   R[1][1]   R[1][2]  |
                    //  \  R[2][0]   R[2][2]   R[2][2]  /
                    double[][] R = new double[3][3];
                    R[0][0] = Hx;
                    R[0][1] = Hy;
                    R[0][2] = Hz;
                    R[1][0] = Mx;
                    R[1][1] = My;
                    R[1][2] = Mz;
                    R[2][0] = Ax;
                    R[2][1] = Ay;
                    R[2][2] = Az;

                    // Azimuth, angle of rotation about the -z axis. likewise: yaw
                    // When facing north, this angle is 0, when facing south, this angle is &pi;.
                    // Likewise, when facing east, this angle is &pi;/2, and when facing west,
                    // this angle is -&pi;/2. The range of values is -&pi; to &pi;.
                    double azimuth = (float) Math.atan2(R[0][1], R[1][1]);
                    // Pitch, angle of rotation about the x axis.
                    // This value represents the angle between a plane parallel to the device's
                    // screen and a plane parallel to the ground. Assuming that the bottom
                    // edge of the device faces the user and that the screen is face-up, tilting
                    // the top edge of the device toward the ground creates a positive pitch angle.
                    // The range of values is -&pi; to &pi;
                    double pitch = (float) Math.asin(-R[2][1]);
                    // Roll, angle of rotation about the y axis. This value represents the angle
                    // between a plane perpendicular to the device's screen and a plane perpendicular
                    // to the ground. Assuming that the bottom edge of the device faces the user
                    // and that the screen is face-up, tilting the left edge of the device toward
                    // the ground creates a positive roll angle.
                    // The range of values is -&pi;/2 to &pi;/2.
                    double roll = (float) Math.atan2(-R[2][0], R[2][2]);

                    // Brief it so just like as below:
                    azimuth = Math.atan2(Hy, My);
                    pitch = Math.asin(-Ay);
                    roll = Math.atan2(-Ax, Az);

                    //Step2: Use gyroscope data to integrate the pose data
                    //       with high temporary accuracy but error grows with time
                    // TODO: 2017/3/5 Make clear the position
                    if (this.mIsFirst) {
                        mLastAccPose.pitch = mAngle[0] = roll;
                        mLastAccPose.roll = mAngle[1] = pitch;
                        mLastAccPose.yaw = mAngle[2] = azimuth;

                        System.out.println("Init the sensors successfully.");

                        // write debug information for debug
                        mDdwDstData.setFileName("MotionPoseData.csv");
                        String hdLine = "acc_x,acc_y,acc_z,vel_x,vel_y,vel_z,"
                                + "path_x,path_y,path_z,roll,pitch,yaw\r\n";
                        mDdwDstData.setDataSetHeader(hdLine);
                        mDdwFusionData.setFileName("CmpSensorFusion.csv");
                        hdLine = "src1_roll,src1_pitch,src1_yaw,src2_roll,src2_pitch,"
                                + "src2_yaw,src1_roll_check,src1_pitch_check," +
                                "src1_yaw_check,result_roll,result_pitch,result_yaw\r\n";
                        mDdwFusionData.setDataSetHeader(hdLine);

                        mCFilter.setLeftRate(0.95);

                        // Disable the first init
                        this.mIsFirst = false;
                        this.mIsNeedCalibrate = false;
                    } else {
                        double dGyroX = (motionData[3] - mGyrOffset[0]) * t;
                        double dGyroY = (motionData[4] - mGyrOffset[1]) * t;
                        // yaw and azimuth have different direction between acc_mag & gyro
                        double dGyroZ = -(motionData[5] - mGyrOffset[2]) * t;

                        // Deal with gimbal lock
                        if (mLastAccPose.pitch > 2 && pitch < -2 ||
                                mLastAccPose.pitch < -2 && pitch > 2) {
                            mAngle[0] = pitch;
                        }
                        if (mLastAccPose.roll > 2 && roll < -2 ||
                                mLastAccPose.roll < -2 && roll > 2) {
                            mAngle[1] = roll;
                        }
                        if (mLastAccPose.yaw > 2 && azimuth < -2 ||
                                mLastAccPose.yaw < -2 && azimuth > 2) {
                            mAngle[2] = azimuth;
                        }

                        mAngle[0] += dGyroX;
                        mAngle[1] += dGyroY;
                        mAngle[2] += dGyroZ;

                        Euler src1 = new Euler(mAngle[0], mAngle[1], mAngle[2]);
                        Euler src2 = new Euler(roll, pitch, azimuth);

                        mCFilter.step(src1, src2);

                        Euler pose = mCFilter.getResultEuler();

                        dstData[0] = roll;
                        dstData[1] = pitch;
                        dstData[2] = azimuth;

                        dstData[3] = mAngle[0];
                        dstData[4] = mAngle[1];
                        dstData[5] = mAngle[2];

                        dstData[9]  = pose.roll;
                        dstData[10] = pose.pitch;
                        dstData[11] = pose.yaw;

                        mLastAccPose.roll = mAngle[0] = pose.roll;
                        mLastAccPose.pitch = mAngle[1] = pose.pitch;
                        mLastAccPose.yaw = mAngle[2] = pose.yaw;

                        // Write info for debug
                        String tcLine = String.valueOf(dstData[0]) + ","
                                + String.valueOf(dstData[1]) + ","
                                + String.valueOf(dstData[2]) + ","
                                + String.valueOf(dstData[3]) + ","
                                + String.valueOf(dstData[4]) + ","
                                + String.valueOf(dstData[5]) + ","
                                + String.valueOf(dstData[6]) + ","
                                + String.valueOf(dstData[7]) + ","
                                + String.valueOf(dstData[8]) + ","
                                + String.valueOf(dstData[9]) + ","
                                + String.valueOf(dstData[10]) + ","
                                + String.valueOf(dstData[11]) + "\r\n";
                        mDdwFusionData.writeData(tcLine);

                        // Compute the path there
                        boolean ret;
                        ret = calcTheWorld(pose, new Point3(motionData[9],
                                motionData[10], motionData[11]), t);
                        if (ret == true) {
                            dstData[0] = mAccW[0];
                            dstData[1] = mAccW[1];
                            dstData[2] = mAccW[2];
                            dstData[3] = mWorldSpeed[0];
                            dstData[4] = mWorldSpeed[1];
                            dstData[5] = mWorldSpeed[2];
                            dstData[6] = mWorldPath[0];
                            dstData[7] = mWorldPath[1];
                            dstData[8] = mWorldPath[2];
                            dstData[9]  = pose.roll;
                            dstData[10] = pose.pitch;
                            dstData[11] = pose.yaw;
                            return true;
                        }
                    }
                    return false;
                }
            }
            return true;
        }


        private volatile double[] mWorldSpeed = {0d, 0d, 0d};
        private volatile double[] mWorldPath = {0d, 0d, 0d};
        private final double mPoseThreshold    = 0.01;
        private final double mLineAccThreshold = 0.03;
        private volatile Euler mLastPose = new Euler();
        private volatile double[] mAccW = new double[4];
        private boolean calcTheWorld(Euler pose, Point3 acc, double t) {
            double dRoll  = pose.roll  - mLastPose.roll;
            double dPitch = pose.pitch - mLastPose.pitch;
            double dYaw   = pose.yaw   - mLastPose.yaw;

            // update the last pose
            mLastPose.roll = pose.roll;
            mLastPose.pitch = pose.pitch;
            mLastPose.yaw = pose.yaw;

            if(dRoll > mPoseThreshold || dPitch > mPoseThreshold || dYaw > mPoseThreshold) {

                double normAcc = Math.sqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
                // TODO: 2017/3/25 Adjust here
                if(true || normAcc > mLineAccThreshold) {

                    double[] r = pose.toRotationMatrix();
                    Mat R = new Mat(4, 4, CvType.CV_64FC1);
                    R.put(0, 0, r);

                    double[] accB_t = {acc.x, acc.y, acc.z, 1};
                    Mat accB = new Mat(4, 1, CvType.CV_64FC1);
                    accB.put(0, 0, accB_t);

                    // Body to World
                    Mat accW = new Mat();
                    Core.gemm(R.inv(), accB, 1.0, new Mat(), 0, accW);

                    accW.get(0, 0, mAccW);

                    mWorldSpeed[0] += mAccW[0] * t;
                    mWorldSpeed[1] += mAccW[1] * t;
                    mWorldSpeed[2] += mAccW[2] * t;

                    mWorldPath[0] += mWorldSpeed[0] * t;
                    mWorldPath[1] += mWorldSpeed[1] * t;
                    mWorldPath[2] += mWorldSpeed[2] * t;

                    String tcLine = String.valueOf(mAccW[0]) + ","
                            + String.valueOf(mAccW[1]) + ","
                            + String.valueOf(mAccW[2]) + ","
                            + String.valueOf(mWorldSpeed[0]) + ","
                            + String.valueOf(mWorldSpeed[1]) + ","
                            + String.valueOf(mWorldSpeed[2]) + ","
                            + String.valueOf(mWorldPath[0]) + ","
                            + String.valueOf(mWorldPath[1]) + ","
                            + String.valueOf(mWorldPath[2]) + ","
                            + String.valueOf(pose.roll) + ","
                            + String.valueOf(pose.pitch) + ","
                            + String.valueOf(pose.yaw) + "\r\n";

                    mDdwDstData.writeData(tcLine);

                } else {
                    // Take as silent
                    return false;
                }
            } else {
                mWorldSpeed[0] = mWorldSpeed[1] = mWorldSpeed[2] = 0;
                return false;
            }

            return true;
        }

        private void init() {
            this.mSpeed[0] = this.mSpeed[1] = this.mSpeed[2] = 0f;
            this.mShift[0] = this.mShift[1] = this.mShift[2] = 0f;
            this.mAngle[0] = this.mAngle[1] = this.mAngle[2] = 0f;
            this.mAccOffset[0] = this.mAccOffset[1] = this.mAccOffset[2] = 0f;
            this.mGyrOffset[0] = this.mGyrOffset[1] = this.mGyrOffset[2] = 0f;
            mInitCounter = 0;
            mIsStart = false;
            mIsFirst = true;
        }

        public SensorDataProcessor() {
            init();
        }

        public void startProc() {
            if(mFusion == null) {

                mFusion = new FilterDispatcher();
                this.mIsStart = true;
            }
        }

        public void calibrate() {
            mWorldSpeed[0] = mWorldSpeed[1] = mWorldSpeed[2] = 0d;
            mIsNeedCalibrate = true;
        }

        public void pauseProc() {
            if(mFusion != null) {
                mFusion = null;
            }
        }
    }
    // </editor-fold>

    // <editor-fold desc="Interactions with UI BLOCK">
    private void initLayouts() {
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        FloatingActionButton fab = (FloatingActionButton) findViewById(R.id.fab);
        fab.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                mSensorDataProcessor.calibrate();
                mSensorDataProcessor.startProc();
            }
        });

        // Example of a call to a native method
        mTvAccX = (TextView) findViewById(R.id.tvAccX);
        mTvAccY = (TextView) findViewById(R.id.tvAccY);
        mTvAccZ = (TextView) findViewById(R.id.tvAccZ);
        mTvVelX = (TextView) findViewById(R.id.tvVelX);
        mTvVelY = (TextView) findViewById(R.id.tvVelY);
        mTvVelZ = (TextView) findViewById(R.id.tvVelZ);
        mTvPathX = (TextView) findViewById(R.id.tvPathX);
        mTvPathY = (TextView) findViewById(R.id.tvPathY);
        mTvPathZ = (TextView) findViewById(R.id.tvPathZ);

        mTvRoll  = (TextView) findViewById(R.id.tvRoll);
        mTvPitch = (TextView) findViewById(R.id.tvPitch);
        mTvYaw   = (TextView) findViewById(R.id.tvYaw);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    // </editor-fold>

}
