package com.example.sensor_fusion;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.Menu;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import fastFourierTransform.Complex;
import fastFourierTransform.FFT;

@SuppressLint("InlinedApi")
public class MainActivity extends Activity implements SensorEventListener {

	private TextView status, stepCounterFSM, maximumFrequency;
	private Button start, stop, step;
	private SensorManager mSensorManager;
	private Sensor accelerometer, gyroscope, compass;
	private ArrayList<SensorData> accelData, gyroData, compassData,
			stepTimeStamps;
	private final float ALPHA = 0.8f;
	private float[] accelSensorVals, gyroSensorVals, compassSensorVals;
	private boolean isStarted = false, isStep = false;
	private static double Thr = 1.75, Pos_Peek_Thr = 2, Neg_Peek_Thr = 1,
			Neg_Thr = 1.2;
	private static int state = 0, FSMStepCount = 0;
	private static ArrayList<Complex> timeDomainData;

	protected float[] lowPassFilter(float[] input, float[] output) {
		if (output == null)
			return input;
		for (int i = 0; i < input.length; i++)
			output[i] = output[i] + ALPHA * (input[i] - output[i]);
		return output;
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		if (isStarted) {
			Sensor sensor = event.sensor;
			if (sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {

				accelSensorVals = lowPassFilter(event.values.clone(),
						accelSensorVals);
				SensorData data = new SensorData(System.currentTimeMillis(),
						accelSensorVals[0], accelSensorVals[1],
						accelSensorVals[2]);
				timeDomainData.add(new Complex(data.mag, 0));
				FSMAlgorithm(data.mag);
				accelData.add(data);
				if (isStep) {
					stepTimeStamps.add(data);
					isStep = false;
				}
			} else if (sensor.getType() == Sensor.TYPE_GYROSCOPE) {
				gyroSensorVals = lowPassFilter(event.values.clone(),
						gyroSensorVals);
				gyroData.add(new SensorData(System.currentTimeMillis(),
						gyroSensorVals[0], gyroSensorVals[1], gyroSensorVals[2]));
			} else if (sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
				compassSensorVals = lowPassFilter(event.values.clone(),
						compassSensorVals);
				compassData.add(new SensorData(System.currentTimeMillis(),
						compassSensorVals[0], compassSensorVals[1],
						compassSensorVals[2]));
			}
		}
	}

	private void FSMAlgorithm(double input) {
		if (state == 0) {
			if (input > Thr)
				state = 1;
		} else if (state == 1) {
			if (input > Thr && input < Pos_Peek_Thr)
				state = 1;
			else if (input > Pos_Peek_Thr)
				state = 2;
			else if (input < Thr)
				state = 4;
		} else if (state == 2) {
			if (input > Pos_Peek_Thr)
				state = 2;
			else if (input < Neg_Peek_Thr)
				state = 3;
		} else if (state == 3) {
			if (input < Neg_Peek_Thr)
				state = 3;
			else
				state = 5;
		} else if (state == 4) {
			if (input > Thr)
				state = 1;
			else
				state = 0;
		} else if (state == 5) {
			if (input > Neg_Peek_Thr && input < Neg_Thr)
				state = 5;
			else if (input < Neg_Thr)
				state = 6;
			else if (input < Neg_Peek_Thr)
				state = 3;
		} else if (state == 6) {
			FSMStepCount++;
			stepCounterFSM.setText(Integer.toString(FSMStepCount));
			if (input > Thr)
				state = 1;
			else
				state = 0;
		}
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		status = (TextView) findViewById(R.id.status);
		status.setText("Off");
		stepCounterFSM = (TextView) findViewById(R.id.stepCounterFSM);
		stepCounterFSM.setText(R.string._0);
		maximumFrequency = (TextView) findViewById(R.id.maximumFrequency);
		maximumFrequency.setText(R.string._0);

		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		if (mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION) != null
				&& mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null
				&& mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) != null)
			Toast.makeText(getApplicationContext(), "Sensors are found",
					Toast.LENGTH_SHORT).show();
		else {
			Toast.makeText(getApplicationContext(),
					"Error: Some sensors are not found", Toast.LENGTH_LONG)
					.show();
			System.exit(0);
		}

		accelerometer = mSensorManager
				.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
		gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		compass = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

		start = (Button) findViewById(R.id.start);
		start.setOnClickListener(new View.OnClickListener() {
			public void onClick(View v) {
				if (!isStarted) {
					FSMStepCount = 0;
					state = 0;
					stepCounterFSM.setText(Integer.toString(0));
					initializeData();
					status.setText("On");
					isStarted = true;
				}
			}
		});
		stop = (Button) findViewById(R.id.stop);
		stop.setOnClickListener(new View.OnClickListener() {
			public void onClick(View v) {
				if (isStarted) {
					state = 0;
					writeToFile();
					status.setText("Off");
					isStarted = false;
				}
			}
		});
		step = (Button) findViewById(R.id.step);
		step.setOnClickListener(new View.OnClickListener() {
			public void onClick(View v) {
				if (isStarted) {
					isStep = true;
				}
			}
		});
	}

	private void initializeData() {
		timeDomainData = new ArrayList<Complex>();
		stepTimeStamps = new ArrayList<SensorData>();
		accelSensorVals = new float[3];
		gyroSensorVals = new float[3];
		compassSensorVals = new float[3];
		accelData = new ArrayList<SensorData>();
		gyroData = new ArrayList<SensorData>();
		compassData = new ArrayList<SensorData>();

		mSensorManager.registerListener(this, accelerometer,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, gyroscope,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, compass,
				SensorManager.SENSOR_DELAY_FASTEST);
	}

	private void writeToFile() {
		try {
			File myFile = new File(getString(R.string._sdcard_measurments_txt));
			myFile.createNewFile();
			FileOutputStream fOut = new FileOutputStream(myFile);
			OutputStreamWriter myOutWriter = new OutputStreamWriter(fOut);

			if (writeData(myOutWriter, stepTimeStamps, "Ste")
					&& writeData(myOutWriter, accelData, "Accelerometer")) {
				int siz = timeDomainData.size();
				int realSize = siz;
				while ((siz & (siz - 1)) != 0) {
					timeDomainData.add(new Complex(0, 0));
					siz++;
				}
				Complex[] x = new Complex[timeDomainData.size()];
				x = timeDomainData.toArray(x);
				Complex[] y = FFT.fft(x);
				double maxFrequency = Double.MIN_VALUE;
				for (int i = 0; i < realSize; i++) {
					myOutWriter.append(y[i].abs() + "\n");
					if (y[i].abs() > maxFrequency)
						maxFrequency = y[i].abs();
				}
				myOutWriter.append("******************\n\n\n\n\n\n\n\n\n\n");
				myOutWriter
						.append("Maximum Frequency = " + maxFrequency + "\n");
				maximumFrequency.setText("" + maxFrequency);
				myOutWriter.close();
				fOut.close();

				Toast.makeText(getApplicationContext(), "Successfully written",
						Toast.LENGTH_SHORT).show();
				mSensorManager.unregisterListener(this);
			}
		} catch (Exception e) {
			Toast.makeText(getApplicationContext(), "Can not construct a file",
					Toast.LENGTH_SHORT).show();
			mSensorManager.unregisterListener(this);
		}
	}

	private boolean writeData(OutputStreamWriter myOutWriter,
			ArrayList<SensorData> data, String sensorName) {
		try {
			// myOutWriter.append(sensorName + " Data\n");
			for (int i = 0; i < data.size(); i++)
				myOutWriter.append(data.get(i).timestamp + "\n");
			myOutWriter.append("------------------\n\n\n\n\n\n\n\n\n\n");
			for (int i = 0; i < data.size(); i++)
				myOutWriter.append(data.get(i).mag + "\n");
			myOutWriter.append("******************\n\n\n\n\n\n\n\n\n\n");
			return true;
		} catch (Exception ex) {
			Toast.makeText(getApplicationContext(), "Writing Failed",
					Toast.LENGTH_SHORT).show();
			mSensorManager.unregisterListener(this);
			return false;
		}
	}

	@Override
	protected void onResume() {
		super.onResume();
	}

	@Override
	protected void onPause() {
		super.onPause();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

	@Override
	public void onAccuracyChanged(Sensor arg0, int arg1) {
	}

}
