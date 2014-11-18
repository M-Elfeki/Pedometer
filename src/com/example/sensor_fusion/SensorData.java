package com.example.sensor_fusion;

class SensorData {
	protected long timestamp;
	protected double x, y, z, mag;

	protected SensorData(long timestamp, double x, double y, double z) {
		this.timestamp = timestamp;
		this.x = x;
		this.y = y;
		this.z = z;
		this.mag = Math.sqrt(Math.pow(this.x, 2)+Math.pow(this.y, 2)+Math.pow(this.z, 2))-0.25;
	}
}