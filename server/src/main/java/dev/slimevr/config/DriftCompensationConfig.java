package dev.slimevr.config;

import dev.slimevr.Main;
import dev.slimevr.vr.trackers.IMUTracker;
import dev.slimevr.vr.trackers.Tracker;


public class DriftCompensationConfig {

	// Is drift compensation enabled
	private boolean enabled = false;

	// Amount of drift compensation applied
	private float amount = 0.8f;

	// Max resets for the calculated average drift
	private int maxResets = 6;

	public DriftCompensationConfig() {
	}

	public void updateTrackersDriftCompensation() {
		for (Tracker t : Main.getVrServer().getAllTrackers()) {
			Tracker tracker = t.get();
			if (tracker instanceof IMUTracker imuTracker) {
				imuTracker
					.setDriftCompensationSettings(
						getEnabled(),
						getAmount(),
						getMaxResets()
					);
			}
		}
	}


	public boolean getEnabled() {
		return enabled;
	}

	public void setEnabled(boolean enabled) {
		this.enabled = enabled;
	}

	public float getAmount() {
		return amount;
	}

	public void setAmount(float amount) {
		this.amount = amount;
	}

	public int getMaxResets() {
		return maxResets;
	}

	public void setMaxResets(int maxResets) {
		this.maxResets = maxResets;
	}
}
