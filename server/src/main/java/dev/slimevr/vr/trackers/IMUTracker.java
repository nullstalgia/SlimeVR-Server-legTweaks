package dev.slimevr.vr.trackers;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import dev.slimevr.VRServer;
import dev.slimevr.config.TrackerConfig;
import dev.slimevr.filtering.CircularArrayList;
import dev.slimevr.filtering.QuaternionMovingAverage;
import dev.slimevr.filtering.TrackerFilters;
import dev.slimevr.vr.Device;
import dev.slimevr.vr.trackers.udp.TrackersUDPServer;
import dev.slimevr.vr.trackers.udp.UDPDevice;
import io.eiren.util.BufferedTimer;
import io.eiren.util.collections.FastList;

import java.util.Optional;


public class IMUTracker
	implements Tracker, TrackerWithTPS, TrackerWithBattery, TrackerWithWireless,
	TrackerWithFiltering {

	public static final float MAX_MAG_CORRECTION_ACCURACY = 5 * FastMath.RAD_TO_DEG;

	// public final Vector3f gyroVector = new Vector3f();
	public final Vector3f accelVector = new Vector3f();
	// public final Vector3f magVector = new Vector3f();
	public final Quaternion rotQuaternion = new Quaternion();
	public final Quaternion rotMagQuaternion = new Quaternion();
	public final Quaternion mountAdjust = new Quaternion();
	public final UDPDevice device;
	public final int trackerNum;
	public final Vector3f rotVector = new Vector3f();
	public final Quaternion gyroFix = new Quaternion();
	public final Quaternion attachmentFix = new Quaternion();
	public final Quaternion mountRotFix = new Quaternion();
	public final Quaternion yawFix = new Quaternion();
	protected final Quaternion correction = new Quaternion();
	protected final int trackerId;
	protected final String name;
	protected final String descriptiveName;
	protected final TrackersUDPServer server;
	protected final VRServer vrserver;
	public int calibrationStatus = 0;
	public int magCalibrationStatus = 0;
	public float magnetometerAccuracy = 0;
	private String customName;
	public boolean hasNewCorrectionData = false;
	private int ping = -1;
	private int signalStrength = -1;
	public float temperature = 0;
	public TrackerPosition bodyPosition = null;
	protected Quaternion mounting = null;
	protected TrackerStatus status = TrackerStatus.OK;
	protected float confidence = 0;
	protected float batteryVoltage = 0;
	protected float batteryLevel = 0;
	protected boolean magnetometerCalibrated = false;
	protected BufferedTimer timer = new BufferedTimer(1f);
	protected QuaternionMovingAverage movingAverage;
	protected boolean allowDriftCompensation = true;
	protected boolean compensateDrift = false;
	protected float driftAmount;
	protected static long DRIFT_COOLDOWN_MS = 30000;
	protected final Quaternion averagedDriftQuat = new Quaternion();
	private final FastList<Float> driftWeights = new FastList<>();
	private final static Quaternion rotationSinceReset = new Quaternion();
	private CircularArrayList<Quaternion> driftQuats;
	private CircularArrayList<Long> driftTimes;
	private long totalDriftTime;
	private long driftSince;
	private long timeAtLastReset;

	public IMUTracker(
		UDPDevice device,
		int trackerId,
		int trackerNum,
		String name,
		String descriptiveName,
		TrackersUDPServer server,
		VRServer vrserver
	) {
		this.device = device;
		this.trackerNum = trackerNum;
		this.name = name;
		this.server = server;
		this.trackerId = trackerId;
		this.descriptiveName = descriptiveName;
		this.vrserver = vrserver;

		if (vrserver != null) {
			setFiltering(
				vrserver.getConfigManager().getVrConfig().getFilters().enumGetType(),
				vrserver.getConfigManager().getVrConfig().getFilters().getAmount()
			);
			setDriftCompensationSettings(
				vrserver.getConfigManager().getVrConfig().getDriftCompensation().getEnabled(),
				vrserver.getConfigManager().getVrConfig().getDriftCompensation().getAmount(),
				vrserver.getConfigManager().getVrConfig().getDriftCompensation().getMaxResets()
			);
		}
	}

	@Override
	public void writeConfig(TrackerConfig config) {
		config.setDesignation(bodyPosition == null ? null : bodyPosition.designation);
		config
			.setMountingOrientation(
				mounting != null ? mounting : new Quaternion().fromAngles(0, FastMath.PI, 0)
			);
		config.setCustomName(customName);
		config.setAllowDriftCompensation(allowDriftCompensation);
	}

	@Override
	public void readConfig(TrackerConfig config) {
		// Loading a config is an act of user editing, therefore it shouldn't
		// be allowed if editing is not allowed
		if (userEditable()) {
			setCustomName(config.getCustomName());

			if (config.getMountingOrientation() != null) {
				mounting = config.getMountingOrientation();
				mountAdjust.set(config.getMountingOrientation());
			} else {
				mountAdjust.loadIdentity();
			}
			Optional<TrackerPosition> trackerPosition = TrackerPosition
				.getByDesignation(config.getDesignation());
			if (trackerPosition.isEmpty()) {
				bodyPosition = null;
			} else {
				bodyPosition = trackerPosition.get();
			}
			if (config.getAllowDriftCompensation() == null) {
				// If value didn't exist, default to true and save
				allowDriftCompensation = true;
				vrserver
					.getConfigManager()
					.getVrConfig()
					.getTracker(this.get())
					.setAllowDriftCompensation(true);
				vrserver.getConfigManager().saveConfig();
			} else {
				allowDriftCompensation = config.getAllowDriftCompensation();
			}
		}
	}

	public Quaternion getMountingOrientation() {
		return mounting;
	}

	public void setMountingOrientation(Quaternion mr) {
		mounting = mr;
		if (mounting != null) {
			mountAdjust.set(mounting);
		} else {
			mountAdjust.loadIdentity();
		}
	}

	public boolean getAllowDriftCompensation() {
		return allowDriftCompensation;
	}

	public void setAllowDriftCompensation(boolean allowDriftCompensation) {
		this.allowDriftCompensation = allowDriftCompensation;
	}

	@Override
	public void setFiltering(TrackerFilters type, float amount) {
		if (type != null) {
			switch (type) {
				case SMOOTHING:
				case PREDICTION:
					movingAverage = new QuaternionMovingAverage(
						type,
						amount,
						rotQuaternion
					);
					break;
				case NONE:
				default:
					movingAverage = null;
					break;
			}
		} else {
			movingAverage = null;
		}
	}

	public void setDriftCompensationSettings(boolean enabled, float amount, int maxResets) {
		compensateDrift = enabled;
		driftAmount = amount;
		if (enabled) {
			if (driftQuats == null || maxResets != driftQuats.capacity()) {
				driftQuats = new CircularArrayList<>(maxResets);
				driftTimes = new CircularArrayList<>(maxResets);
			}
		} else {
			driftQuats = null;
			driftTimes = null;
		}
	}

	@Override
	public void tick() {
		if (magnetometerCalibrated && hasNewCorrectionData) {
			hasNewCorrectionData = false;
			if (magnetometerAccuracy <= MAX_MAG_CORRECTION_ACCURACY) {
				// Adjust gyro rotation to match magnetometer rotation only if
				// magnetometer
				// accuracy is within the parameters
				calculateLiveMagnetometerCorrection();
			}
		}

		// Update moving average (that way movement is smooth even if TPS is
		// stuttering)
		if (movingAverage != null) {
			movingAverage.update();
		}
	}

	@Override
	public String getName() {
		return this.name;
	}

	@Override
	public boolean getPosition(Vector3f store) {
		store.set(0, 0, 0);
		return false;
	}

	@Override
	public boolean getAcceleration(Vector3f store) {
		store.set(accelVector);
		return true;
	}

	@Override
	public boolean getRotation(Quaternion store) {
		if (movingAverage != null) {
			store.set(movingAverage.getFilteredQuaternion());
		} else {
			store.set(rotQuaternion);
		}
		// correction.mult(store, store); // Correction is not used now to
		// prevent accidental errors while debugging other things
		store.multLocal(mountAdjust);
		adjustInternal(store);
		if ((compensateDrift && allowDriftCompensation) && totalDriftTime > 0) {
			store
				.slerpLocal(
					store.mult(averagedDriftQuat),
					driftAmount
						* ((float) (System.currentTimeMillis() - driftSince)
							/ totalDriftTime)
				);
		}
		return true;
	}

	@Override
	public boolean getRawRotation(Quaternion store) {
		store.set(rotQuaternion);
		return true;
	}

	public Quaternion getAdjustedRawRotation() {
		Quaternion rot = new Quaternion(rotQuaternion);
		// correction.mult(store, store); // Correction is not used now to
		// prevent accidental errors while debugging other things
		rot.multLocal(mountAdjust);
		adjustInternal(rot);
		return rot;
	}

	private Quaternion getMountedAdjustedRotation() {
		Quaternion rot = new Quaternion(rotQuaternion);
		rot.multLocal(mountAdjust);
		if ((compensateDrift && allowDriftCompensation) && totalDriftTime > 0) {
			rot
				.slerpLocal(
					rot.mult(averagedDriftQuat),
					driftAmount
						* ((float) (System.currentTimeMillis() - driftSince)
							/ totalDriftTime)
				);
		}
		return rot;
	}

	public void getCorrection(Quaternion store) {
		store.set(correction);
	}

	@Override
	public TrackerStatus getStatus() {
		return status;
	}

	public void setStatus(TrackerStatus status) {
		this.status = status;
	}

	@Override
	public float getTPS() {
		return timer.getAverageFPS();
	}

	@Override
	public void dataTick() {
		timer.update();

		// Add new rotation to moving average
		if (movingAverage != null) {
			movingAverage.addQuaternion(rotQuaternion.clone());
		}
	}

	@Override
	public float getConfidenceLevel() {
		return confidence;
	}

	public void setConfidence(float newConf) {
		this.confidence = newConf;
	}

	@Override
	public float getBatteryLevel() {
		return batteryLevel;
	}

	public void setBatteryLevel(float level) {
		this.batteryLevel = level;
	}

	@Override
	public float getBatteryVoltage() {
		return batteryVoltage;
	}

	public void setBatteryVoltage(float voltage) {
		this.batteryVoltage = voltage;
	}

	/**
	 * Reset the tracker so that its current rotation is counted as (0, HMD Yaw,
	 * 0). This allows the tracker to be strapped to body at any pitch and roll.
	 */
	@Override
	public void resetFull(Quaternion reference) {
		Quaternion rot = getAdjustedRawRotation();
		fixGyroscope(getMountedAdjustedRotation());
		fixAttachment(getMountedAdjustedRotation());
		fixYaw(reference);
		calibrateMag();
		calculateDrift(rot);
	}

	/**
	 * Reset the tracker so that it's current yaw rotation is counted as <HMD
	 * Yaw>. This allows the tracker to have yaw independent of the HMD. Tracker
	 * should still report yaw as if it was mounted facing HMD, mounting
	 * position should be corrected in the source. Also aligns gyro magnetometer
	 * if it's reliable.
	 */
	@Override
	public void resetYaw(Quaternion reference) {
		Quaternion rot = getAdjustedRawRotation();
		fixYaw(reference);
		calibrateMag();
		calculateDrift(rot);
	}

	protected void adjustInternal(Quaternion store) {
		gyroFix.mult(store, store);
		store.multLocal(attachmentFix);
		store.multLocal(mountRotFix);
		yawFix.mult(store, store);
	}

	private void fixGyroscope(Quaternion sensorRotation) {
		sensorRotation.fromAngles(0, sensorRotation.getYaw(), 0);
		gyroFix.set(sensorRotation).inverseLocal();
	}

	private void fixAttachment(Quaternion sensorRotation) {
		gyroFix.mult(sensorRotation, sensorRotation);
		attachmentFix.set(sensorRotation).inverseLocal();
	}

	@Override
	public void resetMounting(boolean reverseYaw) {
		// Get the current calibrated rotation
		Quaternion buffer = getMountedAdjustedRotation();
		gyroFix.mult(buffer, buffer);
		buffer.multLocal(attachmentFix);

		// Reset the vector for the rotation to point straight up
		rotVector.set(0f, 1f, 0f);
		// Rotate the vector by the quat, then flatten and normalize the vector
		buffer.multLocal(rotVector).setY(0f).normalizeLocal();

		// Calculate the yaw angle using tan
		// Just use an angle offset of zero for unsolvable circumstances
		float yawAngle = FastMath.isApproxZero(rotVector.x) && FastMath.isApproxZero(rotVector.z)
			? 0f
			: FastMath.atan2(rotVector.x, rotVector.z);

		// Make an adjustment quaternion from the angle
		buffer.fromAngles(0f, reverseYaw ? yawAngle : yawAngle - FastMath.PI, 0f);

		Quaternion lastRotAdjust = mountRotFix.clone();
		mountRotFix.set(buffer);

		// Get the difference from the last adjustment
		buffer.multLocal(lastRotAdjust.inverseLocal());
		// Apply the yaw rotation difference to the yaw fix quaternion
		yawFix.multLocal(buffer.inverseLocal());
	}

	private void fixYaw(Quaternion reference) {
		// Use only yaw HMD rotation
		Quaternion targetRotation = reference.clone();
		targetRotation.fromAngles(0, targetRotation.getYaw(), 0);

		Quaternion sensorRotation = getMountedAdjustedRotation();
		gyroFix.mult(sensorRotation, sensorRotation);
		sensorRotation.multLocal(attachmentFix);
		sensorRotation.multLocal(mountRotFix);

		sensorRotation.fromAngles(0, sensorRotation.getYaw(), 0);

		yawFix.set(sensorRotation).inverseLocal().multLocal(targetRotation);
	}

	private void calibrateMag() {
		if (magCalibrationStatus >= CalibrationAccuracy.HIGH.status) {
			magnetometerCalibrated = true;
			// During calibration set correction to match magnetometer readings
			// TODO : Correct only yaw
			correction.set(rotQuaternion).inverseLocal().multLocal(rotMagQuaternion);
		}
	}

	/**
	 * Calculates 1 since last reset and store the data related to it in
	 * driftQuat, timeAtLastReset and timeForLastReset
	 */
	synchronized public void calculateDrift(Quaternion beforeQuat) {
		if (compensateDrift && allowDriftCompensation) {
			Quaternion rotQuat = getAdjustedRawRotation();

			if (
				driftSince > 0
					&& System.currentTimeMillis() - timeAtLastReset > DRIFT_COOLDOWN_MS
			) {
				// Check and remove from lists to keep them under the reset
				// limit
				if (driftQuats.size() == driftQuats.capacity()) {
					driftQuats.removeLast();
					driftTimes.removeLast();
				}

				// Add new drift quaternion
				driftQuats
					.add(
						new Quaternion()
							.fromAngles(
								0f,
								rotQuat.mult(beforeQuat.inverse()).getYaw(),
								0f
							)
					);

				// Add drift time to total
				driftTimes.add(System.currentTimeMillis() - driftSince);
				totalDriftTime = 0;
				for (Long time : driftTimes) {
					totalDriftTime += time;
				}

				// Calculate drift Quaternions' weights
				driftWeights.clear();
				for (Long time : driftTimes) {
					driftWeights.add(((float) time) / ((float) totalDriftTime));
				}
				// Make it so recent Quaternions weigh more
				for (int i = driftWeights.size() - 1; i > 0; i--) {
					// Add some of i-1's value to i
					driftWeights
						.set(
							i,
							driftWeights.get(i) + (driftWeights.get(i - 1) / driftWeights.size())
						);
					// Remove the value that was added to i from i-1
					driftWeights
						.set(
							i - 1,
							driftWeights.get(i - 1)
								- (driftWeights.get(i - 1) / driftWeights.size())
						);
				}

				// Set final averaged drift Quaternion
				averagedDriftQuat.fromAveragedQuaternions(driftQuats, driftWeights);

				// Save tracker rotation and current time
				rotationSinceReset.set(rotQuat.mult(beforeQuat.inverse()));
				timeAtLastReset = System.currentTimeMillis();
			} else if (
				System.currentTimeMillis() - timeAtLastReset < DRIFT_COOLDOWN_MS
					&& driftQuats.size() > 0
			) {
				// Replace latest drift quaternion
				rotationSinceReset.multLocal(beforeQuat.mult(rotQuat.inverse()));
				driftQuats
					.set(
						driftQuats.size() - 1,
						new Quaternion()
							.fromAngles(
								0f,
								rotationSinceReset.inverse().getYaw(),
								0f
							)
					);

				// Add drift time to total
				driftTimes
					.set(
						driftTimes.size() - 1,
						driftTimes.getLatest() + System.currentTimeMillis() - driftSince
					);
				totalDriftTime = 0;
				for (Long time : driftTimes) {
					totalDriftTime += time;
				}

				// Calculate drift Quaternions' weights
				driftWeights.clear();
				for (Long time : driftTimes) {
					driftWeights.add(((float) time) / ((float) totalDriftTime));
				}
				// Make it so recent Quaternions weigh more
				for (int i = driftWeights.size() - 1; i > 0; i--) {
					driftWeights
						.set(
							i,
							driftWeights.get(i) + (driftWeights.get(i - 1) / driftWeights.size())
						);
					driftWeights
						.set(
							i - 1,
							driftWeights.get(i - 1)
								- (driftWeights.get(i - 1) / driftWeights.size())
						);
				}

				// Set final averaged drift Quaternion
				averagedDriftQuat.fromAveragedQuaternions(driftQuats, driftWeights);
			} else {
				timeAtLastReset = System.currentTimeMillis();
			}

			driftSince = System.currentTimeMillis();
		}
	}

	/**
	 * Calculate correction between normal and magnetometer readings up to
	 * accuracy threshold
	 */
	protected void calculateLiveMagnetometerCorrection() {
		// TODO Magic, correct only yaw
		// TODO Print "jump" length when correcting if it's more than 1 degree
	}

	@Override
	public TrackerPosition getBodyPosition() {
		return bodyPosition;
	}

	@Override
	public void setBodyPosition(TrackerPosition position) {
		this.bodyPosition = position;
	}

	@Override
	public boolean userEditable() {
		return true;
	}

	@Override
	public boolean hasRotation() {
		return true;
	}

	@Override
	public boolean hasPosition() {
		return false;
	}

	@Override
	public boolean isComputed() {
		return false;
	}

	@Override
	public int getTrackerId() {
		return this.trackerId;
	}

	@Override
	public int getTrackerNum() {
		return this.trackerNum;
	}

	@Override
	public Device getDevice() {
		return this.device;
	}

	@Override
	public String getDisplayName() {
		return "IMU Tracker #" + getTrackerId();
	}

	@Override
	public String getCustomName() {
		return customName;
	}

	public void setCustomName(String customName) {
		this.customName = customName;
	}

	@Override
	public Tracker get() {
		return this;
	}

	@Override
	public int getPing() {
		return this.ping;
	}

	@Override
	public int getSignalStrength() {
		return this.signalStrength;
	}

	public void setPing(int ping) {
		this.ping = ping;
	}

	public void setSignalStrength(int signalStrength) {
		this.signalStrength = signalStrength;
	}

	public enum CalibrationAccuracy {

		UNRELIABLE(0), LOW(1), MEDIUM(2), HIGH(3),;

		private static final CalibrationAccuracy[] byStatus = new CalibrationAccuracy[4];

		static {
			for (CalibrationAccuracy ca : values())
				byStatus[ca.status] = ca;
		}

		public final int status;

		CalibrationAccuracy(int status) {
			this.status = status;
		}

		public static CalibrationAccuracy getByStatus(int status) {
			if (status < 0 || status > 3)
				return null;
			return byStatus[status];
		}
	}
}
