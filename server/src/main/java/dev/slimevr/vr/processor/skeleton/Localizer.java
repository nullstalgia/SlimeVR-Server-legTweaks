package dev.slimevr.vr.processor.skeleton;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

import dev.slimevr.vr.trackers.TrackerStatus;

/* Handels localizing the skeleton in 3d space when no 6dof device is present.
 * This is accomplished by using the foot state calculated by legtweaks. This of course
 * has the problem of the true position and predicted location drifting apart over time.
 */


public class Localizer {
	// constants
	public static final int LEFT_LOCKED = 1;
	public static final int RIGHT_LOCKED = 2;
	public static final int BOTH_LOCKED = 3;
	public static final int NONE_LOCKED = 4;
	public static final int FOLLOW_FOOT = 5;
	public static final int FOLLOW_COM = 6;

	// hyperparameters
	private static final float MAX_PERCENTAGE_OVER_THRESHOLD = 6.0f;
	private static final float GROUND_SNAP_PERCENT = 0.1f;
	private static final int FRAMES_BETWEEN__VEL_CHECK_SUM = 1000;

	// convenience variables
	private HumanSkeleton skeleton;
	private LegTweaks legTweaks;
	private float uncorrectedFloor = 0.0f;
	private LegTweakBuffer bufCur;
	private LegTweakBuffer bufPrev;

	// state
	private boolean enabled = false;
	private Vector3f targetPos = new Vector3f();
	private Vector3f targetCOM = new Vector3f();
	private Vector3f predictedCOMVelocity = new Vector3f();
	private int plantedFoot = BOTH_LOCKED;
	private int worldReference = FOLLOW_FOOT;
	private boolean footInit = false;

	// efficient avg velocity calculation variables
	private Vector3f summedCOMVelocity = new Vector3f();
	private int numCOMVelocitySamples = 0;
	private int framesSinceCheckSum = 0;


	public Localizer(HumanSkeleton skeleton) {
		this.skeleton = skeleton;
		this.legTweaks = skeleton.legTweaks;
	}

	public boolean getEnabled() {
		return enabled;
	}

	public void setEnabled(boolean enabled) {
		this.enabled = enabled;

		// change some state variables in legtweaks when active
		if (enabled) {
			legTweaks.setFloorLevel(0.0f);
			this.uncorrectedFloor = 0.0f - LegTweaks.FLOOR_CALIBRATION_OFFSET;
		}
		legTweaks.setLocalizerMode(enabled);
	}

	public void update() {
		if (!enabled)
			return;

		// check if there is a 6dof device
		if (skeleton.hmdTracker.getStatus() != TrackerStatus.DISCONNECTED)
			return;

		// set the buffer current and previus for easy access
		bufCur = legTweaks.getBuffer();
		bufPrev = bufCur.getParent();

		// get momvent metric to be used for this frame
		worldReference = getWorldReference();

		// init the final travel vector
		Vector3f finalTravel = new Vector3f();

		// get the movment of the skeleton by foot travel
		Vector3f footTravel = getPlantedFootTravel();
		// when using the foot as a reference, it is important to
		// keep the foot firmly planted on the ground
		footTravel.y += getCorrectiveMovment();

		// get the movment of the skeleton by the previus COM velocity
		Vector3f comTravel = getCOMTravel();

		// if the foot is not initialized, do not use it
		if (!footInit) {
			footTravel = new Vector3f(0, 0, 0);
		}

		// if the average velocity is not initialized, do not use it
		if (numCOMVelocitySamples < LegTweakBuffer.BUFFER_LEN) {
			comTravel = new Vector3f(0, 0, 0);
		}

		// update the final travel vector
		if (worldReference == FOLLOW_FOOT) {
			finalTravel = footTravel;
		} else if (worldReference == FOLLOW_COM) {
			finalTravel = comTravel;
		} else {
			new Vector3f(0, 0, 0);
		}

		// update the skeletons root position
		updateSkeletonPos(finalTravel);

		// debug printing (takes over terminal output)
		printState();

	}

	// reset the localizer
	public void reset() {
		skeleton.hmdNode.localTransform.getTranslation().set(Vector3f.ZERO);

		// when localizing without a 6 dof device we choose the floor level
		// 0 happens to be an easy number to use
		legTweaks.setFloorLevel(0.0f);
	}

	private int getPlantedFoot() {
		if (bufPrev == null)
			return BOTH_LOCKED;

		// return the foot planted if a certain state is active
		if (
			bufCur.getLeftLegState() == LegTweakBuffer.LOCKED
				&& bufCur.getRightLegState() == LegTweakBuffer.LOCKED
		)
			return BOTH_LOCKED;

		// the integer value of the state has very low false positives so listen
		// to it
		if (bufCur.getLeftLegState() == LegTweakBuffer.LOCKED)
			return LEFT_LOCKED;

		if (bufCur.getRightLegState() == LegTweakBuffer.LOCKED)
			return RIGHT_LOCKED;

		// if the state is not locked, use the numerical state to determine a
		// foot to follow
		if (
			bufCur.getLeftLegNumericalState() < bufCur.getRightLegNumericalState()
				&& bufCur.getLeftLegNumericalState() < MAX_PERCENTAGE_OVER_THRESHOLD
		)
			return LEFT_LOCKED;

		if (
			bufCur.getRightLegNumericalState() < bufCur.getLeftLegNumericalState()
				&& bufCur.getRightLegNumericalState() < MAX_PERCENTAGE_OVER_THRESHOLD
		)
			return RIGHT_LOCKED;


		return NONE_LOCKED;
	}

	// check if the foot that is planted is actually planted
	private int getWorldReference() {
		// TODO
		// note to self: using the COM should be a last resort
		return FOLLOW_FOOT;
	}

	// get the foot or feet that are planted
	// also sets the planted foot, foot init, and target pos variables
	private Vector3f getPlantedFootTravel() {
		if (bufPrev == null)
			return new Vector3f();

		// get the foot that is planted
		int foot = getPlantedFoot();

		// both feet being locked is treated as the locked foot not changing
		if (foot == BOTH_LOCKED) {
			// if neither foot was locked, default to the left
			// this should be unlikely to happen
			if (plantedFoot == NONE_LOCKED)
				plantedFoot = LEFT_LOCKED;
			foot = plantedFoot;
		}

		if (foot == LEFT_LOCKED) {
			Vector3f footLoc = bufCur.getLeftFootPosition(null);
			updateTargetPos(footLoc, foot);
			return getFootTravel(footLoc);

		} else if (foot == RIGHT_LOCKED) {
			Vector3f footLoc = bufCur.getRightFootPosition(null);
			updateTargetPos(footLoc, foot);
			return getFootTravel(
				footLoc
			);
		}

		return new Vector3f();
	}

	// get the travel of a foot over a frame
	private Vector3f getFootTravel(Vector3f loc) {
		return loc.subtract(targetPos);
	}

	// update the target position of the foot
	private void updateTargetPos(Vector3f loc, int foot) {
		if (foot == plantedFoot) {
			footInit = true;
		} else {
			footInit = false;
			plantedFoot = foot;
			targetPos.set(loc);
		}
	}

	// when neither foot is planted, guess the travel of the COM from its last
	// positions
	private Vector3f getCOMTravel() {

		// update COM attributes
		updateTargetCOM();

		// return the distance needed to move the COM to the target COM
		Vector3f dist = bufCur.getCenterOfMass(null).subtract(targetCOM);

		// prevent overshoots
		if (bufCur.getLeftFootPosition(null).y - dist.y < uncorrectedFloor)
			dist.y = bufCur.getLeftFootPosition(null).y - uncorrectedFloor;
		if (bufCur.getRightFootPosition(null).y - dist.y < uncorrectedFloor)
			dist.y = bufCur.getRightFootPosition(null).y - uncorrectedFloor;

		return dist;
	}

	// update the COM target position
	private void updateTargetCOM() {
		// set the target COM to the current COM
		// and the predicted COM velocity to the current COM velocity

		if (worldReference == FOLLOW_FOOT) {
			predictedCOMVelocity.set(getAverageCOMVelocity());
			bufCur.getCenterOfMass(targetCOM);
			return;
		}

		// if the foot is not locked, use the data that was collected above to
		// predict the COM location
		predictedCOMVelocity.addLocal(LegTweakBuffer.GRAVITY.divide(bufCur.getTimeDelta()));
		targetCOM.addLocal(predictedCOMVelocity);
	}

	// makes sure the skeleton stays on the ground and doesn't fall through it
	private float getCorrectiveMovment() {
		// first get the distance to the ground and the current COM velocity
		float distToGround;
		if (plantedFoot == LEFT_LOCKED) {
			distToGround = bufCur.getLeftFootPosition(null).y - uncorrectedFloor;
		} else if (plantedFoot == RIGHT_LOCKED) {
			distToGround = bufCur.getRightFootPosition(null).y - uncorrectedFloor;
		} else {
			// if no foot is planted return the average of the two feet
			distToGround = (bufCur.getLeftFootPosition(null).y
				- uncorrectedFloor
				+ bufCur.getRightFootPosition(null).y
				- uncorrectedFloor) / 2.0f;
		}

		// if the foot is below the ground, return the distance to the ground
		// to imidiatly correct
		if (distToGround < 0.0f)
			return distToGround;

		// if the foot is above the ground, return GROUND_SNAP_PERCENT of the
		// distance to the ground to correct
		return distToGround * GROUND_SNAP_PERCENT;
	}

	// gets the average COM velocity over the last LegTweakBuffer.BUFFER_LEN
	// frames
	private Vector3f getAverageCOMVelocity() {
		LegTweakBuffer buf = bufCur;
		int i = 0;

		while (buf != null) {
			// if this buffers velocity is not yet in the sum add it
			if (i == 0) {
				summedCOMVelocity.addLocal(buf.getCenterOfMassVelociy(null));
				numCOMVelocitySamples++;
			}
			// remove the oldest velocity from the sum
			if (i == LegTweakBuffer.BUFFER_LEN) {
				summedCOMVelocity.subtractLocal(buf.getCenterOfMassVelociy(null));
				numCOMVelocitySamples--;
			}

			// update the state of the loop
			buf = buf.getParent();
			i++;
		}
		// return the average velocity
		return summedCOMVelocity.divide(numCOMVelocitySamples);
	}

	// update the hmd position and rotation
	private void updateSkeletonPos(Vector3f travel) {
		Quaternion rot = new Quaternion();
		skeleton.hmdTracker.getRotation(rot);
		skeleton.hmdNode.localTransform.getTranslation().subtractLocal(travel);
	}

	private void printState() {
		String left;
		String right;
		Vector3f headPos = skeleton.hmdNode.localTransform.getTranslation().clone();
		left = (plantedFoot == LEFT_LOCKED || plantedFoot == BOTH_LOCKED)
			? "\033[92m L"
			: "\033[91m L";
		right = (plantedFoot == RIGHT_LOCKED || plantedFoot == BOTH_LOCKED)
			? "\033[92m R"
			: "\033[91m R";

		System.out
			.println(
				String.format("%.2f", headPos.x)
					+ "\t"
					+ String.format("%.2f", headPos.y)
					+ "\t"
					+ String.format("%.2f", headPos.z)
					+ "\t"
					+ left
					+ "\t"
					+ right
					+ "\033[0m"
					+ "\t"
					+ worldReference
					+ "\t"
					+ predictedCOMVelocity
					+ "\t"
					+ bufCur.getCenterOfMassVelociy(null)
			);
	}
}
