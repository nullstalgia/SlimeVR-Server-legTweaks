package dev.slimevr.vr.processor.skeleton;

import com.jme3.math.Vector3f;

import dev.slimevr.vr.trackers.TrackerStatus;

/* Handels localizing the skeleton in 3d space when no 6dof device is present.
 * This is accomplished by using the foot state calculated by legtweaks. This of course
 * has the problem of the true position and predicted location drifting apart over time.
 */


public class Localizer {
	public static final int LEFT_LOCKED = 1;
	public static final int RIGHT_LOCKED = 2;
	public static final int BOTH_LOCKED = 3;
	public static final int NONE_LOCKED = 4;

	// hyperparameters
	private static final float MAX_PERCENTAGE_OVER_THRESHOLD = 3.0f;

	// classes
	private HumanSkeleton skeleton;
	private LegTweaks legTweaks;

	// state
	private boolean enabled = false;
	private LegTweakBuffer bufCur;
	private LegTweakBuffer bufPrev;
	private Vector3f targetPos = new Vector3f();
	private int plantedFoot = NONE_LOCKED;
	private boolean footInit = false;

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

		// get the movment of the skeleton by foot travel
		Vector3f footTravel = getPlantedFootTravel();

		// get the movment of the skeleton by the previus COM velocity
		Vector3f COMTravel = getCOMTravel();

		// TODO use COM velocity and acceleration to allow for jumping and other
		// movements where the foot may temporarily be off the ground

		// if neither foot is on the floor level accelerate towards the ground
		// until one foot is on the ground
		float yShift = keepOnGround();


		// if the foot is not initialized, use the COM velocity as a guess of
		// movement for this frame
		if (!footInit) {
			footTravel = COMTravel;
		}

		// add the y shift to the foot travel
		footTravel.setY(footTravel.y - yShift);

		// update the skeletons root position
		updateSkeletonPos(footTravel);

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
			return NONE_LOCKED;

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

	// get the foot or feet that are planted
	// also sets the planted foot, foot init, and target pos variables
	private Vector3f getPlantedFootTravel() {
		if (bufPrev == null)
			return new Vector3f();

		// get the foot that is planted
		int foot = getPlantedFoot();

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
		} else if (foot == BOTH_LOCKED) {
			// if both feet are planted, average the travel to minimize error
			Vector3f footloc = getAverageFootTravel();
			updateTargetPos(footloc, foot);
			return getFootTravel(footloc);
		}


		return new Vector3f();
	}

	// get the travel of a foot over a frame
	private Vector3f getFootTravel(Vector3f loc) {
		return loc.subtract(targetPos);
	}

	// get the average foot travel of the two feet
	private Vector3f getAverageFootTravel() {
		return getFootTravel(
			bufCur.getLeftFootPosition(null)
		)
			.addLocal(
				getFootTravel(
					bufCur.getRightFootPosition(null)
				)
			)
			.divideLocal(2.0f);
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

	// when neither foot is planted, guess the travel
	private Vector3f getCOMTravel() {
		// for now return the velocity of the center of mass
		// TODO implement a better guess
		return bufCur
			.getCenterOfMassVelociy(null)
			.divide(legTweaks.getBuffer().getTimeDelta());

	}

	// makes sure the skeleton stays on the ground unless a jump is detected
	// TODO use actual physics
	private float keepOnGround() {
		if (
			bufCur.getLeftFootPosition(null).y > bufCur.getLeftFloorLevel()
				&& bufCur.getRightFootPosition(null).y > bufCur.getRightFloorLevel()
		)
			return -0.01f;
		if (
			bufCur.getLeftFootPosition(null).y < bufCur.getLeftFloorLevel()
				|| bufCur.getRightFootPosition(null).y < bufCur.getRightFloorLevel()
		)
			return 0.01f;
		return 0.00f;
	}

	// update the hmd position
	private void updateSkeletonPos(Vector3f travel) {
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
			);
	}
}
