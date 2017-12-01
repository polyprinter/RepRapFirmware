/*
 * DriveMovement.cpp
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#include "DriveMovement.h"
#include "DDA.h"
#include "Move.h"
#include "RepRap.h"
#include "Libraries/Math/Isqrt.h"
#include "Kinematics/LinearDeltaKinematics.h"

// Static members

DriveMovement *DriveMovement::freeList = nullptr;
int DriveMovement::numFree = 0;
int DriveMovement::minFree = 0;

void DriveMovement::InitialAllocate(unsigned int num)
{
	while (num != 0)
	{
		freeList = new DriveMovement(freeList);
		++numFree;
		--num;
	}
	ResetMinFree();
}

DriveMovement *DriveMovement::Allocate(size_t drive, DMState st)
{
	DriveMovement * const dm = freeList;
	if (dm != nullptr)
	{
		freeList = dm->nextDM;
		--numFree;
		if (numFree < minFree)
		{
			minFree = numFree;
		}
		dm->nextDM = nullptr;
		dm->drive = (uint8_t)drive;
		dm->state = st;
	}
	return dm;
}

// Constructors
DriveMovement::DriveMovement(DriveMovement *next) : nextDM(next)
{
}

// Non static members

// Prepare this DM for a Cartesian axis move
void DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params)
{
	const float stepsPerMm = (float)totalSteps/dda.totalDistance;
	mp.cart.twoCsquaredTimesMmPerStepDivA = roundU64((double)(DDA::stepClockRateSquared * 2)/((double)stepsPerMm * (double)dda.acceleration));

	// Acceleration phase parameters
	mp.cart.accelStopStep = (uint32_t)(dda.accelDistance * stepsPerMm) + 1;
	mp.cart.compensationClocks = mp.cart.accelCompensationClocks = 0;

	// Constant speed phase parameters
	mp.cart.mmPerStepTimesCKdivtopSpeed = roundU32(((float)((uint64_t)DDA::stepClockRate * K1))/(stepsPerMm * dda.topSpeed));

	// Deceleration phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)
	{
		mp.cart.decelStartStep = totalSteps + 1;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)(params.decelStartDistance * stepsPerMm) + 1;
		const uint64_t initialDecelSpeedTimesCdivASquared = isquare64(params.topSpeedTimesCdivA);
		twoDistanceToStopTimesCsquaredDivA = initialDecelSpeedTimesCdivASquared + roundU64((params.decelStartDistance * (DDA::stepClockRateSquared * 2))/dda.acceleration);
	}

	// No reverse phase
	reverseStartStep = totalSteps + 1;
	mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
}

// Prepare this DM for a Delta axis move
void DriveMovement::PrepareDeltaAxis(const DDA& dda, const PrepParams& params)
{
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float A = params.initialX - params.dparams->GetTowerX(drive);
	const float B = params.initialY - params.dparams->GetTowerY(drive);
	const float aAplusbB = A * dda.directionVector[X_AXIS] + B * dda.directionVector[Y_AXIS];
	const float dSquaredMinusAsquaredMinusBsquared = params.diagonalSquared - fsquare(A) - fsquare(B);
	const float h0MinusZ0 = sqrtf(dSquaredMinusAsquaredMinusBsquared);
	mp.delta.hmz0sK = roundS32(h0MinusZ0 * stepsPerMm * DriveMovement::K2);
	mp.delta.minusAaPlusBbTimesKs = -roundS32(aAplusbB * stepsPerMm * DriveMovement::K2);
	mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared = roundS64(dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm * DriveMovement::K2));
	mp.delta.twoCsquaredTimesMmPerStepDivA = roundU64((double)(2 * DDA::stepClockRateSquared)/((double)stepsPerMm * (double)dda.acceleration));

	// Calculate the distance at which we need to reverse direction.
	if (params.a2plusb2 <= 0.0)
	{
		// Pure Z movement. We can't use the main calculation because it divides by a2plusb2.
		direction = (dda.directionVector[Z_AXIS] >= 0.0);
		reverseStartStep = totalSteps + 1;
	}
	else
	{
		// The distance to reversal is the solution to a quadratic equation. One root corresponds to the carriages being below the bed,
		// the other root corresponds to the carriages being above the bed.
		const float drev = ((dda.directionVector[Z_AXIS] * sqrtf(params.a2b2D2 - fsquare(A * dda.directionVector[Y_AXIS] - B * dda.directionVector[X_AXIS])))
							- aAplusbB)/params.a2plusb2;
		if (drev > 0.0 && drev < dda.totalDistance)		// if the reversal point is within range
		{
			// Calculate how many steps we need to move up before reversing
			const float hrev = dda.directionVector[Z_AXIS] * drev + sqrtf(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - params.a2plusb2 * fsquare(drev));
			const int32_t numStepsUp = (int32_t)((hrev - h0MinusZ0) * stepsPerMm);

			// We may be almost at the peak height already, in which case we don't really have a reversal.
			if (numStepsUp < 1 || (direction && (uint32_t)numStepsUp <= totalSteps))
			{
				reverseStartStep = totalSteps + 1;
			}
			else
			{
				reverseStartStep = (uint32_t)numStepsUp + 1;

				// Correct the initial direction and the total number of steps
				if (direction)
				{
					// Net movement is up, so we will go up a bit and then down by a lesser amount
					totalSteps = (2 * numStepsUp) - totalSteps;
				}
				else
				{
					// Net movement is down, so we will go up first and then down by a greater amount
					direction = true;
					totalSteps = (2 * numStepsUp) + totalSteps;
				}
			}
		}
		else
		{
			reverseStartStep = totalSteps + 1;
		}
	}

	// Acceleration phase parameters
	mp.delta.accelStopDsK = roundU32(dda.accelDistance * stepsPerMm * K2);

	// Constant speed phase parameters
	mp.delta.mmPerStepTimesCKdivtopSpeed = roundU32(((float)DDA::stepClockRate * K1)/(stepsPerMm * dda.topSpeed));

	// Deceleration phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)
	{
		mp.delta.decelStartDsK = 0xFFFFFFFF;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		mp.delta.decelStartDsK = roundU32(params.decelStartDistance * stepsPerMm * K2);
		twoDistanceToStopTimesCsquaredDivA = isquare64(params.topSpeedTimesCdivA) + roundU64((params.decelStartDistance * (DDA::stepClockRateSquared * 2))/dda.acceleration);
	}
}

// Prepare this DM for an extruder move
void DriveMovement::PrepareExtruder(const DDA& dda, const PrepParams& params, bool doCompensation)
{
	const float dv = dda.directionVector[drive];
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive) * fabsf(dv);
	mp.cart.twoCsquaredTimesMmPerStepDivA = roundU64((double)(DDA::stepClockRateSquared * 2)/((double)stepsPerMm * (double)dda.acceleration));

	// Calculate the pressure advance parameter
	const float compensationTime_SEC_or_MMpMMpSEC = (doCompensation && dv > 0.0) ? reprap.GetPlatform().GetPressureAdvance(drive - reprap.GetGCodes().GetTotalAxes()) : 0.0;
	const uint32_t compensationClocks = lrintf(compensationTime_SEC_or_MMpMMpSEC * DDA::stepClockRate);
	mp.cart.accelCompensationClocks = roundU32(compensationTime_SEC_or_MMpMMpSEC * (float)DDA::stepClockRate * params.compFactor);

	// Calculate the net total step count to allow for compensation. It may be negative.
	// for the vectorial speed change across the whole move, determine the net additional distance from Advance (could be negative)
	const float compensationDistanceForEntireMoveSpeedChange_vect_mm = (dda.endSpeed - dda.startSpeed) * compensationTime_SEC_or_MMpMMpSEC;
	// and convert it into steps for this axis and correct the number of steps in the whole move
	const int32_t netSteps = (int32_t)(compensationDistanceForEntireMoveSpeedChange_vect_mm * stepsPerMm) + (int32_t)totalSteps;

	// Calculate the acceleration phase parameters
	// for the vectorial speed change in acceleration, determine the net additional distance from Advance
	const float accelCompensationDistance_vect_MM = compensationTime_SEC_or_MMpMMpSEC * (dda.topSpeed - dda.startSpeed); // distance = speed * time

	// Acceleration phase parameters
	mp.cart.accelStopStep = (uint32_t)((dda.accelDistance + accelCompensationDistance_vect_MM) * stepsPerMm) + 1;

	// Constant speed phase parameters
	mp.cart.mmPerStepTimesCKdivtopSpeed = (uint32_t)((float)((uint64_t)DDA::stepClockRate * K1)/(stepsPerMm * dda.topSpeed));


	// modify the clock count for cruise speed zero intercept by the time taken at cruise to travel the compensation distance added
	// - we are adding accelCompensationDistance_MM which takes accelCompensationDistance_MM/dda.topSpeed seconds
	const float compensationDistanceTimeAtTopSpeed_SEC = accelCompensationDistance_vect_MM / dda.topSpeed;  // vectorially
	int32_t compensationClocksDeltaToCruiseInterceptClocks = compensationDistanceTimeAtTopSpeed_SEC * DDA::stepClockRate;
	// since more advance means more steps, the intercept number becomes smaller when there is advance
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)params.accelClocksMinusAccelDistanceTimesCdivTopSpeed - compensationClocksDeltaToCruiseInterceptClocks;

	// Calculate the deceleration and reverse phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)		// if less than 1 deceleration step
	{
		totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
		mp.cart.decelStartStep = reverseStartStep = netSteps + 1;
		mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		// during cruise, we have kept a constant amount of advance
		mp.cart.decelStartStep = (uint32_t)((params.decelStartDistance + accelCompensationDistance_vect_MM) * stepsPerMm) + 1;
		const int32_t initialDecelSpeedTimesCdivA = (int32_t)params.topSpeedTimesCdivA - (int32_t)mp.cart.compensationClocks;	// signed because it may be negative and we square it
		const uint64_t initialDecelSpeedTimesCdivASquared = isquare64(initialDecelSpeedTimesCdivA);
		twoDistanceToStopTimesCsquaredDivA =
			initialDecelSpeedTimesCdivASquared + llrintf(((params.decelStartDistance + accelCompensationDistance) * (DDA::stepClockRateSquared * 2))/dda.acceleration);

		// Calculate the move distance to the point of zero speed, where reverse motion starts
		// the "speed" at the beginning of deceleration is actually instantaneously lower than it was at cruise.
		const float initialDecelSpeed = dda.topSpeed - dda.acceleration * compensationTime_SEC_or_MMpMMpSEC;
		const float reverseStartDistance = (initialDecelSpeed > 0.0)
												? fsquare(initialDecelSpeed)/(2 * dda.acceleration) + params.decelStartDistance
												: params.decelStartDistance;
		// Reverse phase parameters
		if (reverseStartDistance >= dda.totalDistance)
		{
			// No reverse phase
			totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
			reverseStartStep = netSteps + 1;
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		}
		else
		{
			reverseStartStep = (initialDecelSpeed < 0.0)
								? mp.cart.decelStartStep
								: (twoDistanceToStopTimesCsquaredDivA/mp.cart.twoCsquaredTimesMmPerStepDivA) + 1;
			// Because the step numbers are rounded down, we may sometimes get a situation in which netSteps = 1 and reverseStartStep = 1.
			// This would lead to totalSteps = -1, which must be avoided.
			const int32_t overallSteps = (int32_t)(2 * (reverseStartStep - 1)) - netSteps;
			if (overallSteps > 0)
			{
				totalSteps = (uint32_t)overallSteps;
				mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA =
						(int64_t)((2 * (reverseStartStep - 1)) * mp.cart.twoCsquaredTimesMmPerStepDivA) - (int64_t)twoDistanceToStopTimesCsquaredDivA;
			}
			else
			{
				totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
				reverseStartStep = totalSteps + 1;
				mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
			}
		}
	}
}

#ifdef POLYPRINTER
// TODO: POLYPRINTER - make sure that any improvements, above, are incorporated e.g. from v1.10 or so. Possibly, drop all this custom code if it does nothing.
// Prepare this DM for an extruder move with linear Advance
void DriveMovement::PrepareExtruderWithLinearAdvance(const DDA& dda, const PrepParams& params, bool doCompensation)
{
	const float dv = dda.directionVector[drive];
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive) * fabsf(dv);  // steps of this axis per mm of overall move distance
	mp.cart.twoCsquaredTimesMmPerStepDivA = (uint64_t)(((float)DDA::stepClockRate * (float)DDA::stepClockRate)/(stepsPerMm * dda.acceleration)) * 2;

	// Calculate the pressure advance parameter
	const float compensationTime_SEC_or_MMpMMpSEC = (doCompensation && dv > 0.0) ? reprap.GetPlatform().GetPressureAdvance(drive - reprap.GetGCodes().GetTotalAxes()) : 0.0;
	const uint32_t compensationClocks = (uint32_t)(compensationTime_SEC_or_MMpMMpSEC * DDA::stepClockRate);

	// Calculate the net total step count to allow for compensation. It may be negative.
	// Note that we add totalSteps in floating point mode, to round the number of steps down consistently
	// for the vectorial speed change across the whole move, determine the net additional distance from Advance (could be negative)
	const float compensationDistanceForEntireMoveSpeedChange_vect_mm = (dda.endSpeed - dda.startSpeed) * compensationTime_SEC_or_MMpMMpSEC;
	// and convert it into steps for this axis and correct the number of steps in the whole move
	const int32_t netSteps = (int32_t)(compensationDistanceForEntireMoveSpeedChange_vect_mm * stepsPerMm) + (int32_t)totalSteps;

	// Calculate the acceleration phase parameters
	// for the vectorial speed change in acceleration, determine the net additional distance from Advance
	const float accelCompensationDistance_vect_MM = compensationTime_SEC_or_MMpMMpSEC * (dda.topSpeed - dda.startSpeed); // distance = speed * time
	// alternatively: const float accelTime_SEC = (dda.topSpeed - dda.startSpeed) / dda.acceleration;
	//                const float compensationSpeed_vect_MMpSEC = compensationTime_SEC_or_MMpMMpSEC * dda.acceleration;	// work in vectorial terms even though the vectorial speed is unaffected
	//                const float accelCompensationDistance_vect_MM = compensationSpeed_vect_MMpSEC * accelTime_SEC; // still in vectorial terms

	// Acceleration phase parameters
	mp.cart.accelStopStep = (uint32_t)((dda.accelDistance + accelCompensationDistance_vect_MM) * stepsPerMm) + 1;
	// a bit trickier making sure all the derived elements are actually working together
	// since K is an amount of time, we know how much sooner acceleration would have needed to start to get to the initial velocity
	// - that is a constant amount of time, regardless of acceleration. (Low accel means little advance in that same time.)
	// - let's do everything necessary to fake it into simply executing the equivalent (modified) trapezoid
	startSpeedTimesCdivA = params.startSpeedTimesCdivA + compensationClocks;  // effective time taken to get to start speed with compensation speed added to it

	// Constant speed phase parameters
	mp.cart.mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)DDA::stepClockRate * K1)/(stepsPerMm * dda.topSpeed));
	// compfactor represents the fraction of the advance needed at a top speed that is actually going to be added, given an initial speed having reduced the need for it.
	// accelClocksMinusAccelDistanceTimesCdivTopSpeed is used to convert a step count in cruise into a time count for that step
	// by working back from the intercept at top speed. Take the time into the move that top speed would have taken, off the top.
	// so accelClocksMinusAccelDistanceTimesCdivTopSpeed is the difference in time between actual move start and the start if  it was all at top speed

	// modify the clock count for cruise speed zero intercept by the time taken at cruise to travel the compensation distance added
	// - we are adding accelCompensationDistance_MM which takes accelCompensationDistance_MM/dda.topSpeed seconds
	const float compensationDistanceTimeAtTopSpeed_SEC = accelCompensationDistance_vect_MM / dda.topSpeed;  // vectorially
	int32_t compensationClocksDeltaToCruiseInterceptClocks = compensationDistanceTimeAtTopSpeed_SEC * DDA::stepClockRate;
	// since more advance means more steps, the intercept number becomes smaller when there is advance
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)params.accelClocksMinusAccelDistanceTimesCdivTopSpeed - compensationClocksDeltaToCruiseInterceptClocks;

	//accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)params.accelClocksMinusAccelDistanceTimesCdivTopSpeed - (int32_t)(compensationClocks * params.compFactor);

	// Calculate the deceleration and reverse phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)		// if less than 1 deceleration step
	{
		totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
		mp.cart.decelStartStep = reverseStartStep = netSteps + 1;
		topSpeedTimesCdivAPlusDecelStartClocks = 0;
		mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)((params.decelStartDistance + accelCompensationDistance_vect_MM) * stepsPerMm) + 1;
		const int32_t initialDecelSpeedTimesCdivA = (int32_t)params.topSpeedTimesCdivA - (int32_t)compensationClocks;	// signed because it may be negative and we square it
		const uint64_t initialDecelSpeedTimesCdivASquared = isquare64(initialDecelSpeedTimesCdivA);
		topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivAPlusDecelStartClocks - compensationClocks;
		twoDistanceToStopTimesCsquaredDivA =
			initialDecelSpeedTimesCdivASquared + (uint64_t)(((params.decelStartDistance + accelCompensationDistance_vect_MM) * (DDA::stepClockRateSquared * 2))/dda.acceleration);

		// Calculate the move distance to the point of zero speed, where reverse motion starts
		// the "speed" at the beginning of deceleration is actually instantaneously lower than it was at cruise.
		const float initialDecelSpeed = dda.topSpeed - dda.acceleration * compensationTime_SEC_or_MMpMMpSEC;
		const float reverseStartDistance = (initialDecelSpeed > 0.0)
												? fsquare(initialDecelSpeed)/(2 * dda.acceleration) + params.decelStartDistance
												: params.decelStartDistance;

		// Reverse phase parameters
		if (reverseStartDistance >= dda.totalDistance)
		{
			// No reverse phase
			totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
			reverseStartStep = netSteps + 1;
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		}
		else
		{
			reverseStartStep = (initialDecelSpeed < 0.0)
								? mp.cart.decelStartStep
								: (twoDistanceToStopTimesCsquaredDivA/mp.cart.twoCsquaredTimesMmPerStepDivA) + 1;
			// Because the step numbers are rounded down, we may sometimes get a situation in which netSteps = 1 and reverseStartStep = 1.
			// This would lead to totalSteps = -1, which must be avoided.
			const int32_t overallSteps = (int32_t)(2 * (reverseStartStep - 1)) - netSteps;
			if (overallSteps > 0)
			{
				totalSteps = overallSteps;
				mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA =
						(int64_t)((2 * (reverseStartStep - 1)) * mp.cart.twoCsquaredTimesMmPerStepDivA) - (int64_t)twoDistanceToStopTimesCsquaredDivA;
#ifdef POLYPRINTER
				// make sure there's not going to be a problem using this
				int64_t rootvalue = (int64_t)(mp.cart.twoCsquaredTimesMmPerStepDivA * reverseStartStep) - mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA;
				if ( rootvalue < 0 )
				{
					// it going to barf!!!
					debugPrintf("Error in Extruder deceleration - root val %lld\n", rootvalue );
					DebugPrint('E', false );
				}
#endif
			}
			else
			{
				totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
				reverseStartStep = totalSteps + 1;
				mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
			}
		}
	}
}

void DriveMovement::PrepareExtruderWithVibration(const DDA& dda, const PrepParams& params, const GCodes::PolyZHomeParams& zHomingParams )
{
	// calculate how many total steps we will need for the overall time the move should take
	// we can take the amplitude, convert it to steps, multiply by the frequency and the time, to get total steps in total cycles
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);  // steps of this axis per mm of overall move distance
	// we will underestimate the time allowed for the move by assuming nominal speed throughout. We may stop vibrating just before
	// the move stops, when no contact is found. That is OK. All these probing moves must normally aim far below the expected
	// stopping point.
	const float expectedTime_SEC = dda.totalDistance / dda.requestedSpeed;
	float cycleCount = expectedTime_SEC * zHomingParams.frequency_HZ;
	mp.vibration.stepsPerHalfCycle = zHomingParams.amplitude_MM * stepsPerMm;		// we could possibly just as well specify this in steps instead of MM?
	totalSteps = 2 * cycleCount * mp.vibration.stepsPerHalfCycle;
	debugPrintf("vibration with %d phases per cycle, %f HZ\n", vibrationPhasesPerCycle, (double)zHomingParams.frequency_HZ );
	mp.vibration.clocksPerPhase = DDA::stepClockRate / ( vibrationPhasesPerCycle * zHomingParams.frequency_HZ );
	mp.vibration.phase = 0; // start things off
	debugPrintf("Set up vibration with %d clks/phase and %d steps per half cycle\n", mp.vibration.clocksPerPhase, mp.vibration.stepsPerHalfCycle );
}
#endif

void DriveMovement::DebugPrint(char c, bool isDeltaMovement) const
{
	if (state != DMState::idle)
	{
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32
					" 2dtstc2diva=%" PRIu64 "\n",
					c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval,
					twoDistanceToStopTimesCsquaredDivA);

		if (isDeltaMovement)
		{
			debugPrintf("hmz0sK=%" PRIi32 " minusAaPlusBbTimesKs=%" PRIi32 " dSquaredMinusAsquaredMinusBsquared=%" PRId64 "\n"
						"2c2mmsda=%" PRIu64 " asdsk=%" PRIu32 " dsdsk=%" PRIu32 " mmstcdts=%" PRIu32 "\n",
						mp.delta.hmz0sK, mp.delta.minusAaPlusBbTimesKs, mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared,
						mp.delta.twoCsquaredTimesMmPerStepDivA, mp.delta.accelStopDsK, mp.delta.decelStartDsK, mp.delta.mmPerStepTimesCKdivtopSpeed
						);
		}
		else
		{
			debugPrintf("accelStopStep=%" PRIu32 " decelStartStep=%" PRIu32 " 2CsqtMmPerStepDivA=%" PRIu64 "\n"
						"mmPerStepTimesCdivtopSpeed=%" PRIu32 " fmsdmtstdca2=%" PRId64 " cc=%" PRIu32 " acc=%" PRIu32 "\n",
						mp.cart.accelStopStep, mp.cart.decelStartStep, mp.cart.twoCsquaredTimesMmPerStepDivA,
						mp.cart.mmPerStepTimesCKdivtopSpeed, mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA, mp.cart.compensationClocks, mp.cart.accelCompensationClocks
						);
		}
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do.
// This is also used for extruders on delta machines.
bool DriveMovement::CalcNextStepTimeCartesianFull(const DDA &dda, bool live)
pre(nextStep < totalSteps; stepsTillRecalc == 0)
{
	// Work out how many steps to calculate at a time.
	// The last step before reverseStartStep must be single stepped to make sure that we don't reverse the direction too soon.
	uint32_t shiftFactor = 0;		// assume single stepping
	if (stepInterval < DDA::MinCalcIntervalCartesian)
	{
		uint32_t stepsToLimit = ((nextStep <= reverseStartStep && reverseStartStep <= totalSteps)
									? reverseStartStep
									: totalSteps
								) - nextStep;
		if (stepInterval < DDA::MinCalcIntervalCartesian/4 && stepsToLimit > 8)
		{
			shiftFactor = 3;		// octal stepping
		}
		else if (stepInterval < DDA::MinCalcIntervalCartesian/2 && stepsToLimit > 4)
		{
			shiftFactor = 2;		// quad stepping
		}
		else if (stepsToLimit > 2)
		{
			shiftFactor = 1;		// double stepping
		}
	}

	stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate

	const uint32_t nextCalcStep = nextStep + stepsTillRecalc;
	const uint32_t lastStepTime = nextStepTime;					// pick up the time of the last step
	if (nextCalcStep < mp.cart.accelStopStep)
	{
		// acceleration phase
		const uint32_t adjustedStartSpeedTimesCdivA = dda.startSpeedTimesCdivA + mp.cart.compensationClocks;
		nextStepTime = isqrt64(isquare64(adjustedStartSpeedTimesCdivA) + (mp.cart.twoCsquaredTimesMmPerStepDivA * nextCalcStep)) - adjustedStartSpeedTimesCdivA;
	}
	else if (nextCalcStep < mp.cart.decelStartStep)
	{
		// steady speed phase
		nextStepTime = (uint32_t)(  (int32_t)(((uint64_t)mp.cart.mmPerStepTimesCKdivtopSpeed * nextCalcStep)/K1)
								  + dda.extraAccelerationClocks
								  - (int32_t)mp.cart.accelCompensationClocks
								 );
	}
	else if (nextCalcStep < reverseStartStep)
	{
		// deceleration phase, not reversed yet
		const uint64_t temp = mp.cart.twoCsquaredTimesMmPerStepDivA * nextCalcStep;
		const uint32_t adjustedTopSpeedTimesCdivAPlusDecelStartClocks = dda.topSpeedTimesCdivAPlusDecelStartClocks - mp.cart.compensationClocks;
		// Allow for possible rounding error when the end speed is zero or very small
		nextStepTime = (temp < twoDistanceToStopTimesCsquaredDivA)
						? adjustedTopSpeedTimesCdivAPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivA - temp)
						: adjustedTopSpeedTimesCdivAPlusDecelStartClocks;
	}
	else
	{
		// deceleration phase, reversing or already reversed
		if (nextCalcStep == reverseStartStep)
		{
			direction = !direction;
			if (live)
			{
				reprap.GetPlatform().SetDirection(drive, direction);
			}
		}
		const uint32_t adjustedTopSpeedTimesCdivAPlusDecelStartClocks = dda.topSpeedTimesCdivAPlusDecelStartClocks - mp.cart.compensationClocks;
		nextStepTime = adjustedTopSpeedTimesCdivAPlusDecelStartClocks
							+ isqrt64((int64_t)(mp.cart.twoCsquaredTimesMmPerStepDivA * nextCalcStep) - mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA);
	}

	stepInterval = (nextStepTime - lastStepTime) >> shiftFactor;	// calculate the time per step, ready for next time

	if (nextStepTime > dda.clocksNeeded)
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// Very rarely on a delta, the penultimate step may also be calculated late. Allow for that here in case it affects Cartesian axes too.
		if (nextStep + 1 >= totalSteps)
		{
			nextStepTime = dda.clocksNeeded;
		}
		else
		{
			// We don't expect any step except the last to be late
			state = DMState::stepError;
			stepInterval = 10000000 + nextStepTime;				// so we can tell what happened in the debug print
			return false;
		}
	}
	return true;
}

// Calculate the time since the start of the move when the next step for the specified DriveMovement is due
// Return true if there are more steps to do
bool DriveMovement::CalcNextStepTimeDeltaFull(const DDA &dda, bool live)
pre(nextStep < totalSteps; stepsTillRecalc == 0)
{
	// Work out how many steps to calculate at a time.
	// The last step before reverseStartStep must be single stepped to make sure that we don't reverse the direction too soon.
	// The simulator suggests that at 200steps/mm, the minimum step pulse interval for 400mm/sec movement is 4.5us
	uint32_t shiftFactor = 0;		// assume single stepping
	if (stepInterval < DDA::MinCalcIntervalDelta)
	{
		const uint32_t stepsToLimit = ((nextStep < reverseStartStep && reverseStartStep <= totalSteps)
										? reverseStartStep
										: totalSteps
									  ) - nextStep;
		if (stepInterval < DDA::MinCalcIntervalDelta/8 && stepsToLimit > 16)
		{
			shiftFactor = 4;		// hexadecimal stepping
		}
		else if (stepInterval < DDA::MinCalcIntervalDelta/4 && stepsToLimit > 8)
		{
			shiftFactor = 3;		// octal stepping
		}
		else if (stepInterval < DDA::MinCalcIntervalDelta/2 && stepsToLimit > 4)
		{
			shiftFactor = 2;		// quad stepping
		}
		else if (stepsToLimit > 2)
		{
			shiftFactor = 1;		// double stepping
		}
	}

	stepsTillRecalc = (1u << shiftFactor) - 1;					// store number of additional steps to generate

	if (nextStep == reverseStartStep)
	{
		direction = false;
		if (live)
		{
			reprap.GetPlatform().SetDirection(drive, false);	// going down now
		}
	}

	// Calculate d*s*K as an integer, where d = distance the head has travelled, s = steps/mm for this drive, K = a power of 2 to reduce the rounding errors
	{
		int32_t shiftedK2 = (int32_t)(K2 << shiftFactor);
		if (!direction)
		{
			shiftedK2 = -shiftedK2;
		}
		mp.delta.hmz0sK += shiftedK2;
	}

	const int32_t hmz0scK = (int32_t)(((int64_t)mp.delta.hmz0sK * dda.cKc)/Kc);
	const int32_t t1 = mp.delta.minusAaPlusBbTimesKs + hmz0scK;
	// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
	const int64_t t2a = mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared - (int64_t)isquare64(mp.delta.hmz0sK) + (int64_t)isquare64(t1);
	const int32_t t2 = (t2a > 0) ? isqrt64(t2a) : 0;
	const int32_t dsK = (direction) ? t1 - t2 : t1 + t2;

	// Now feed dsK into a modified version of the step algorithm for Cartesian motion without elasticity compensation
	if (dsK < 0)
	{
		state = DMState::stepError;
		nextStep += 1000000;						// so that we can tell what happened in the debug print
		return false;
	}

	const uint32_t lastStepTime = nextStepTime;		// pick up the time of the last step
	if ((uint32_t)dsK < mp.delta.accelStopDsK)
	{
		// Acceleration phase
		nextStepTime = isqrt64(isquare64(dda.startSpeedTimesCdivA) + (mp.delta.twoCsquaredTimesMmPerStepDivA * (uint32_t)dsK)/K2) - dda.startSpeedTimesCdivA;
	}
	else if ((uint32_t)dsK < mp.delta.decelStartDsK)
	{
		// Steady speed phase
		nextStepTime = (uint32_t)(  (int32_t)(((uint64_t)mp.delta.mmPerStepTimesCKdivtopSpeed * (uint32_t)dsK)/(K1 * K2))
								  + dda.extraAccelerationClocks
								 );
	}
	else
	{
		const uint64_t temp = (mp.delta.twoCsquaredTimesMmPerStepDivA * (uint32_t)dsK)/K2;
		// Because of possible rounding error when the end speed is zero or very small, we need to check that the square root will work OK
		nextStepTime = (temp < twoDistanceToStopTimesCsquaredDivA)
						? dda.topSpeedTimesCdivAPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivA - temp)
						: dda.topSpeedTimesCdivAPlusDecelStartClocks;
	}

	stepInterval = (nextStepTime - lastStepTime) >> shiftFactor;	// calculate the time per step, ready for next time

	if (nextStepTime > dda.clocksNeeded)
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// Very rarely, the penultimate step may be calculated late, so allow for that too.
		if (nextStep + 1 >= totalSteps)
		{
			nextStepTime = dda.clocksNeeded;
		}
		else
		{
			// We don't expect any steps except the last two to be late
			state = DMState::stepError;
			stepInterval = 10000000 + nextStepTime;		// so we can tell what happened in the debug print
			return false;
		}
	}
	return true;
}

#ifdef POLYPRINTER
// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do.
// This is also used for extruders on delta machines.
bool DriveMovement::CalcNextStepTimeVibrationFull(const DDA &dda, bool live)
{
#ifdef DO_QUADRATURE_MODULATION_OUTPUT // doesn't work well -timing is very unexpected
	// we change the clock state each quadrature, to get 2x the frequency of the applied wave
	reprap.GetPlatform().SetProbeModulationOut( mp.vibration.phase & 1 );

	// the next time we need to do anything is simply always 1/4 the period
	// because we have 4 phases
	// phase 0: change clock state, do some steps
	//       1: change clock state
	//       2: change clock state, do some steps the other way
	//       3: change clock state
	if ( ( mp.vibration.phase & 1 ) == 0 )
	{
		// we need to generate the steps
		// We do that by setting it up to step quickly, without coming back into here, for all the steps in the count
		stepsTillRecalc = mp.vibration.stepsPerHalfCycle;
		// change direction
		direction = ! direction;
		if (live)
		{
			reprap.GetPlatform().SetDirection(drive, direction);
		}
	}
	else {
		// this approach, unfortunately, will generate one more step between movements than we actually want
		// - there may be a clean way to eliminate this, but it probably makes very little actual difference as long
		//   as there is no accumulated rotation i.e. one CW, one CCW
		stepsTillRecalc = 0;  // no additional steps this time
	}

#else

	// the next time we need to do anything is simply always 1/4 the period
	// because we have 4 phases
	// phase 0: change clock state, do some steps
	//       1: change clock state, do some steps the other way
	// we need to generate the steps
	// We do that by setting it up to step quickly, without coming back into here, for all the steps in the count
	stepsTillRecalc = mp.vibration.stepsPerHalfCycle;
	// change direction
	if (live)
	{
		reprap.GetPlatform().SetDirection(drive, mp.vibration.phase & 1);
		// plain modulation. Output a signal analogous to the direction pin - we can choose which one to use later.
		// we change the clock state each quadrature, to get 2x the frequency of the applied wave
		reprap.GetPlatform().SetProbeModulationOut( mp.vibration.phase & 1 );
	}

#endif

	++mp.vibration.phase;

	// change the clock output state
	// TODO: define the pin, implement the IO
	nextStepTime += mp.vibration.clocksPerPhase;

	if (nextStepTime > dda.clocksNeeded)
	{
		nextStepTime = dda.clocksNeeded;
		//debugPrintf("vibration ended at %d\n", nextStepTime );
	}

	return 	nextStepTime < dda.clocksNeeded ? true : false;
}
#endif

// Reduce the speed of this movement. Called to reduce the homing speed when we detect we are near the endstop for a drive.
void DriveMovement::ReduceSpeed(const DDA& dda, uint32_t inverseSpeedFactor)
{
	if (dda.isDeltaMovement)
	{
		// Force the linear motion phase
		mp.delta.accelStopDsK = 0;
		mp.delta.decelStartDsK = 0xFFFFFFFF;

		// Adjust the speed
		mp.delta.mmPerStepTimesCKdivtopSpeed *= inverseSpeedFactor;
	}
	else
	{
		// Force the linear motion phase
		mp.cart.accelStopStep = 0;
		mp.cart.decelStartStep = totalSteps + 1;

		// Adjust the speed
		mp.cart.mmPerStepTimesCKdivtopSpeed *= inverseSpeedFactor;
	}
}

// End
