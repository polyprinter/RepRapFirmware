/*
 * DriveMovement.h
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#ifndef DRIVEMOVEMENT_H_
#define DRIVEMOVEMENT_H_

#include <gcodes/gcodes.h>
#include "RepRapFirmware.h"

class LinearDeltaKinematics;

#define ROUND_TO_NEAREST	(0)			// 1 for round to nearest (as used in 1.20beta10), 0 for round down (as used prior to 1.20beta10)

// Rounding functions, to improve code clarity. Also allows a quick switch between round-to-nearest and round down in the movement code.
inline uint32_t roundU32(float f)
{
#if ROUND_TO_NEAREST
	return (uint32_t)lrintf(f);
#else
	return (uint32_t)f;
#endif
}

inline uint32_t roundU32(double d)
{
#if ROUND_TO_NEAREST
	return lrint(d);
#else
	return (uint32_t)d;
#endif
}

inline int32_t roundS32(float f)
{
#if ROUND_TO_NEAREST
	return lrintf(f);
#else
	return (int32_t)f;
#endif
}

inline int32_t roundS32(double d)
{
#if ROUND_TO_NEAREST
	return lrint(d);
#else
	return (int32_t)d;
#endif
}

inline uint64_t roundU64(float f)
{
#if ROUND_TO_NEAREST
	return (uint64_t)llrintf(f);
#else
	return (uint64_t)f;
#endif
}

inline uint64_t roundU64(double d)
{
#if ROUND_TO_NEAREST
	return (uint64_t)llrint(d);
#else
	return (uint64_t)d;
#endif
}

inline int64_t roundS64(float f)
{
#if ROUND_TO_NEAREST
	return llrintf(f);
#else
	return (int64_t)f;
#endif
}

inline int64_t roundS64(double d)
{
#if ROUND_TO_NEAREST
	return llrint(d);
#else
	return (int64_t)d;
#endif
}

// Struct for passing parameters to the DriveMovement Prepare methods
struct PrepParams
{
	// Parameters used for all types of motion
	float decelStartDistance;
	uint32_t topSpeedTimesCdivA;

	// Parameters used only for extruders
	float compFactor;

	// Parameters used only for delta moves
	float initialX;
	float initialY;
	const LinearDeltaKinematics *dparams;
	float diagonalSquared;
	float a2plusb2;								// sum of the squares of the X and Y movement fractions
	float a2b2D2;
};

enum class DMState : uint8_t
{
	idle = 0,
	moving = 1,
	stepError = 2
};

// This class describes a single movement of one drive
class DriveMovement
{
public:
	friend class DDA;

	DriveMovement(DriveMovement *next);

	bool CalcNextStepTimeCartesian(const DDA &dda, bool live) __attribute__ ((hot));
	bool CalcNextStepTimeDelta(const DDA &dda, bool live) __attribute__ ((hot));
#ifdef POLYPRINTER
	bool CalcNextStepTimeVibration(const DDA &dda, bool live) __attribute__ ((hot));
#endif
	void PrepareCartesianAxis(const DDA& dda, const PrepParams& params) __attribute__ ((hot));
	void PrepareDeltaAxis(const DDA& dda, const PrepParams& params) __attribute__ ((hot));
	void PrepareExtruder(const DDA& dda, const PrepParams& params, bool doCompensation) __attribute__ ((hot));

#ifdef POLYPRINTER_PrepareExtruderWithLinearAdvance
	void PrepareExtruderWithLinearAdvance(const DDA& dda, const PrepParams& params, bool doCompensation) __attribute__ ((hot));
#endif
#ifdef POLYPRINTER
	void PrepareExtruderWithVibration(const DDA& dda, const PrepParams& params, const GCodes::PolyZHomeParams& zHomingParams ) __attribute__ ((hot));
#endif
	void ReduceSpeed(const DDA& dda, uint32_t inverseSpeedFactor);
	void DebugPrint(char c, bool withDelta) const;
	int32_t GetNetStepsLeft() const;
	int32_t GetNetStepsTaken() const;

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t microstepShift) const;	// Get the current full step interval for this axis or extruder
#endif

	static void InitialAllocate(unsigned int num);
	static int NumFree() { return numFree; }
	static int MinFree() { return minFree; }
	static void ResetMinFree() { minFree = numFree; }
	static DriveMovement *Allocate(size_t drive, DMState st);
	static void Release(DriveMovement *item);

private:
	bool CalcNextStepTimeCartesianFull(const DDA &dda, bool live) __attribute__ ((hot));
	bool CalcNextStepTimeDeltaFull(const DDA &dda, bool live) __attribute__ ((hot));

#ifdef POLYPRINTER
	bool CalcNextStepTimeVibrationFull(const DDA &dda, bool live) __attribute__ ((hot));
#ifdef DO_QUADRATURE_MODULATION_OUTPUT
	enum { vibrationPhasesPerCycle = 4 };
#else
	enum { vibrationPhasesPerCycle = 2 };
#endif
#endif

	static DriveMovement *freeList;
	static int numFree;
	static int minFree;

	// Parameters common to Cartesian, delta and extruder moves

	DriveMovement *nextDM;								// link to next DM that needs a step

	DMState state;										// whether this is active or not
	uint8_t drive;										// the drive that this DM controls
	uint8_t microstepShift : 4,							// log2 of the microstepping factor (for when we use dynamic microstepping adjustment)
			direction : 1,								// true=forwards, false=backwards
			fullCurrent : 1;							// true if the drivers are set to the full current, false if they are set to the standstill current
	uint8_t stepsTillRecalc;							// how soon we need to recalculate

	uint32_t totalSteps;								// total number of steps for this move

	// These values change as the step is executed, except for reverseStartStep
	uint32_t nextStep;									// number of steps already done
	uint32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement
	uint32_t nextStepTime;								// how many clocks after the start of this move the next step is due
	uint32_t stepInterval;								// how many clocks between steps

	// The following only needs to be stored per-drive if we are supporting pressure advance
	uint64_t twoDistanceToStopTimesCsquaredDivA;

	// Parameters unique to a style of move (Cartesian, delta or extruder). Currently, extruders and Cartesian moves use the same parameters.
	union MoveParams
	{
		struct CartesianParameters						// Parameters for Cartesian and extruder movement, including extruder pressure advance
		{
			// The following don't depend on how the move is executed, so they could be set up in Init()
			uint64_t twoCsquaredTimesMmPerStepDivA;		// 2 * clock^2 * mmPerStepInHyperCuboidSpace / acceleration

			// The following depend on how the move is executed, so they must be set up in Prepare()
			int64_t fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA;		// this one can be negative
			uint32_t accelStopStep;						// the first step number at which we are no longer accelerating
			uint32_t decelStartStep;					// the first step number at which we are decelerating
			uint32_t mmPerStepTimesCKdivtopSpeed;		// mmPerStepInHyperCuboidSpace * clock / topSpeed
			uint32_t compensationClocks;				// the pressure advance time in clocks
			uint32_t accelCompensationClocks;			// compensationClocks * (1 - startSpeed/topSpeed)
		} cart;

		struct DeltaParameters							// Parameters for delta movement
		{
			// The following don't depend on how the move is executed, so they can be set up in Init
			uint64_t twoCsquaredTimesMmPerStepDivA;		// this could be stored in the DDA if all towers use the same steps/mm
			int64_t dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared;
			int32_t hmz0sK;								// the starting step position less the starting Z height, multiplied by the Z movement fraction and K (can go negative)
			int32_t minusAaPlusBbTimesKs;

			// The following depend on how the move is executed, so they must be set up in Prepare()
			uint32_t accelStopDsK;
			uint32_t decelStartDsK;
			uint32_t mmPerStepTimesCKdivtopSpeed;
		} delta;

#ifdef POLYPRINTER
		// for vibration of the extruder, we need a bit of info to use during the move
		struct ExtruderVibration
		{
			// in the future we might choose to do the excitation differently, so a little abstraction is called for
			int stepsPerHalfCycle; 			// With the basic square wave excitation, each time we reverse, we will do a full set of steps
			int clocksPerPhase;				// for 4 phases, this is 1/4 the period
			uint32_t phase;					// keep track of where we are in the cycle
		} vibration;
#endif
	} mp;

	static constexpr uint32_t NoStepTime = 0xFFFFFFFF;	// value to indicate that no further steps are needed when calculating the next step time
	static constexpr uint32_t K1 = 1024;				// a power of 2 used to multiply the value mmPerStepTimesCdivtopSpeed to reduce rounding errors
	static constexpr uint32_t K2 = 512;					// a power of 2 used in delta calculations to reduce rounding errors (but too large makes things worse)
	static constexpr int32_t Kc = 1024 * 1024;			// a power of 2 for scaling the Z movement fraction
};

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1.
// This is also used for extruders on delta machines.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTimeCartesian(const DDA &dda, bool live)
{
	++nextStep;
	if (nextStep <= totalSteps)
	{
#define POLYPRINTER_FAST_STEP
#ifdef POLYPRINTER_FAST_STEP
		uint32_t driverBit = reprap.GetPlatform().GetDriversBitmap( drive );
		if ( (driverBit & reprap.GetPlatform().GetSlowDrivers()) != 0 )
		{
			++nextStep;
			// we need to slow the steps down anyway - do it the old, formal way
			if (stepsTillRecalc != 0)
			{
				--stepsTillRecalc;			// we are doing double or quad stepping
				return true;
			}
			else
			{
				return CalcNextStepTimeCartesianFull(dda, live);
			}
		}
		else
		{
			++nextStep;
			bool moreStepsAfterThese = CalcNextStepTimeCartesianFull(dda, live);
			// we don't want to make the main code spend time inserting and looping for all these extra steps
			// plus we want them done with low overhead. So just do them. Could be slightly noisier because of
			// bunching them together. But this way we can do much higher speeds.
			while ( stepsTillRecalc-- )
			{
				// NOTE: won't work for slow-stepping drives. But the native TMC2660s accept pulses very quickly
				Platform::StepDriversHigh( driverBit );	// generate the step
				delayMicroseconds(1);
				Platform::StepDriversLow();
				delayMicroseconds(1);
				++nextStep;
			}
			return moreStepsAfterThese;
		}

#else
		if (stepsTillRecalc != 0)
		{
			--stepsTillRecalc;			// we are doing double/quad/octal stepping
			return true;
		}
		return CalcNextStepTimeCartesianFull(dda, live);
#endif
	}

	state = DMState::idle;
	return false;
}

// Calculate the time since the start of the move when the next step for the specified DriveMovement is due
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTimeDelta(const DDA &dda, bool live)
{
	++nextStep;
	if (nextStep <= totalSteps)
	{
		++nextStep;
#ifdef POLYPRINTER_FAST_STEP
		uint32_t driverBit = reprap.GetPlatform().GetDriversBitmap( drive );
		if ( (driverBit & reprap.GetPlatform().GetSlowDrivers()) != 0 )
		{
			// we need to slow the steps down anyway - do it the old, formal way
			if (stepsTillRecalc != 0)
			{
				--stepsTillRecalc;			// we are doing double or quad stepping
				return true;
			}
			else
			{
				return CalcNextStepTimeDeltaFull(dda, live);
			}
		}
		else
		{
			bool moreStepsAfterThese = CalcNextStepTimeDeltaFull(dda, live);
			// we don't want to make the main code spend time inserting and looping for all these extra steps
			// plus we want them done with low overhead. So just do them. Could be slightly noisier because of
			// bunching them together. But this way we can do much higher speeds.
			while ( stepsTillRecalc-- )
			{
				// NOTE: won't work for slow-stepping drives. But the native TMC2660s accept pulses very quickly
				Platform::StepDriversHigh( driverBit );	// generate the step
				delayMicroseconds(1);
				Platform::StepDriversLow();
				delayMicroseconds(1);
			}
			return moreStepsAfterThese;
		}

#else
		if (stepsTillRecalc != 0)
		{
			--stepsTillRecalc;			// we are doing double or quad stepping
			return true;
		}
		else
		{
			return CalcNextStepTimeDeltaFull(dda, live);
		}
#endif
	}

	state = DMState::idle;
	return false;
}

#ifdef POLYPRINTER
// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do.
// This is also used for extruders on delta machines.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTimeVibration(const DDA &dda, bool live)
{
#define DO_EXTRA_STEPS_HERE
#ifdef DO_EXTRA_STEPS_HERE
	bool moreStepsAfterThese = CalcNextStepTimeVibrationFull(dda, live);
	// we don't want to make the main code spend time inserting and looping for all these extra steps
	// plus we want them done instantly. So just do them.
	while ( stepsTillRecalc-- )
	{
		// NOTE: won't work for slow-stepping drives.
		Platform::StepDriversHigh( reprap.GetPlatform().GetDriversBitmap( drive ) );	// generate the steps
		delayMicroseconds(1);
		//volatile uint32_t x=10;
		//while ( --x ) {
		//}
		Platform::StepDriversLow();
		delayMicroseconds(1);
		//x = 9;
		//while ( --x ) {
		//}

	}
	return moreStepsAfterThese;
#else
		if (stepsTillRecalc != 0)
		{
			--stepsTillRecalc;			// we are doing quick stepping
			return true;
		}
		return CalcNextStepTimeVibrationFull(dda, live);
#endif
}

#endif

// Return the number of net steps left for the move in the forwards direction.
// We have already taken nextSteps - 1 steps, unless nextStep is zero.
inline int32_t DriveMovement::GetNetStepsLeft() const
{
	int32_t netStepsLeft;
	if (reverseStartStep > totalSteps)		// if no reverse phase
	{
		netStepsLeft = (nextStep == 0) ? (int32_t)totalSteps : (int32_t)totalSteps - (int32_t)nextStep + 1;
	}
	else if (nextStep >= reverseStartStep)
	{
		netStepsLeft = (int32_t)totalSteps - (int32_t)nextStep + 1;
	}
	else
	{
		const int32_t totalNetSteps = (int32_t)(2 * reverseStartStep) - (int32_t)totalSteps - 2;
		netStepsLeft = (nextStep == 0) ? totalNetSteps : totalNetSteps - (int32_t)nextStep + 1;
	}
	return (direction) ? netStepsLeft : -netStepsLeft;
}

// Return the number of net steps already taken for the move in the forwards direction.
// We have already taken nextSteps - 1 steps, unless nextStep is zero.
inline int32_t DriveMovement::GetNetStepsTaken() const
{
	int32_t netStepsTaken;
	if (nextStep < reverseStartStep || reverseStartStep > totalSteps)				// if no reverse phase, or not started it yet
	{
		netStepsTaken = (nextStep == 0) ? 0 : (int32_t)nextStep - 1;
	}
	else
	{
		netStepsTaken = (int32_t)nextStep - (int32_t)(2 * reverseStartStep) + 2;	// allowing for direction having changed
	}
	return (direction) ? netStepsTaken : -netStepsTaken;
}

// This is inlined because it is only called from one place
inline void DriveMovement::Release(DriveMovement *item)
{
	item->nextDM = freeList;
	freeList = item;
	++numFree;
}

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder
inline uint32_t DriveMovement::GetStepInterval(uint32_t microstepShift) const
{
	return ((nextStep >> microstepShift) != 0)		// if at least 1 full step done
		? stepInterval << microstepShift			// return the interval between steps converted to full steps
			: 0;
}

#endif

#endif /* DRIVEMOVEMENT_H_ */
