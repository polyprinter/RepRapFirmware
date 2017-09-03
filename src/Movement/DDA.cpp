/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "DDA.h"
#include "RepRap.h"
#include "Platform.h"
#include "Move.h"
#include "Kinematics/LinearDeltaKinematics.h"		// for DELTA_AXES

//#ifdef DUET_NG
# define DDA_MOVE_DEBUG	(1)
//#else
// On the wired Duets we don't have enough RAM to support this
//# define DDA_MOVE_DEBUG	(0)
//#endif

#if DDA_MOVE_DEBUG
// Structure to hold the essential parameters of a move, for debugging
struct MoveParameters
{
	float accelDistance;
	float steadyDistance;
	float decelDistance;
	float startSpeed;
	float topSpeed;
	float endSpeed;

	MoveParameters()
	{
		accelDistance = steadyDistance = decelDistance = startSpeed = topSpeed = endSpeed = 0.0;
	}

	inline bool isUsed() const { return accelDistance != 0 || steadyDistance != 0 || decelDistance != 0 || startSpeed != 0 || topSpeed != 0 || endSpeed != 0; }
};

//const size_t NumSavedMoves = 256;
const size_t NumSavedMoves = 50;
static MoveParameters savedMoves[NumSavedMoves];
static size_t savedMovePointer = 0;

// Print the saved moves in CSV format for analysis
/*static*/ void DDA::PrintMoves()
{
	// Print the saved moves in CSV format
	reprap.GetPlatform().MessageF(DEBUG_MESSAGE, "Moves:%s,%s,%s,%s,%s,%s ptr %d\n","accelDistance","steadyDistance","decelDistance","startSpeed","topSpeed","endSpeed", savedMovePointer );
	size_t movePointer = savedMovePointer;
	for (size_t i = 0; i < NumSavedMoves; ++i)
	{
		const MoveParameters& m = savedMoves[movePointer];
		movePointer = (movePointer + 1) % NumSavedMoves;
		if ( m.isUsed() ) {
			reprap.GetPlatform().MessageF(DEBUG_MESSAGE, "%f,%f,%f,%f,%f,%f\n", m.accelDistance, m.steadyDistance, m.decelDistance, m.startSpeed, m.topSpeed, m.endSpeed);
		}
	}
}

#else

/*static*/ void DDA::PrintMoves() { }

#endif

#if DDA_LOG_PROBE_CHANGES

size_t DDA::numLoggedProbePositions = 0;
int32_t DDA::loggedProbePositions[XYZ_AXES * MaxLoggedProbePositions];
bool DDA::probeTriggered = false;

void DDA::LogProbePosition()
{
	if (numLoggedProbePositions < MaxLoggedProbePositions)
	{
		int32_t *p = loggedProbePositions + (numLoggedProbePositions * XYZ_AXES);
		for (size_t drive = 0; drive < XYZ_AXES; ++drive)
		{
			DriveMovement *dm = pddm[drive];
			if (dm != nullptr && dm->state == DMState::moving)
			{
				p[drive] = endPoint[drive] - dm->GetNetStepsLeft();
			}
			else
			{
				p[drive] = endPoint[drive];
			}
		}
		++numLoggedProbePositions;
	}
}

#endif


#ifdef POLYPRINTER
#define DO_FORWARD_PASS
#define AUTOMATIC_JERK_VARIATION
#define USE_GREATER_JERK_VALUE			// when using AUTOMATIC_JERK_VARIATION

#ifdef AUTOMATIC_JERK_VARIATION
#define DEFAULT_FULL_JERK (25.1f)		// this is a sentinel value mechanically unsupported to exceed this but is allowed to be used e.g. for G0 moves in particular
#define DEFAULT_MIN_JERK   (10.0f)		// not worth using less than this. Less seems to give poorer quality on facets. Corners seem about the same.
#define DEFAULT_FULL_JERK_SPEED (250.f)		// set to the speed when we are not trying for quality
#define DEFAULT_MIN_JERK_SPEED   (90.f)	// set to the drawing speed that we deem to be an attempt at full quality
#define MINIMUM_PLANNER_SPEED (1.0f)	// TODO: check to see what this should be

inline float interpolate( float fraction, float a, float b ) { return a + fraction * ( b - a ); }
//inline float interpolate( float fraction, int a, int b ) { return a + fraction * ( b - a ); }   // truncates- watch out - may be very unsatisfactory

inline float percentThrough( float value, float lowInput, float highInput ) {
	return ( value - lowInput ) / ( highInput - lowInput );
}

inline float constrainto1( float value ) { return min( value, 1.0f ); }
inline float constrain0to1( float value ) { return max( min( value, 1.0f ), 0.0f ); }

#define DEBUG_BUILD

#ifdef CHECK_REPLAN
float DDA::EffectiveAxisJerkLimit( size_t axis )
{
	float officialJerkLimit = reprap.GetPlatform().ConfiguredInstantDv( axis );
#ifdef AUTOMATIC_JERK_VARIATION
	return axis <= Y_AXIS ? max( officialJerkLimit, DEFAULT_FULL_JERK ) : officialJerkLimit; // the planner is going to use at least the DEFAULT_MIN_JERK for X,Y instead of the setting but it's not an error unless it's over the MAX jerk the auto code might set
#else
	return officialJerkLimit;
#endif
}

// returns true if error
bool DDA::CheckForZeroVectorJerkError( const DDA* firstBlock, const DDA* secondBlock ) {
	// the governing strategy for easy replanning is that the vectorial movement of the print
	// head in XY has zero junction jerk.
	float difference = secondBlock->startSpeed - firstBlock->endSpeed;
	const float VECTORIAL_JERK_LIMIT_MMpSEC = 3.0f;
	if ( fabs( difference )  > VECTORIAL_JERK_LIMIT_MMpSEC ) {
		debugPrintf( "Error: Vectorial Jerk: %f - ", difference );
		debugPrintf( " First DDA: %d", firstBlock->ringIndex );
		debugPrintf( " 2nd DDA: %d", secondBlock->ringIndex );
		debugPrintf( "  First Exit: %f vs.", firstBlock->endSpeed );
		debugPrintf( " Second Entry: %f\n", secondBlock->startSpeed );
		//debugPrintf("\n");
		return true;
	}

	return false;
}

// can only be run after trapezoids are recalculated
float DDA::SingleBlockJerkVel( const DDA* firstBlock, size_t drive ) {
	// assume it's an isolated move
	float initialJerk_MMpSEC = firstBlock->directionVector[drive] * firstBlock->startSpeed;
	float finalJerk_MMpSEC = firstBlock->directionVector[drive] * firstBlock->endSpeed;;

#ifdef TRACE_SINGLE_BLOCK
	if ( initialJerk_MMpSEC > 0 || finalJerk_MMpSEC > 0 ) {
		SERIAL_ECHOLN( "\nTracedSingleJerkVel:" );
		SERIAL_ECHOPAIR( "\ninitial jerk_MMpSEC:", initialJerk_MMpSEC );
		SERIAL_ECHOPAIR( "\n  final jerk_MMpSEC:", finalJerk_MMpSEC );
		SERIAL_ECHOLN("");
	}
#endif

	return max( initialJerk_MMpSEC, finalJerk_MMpSEC );
}

// we must always (in development) check for Jerk violations whenever we are messing with planning code.
// can only be run after trapezoids are recalculated
float DDA::JerkVel( const DDA* firstBlock, const DDA* secondBlock, size_t drive ) {
	float firsExitVel_MMpSEC = firstBlock->directionVector[drive] * firstBlock->endSpeed;
	float secondEntryVel_MMpSEC = secondBlock->directionVector[drive] * secondBlock->startSpeed;

	float jerk_MMpSEC = secondEntryVel_MMpSEC - firsExitVel_MMpSEC;
	return jerk_MMpSEC;
}

float DDA::TracedJerkVel( const DDA* firstBlock, const DDA* secondBlock, size_t drive ) {
	float firsExitVel_MMpSEC = firstBlock->directionVector[drive] * firstBlock->endSpeed;
	float secondEntryVel_MMpSEC = secondBlock->directionVector[drive] * secondBlock->startSpeed;
	float jerk_MMpSEC = secondEntryVel_MMpSEC - firsExitVel_MMpSEC;
	debugPrintf( "   exit speed: %f, ", firsExitVel_MMpSEC );
	debugPrintf( " entry speed: %f\n", secondEntryVel_MMpSEC );
	return jerk_MMpSEC;
}

// can only be run after trapezoids are recalculated
void DDA::TraceAxisJunctionPlan( const DDA* firstBlock, const DDA* secondBlock, int drive )
{
	debugPrintf( "Axis Planning: drive %d - \n", drive );
	//debugPrintf("");
	debugPrintf( " First Block: %d", firstBlock->ringIndex );
	//float firstAxisFinalSpeed_StepsPerSec    = TraceAxisBlockPlanFinal(   firstBlock,  axis );
	debugPrintf( "   2nd Block: %d\n", secondBlock->ringIndex );
	//float secondAxisInitialSpeed_StepsPerSec = TraceAxisBlockPlanInitial( secondBlock, axis );
	//float jerkStepsPerSecond = secondAxisInitialSpeed_StepsPerSec - firstAxisFinalSpeed_StepsPerSec;
	float jerk_MMpSEC = TracedJerkVel( firstBlock, secondBlock, drive );
	//debugPrintf( " Axis Jerk steps/sec:", jerkStepsPerSecond );
	debugPrintf( " Axis Jerk mm/sec: %f\n", jerk_MMpSEC );
	//debugPrintf("");
}

// can only be run after trapezoids are recalculated
float DDA::JerkFactor( const DDA* firstBlock, const DDA* secondBlock, size_t drive ) {
	float jerkVel_MMpSEC = JerkVel( firstBlock, secondBlock, drive );
	// compare to axis Jerk limit
	//const DriveMovement& first_dm = firstBlock->ddm[drive];
	//const DriveMovement& second_dm = secondBlock->ddm[drive];

	float jerkLimit = EffectiveAxisJerkLimit(drive);
	return jerkVel_MMpSEC /jerkLimit;
}

// can only be run after trapezoids are recalculated
float DDA::JerkFactor( const DDA* firstBlock, size_t drive ) {
	float jerkVel_MMpSEC = SingleBlockJerkVel( firstBlock, drive );
	// compare to axis Jerk limit
	float jerkLimit =  EffectiveAxisJerkLimit(drive);
	return jerkVel_MMpSEC /jerkLimit;
}

bool DDA::CheckForJerkError( const DDA* firstBlock, const DDA* secondBlock, float errorFactorThreshold, size_t toAxis ) {

	bool hasError = CheckForZeroVectorJerkError( firstBlock, secondBlock );

	for (size_t drive = 0; drive < DRIVES && drive <= toAxis; drive++) {
		float factor = JerkFactor( firstBlock, secondBlock, drive );
		if ( fabs(factor) > errorFactorThreshold ) {
			debugPrintf( "Drive %d Jerk Factor high: %f", drive, factor );
			debugPrintf( " Block:  %d", firstBlock->ringIndex );
			debugPrintf( " and:  %d", secondBlock->ringIndex );
			debugPrintf( " vs. allowed max jerk: %f\n",  EffectiveAxisJerkLimit(drive)  );
			debugPrintf("\n");

			// if disagreement:
			//TracedJerkVel( firstBlock, secondBlock, drive );
			TraceAxisJunctionPlan( firstBlock, secondBlock, drive );
			//SERIAL_ECHOLN("");
			hasError = true;
		}
	}

	return hasError;
}

// returns false if there's a problem
bool DDA::TestTrapezoidJerk( const DDA* prev, const DDA* following ) {
	if ( ( prev->state == DDA::provisional ) // we can't change the older ones... || prev->state == DDA::frozen || prev->state == DDA::executing ) // possibly completed too?
			&& following->state == DDA::provisional ) {
		const float PLANNED_JERK_ERROR_ALLOWED = 1.1f;
		// check our speeds vs the previous block that was handed in. (Is Prev even necessary?)
		bool bIsJerkError = CheckForJerkError( prev, following, PLANNED_JERK_ERROR_ALLOWED, Y_AXIS );  // will issue messages if there's a problem with X or Y jerk
		if ( bIsJerkError ) {
			debugPrintf( "prev DDA %d plan:\n", prev->ringIndex );
			prev->DebugPrint();
			debugPrintf( "foll DDA %d plan:\n", following->ringIndex );
			following->DebugPrint();
			PrintMoves();
		}
		return bIsJerkError;
	}
	return true;
}

#endif
#endif

DDA::DDA(DDA* n) : next(n), prev(nullptr), state(empty)
{
	for (DriveMovement*& p : pddm)
	{
		p = nullptr;
	}
}

void DDA::ReleaseDMs()
{
	for (DriveMovement*& p : pddm)
	{
		if (p != nullptr)
		{
			DriveMovement::Release(p);
			p = nullptr;
		}
	}
}

// Return the number of clocks this DDA still needs to execute.
// This could be slightly negative, if the move is overdue for completion.
int32_t DDA::GetTimeLeft() const
pre(state == executing || state == frozen || state == completed)
{
	return (state == completed) ? 0
			: (state == executing) ? (int32_t)(moveStartTime + clocksNeeded - Platform::GetInterruptClocks())
			: (int32_t)clocksNeeded;
}

// Insert the specified drive into the step list, in step time order.
// We insert the drive before any existing entries with the same step time for best performance. Now that we generate step pulses
// for multiple motors simultaneously, there is no need to preserve round-robin order.
inline void DDA::InsertDM(DriveMovement *dm)
{
	DriveMovement **dmp = &firstDM;
	while (*dmp != nullptr && (*dmp)->nextStepTime < dm->nextStepTime)
	{
		dmp = &((*dmp)->nextDM);
	}
	dm->nextDM = *dmp;
	*dmp = dm;
}

// Remove this drive from the list of drives with steps due, and return its DM or nullptr if not there
// Called from the step ISR only.
DriveMovement *DDA::RemoveDM(size_t drive)
{
	DriveMovement **dmp = &firstDM;
	while (*dmp != nullptr)
	{
		DriveMovement *dm = *dmp;
		if (dm->drive == drive)
		{
			(*dmp) = dm->nextDM;
			return dm;
		}
		dmp = &(dm->nextDM);
	}
	return nullptr;
}

void DDA::DebugPrintVector(const char *name, const float *vec, size_t len) const
{
	debugPrintf("%s=", name);
	for (size_t i = 0; i < len; ++i)
	{
		debugPrintf("%c%f", ((i == 0) ? '[' : ' '), vec[i]);
	}
	debugPrintf("]");
}

void DDA::DebugPrint() const
{
	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	debugPrintf("DDA:");
	if (endCoordinatesValid)
	{
		float startCoordinates[MaxAxes];
		for (size_t i = 0; i < numAxes; ++i)
		{
			startCoordinates[i] = endCoordinates[i] - (totalDistance * directionVector[i]);
		}
		DebugPrintVector(" start", startCoordinates, numAxes);
		DebugPrintVector(" end", endCoordinates, numAxes);
	}

	debugPrintf(" d=%f", totalDistance);
	DebugPrintVector(" vec", directionVector, 5);
	debugPrintf("\na=%f reqv=%f topv=%f startv=%f endv=%f\n"
				"daccel=%f ddecel=%f cks=%u\n",
				acceleration, requestedSpeed, topSpeed, startSpeed, endSpeed,
				accelDistance, decelDistance, clocksNeeded);
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		if (pddm[axis] != nullptr)
		{
			pddm[axis]->DebugPrint(reprap.GetGCodes().axisLetters[axis], isDeltaMovement);
		}
	}
	for (size_t i = numAxes; i < DRIVES; ++i)
	{
		if (pddm[i] != nullptr && pddm[i]->state != DMState::idle)
		{
			pddm[i]->DebugPrint((char)('0' + (i - numAxes)), false);
		}
	}
}

// This is called by Move to initialize all DDAs
void DDA::Init()
{
	// Set the endpoints to zero, because Move asks for them.
	// They will be wrong if we are on a delta. We take care of that when we process the M665 command in config.g.
	for (int32_t& ep : endPoint)
	{
		ep = 0;
	}
	state = empty;
	endCoordinatesValid = false;
	virtualExtruderPosition = 0;
	filePos = noFilePosition;

#ifdef POLYPRINTER
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		relativeExtrusionDebt[drive] = 0;
	}
	hadNutSwitch = false;
	hadBedContact = false;
#endif
#if SUPPORT_IOBITS
	ioBits = 0;
#endif
}

// Set up a real move. Return true if it represents real movement, else false.
// Either way, return the amount of extrusion we didn't do in the extruder coordinates of nextMove
bool DDA::Init(GCodes::RawMove &nextMove, bool doMotorMapping)
{
	// 1. Compute the new endpoints and the movement vector
	const int32_t * const positionNow = prev->DriveCoordinates();
	const Move& move = reprap.GetMove();
	if (doMotorMapping)
	{
		if (!move.CartesianToMotorSteps(nextMove.coords, endPoint))		// transform the axis coordinates if on a delta or CoreXY printer
		{
			return false;												// throw away the move if it couldn't be transformed
		}
		isDeltaMovement = move.IsDeltaMode()
							&& (endPoint[X_AXIS] != positionNow[X_AXIS] || endPoint[Y_AXIS] != positionNow[Y_AXIS] || endPoint[Z_AXIS] != positionNow[Z_AXIS]);
	}
	else
	{
		isDeltaMovement = false;
	}

	isPrintingMove = false;
	xyMoving = false;
	bool extruding = false;												// we set this true if extrusion was commanded, even if it is too small to do
	bool realMove = false;
	float accelerations[DRIVES];
	const float * const normalAccelerations = reprap.GetPlatform().Accelerations();
	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		accelerations[drive] = normalAccelerations[drive];

		if (drive >= numAxes || !doMotorMapping)
		{
#ifdef POLYPRINTER
			// we must keep track of extrusion distance that gets deferred for being less than one step
			// here, add any debt to the current relative extrusion amount. When the total gets to be more than one step,
			// we will subtract the actual movement from the debt.
			// Debt could be negative, if we round up 1/2 step, but that's a separate issue.
			endPoint[drive] = Move::MotorEndPointToMachine(drive, (drive >= numAxes ) ? ( nextMove.coords[drive] + prev->relativeExtrusionDebt[drive] ) : nextMove.coords[drive]);
#else
			endPoint[drive] = Move::MotorEndPointToMachine(drive, nextMove.coords[drive]);
#endif
		}


		endCoordinates[drive] = nextMove.coords[drive];
		const int32_t delta = (drive < numAxes) ? endPoint[drive] - positionNow[drive] : endPoint[drive];

		if (drive < numAxes && doMotorMapping)
		{
			const float positionDelta = nextMove.coords[drive] - prev->GetEndCoordinate(drive, false);
			directionVector[drive] = positionDelta;
			if (positionDelta != 0.0 && (IsBitSet(nextMove.yAxes, drive) || IsBitSet(nextMove.xAxes, drive)))
			{
				xyMoving = true;
			}
		}
		else
		{
#ifdef POLYPRINTER
			// extruders are always handled here, but sometimes other drives
			if ( drive >= numAxes )
			{
				const float coorddelta = nextMove.coords[drive] - nextMove.initialCoords[drive];
				// to properly class the type of move, for planner purposes, it's important to know the inent, even
				// though it may turn out to be zero extruder steps, for example.
				if ( coorddelta > 0 && xyMoving )
				{
					isPrintingMove = true;				// we have both XY movement and extrusion intent
					// TODO: possibly reset debt each retraction or true non-printing move?
					// we must keep track of extrusion distance that gets deferred for being less than one step
					float distanceRequested = nextMove.coords[drive] + prev->relativeExtrusionDebt[drive];
					float distanceMoved = (float)delta/reprap.GetPlatform().DriveStepsPerUnit(drive);  // might be zero. If we round 1/2 step might be more than requested.
					relativeExtrusionDebt[drive] =  distanceRequested;
					relativeExtrusionDebt[drive] -= distanceMoved;
					if ( relativeExtrusionDebt[drive] > 1.0f/reprap.GetPlatform().DriveStepsPerUnit(drive) )
					{
						debugPrintf("Error in relativeExtrusionDebt - > 1.0 steps\n", relativeExtrusionDebt[drive] );
					}
				}
				else {
					// if it's not intended to be a printing move, cancel any accumulated debt
					relativeExtrusionDebt[drive] = 0;
				}

				// make note of whether this is a vibration "move" for the extruder, and save the parameters
				zHomingParams = nextMove.zHomingParams;
				doZHomingVibration = nextMove.doZHomingVibration;
			}
#endif
			directionVector[drive] = (float)delta/reprap.GetPlatform().DriveStepsPerUnit(drive);
			if (drive >= numAxes && nextMove.coords[drive] > 0.0)
			{
				extruding = true;
			}
		}

		if (delta != 0)
		{
			realMove = true;
			DriveMovement*& pdm = pddm[drive];
			pdm = DriveMovement::Allocate(drive, DMState::moving);
			pdm->totalSteps = labs(delta);				// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
			pdm->direction = (delta >= 0);				// for now this is the direction of net movement, but gets adjusted later if it is a delta movement

			if (drive >= numAxes)
			{

				// It's an extruder movement
				nextMove.coords[drive] -= directionVector[drive];
														// subtract the amount of extrusion we actually did to leave the residue outstanding

				if (xyMoving && nextMove.usePressureAdvance)
				{
					const float compensationTime = reprap.GetPlatform().GetPressureAdvance(drive - numAxes);
					if (compensationTime > 0.0)
					{
						// Compensation causes instant velocity changes equal to acceleration * k, so we may need to limit the acceleration
#ifdef POLYPRINTER_DEBUG_E_ACCEL_LIMIT
						// give one warning if the compensation would cause a reduction of acceleration for the E axis
						static bool warnedAccelDrop = false;
						if ( ! warnedAccelDrop && accelerations[drive] > EffectiveAxisJerkLimit(drive)/compensationTime ) {
							debugPrintf("NOTE: E acceleration of %f being reduced by Advance!!!\n", accelerations[drive]);
							debugPrintf("adv %f, extruder %d InstantDv %f mm/sec\n", compensationTime, drive, EffectiveAxisJerkLimit(drive) );
							debugPrintf("acceleration becomes %f\n",  min<float>(accelerations[drive], EffectiveAxisJerkLimit(drive)/compensationTime) );
							warnedAccelDrop = true;
						}
#endif
						accelerations[drive] = min<float>(accelerations[drive], EffectiveAxisJerkLimit(drive)/compensationTime);
					}
				}
			}
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		// Update the end position in the previous move, so that on the next move we don't think there is XY movement when the user didn't ask for any
		if (doMotorMapping)
		{
			for (size_t drive = 0; drive < numAxes; ++drive)
			{
				prev->endCoordinates[drive] = nextMove.coords[drive];
			}
		}
		return false;
	}

	// 2a. If it's a delta move, we need a DM for each tower even if its carriage has no net movement
	if (isDeltaMovement)
	{
		for (size_t drive = 0; drive < DELTA_AXES; ++drive)
		{
			DriveMovement*& pdm = pddm[drive];
			if (pdm == nullptr)
			{
				pdm = DriveMovement::Allocate(drive, DMState::moving);
				pdm->totalSteps = 0;
				pdm->direction = true;
			}
		}
	}

	// 3. Store some values
	xAxes = nextMove.xAxes;
	yAxes = nextMove.yAxes;
	endStopsToCheck = nextMove.endStopsToCheck;
	canPauseBefore = nextMove.canPauseBefore;
	canPauseAfter = nextMove.canPauseAfter;
	filePos = nextMove.filePos;
	isPrintingMove = xyMoving && extruding;
	usePressureAdvance = nextMove.usePressureAdvance;
	virtualExtruderPosition = nextMove.virtualExtruderPosition;
	hadLookaheadUnderrun = false;

#ifdef POLYPRINTER
	hadNutSwitch = false;
	hadBedContact = false;
#endif

	isLeadscrewAdjustmentMove = false;
	goingSlow = false;


#if SUPPORT_IOBITS
	ioBits = nextMove.ioBits;
#endif

	// If it's a Z probing move, limit the Z acceleration to better handle nozzle-contact probes
	if ((endStopsToCheck & ZProbeActive) != 0 && accelerations[Z_AXIS] > ZProbeMaxAcceleration)
	{
		accelerations[Z_AXIS] = ZProbeMaxAcceleration;
	}

	// The end coordinates will be valid at the end of this move if it does not involve endstop checks and is not a raw motor move
	endCoordinatesValid = (endStopsToCheck == 0) && doMotorMapping;

	// 4. Normalise the direction vector and compute the amount of motion.
	if (xyMoving)
	{
		// There is some XY movement, so normalise the direction vector so that the total XYZ movement has unit length and 'totalDistance' is the XYZ distance moved.
		// This means that the user gets the feed rate that he asked for. It also makes the delta calculations simpler.
		// First do the bed tilt compensation for deltas.
		const Kinematics& k = move.GetKinematics();
		directionVector[Z_AXIS] += (directionVector[X_AXIS] * k.GetTiltCorrection(X_AXIS)) + (directionVector[Y_AXIS] * k.GetTiltCorrection(Y_AXIS));

		totalDistance = NormaliseXYZ();
#ifdef POLYPRINTER
		// there is some redundancy between our moveClass and the other flags
		if ( isPrintingMove ) {
			motionClass = ( directionVector[E0_AXIS] < 0 || directionVector[E0_AXIS+1] < 0 ) ? MOTION_CLASS_XYZ_RETRACT : MOTION_CLASS_XYZ_DRAW;
		} else {
			motionClass = MOTION_CLASS_XYZ_MOVE;
		}

#endif
	}
	else
	{
		// Extruder-only movement, or movement of additional axes, or a combination.
		// Currently we normalise vector sum of all drive movement to unit length.
		// Alternatives would be:
		// 1. Normalise the largest one to unit length. This means that when retracting multiple filaments, they all get the requested retract speed.
		// 2. Normalise the sum to unit length. This means that when we use mixing, we get the requested extrusion rate at the nozzle.
		// 3. Normalise the sum to the sum of the mixing coefficients (which we would have to include in the move details).
		totalDistance = Normalise(directionVector, DRIVES, DRIVES);
#ifdef POLYPRINTER
		// there is some redundancy between our moveClass and the other flags
		motionClass = ( directionVector[E0_AXIS] < 0 || directionVector[E0_AXIS+1] < 0 ) ? MOTION_CLASS_XYZ_RETRACT : MOTION_CLASS_E_ONLY;
#endif
	}

	// 5. Compute the maximum acceleration available
	float normalisedDirectionVector[DRIVES];			// Used to hold a unit-length vector in the direction of motion
	memcpy(normalisedDirectionVector, directionVector, sizeof(normalisedDirectionVector));
	Absolute(normalisedDirectionVector, DRIVES);
	acceleration = VectorBoxIntersection(normalisedDirectionVector, accelerations, DRIVES);
	if (xyMoving)
	{
		acceleration = min<float>(acceleration, (isPrintingMove) ? reprap.GetPlatform().GetMaxPrintingAcceleration() : reprap.GetPlatform().GetMaxTravelAcceleration());
	}

	// 6. Set the speed to the smaller of the requested and maximum speed.
	// Also enforce a minimum speed of 0.5mm/sec. We need a minimum speed to avoid overflow in the movement calculations.
	float reqSpeed = nextMove.feedRate;
	if (!doMotorMapping)
	{
		// Special case of a raw or homing move on a delta printer
		// We use the Cartesian motion system to implement these moves, so the feed rate will be interpreted in Cartesian coordinates.
		// This is wrong, we want the feed rate to apply to the drive that is moving the farthest.
		float maxDistance = 0.0;
		for (size_t axis = 0; axis < DELTA_AXES; ++axis)
		{
			if (normalisedDirectionVector[axis] > maxDistance)
			{
				maxDistance = normalisedDirectionVector[axis];
			}
		}
		if (maxDistance != 0.0)				// should always be true
		{
			reqSpeed /= maxDistance;		// because normalisedDirectionVector is unit-normalised
		}
	}

	// Don't use the constrain function in the following, because if we have a very small XY movement and a lot of extrusion, we may have to make the
	// speed lower than the 0.5mm/sec minimum. We must apply the minimum speed first and then limit it if necessary after that.
	requestedSpeed = min<float>(max<float>(reqSpeed, 0.5), VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform().MaxFeedrates(), DRIVES));

	// On a Cartesian printer, it is OK to limit the X and Y speeds and accelerations independently, and in consequence to allow greater values
	// for diagonal moves. On a delta, this is not OK and any movement in the XY plane should be limited to the X/Y axis values, which we assume to be equal.
	if (doMotorMapping)
	{
		switch (reprap.GetMove().GetKinematics().GetKinematicsType())
		{
		case KinematicsType::cartesian:
			// On a Cartesian printer the axes accelerate independently
			break;

		case KinematicsType::coreXY:
		case KinematicsType::coreXYU:
			// Preferably we would specialise these, but for now fall through to the default implementation
		default:
			// On other types of printer, the relationship between motor movement and head movement is complex.
			// Limit all moves to the lower of X and Y speeds and accelerations.
			{
				const float xyFactor = sqrtf(fsquare(normalisedDirectionVector[X_AXIS]) + fsquare(normalisedDirectionVector[Y_AXIS]));
				const float * const maxSpeeds = reprap.GetPlatform().MaxFeedrates();
				const float maxSpeed = min<float>(maxSpeeds[X_AXIS], maxSpeeds[Y_AXIS]);
				if (requestedSpeed * xyFactor > maxSpeed)
				{
					requestedSpeed = maxSpeed/xyFactor;
				}

				const float maxAcceleration = min<float>(normalAccelerations[X_AXIS], normalAccelerations[Y_AXIS]);
				if (acceleration * xyFactor > maxAcceleration)
				{
					acceleration = maxAcceleration/xyFactor;
				}
			}
			break;
		}
	}
#ifdef POLYPRINTER
	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	// TODO: leave at low rate if drawing, but use Jerk speed otherwise?
	endSpeed = 0.0;					// until the next move asks us to adjust it
	recalculate_flag = true;		// must always start out being set

	if (prev->state != provisional)
	{
		// There is no previous move that we can adjust, so this move must start at zero speed.
		startSpeed = 0.0;
	}
	else
	{
		startSpeed = prev->endSpeed;	// it's probably quite low, but may be as high as the allowed Jerk
		// Try to meld this move to the previous move to avoid stop/start
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		//prev->targetNextSpeed = min<float>(sqrtf(acceleration * totalDistance * 2.0), requestedSpeed);
		// TODO: add filtering if necessary
		//float filteredPreviousVelocity = prev->endSpeed;
		SetBlockMaxAllowableJerkEntrySpeed( prev,  this );
		//SetBlockMaxAllowableJerkEntrySpeed( prev->requestedSpeed, prev->endSpeed, this, requestedSpeed, prev->totalDistance, prev->motionClass, filteredPreviousVelocity );

		// initial calc, based on stopping at the end
		//block->nominal_length_flag = 0; // must be cleared first, or the calc will just use the nominal speed
		SetBlockMaxAllowableDecelerationEntrySpeed( this, MINIMUM_PLANNER_SPEED ); // this MUST be calculated and set every time the exit speed is changed
		//float v_allowable = block->decelLimitedEntrySpeed_MMpSEC; // initially this will be correct

		DoPlannerLookahead( this );
		//startSpeed = prev->targetNextSpeed;
	}

#else
	// original planning strategy
	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	endSpeed = 0.0;					// until the next move asks us to adjust it

	if (prev->state != provisional || isPrintingMove != prev->isPrintingMove)
	{
		// There is no previous move that we can adjust, so this move must start at zero speed.
		startSpeed = 0.0;
	}
	else
	{
		// Try to meld this move to the previous move to avoid stop/start
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		prev->targetNextSpeed = min<float>(sqrtf(acceleration * totalDistance * 2.0), requestedSpeed);
		DoLookahead(prev);
		startSpeed = prev->targetNextSpeed;
	}
#endif

	RecalculateMove();
	state = provisional;
#ifdef POLYPRINTER
#ifdef CHECK_REPLAN
	// we must always (in development) check for Jerk violations whenever we are messing with planning code.
	// just check this new block and its predecessor, to start with.
	TestTrapezoidJerk( prev, this );
#endif
#endif
	return true;
}


// Set up a raw (unmapped) motor move returning true if the move does anything
bool DDA::Init(const float_t adjustments[DRIVES])
{
	// 1. Compute the new endpoints and the movement vector
	const float ZAcceleration = reprap.GetPlatform().Accelerations()[Z_AXIS];
	const float ZSpeed = reprap.GetPlatform().MaxFeedrate(Z_AXIS);

	float accelerations[DRIVES];
	float maxSpeeds[DRIVES];
	bool realMove = false;

	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		accelerations[drive] = ZAcceleration;					// all motors moving are Z motors
		maxSpeeds[drive] = ZSpeed;								// all motors moving are Z motors
		endPoint[drive] = prev->endPoint[drive];				// adjusting leadscrews doesn't change the endpoint
		endCoordinates[drive] = prev->endCoordinates[drive];	// adjusting leadscrews doesn't change the position

		directionVector[drive] = adjustments[drive];
		const int32_t delta = lrintf(directionVector[drive] * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));

		if (delta != 0)
		{
			DriveMovement*& pdm = pddm[drive];
			pdm = DriveMovement::Allocate(drive + DRIVES, DMState::moving);
			pdm->totalSteps = labs(delta);
			pdm->direction = (delta >= 0);
			realMove = true;
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		ReleaseDMs();
		return false;
	}

	// 3. Store some values
	isLeadscrewAdjustmentMove = true;
	isDeltaMovement = false;
	isPrintingMove = false;
	xyMoving = false;
	endStopsToCheck = 0;
	canPauseBefore = true;
	canPauseAfter = true;
	usePressureAdvance = false;
	virtualExtruderPosition = prev->virtualExtruderPosition;
	hadLookaheadUnderrun = false;
	xAxes = prev->xAxes;
	yAxes = prev->yAxes;
	filePos = prev->filePos;
	endCoordinatesValid = prev->endCoordinatesValid;
	goingSlow = false;

#if SUPPORT_IOBITS
	ioBits = prev->ioBits;
#endif

	// 4. Normalise the direction vector and compute the amount of motion.
	// Currently we normalise the vector sum of all Z motor movement to unit length.
	totalDistance = Normalise(directionVector, DRIVES, DRIVES);

	// 5. Compute the maximum acceleration available
	float normalisedDirectionVector[DRIVES];			// Used to hold a unit-length vector in the direction of motion
	memcpy(normalisedDirectionVector, directionVector, sizeof(normalisedDirectionVector));
	Absolute(normalisedDirectionVector, DRIVES);
	acceleration = VectorBoxIntersection(normalisedDirectionVector, accelerations, DRIVES);

	// 6. Set the speed to the smaller of the requested and maximum speed.
	requestedSpeed = VectorBoxIntersection(normalisedDirectionVector, maxSpeeds, DRIVES);

	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	startSpeed = endSpeed = 0.0;

	RecalculateMove();
	state = provisional;
	return true;
}


#ifdef POLYPRINTER
// this linear rule may not be the most effective at all.
#define DEFAULT_JERK_SPEED_FACTOR ( (float)( DEFAULT_FULL_JERK - DEFAULT_MIN_JERK ) / (float)( DEFAULT_FULL_JERK_SPEED - DEFAULT_MIN_JERK_SPEED ) )
// to calculate the dynamic jerk we should just need the factor and the minimum speed. e.g. jerk = min_jerk + ( speed - min_speed ) * factor. Then clamped at max.
// - alternatively, calculate a different intercept and then clamp both. jerk = jerkIntercept + speed * factor, clamp high and low
#define DEFAULT_JERK_INTERCEPT (DEFAULT_MIN_JERK-(float)DEFAULT_JERK_SPEED_FACTOR*DEFAULT_MIN_JERK_SPEED)

 #ifdef USE_GREATER_JERK_VALUE
  #define DEFAULT_XYJERK                DEFAULT_MIN_JERK    // at low speeds this will be the Jerk for G1. Higher speeds will override this higher.
 #else
	// we'll turn off the modulation if the max has been set to something other than this default, since that typically indicates an explicit control of jerk.
	#define DEFAULT_XYJERK                DEFAULT_FULL_JERK    // if axis jerk (which defaults to this) is not our sentinel value we will not modulate jerk.
 #endif
#else
// default to something just a bit softer when we don't modulate it
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#endif

#define POW2( x ) ( x * x )

inline float EstimateAccelerationDistance_precalc( float initial_rate_sq, float target_rate_sq, float acceleration_x2 )
{
	return ( ( target_rate_sq - initial_rate_sq ) / acceleration_x2 );
}

// floating point, based on mm distances
// otherSpeed must be <= the requestedSpeed
inline float DDA::BlockAccelerationDistance( const DDA* block, float otherSpeed ) {
	float initial_rate_sq = POW2( otherSpeed );
	float target_rate_sq = POW2( block->requestedSpeed );
	float acceleration_x2 = block->acceleration * 2;
	return EstimateAccelerationDistance_precalc( initial_rate_sq, target_rate_sq, acceleration_x2 );
	}

inline float DDA::SpeedAchieved( float initialVel, float accel, float distance ) {
	return sqrt( POW2( initialVel ) + 2 * accel * distance );
	}

inline float DDA::AverageSpeed( float initialVel, float accel, float distance ) {
	float finalVel = SpeedAchieved( initialVel, accel, distance );
	return ( initialVel + finalVel ) * .5f;
	}

inline float DDA::BlockMaxAllowableDecelerationEntrySpeed( DDA* block, float exitSpeed ) {

	//SERIAL_ECHOPAIR( "BlockMaxAllowableDecelerationEntrySpeed for Block: ", (uint32_t)BlockNum( block ) );
	//SERIAL_ECHOPAIR( "nominal_length_flag: ", (uint32_t)block->nominal_length_flag );
	//SERIAL_ECHOLN( "" );

	//if ( block->nominal_length_flag ) return block->requestedSpeed; // the block can decelerate to zero from nominal in the length. No need to calculate further.

	float maxDecelEntrySpeed = block->requestedSpeed;
	// not sure if this is useful enough to save any time.
	//SERIAL_ECHOPAIR( "exitSpeed: ", exitSpeed );
	//SERIAL_ECHOPAIR( "acceleration: ", block->acceleration );
	//SERIAL_ECHOPAIR( "millimeters: ", block->totalDistance );
	//SERIAL_ECHOPAIR( "BlockAccelerationDistance: ", BlockAccelerationDistance( block, exitSpeed ) );
	//SERIAL_ECHOPAIR( "Speed Achievable: ", SpeedAchieved( exitSpeed, block->acceleration, block->totalDistance ) );
	if ( BlockAccelerationDistance( block, exitSpeed ) > block->totalDistance ) {
		// it can't enter at nominal rate, because it can't slow down fast enough
		float maxEntrySpeed = SpeedAchieved( exitSpeed, block->acceleration, block->totalDistance );
		//SERIAL_ECHOPAIR( "maxEntrySpeed: ", maxEntrySpeed );
#ifdef CHECK_PLANNING_ASSUMPTIONS
		if ( maxEntrySpeed >= maxDecelEntrySpeed ) {
			// there's an error in the assumptions or the functions
			}
#endif
		maxDecelEntrySpeed = maxEntrySpeed;
		}

	//SERIAL_ECHOLN( "ret" );
	return maxDecelEntrySpeed;
	}

// this MUST be called both initially, and whenever any block's exit speed is modified.
inline void DDA::SetBlockMaxAllowableDecelerationEntrySpeed( DDA* block, float exitSpeed ) {
	block->decelLimitedEntrySpeed_MMpSEC = BlockMaxAllowableDecelerationEntrySpeed( block, exitSpeed );
	}

//#define TRACE_SetBlockMaxAllowableJerkEntrySpeed
#define CHECK_SetBlockMaxAllowableJerkEntrySpeed

// this MUST be called initially to set up the invariant limit
// based on jerk, and the lesser of the two nominal vectorial speeds.
// It will be zero, when there is no previous block.
inline void DDA::SetBlockMaxAllowableJerkEntrySpeed( const DDA* prevblock, DDA* currentblock )
{
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
	// check conditions
	if ( currentblock->requestedSpeed == 0 ) {
		debugPrintf("SetBlockMaxAllowableJerkEntrySpeed: Error - requestedSpeed 0\n");
	}
#endif
	//float prevNominalSpeed;
	//float previousAxisSpeeds[];
	//float curAxisSpeeds[];
	//float prevBlockLength_MM;
	//MotionClass prevBlockMotionClass;
	//float filteredPreviousVelocityXY[];
	// lower of the two nominal speeds
	float jerkLimitedEntrySpeed_MMpSEC = min( currentblock->requestedSpeed, prevblock->requestedSpeed );
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
	// this currently always ends up with zero. Fix.
	debugPrintf( "Initial Setup for Block: %d", currentblock->ringIndex );
	debugPrintf( " - initial jerkLimit: %f\n", jerkLimitedEntrySpeed_MMpSEC );
#endif


	// there is a big problem with this - very short segments sometimes have zero extruder steps, even though they
	// are part of a chain of segments that all form an extrusion path.
	// It doesn't look like a problem here.... but the axis speeds are actually calculated from the number of steps
	// so they go to zero when there's not enough to make a step within the block.
// this may someday help improve transitions, if we can get it reliable information:
#define APPLY_EXTRUSION_ISOLATION
#define FORCE_SLOW_ENDING_EXTRUSION_BEFORE_RETRACT

#ifdef APPLY_EXTRUSION_ISOLATION
	// the highest-joint speed goal only applies between blocks of the same type.
	// When transitioning from pure movement to a drawing-extrusion block, we want the head to come to zero velocity before starting to draw.
	// Similarly, we probably want the head to come to a stop before retracting and moving, though it's possible that this does not matter very much.
	// We can accomplish this in two ways:
	// a. Modify the acceleration-limited entry speed - would have to be done every replan
	// better: b. Modify the jerk-limited entry speed. Only needs to be done once, here.
	// keep it simple
	if ( prevblock->motionClass == MOTION_CLASS_XYZ_DRAW ) {
		// extruder was moving before
#ifdef FORCE_SLOW_ENDING_EXTRUSION_BEFORE_RETRACT
		if ( currentblock->motionClass == MOTION_CLASS_XYZ_RETRACT) {
			// ending moving while drawing, now retracting.
			// NOTE: making this only apply to retraction has the effect of allowing short moves (between draws) that do NOT involve retraction to avoid this slow junction speed. Hopefully that is best.
			// Since we are about to retract, we need to relieve pressure and unwind any advance.
			// (Retract amount is then effectively the advance that is required at the MIN_JERK speed)
			// We do not want to carry over any vectorial planning speed at all. We want the
			// last path to come smoothly to an end, and we want the extruder to accelerate smoothly too.
			//SERIAL_ECHOLN( "(end of draw, starting retraction)" );
			//jerkLimitedEntrySpeed_MMpSEC = MINIMUM_PLANNER_SPEED; // a tiny bit dangerous, if made to go too slow. Possibly needs a little speed left over like XY jerk?
			jerkLimitedEntrySpeed_MMpSEC = min( jerkLimitedEntrySpeed_MMpSEC, DEFAULT_MIN_JERK );
		}
#endif
	}
	else {
		if ( currentblock->motionClass == MOTION_CLASS_XYZ_DRAW ) {
			// starting to move the extruder but haven't been before
			//SERIAL_ECHOLN( "(begin draw)" );
			// TODO: possibly use less Jerk when starting?
			jerkLimitedEntrySpeed_MMpSEC = min( jerkLimitedEntrySpeed_MMpSEC, DEFAULT_MIN_JERK );
		}

	}
#endif

	// another important restriction is to avoid an accumulation of Jerk from several small segments, each a small jerk.
	// we should really look across the last few blocks to do this right.
	// but it may help a lot just to simply look at the path length of this block, and adjust the allowed jerk based on just
	// the path length.
	float axisJerkFactor = 1.0f;

#ifdef APPLY_SHORT_SEGMENT_RULES
	const float MIN_SEGMENT_PAIR_LENGTH_FOR_FULL_JERK_MM = motion_seglen_d;
	const float MIN_SEGMENT_LENGTH_FOR_FULL_JERK_MM      = motion_seglen_s;				// for the first short segment found, after a longer segment

	float pairedBlockLength_MM = currentblock->totalDistance + prevBlockLength_MM;
	if ( pairedBlockLength_MM < MIN_SEGMENT_PAIR_LENGTH_FOR_FULL_JERK_MM ) {
		// this is the second of two short segments. We could end up with too much
		// accumulated jerk within a small distance.
		// Reduce the jerk proportionally.
		//axisJerkFactor = max( .10f, pairedBlockLength_MM / MIN_SEGMENT_PAIR_LENGTH_FOR_FULL_JERK_MM );
		axisJerkFactor = pairedBlockLength_MM / MIN_SEGMENT_PAIR_LENGTH_FOR_FULL_JERK_MM;
		//SERIAL_ECHOPAIR( "\npaired Block Length short: ", pairedBlockLength_MM );
		//SERIAL_ECHOPAIR( "  axisJerkFactor: ", axisJerkFactor );
		//SERIAL_ECHOLN( "" );
	}
	else {
		// the pair is of adequate length... but perhaps we've got a single short one.
		// The jerk is possibly split between a short and long segment.
		if ( currentblock->totalDistance < MIN_SEGMENT_LENGTH_FOR_FULL_JERK_MM ) {
			// don't drop it too much - this could simply be a small one before another long one. Split the jerk between them
			// by limiting this short segment's jerk to half the maximum
			axisJerkFactor = max( .5f, currentblock->totalDistance / MIN_SEGMENT_LENGTH_FOR_FULL_JERK_MM );
			//SERIAL_ECHOPAIR( "\nsingle Block Length short: ", currentblock->totalDistance );
			//SERIAL_ECHOPAIR( "  axisJerkFactor: ", axisJerkFactor );
			//SERIAL_ECHOLN( "" );
		}
	}
#endif

#define JERK_MAKES_INVARIANT // if vectorial jerk is zero, then axis jerk limits can be calculated once
#ifdef JERK_MAKES_INVARIANT  // this only needs to be set up if there is a way for replanning to skip limiting by Jerk

	if ( jerkLimitedEntrySpeed_MMpSEC > MINIMUM_PLANNER_SPEED ) {
		// it may also be axis Jerk limited
		// NOTE: it's likely that this isolated-axis approach, while best for purely mechanical limitation jerk clamping
		//       is not vectorially jerk-constant when both X and Y have significant Jerk. It would tend to have 1.414 times the vectorial jerk then.
		for ( size_t axis=0; axis < DRIVES; ++axis ) {

			if ( prevblock->directionVector[axis] == 0 && currentblock->directionVector[axis] == 0 ) continue; // skip if this axis not doing anything this move
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
			debugPrintf("\nDRIVE %d:\n", axis );
#endif
			float axisSpeedPerBlockNominal = currentblock->directionVector[axis];		// the fraction of vectorial speed this axis will be  going
			float prevAxisSpeedPerNominal  = prevblock->directionVector[axis];		// the fraction of vectorial speed this axis has been going

			// now we get a constant factor for this axis, that represents the (desired, nominal) change in axis speed as a fraction of our nominal speed.
			// so if nothing slows this junction down, this factor * nominal is the actual amount of jerk this axis will see
			// it can range from 0 to 2.0 if both blocks have the same nominal speed. Can go higher if speeds are different e.g. prev has higher nominal speed.
			float kAxisJerk = fabs( axisSpeedPerBlockNominal - prevAxisSpeedPerNominal );  // jerk per mm/sec of junction speed
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
			debugPrintf( "    axisSpeedPerBlockNominal %ff\n", axisSpeedPerBlockNominal );
			debugPrintf( "    prevAxisSpeedPerNominal %f\n", prevAxisSpeedPerNominal );
			//debugPrintf( "    kAxisJerk: %f\n", kAxisJerk );
#endif
			if ( axis <= Y_AXIS ) {
#ifdef APPLY_PREV_SHORT_SEGMENT_RULES
				// the prev. Nominal may not be accurate here... but it may be better.
				float filteredAxisSpeedPerNominal = filteredPreviousVelocityXY[ axis ] / currentblock->requestedSpeed;
				float kAxisJerkToFiltered = fabs( axisSpeedPerBlockNominal - filteredAxisSpeedPerNominal );  // jerk per mm/sec of junction speed

				if ( kAxisJerkToFiltered > kAxisJerk ) {
#ifdef TRACE_FILTERED_AXIS_JERK
					SERIAL_ECHOPAIR( " axisSpeedPerBlockNominal: ", axisSpeedPerBlockNominal );
					SERIAL_ECHOPAIR( " prevAxisSpeedPerNominal: ", prevAxisSpeedPerNominal );
					SERIAL_ECHOPAIR( " kAxisJerk: ", kAxisJerk );
					SERIAL_ECHOPAIR( " filteredAxisSpeedPerNominal: ", prevAxisSpeedPerNominal );
					SERIAL_ECHOPAIR( " kAxisJerkToFiltered: ", kAxisJerkToFiltered );
					if ( kAxisJerk > 0 ) {
						SERIAL_ECHOPAIR( "use filt ratio: ", kAxisJerkToFiltered / kAxisJerk );
					}
					SERIAL_LF();
#endif
					kAxisJerk = kAxisJerkToFiltered;
				}

#ifdef IGNORE_SHORT_PREV_SEG_JERK  // did not seem to help the hairband. Causes apparent (spurious) jerk error checks to prev. segment (since ignored).
				const float MIN_SEGMENT_LENGTH_FOR_FULL_JERK_MM = motion_seglen_d;
				// ignore the jerk to the previous segment if it was extremely short - the filtered one matters more in that case.
				kAxisJerk = interpolate( prevBlockLength_MM / MIN_SEGMENT_LENGTH_FOR_FULL_JERK_MM, kAxisJerkToFiltered, max( kAxisJerk, kAxisJerkToFiltered ) );
#endif
#endif
			}
			else {
				// restore full jerk regardless of segment length if it's not X or Y
				axisJerkFactor = 1.0f;
			}


#define USE_requestedSpeed
#ifdef USE_requestedSpeed
			// calculate the Jerk if we ran at our nominal speed
			float maxPossibleAxisJerk = currentblock->requestedSpeed * kAxisJerk;
#else
			// it's a little more efficient to ratchet the speed down - fewer alterations needed
			// calculate the Jerk if we ran the calculated limit so far - if it's already low enough, we won't change it
			float maxPossibleAxisJerk = jerkLimitedEntrySpeed_MMpSEC * kAxisJerk;

#endif
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
			debugPrintf( "    maxPossibleAxisJerk: %f\n", maxPossibleAxisJerk );
#endif
#ifdef AUTOMATIC_JERK_VARIATION
//#define TRACE_AUTOMATIC_JERK
			float thisAxisJerk = reprap.GetPlatform().ConfiguredInstantDv(axis);		// look at how much Jerk is allowed for this axis

			if ( axis <= Y_AXIS ) {
				// affects only the XY Jerk
				if ( currentblock->motionClass == MOTION_CLASS_XYZ_DRAW ) {
//					if ( true ) {
					const float conservativeBoostFactor = 2.0f;
	 #ifdef USE_GREATER_JERK_VALUE
					// The M205 X or Y value serves as a Minimum value. This can raise it, if speed seems to request it, or angle allows it.
					// Raise the Jerk on shallow angles (effectively) - sharp angles have a greater kAxisJerk, so we use that as an analog for the angle
					// when this code is active, the default: DEFAULT_XYJERK gets set to DEFAULT_MIN_JERK
					// and therefore the job of this code is to opportunistically increase the allowed jerk
					// when the angle is shallow, even though the speed may indicate low jerk should be used.
	#ifdef TRACE_AUTOMATIC_JERK
					static bool once = true;
					if ( once ) {
						once = false;
						SERIAL_ECHO_START;
						SERIAL_ECHOPAIR( "DEFAULT_JERK_INTERCEPT:", DEFAULT_JERK_INTERCEPT );
						SERIAL_ECHOPAIR( "DEFAULT_JERK_SPEED_FACTOR:", DEFAULT_JERK_SPEED_FACTOR );
						SERIAL_LF();
					}
	#endif
					float speedBasedAxisJerk = DEFAULT_JERK_INTERCEPT + currentblock->requestedSpeed * DEFAULT_JERK_SPEED_FACTOR;
					speedBasedAxisJerk = constrain( speedBasedAxisJerk, DEFAULT_MIN_JERK, DEFAULT_FULL_JERK );
					// thisAxisJerk may actually be high if the file was post-processed
					thisAxisJerk = max( thisAxisJerk, speedBasedAxisJerk );		// we raise the Jerk based on the speed.
					// and then raise it (possibly more) if the angle is shallow. When kAxis Jerk is toward 0, meaning straighter, it will interpolate to the
					// maximum (default). When facing a definite change in direction, the possibly-lower axis jerk will be used.

					float autoThisAxisJerk = interpolate( constrainto1( kAxisJerk * conservativeBoostFactor ), DEFAULT_FULL_JERK, thisAxisJerk );
					thisAxisJerk = max( thisAxisJerk, autoThisAxisJerk );
	#ifdef TRACE_AUTOMATIC_JERK
					SERIAL_ECHO_START;
					SERIAL_ECHOPAIR( "spd:", currentblock->requestedSpeed );
					SERIAL_ECHOPAIR( " k:", kAxisJerk );
					SERIAL_ECHOPAIR( " s:", speedBasedAxisJerk );
					SERIAL_ECHOPAIR( " a:", autoThisAxisJerk );
					SERIAL_ECHOPAIR( " j:", thisAxisJerk );
					SERIAL_LF();
	#endif

	 #else
					// modulate it if we should. We reduce the Jerk limit if the speed is low. Low speed is taken to mean higher quality is desired.
					// actively manage the Jerk according to speed, if it appears unmanaged (no post-proc)
					if ( thisAxisJerk == DEFAULT_FULL_JERK ) {
						// the jerk has not been messed with that we know of
		 #define KEEP_HIGHER_JERK_ON_SHALLOW_JUNCTIONS
		 #ifdef KEEP_HIGHER_JERK_ON_SHALLOW_JUNCTIONS
						float speedBasedAxisJerk = DEFAULT_JERK_INTERCEPT + currentblock->requestedSpeed * DEFAULT_JERK_SPEED_FACTOR;
						// kAxisJerk will be low when we want to use the full Jerk value, and high when we need to reduce Jerk (1.0 is an axis-aligned right-angle, for example)
						// but even at e.g. .5 it's a significant angle, and we want to be careful with Jerk.
						// so use a factor to boost the interpolation over to the more conservative jerk
						// However, we do probably want a bit more jerk when doing e.g. a hexagon.
						thisAxisJerk = interpolate( constrainto1( kAxisJerk * conservativeBoostFactor ), thisAxisJerk, speedBasedAxisJerk );
						// NOTE: this may still not be appropriate if we're not looking at whether it's likely that raising the Jerk will allow constant speed across the junction
		 #else
						thisAxisJerk = DEFAULT_JERK_INTERCEPT + currentblock->requestedSpeed * DEFAULT_JERK_SPEED_FACTOR;
		 #endif
						// we must clamp bidirectionally
						thisAxisJerk = constrain( thisAxisJerk, DEFAULT_MIN_JERK, DEFAULT_FULL_JERK );
		 #ifdef TRACE_AUTOMATIC_JERK
						SERIAL_ECHO_START;
						SERIAL_ECHOPAIR( "speed:", currentblock->requestedSpeed );
		  #ifdef KEEP_HIGHER_JERK_ON_SHALLOW_JUNCTIONS
						SERIAL_ECHOPAIR( " kAxisJerk:", kAxisJerk );
		  #endif
						SERIAL_ECHOPAIR( " jerk:", thisAxisJerk );
						SERIAL_LF();
		 #endif
					}

	 #endif
				}
				else {
					// not drawing. Use maximum allowed Jerk, always.
					thisAxisJerk = DEFAULT_FULL_JERK;
				}
			}

			float axisJerkAllowed = axisJerkFactor * thisAxisJerk;
#else
			float axisJerkAllowed = axisJerkFactor * reprap.GetPlatform().ConfiguredInstantDv(axis);
#endif
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
			debugPrintf( "    axisJerkAllowed: %f\n", axisJerkAllowed );
#endif
			if ( maxPossibleAxisJerk > axisJerkAllowed ) {
				// the jerk would exceed the limit for this axis
				// adjust the entry speed limit
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
				debugPrintf("\n");
				debugPrintf( "    maxPossibleAxisJerk > axisJerkAllowed for axis: %d\n", (int)axis );
				debugPrintf( "    maxPossibleAxisJerk: %f\n", maxPossibleAxisJerk );
				debugPrintf( "    reprap.GetPlatform().ConfiguredInstantDv(axis): %f\n", reprap.GetPlatform().ConfiguredInstantDv(axis) );
				debugPrintf( "                                 EffectiveaxisJerk: %f\n", EffectiveaxisJerk(axis) );
				if ( axisJerkFactor != 1.0f ) debugPrintf( "  axisJerkFactor: %f\n", axisJerkFactor );
				debugPrintf( "    axisSpeedPerBlockNominal: %f\n", axisSpeedPerBlockNominal );
				debugPrintf( "    prevAxisSpeedPerNominal: %f\n", prevAxisSpeedPerNominal );
				debugPrintf( "    kAxisJerk: %f\n", kAxisJerk );
#endif

				float allowedEntryFraction = axisJerkAllowed / maxPossibleAxisJerk;
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
				debugPrintf( "    allowedEntryFraction: %f\n", allowedEntryFraction );
#endif
#ifdef USE_requestedSpeed
				jerkLimitedEntrySpeed_MMpSEC = min( jerkLimitedEntrySpeed_MMpSEC, currentblock->requestedSpeed * allowedEntryFraction );
#else
				if ( jerkLimitedEntrySpeed_MMpSEC * allowedEntryFraction > jerkLimitedEntrySpeed_MMpSEC ) {
					debugPrintf("*** jerkLimitedEntrySpeed_MMpSEC going up??? *****" );
				}
				jerkLimitedEntrySpeed_MMpSEC = jerkLimitedEntrySpeed_MMpSEC * allowedEntryFraction;
#endif
#ifdef CHECK_SetBlockMaxAllowableJerkEntrySpeed
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
				debugPrintf( "    jerkLimitedEntrySpeed_MMpSEC: %f\n", jerkLimitedEntrySpeed_MMpSEC );
#endif
				// check the result. In this case the current block is Q, and previous is P
				//float fractionalSpeedP = jerkLimitedEntrySpeed_MMpSEC / prevblock->requestedSpeed;
				//float fractionalSpeedQ = jerkLimitedEntrySpeed_MMpSEC / currentblock->requestedSpeed;
				//float axisSpeedP = ( prevblock->directionVector[axis]    * prevblock->requestedSpeed )    * fractionalSpeedP;
				//float axisSpeedQ = ( currentblock->directionVector[axis] * currentblock->requestedSpeed ) * fractionalSpeedQ;
				float axisSpeedP = ( prevblock->directionVector[axis]    * jerkLimitedEntrySpeed_MMpSEC );
				float axisSpeedQ = ( currentblock->directionVector[axis] * jerkLimitedEntrySpeed_MMpSEC );
				float jerk = axisSpeedQ - axisSpeedP;
				if ( fabs(jerk) > axisJerkAllowed * 1.01f ) { // a little tolerance cuts down noise lines
					debugPrintf( "  Check FAILED!: axis %d jerk: %f vs %f allowed, from maxPossibleAxisJerk %f (%f to %f), entrySpeed of %f\n", axis, jerk, axisJerkAllowed, maxPossibleAxisJerk, axisSpeedP, axisSpeedQ, jerkLimitedEntrySpeed_MMpSEC );
				}
#endif
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
				debugPrintf( "axis %d limit: %f", axis, jerkLimitedEntrySpeed_MMpSEC );
				//debugPrintf("\n");
#endif
			}
			else {
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
				debugPrintf("doesn't hit jerk limit\n");
#endif
			}
		}

	}
#endif

	currentblock->maxStartSpeed_MMpSEC = jerkLimitedEntrySpeed_MMpSEC;
#ifdef TRACE_SetBlockMaxAllowableJerkEntrySpeed
	debugPrintf( " final limit: %f\n", jerkLimitedEntrySpeed_MMpSEC );
#endif
	}




//#define TRACE_MaximizeJunctionSpeed
#define CHECK_PLANNING_ASSUMPTIONS
// given two blocks, make the junction between them as quick as possible
// P is the first block, whose final speed we will control
// Q is the second block, whose entry speed we will control
bool DDA::MaximizeJunctionSpeed( DDA* p, DDA* q ) {
#ifdef TRACE_MaximizeJunctionSpeed
	debugPrintf( "MaximizeJunctionSpeed p: %d", p->ringIndex );
	debugPrintf( " q: %d", q->ringIndex );
	debugPrintf("\n");
#endif
	// it's possible that P already has a full-speed exit velocity.
	// Assumption: It is impossible for new blocks or any backwards planning to REDUCE the exit factor of P
	//             - assuming that all blocks begin life with an exit speed that represents maximum allowable final stopping jerk, at most,
	//               and minimum planner speed for the combined vectorial velocity, at the least
	if ( q->startSpeed == p->requestedSpeed  ) {
#ifdef CHECK_PLANNING_ASSUMPTIONS
		// TODO: check to make sure that Q's entry is compatible with P's exit.
		if ( q->startSpeed > q->requestedSpeed || q->startSpeed > q->decelLimitedEntrySpeed_MMpSEC ) {
			debugPrintf( "MaximizeJunctionSpeed: q->startSpeed high: %f\n", q->startSpeed );
		}

#endif
#ifdef TRACE_MaximizeJunctionSpeed
		debugPrintf("    - same\n");
#endif
		return false; // can't go any higher. We can assume that Q has already been correctly planned for P's existing exit rate.
	}

	bool changed = false;

	// with invariant jerk limited speed, all we need to do is set the entry speed to the lower
	// of the deceleration limited speed and the jerk limited speed
	float maxAllowedEntrySpeedQ = min( q->maxStartSpeed_MMpSEC, q->decelLimitedEntrySpeed_MMpSEC );

#ifndef DO_FORWARD_PASS
	// TODO: a forward pass to make sure that P can actually achieve the desired exit speed by accelerating.
	// for now, we can knock it down to a safe amount by using whatever the initial or replanning may have set it to, as a conservative limit.
	float maxAccelerationLimitedExitSpeedP = SpeedAchieved( p->startSpeed, p->acceleration, p->totalDistance );
	maxAllowedEntrySpeedQ = min( maxAllowedEntrySpeedQ, maxAccelerationLimitedExitSpeedP );
#endif

	if ( q->startSpeed != maxAllowedEntrySpeedQ ) {
		// it needs to be altered
#ifdef TRACE_MaximizeJunctionSpeed
		debugPrintf( " q->maxStartSpeed_MMpSEC: %f ", q->maxStartSpeed_MMpSEC );
		debugPrintf( " q->decelLimitedEntrySpeed_MMpSEC: %f ", q->decelLimitedEntrySpeed_MMpSEC );
#ifndef DO_FORWARD_PASS
		debugPrintf( " maxAccelerationLimitedExitSpeedP: %f ", maxAccelerationLimitedExitSpeedP );
#endif
		debugPrintf( " q->startSpeed != maxAllowedEntrySpeedQ: is %f\n ", q->startSpeed );
		//SERIAL_ECHOPAIR( " diff: ", q->startSpeed - maxAllowedEntrySpeedQ );
#endif
		if ( ! ( p->state == DDA::executing || p->state == DDA::frozen ) ) {
			// since it would be redundant to store both the entry speed and the exit speed, only the entry speed is stored (last block always has implied zero exit speed)
			q->recalculate_flag = true;
			p->recalculate_flag = true; // since we changed the junction speed, both blocks are affected and need trapezoids recalculated.
			q->startSpeed = maxAllowedEntrySpeedQ;
			p->endSpeed = q->startSpeed;
			SetBlockMaxAllowableDecelerationEntrySpeed( p, q->startSpeed ); // this MUST be calculated and set every time the exit speed is changed
#ifdef TRACE_MaximizeJunctionSpeed
			debugPrintf( " CHANGED start to: %f\n", q->startSpeed );
#endif
			changed = true;
			// NOTE: the full change is not yet complete. P's exit factor must be updated and its trapezoid recalculated.
		}

	}

#ifdef TRACE_MaximizeJunctionSpeed
	debugPrintf("\n");
#endif
	return changed;
}

/* Needs:
 * 	To know how far back we can go to get to the executing block or oldest changeable block.
 */
//#define TRACE_RippleChangesBack

// goes from the new head back through the last blocks until no change is made
// Usually can raise the exit velocity of earlier blocks.
// Returns pointer to last block altered, in case a forward pass is required.
DDA* DDA::RippleChangesBack( DDA* newestBlock ) {
	// this is not normally called when tail == head, because it is always called and only called when a new block is added to the buffer.
	// but the tail could finish executing just as we get here. Assume nothing about the tail will remain constant.
	//uint8_t originalTail = block_buffer_tail;
#ifdef TRACE_RippleChangesBack
	debugPrintf("RippleChangesBack\n");
#endif
#ifdef REPLAN_LIMIT_TIME_BASED_US
	// add up the total nominal time of all the segments from (but not including) the currently executing block
	// until we accumulate the minimum planning buffer time that guarantees we can finish all this planning
	// This can be a bit time-consuming, but if it keeps us from balking, it's probably worth it.
	// so last real block is one prev to head. It does not need changes rippled back, because it can only act as a "next" block,
	// not a "current" block from the point of view of the main maximization loop below.
	uint32_t total_us = 0;
	// the tail is currently executing, and may finish at any time
	//uint8_t timeBlock = next_block_index( block_buffer_tail );
	uint8_t timeBlock = originalTail;  // we increment before examining, so first one examined will not be the one executing

	// there was at least one block in the queue. It could be just the newest, unprocessed one. In that case, currentBlockNo is originalTail.
	if ( timeBlock == newestBlock ) {
		// not an error, just a waste of time, and possible risk of loop being mishandled.
		//SERIAL_ECHOLN( "only one in buffer looking for replan time");
		return NULL; // no replanning possible
		}

	// there are guaranteed to be two or more blocks in the queue.
	// Two is not enough for replanning. The executing one is off-limits, and the other one would be "current".
	// In that case, the loop will immediately exit when it checks the exit condition.

	bool bEnoughTime = false;
	// the more blocks in the queue, the more time would be required to replan them all
	// so if we only need to replan a couple, we don't need the full time.
	// but there's a bit of overhead on the whole process, so allow for that, too.
	// in collecting some basic stats, it looks like there is a bit less than one ms needed per actual replanned block.
	// The overhead is probably quite small.
	// motion_replan_time_us is PER BLOCK
	const uint32_t REPLAN_OVERHEAD_US = 1000;
	// this will actually get a count that excludes the newest block.
	// So with one planned and one newest block, this will be 1.
	// But if one is executing... the "planned" one is actually off-limits.
	// if movesplanned is 1, it refers to the executing one, which is off-limits.
	// if movesplanned is 2, there is one replanning (it and the newest) that may take place
	uint8_t movesPlanned = movesplanned();

	uint32_t timeNeeded_us = motion_replan_time_us * movesPlanned + REPLAN_OVERHEAD_US;
	// with every block added up, we reduce the time needed, since there are fewer remaining to plan
	do {
		timeBlock = next_block_index( timeBlock );
		// check first, because we can't use the newest block (no replanning possible), and it's no use if we get that far.
		if ( timeBlock == newestBlock ) {
			// we got to the current (last) block, meaning that there is not enough time to
			// plan even the last block
			// no time at all
#ifdef DEBUG_BUILD
			if ( movesPlanned > 2 ) SERIAL_ECHOLN("z");  // if there's a queue but it's too short we'd like to know
			//use this to see how many are in the buffer when there's not enough total time
			//SERIAL_ECHOPAIR("z", (unsigned long)movesplanned() ); SERIAL_ECHOLN("");
#endif
				return NULL;
		}


		DDA* pForward = &block_buffer[ timeBlock ];
		if ( ! ( pForward->state == DDA::executing ) )
			{
			total_us += pForward->estimatedNominalTime_USEC;
			}

		bEnoughTime = ( total_us >= timeNeeded_us );  // controllable by M code
		timeNeeded_us -= motion_replan_time_us; // decrease the time needed to process the remainder
		} while ( ! bEnoughTime );

	// if the loop exited and got here, timeBlock represents the block AFTER which
	// there would be enough time to replan. So it is appropriate for the loop below to
	// stop when the "current" becomes that block. It should not be replanned.
	// tooFarBack is not allowed to point to the current block. It must point to an earlier block.
	// - one previous will be useless, it takes two to be "too far" when one can be replanned.
	uint8_t tooFarBack = timeBlock;

#else
	// still triggered "r" debug tracing sometimes: const int DISTANCE_FROM_TAIL_TO_REPLAN = 4; // don't get closer than this to the tail
	//const int DISTANCE_FROM_TAIL_TO_REPLAN = 6; // don't get closer than this to the tail
	//const int DISTANCE_FROM_TAIL_TO_REPLAN = 14; // don't get closer than this to the tail - MUST BE < 1/2 of BLOCK_BUFFER_SIZE
	//uint8_t tooFarBack = relative_block_index( block_buffer_tail, DISTANCE_FROM_TAIL_TO_REPLAN );
	//uint8_t tooFarBack = block_buffer_tail;  // all the way, all the time
#endif

	// current is the very last, as yet unprocessed block. It can only act as a "next" block right now.
	// the tail is dynamic. We must keep checking to make sure we don't try to alter it.
	// we can either work in integer block numbers, or pointers.
	DDA* lastChanged = NULL;

	DDA* current = newestBlock;  // the newest block (not fully planned yet)
	DDA* next = nullptr;
	//uint8_t currentBlockNo = newestBlock;
	bool changed = false;

	do {
		// move one block backwards
#ifdef TRACE_RippleChangesBack
		debugPrintf( " before maximize, current ringIndex: %d\n", current->ringIndex );
#endif
		next = current;
		//currentBlockNo = prev_block_index( currentBlockNo );
#ifdef REPLAN_LIMIT_TIME_BASED_US
		// check the original tail in case it catches up to us while  we're processing everything.
		// - if our calculations above are correct, we will still be able to do the replanning
		//   during the execution of the current tail, so it's not necessarily an error... but it's risky. Also likely to be marked "busy".
		//if ( currentBlockNo == tooFarBack || currentBlockNo == block_buffer_tail ) {
#else
		//if ( currentBlockNo == tooFarBack || currentBlockNo == originalTail || currentBlockNo == block_buffer_tail ) {
#endif
			//SERIAL_ECHOLN( "(tail)");
		//	break;
		//}
		current = current->prev;
		//SERIAL_ECHOPAIR( " after, currentBlockNo: ", (uint32_t)currentBlockNo );
		//SERIAL_ECHOPAIR( " num: ", (uint32_t)BlockNum( current ) );
		//SERIAL_ECHOLN("");
#ifdef TRACE_RippleChangesBack
		debugPrintf( " current: %d, next %d\n", current->ringIndex, next->ringIndex );
#endif

		if ( current->state == DDA::DDAState::executing ) {
#ifdef DEBUG_BUILD
			debugPrintf("b\n");
#endif
			break;
		}
		else if ( current->state == DDA::DDAState::frozen ) {
#ifdef DEBUG_BUILD
			debugPrintf("f\n");
#endif
			break;
		}
		else if ( current->state == DDA::DDAState::empty ) {
#ifdef DEBUG_BUILD
			// no surprise debugPrintf("e\n");
#endif
			break;
		}
		else if ( current->state == DDA::DDAState::completed ) {
#ifdef DEBUG_BUILD
			debugPrintf("c\n");
#endif
			break;
		}
		else if ( current->state != DDA::DDAState::provisional ) {
			// unexpected condition for beginning. Possibly could happen initially though. Refine as needed.
#ifdef DEBUG_BUILD
			debugPrintf("#\n");
#endif
			break;
		}

		// replan the junction between the current and next block. The first time through this, "next" is the newly added block.
		// Both blocks may be altered.
#ifdef TRACE_RippleChangesBack
		debugPrintf( "current is provisional - maximizing\n" );
#endif
		changed = MaximizeJunctionSpeed( current, next );  // returns true if changed "current". Will not change an executing block.
#ifdef TRACE_RippleChangesBack
		debugPrintf( " MaximizeJunctionSpeed() changed? %d\n", changed );
#endif
		if ( changed ) lastChanged = current;
	} while ( changed );
	// experiment with doing them all repeatedly (in case there are sections that need it, separated by full-speed bits that don't need changes)
	//} while ( true );

#ifdef TRACE_RippleChangesBack
	if ( lastChanged ) debugPrintf("returning lastChanged of %d", lastChanged->ringIndex );
	else debugPrintf("none changed\n");
#endif
	return lastChanged;
}

#ifdef DO_FORWARD_PASS

/* to suppress resonances:
(Y only, really)
 Planning needs to chain predicted segment times during the forward pass.
 If a set of segments forms a complete cycle of direction change in a particular axis of interest, if the time would be too short, then the latest segment is prolonged by keeping the
 jerk artificially low. This takes energy out and lengthens the time, moving it away from the resonant period.

*/
//#define TRACE_MaximizeForwardSpeeds

void DDA::MaximizeForwardSpeeds( DDA* firstChangedBlock, DDA* newestBlock ) {
	// Maximizes the junction speeds by projecting the acceleration that is possible
	//float prevExitSpeed = 0;
#ifdef TRACE_MaximizeForwardSpeeds
	debugPrintf( "\nMaximizeForwardSpeeds\n" );
	debugPrintf( " Starting with Block: %d (newest block %d)\n", firstChangedBlock->ringIndex, newestBlock->ringIndex );
#endif
	//uint8_t blockNo = BlockNum( firstChangedBlock );
	if ( firstChangedBlock == newestBlock ) return; // should never happen - nothing buffered at all

	//uint8_t nextBlockNo = next_block_index( blockNo );

	DDA* block     = firstChangedBlock;
	DDA* nextBlock =  firstChangedBlock->next;

	DDA* newHead = newestBlock->next;

	for ( ; nextBlock != newHead; block = nextBlock, nextBlock = nextBlock->next ) {
#ifdef TRACE_MaximizeForwardSpeeds
		debugPrintf( " Block: %d", block->ringIndex );
		debugPrintf( " Next: %d",  nextBlock->ringIndex );
#endif

#ifdef DEFER_REPLAN_BLOCKS
		// impossible for any of the blocks to be busy while their recalculate flag is still set
		// basic assert here, because we don't go forward on blocks that were not replanned backwards
		//if ( ! block->recalculate_flag ) SERIAL_ERRORLN( "Forward Pass - Block not already marked as needing recalc!!! " );

		// we are not yet marked busy as of this instant. We may have time to recalculate the trapezoid
		// to take advantage of replanning.
		float maxAllowedExitSpeedP;

		if ( nextBlock->startSpeed <= block->startSpeed ) {
			// it doesn't matter if this block can accelerate, because the next block won't accept a higher speed.
			// So it's automatically planned to exit as quickly as possible.
			// That means the next block is correctly set up with its entry speed.
			// It still needs its trapezoid recalculated, because it obviously was changed during the backwards pass.
			maxAllowedExitSpeedP = nextBlock->startSpeed;  // this will leave it unchanged
		}
		else {
			// the backward pass set an optimistic entry speed, ignoring whether or not that speed was possible to achieve
			// from earlier segments accelerating.
			// We must now look whether we can actually achieve those speeds.
			float maxAccelerationLimitedExitSpeedP = SpeedAchieved( block->startSpeed, block->acceleration, block->totalDistance );
			maxAllowedExitSpeedP  = min( nextBlock->startSpeed, maxAccelerationLimitedExitSpeedP );
		}

		// It's almost certain that the trapezoid must be replanned, because this or another segment's parameters have been changed on the backwards pass.
		// So we can't skip this just because we're not actually changing the exit speed.
		// modify the next block's entry speed
		// we are going to try to change its exit speed
		// must do this as soon as possible to stay ahead of the stepper execution
		calculate_trapezoid_for_block( block, block->startSpeed / block->requestedSpeed, maxAllowedExitSpeedP / block->requestedSpeed );
#ifdef DEBUG_BUILD
		if ( block->startSpeed < block->requestedSpeed && block->planState < PLAN_STATE_FULLSPEED ) {
			++block->planState;		// count the replans
		}
		else {
			block->planState = PLAN_STATE_FULLSPEED;
		}
#endif
		nextBlock->startSpeed = maxAllowedExitSpeedP;
#ifdef DEBUG_BUILD
		if ( block->recalculate_flag ) {
			// impossible!
			debugPrintf( "Forward Pass - not retrapezoided!!! " ); // should not be possible, if we are balking when needing recalc
		}
#endif
		// don't bother checking this: if ( block->planState == PLAN_STATE_COMPLETE ) {
			// this is not the latest block. It's possible that the motion plan has executed
			// to this point and caught up with us while we were replanning.
			// to do this correctly, we must turn off interrupts, to avoid kicking
			// the ISR just when it has fired off on its own (not sure if that is possible)
			if ( executionBalked ) {
				// it could be waiting for this block to have been planned
				// This is probably really bad. Should never happen during printing.
				// Trigger the ISR
				st_kick();
				executionBalked = false;
#ifdef DEBUG_BUILD
				debugPrintf( "k" );  // for Kicked - possibly this serial action helps the stalling a little? Seems to run more quickly with this, and don't see the'r's that we otherwise see.
#endif
			}
		//}

#else
		// blocks can be made busy at any point in this process.
		if ( block->state == DDA::executing ) {
			// the true first block will almost always be busy and unchangeable if it's really the first block. But we should never see that.
			debugPrintf( "\tMaximizeForwardSpeeds BUSY!!!\t" );
			// its trapezoid will not have been recalculated! But it's too late now.

#ifdef SKIP_FIRST_BLOCK

			// let's skip an additional block to buy us enough time to be sure that the second block does not begin to be executed while we are replanning it.
			nextBlockNo = next_block_index( nextBlockNo );
			block = nextBlock;

			//SERIAL_ECHOPAIR( " skip Block: ", (uint32_t)nextBlockNo );
			if ( nextBlockNo == newestBlock ) break;
			nextBlock = &block_buffer[ nextBlockNo ];
#endif
			// perhaps we should carry on. After all, what we are really doing is fixing up the NEXT block's entry, not our
			// own. But we can skip re-doing our trapezoid. Can flag it as done though to prevent a big stall.
			// go on and modify the next block entry continue;
			// So: do we need to fix our own entries at all, before carrying on?
			// it's too late for us - but make sure the next block's entry is reset.block->startSpeed = (float)block->initial_rate / block->nominal_rate;
			//nextBlock->startSpeed = nextBlock->startSpeed;  // fix it back up to what's actually executing
			continue;
		}

		// basic assert here, because we don't go forward on blocks that were not replanned backwards
		if ( ! block->recalculate_flag ) {
			debugPrintf( "\nForward Pass - Block not already marked as needing recalc!!!\n" );
		}

		// we are not yet marked busy as of this instant. We may have time to recalculate the trapezoid
		// to take advantage of replanning.
		// However, at any time during this process, execution could begin on our block, and even possibly on the next block.
		// We really only need to check the next block down below, because it's the worst case.
		// The busy flags stay set after the block has been executed, so if the next block's flag is busy, so is ours.

		// the backward pass already calculated this, and set startSpeed to it. No need to redo that calculation here.
		//float maxAllowedEntrySpeedQ = min( nextBlock->decelLimitedEntrySpeed_MMpSEC, nextBlock->maxStartSpeed_MMpSEC );  // combo of deceleration and jerk limitations

		float maxAllowedExitSpeedP;

		if ( nextBlock->startSpeed <= block->startSpeed ) {
			// it doesn't matter if this block can accelerate, because the next block won't accept a higher speed.
			// So it's automatically planned to exit as quickly as possible.
			// That means the next block is correctly set up with its entry speed.
			// It still needs its trapezoid recalculated, because it obviously was changed during the backwards pass.
			maxAllowedExitSpeedP = nextBlock->startSpeed;  // this will leave it unchanged
		}
		else {
			// the backward pass set an optimistic entry speed, ignoring whether or not that speed was possible to achieve
			// from earlier segments accelerating.
			// We must now look whether we can actually achieve those speeds.
			float maxAccelerationLimitedExitSpeedP = SpeedAchieved( block->startSpeed, block->acceleration, block->totalDistance );
			maxAllowedExitSpeedP  = min( nextBlock->startSpeed, maxAccelerationLimitedExitSpeedP );
		}

		// setting this flag signals the block execution unit to wait. We really don't want this to happen, though.
		//block->recalculate_flag = true;  // should really already be true, if we are starting at the latest block modified by the backward pass.
		if ( nextBlock->state == DDA::executing  ) { // aw shucks - all that calculation and we can't use it
#if defined(DEFER_REPLAN_BLOCKS) || defined(DEFER_UNPLANNED_BLOCK_EXECUTION)
			debugPrintf( "Forward Pass - next block Busy!!! " ); // should not be possible, if it's marked as needing recalc.
#endif
			// NOTE: need to examine how the forward planning is affected by this.
			// We may need to reset the next block's entry speed back to what's actually being executed.
			// e.g.
			// TODO: check to see if this is actually correct
			//nextBlock->startSpeed = (float)nextBlock->initial_rate / nextBlock->nominal_rate;  // fix it back up to what's actually executing
		}
		else {
			// next block is not busy
			// It's almost certain that the trapezoid must be replanned, because this or another segment's parameters have been changed on the backwards pass.
			// So we can't skip this just because we're not actually changing the exit speed.
			// modify the next block's entry speed
			// we are going to try to change its exit speed
			// must do this as soon as possible to stay ahead of the stepper execution
			// todo: find out if this was successful. If not, revert the startSpeed. Or handle that in the function.
			//calculate_trapezoid_for_block( block, block->startSpeed / block->requestedSpeed, maxAllowedExitSpeedP / block->requestedSpeed );
			block->endSpeed = maxAllowedExitSpeedP;
			block->RecalculateMove();
			// TODO: check the replan flag - if it's still set, the block was already busy and could not actually be replanned
			if ( ! block->recalculate_flag ) {
				// flag was reset - it was not busy. It's safe to set the next block's entry speed to the new value.
				nextBlock->startSpeed = maxAllowedExitSpeedP;
			}

		}

#endif // not balking
	}

#ifdef TRACE_MaximizeForwardSpeeds
	debugPrintf("done MaximizeForwardSpeeds()\n");
#endif
}
#endif // POLYPRINTER



void DDA::DoPlannerLookahead( DDA* newestMove )
{
	DDA* earliestChangedBlock = RippleChangesBack( newestMove );

	if ( earliestChangedBlock ) {
		MaximizeForwardSpeeds( earliestChangedBlock, newestMove );
	}

}

#else


// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
// We declare this inline because it is only used once, in DDA::DoLookahead
inline bool DDA::IsDecelerationMove() const
{
	return decelDistance == totalDistance					// the simple case - is a deceleration-only move
			|| (topSpeed < requestedSpeed					// can't have been intended as deceleration-only if it reaches the requested speed
				&& decelDistance > 0.98 * totalDistance		// rounding error can only go so far
				&& prev->state == DDA::provisional			// if we can't adjust the previous move then we don't care (and its figures may not be reliable if it has been recycled already)
				&& prev->decelDistance > 0.0);				// if the previous move has no deceleration phase then no point in adjus6ting it
}

// Try to increase the ending speed of this move to allow the next move to start at targetNextSpeed
/*static*/ void DDA::DoLookahead(DDA *laDDA)
pre(state == provisional)
{
//	if (reprap.Debug(moduleDda)) debugPrintf("Adjusting, %f\n", laDDA->targetNextSpeed);
	unsigned int laDepth = 0;
	bool goingUp = true;

	for(;;)					// this loop is used to nest lookahead without making recursive calls
	{
		bool recurse = false;
		if (goingUp)
		{
			// We have been asked to adjust the end speed of this move to match the next move starting at targetNextSpeed
			if (laDDA->topSpeed >= laDDA->requestedSpeed)
			{
				// This move already reaches its top speed, so just need to adjust the deceleration part
				laDDA->endSpeed = laDDA->requestedSpeed;		// remove the deceleration phase
				laDDA->CalcNewSpeeds();							// put it back if necessary
			}
			else if (laDDA->IsDecelerationMove())
			{
				// This is a deceleration-only move, so we may have to adjust the previous move as well to get optimum behaviour
				if (laDDA->prev->state == provisional)
				{
					laDDA->endSpeed = laDDA->requestedSpeed;
					laDDA->CalcNewSpeeds();
					const float maxStartSpeed = sqrtf(fsquare(laDDA->endSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
					if (maxStartSpeed < laDDA->requestedSpeed)
					{
						laDDA->prev->targetNextSpeed = maxStartSpeed;
						// Still provisionally a decelerate-only move
					}
					else
					{
						laDDA->prev->targetNextSpeed = laDDA->requestedSpeed;
					}
					recurse = true;
				}
				else
				{
					// This move is a deceleration-only move but we can't adjust the previous one
					laDDA->hadLookaheadUnderrun = true;
					laDDA->endSpeed = min<float>(sqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance)),
													laDDA->requestedSpeed);
					laDDA->CalcNewSpeeds();
				}
			}
			else
			{
				// This move doesn't reach its requested speed, but it isn't a deceleration-only move
				// Set its end speed to the minimum of the requested speed and the highest we can reach
				laDDA->endSpeed = min<float>(sqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance)),
												laDDA->requestedSpeed);
				laDDA->CalcNewSpeeds();
			}
		}
		else
		{
			// Going back down the list
			// We have adjusted the end speed of the previous move as much as is possible and it has adjusted its targetNextSpeed accordingly.
			// Adjust this move to match it.
			laDDA->startSpeed = laDDA->prev->targetNextSpeed;
			const float maxEndSpeed = sqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
			if (maxEndSpeed < laDDA->endSpeed)
			{
				// Oh dear, we were too optimistic! Have another go.
				laDDA->endSpeed = maxEndSpeed;
				laDDA->CalcNewSpeeds();
			}
		}

		if (recurse)
		{
			// Still going up
			laDDA = laDDA->prev;
			++laDepth;
			if (reprap.Debug(moduleDda))
			{
				debugPrintf("Recursion start %u\n", laDepth);
			}
		}
		else
		{
			// Either just stopped going up. or going down
			laDDA->RecalculateMove();

			if (laDepth == 0)
			{
//				if (reprap.Debug(moduleDda)) debugPrintf("Complete, %f\n", laDDA->targetNextSpeed);
				return;
			}

			laDDA = laDDA->next;
			--laDepth;
			goingUp = false;
		}
	}
}
#endif


// Try to push babystepping earlier in the move queue, returning the amount we pushed
float DDA::AdvanceBabyStepping(float amount)
{
	DDA *cdda = this;
	while (cdda->prev->state == DDAState::provisional)
	{
		cdda = cdda->prev;
	}

	// cdda addresses the earliest un-prepared move, which is the first one we can apply babystepping to
	// Allow babystepping Z speed up to 10% of the move top speed or up to half the Z jerk rate, whichever is lower
	float babySteppingDone = 0.0;
	while(cdda != this)
	{
		float babySteppingToDo = 0.0;
		if (amount != 0.0 && cdda->xyMoving)
		{
			// If not on a delta printer, check that we have a DM for the Z axis
			bool ok = (cdda->isDeltaMovement || cdda->pddm[Z_AXIS] != nullptr);
			if (!ok)
			{
				cdda->pddm[Z_AXIS] = DriveMovement::Allocate(Z_AXIS, DMState::idle);
				ok = (cdda->pddm[Z_AXIS] != nullptr);
			}

			if (ok)
			{
				// Limit the babystepping Z speed to the lower of 0.1 times the original XYZ speed and 0.5 times the Z jerk
				const float maxBabySteppingAmount = cdda->totalDistance * min<float>(0.1, 0.5 * reprap.GetPlatform().ConfiguredInstantDv(Z_AXIS)/cdda->topSpeed);
				babySteppingToDo = constrain<float>(amount, -maxBabySteppingAmount, maxBabySteppingAmount);
				cdda->directionVector[Z_AXIS] += babySteppingToDo/cdda->totalDistance;
				cdda->totalDistance *= cdda->NormaliseXYZ();
				cdda->RecalculateMove();
				babySteppingDone += babySteppingToDo;
				amount -= babySteppingToDo;
			}
		}

		// Even if there is no babystepping to do this move, we may need to adjust the end coordinates
		cdda->endCoordinates[Z_AXIS] += babySteppingDone;
		if (cdda->isDeltaMovement)
		{
			for (size_t tower = 0; tower < DELTA_AXES; ++tower)
			{
				cdda->endPoint[tower] += (int32_t)(babySteppingDone * reprap.GetPlatform().DriveStepsPerUnit(tower));
				if (babySteppingToDo != 0.0)
				{
					int32_t steps = (int32_t)(babySteppingToDo * reprap.GetPlatform().DriveStepsPerUnit(tower));
					DriveMovement* const pdm = cdda->pddm[tower];
					if (pdm != nullptr)
					{
						if (pdm->direction)		// if moving up
						{
							steps += (int32_t)pdm->totalSteps;
						}
						else
						{
							steps -= (int32_t)pdm->totalSteps;
						}
						if (steps >= 0)
						{
							pdm->direction = true;
							pdm->totalSteps = (uint32_t)steps;
						}
						else
						{
							pdm->direction = false;
							pdm->totalSteps = (uint32_t)(-steps);
						}
					}
				}
			}
		}
		else
		{
			cdda->endPoint[Z_AXIS] += (int32_t)(babySteppingDone * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));
			if (babySteppingToDo != 0.0)
			{
				int32_t steps = (int32_t)(babySteppingToDo * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));
				DriveMovement* const pdm = cdda->pddm[Z_AXIS];			// must be non-null because we allocated one earlier if necessary
				if (pdm->state == DMState::moving)
				{
					if (pdm->direction)		// if moving up
					{
						steps += (int32_t)pdm->totalSteps;
					}
					else
					{
						steps -= (int32_t)pdm->totalSteps;
					}
				}
				else
				{
					pdm->state = DMState::moving;
				}

				if (pdm != nullptr)
				{
					if (steps >= 0)
					{
						pdm->direction = true;
						pdm->totalSteps = (uint32_t)steps;
					}
					else
					{
						pdm->direction = false;
						pdm->totalSteps = (uint32_t)(-steps);
					}
				}
			}
		}

		// Now do the next move
		cdda = cdda->next;
	}

	return babySteppingDone;
}

// Recalculate the top speed, acceleration distance and deceleration distance, and whether we can pause after this move
// This may cause a move that we intended to be a deceleration-only move to have a tiny acceleration segment at the start
void DDA::RecalculateMove()
{
	accelDistance = (fsquare(requestedSpeed) - fsquare(startSpeed))/(2.0 * acceleration);
	decelDistance = (fsquare(requestedSpeed) - fsquare(endSpeed))/(2.0 * acceleration);
	if (accelDistance + decelDistance >= totalDistance)
	{
		// This move has no steady-speed phase, so it's accelerate-decelerate or accelerate-only move.
		// If V is the peak speed, then (V^2 - u^2)/2a + (V^2 - v^2)/2a = distance.
		// So (2V^2 - u^2 - v^2)/2a = distance
		// So V^2 = a * distance + 0.5(u^2 + v^2)
		float vsquared = (acceleration * totalDistance) + 0.5f * (fsquare(startSpeed) + fsquare(endSpeed));
		// Calculate accelerate distance from: V^2 = u^2 + 2as
		if (vsquared >= 0.0)
		{
			accelDistance = max<float>((vsquared - fsquare(startSpeed))/(2.0f * acceleration), 0.0);
			decelDistance = totalDistance - accelDistance;
			topSpeed = sqrtf(vsquared);
		}
		else
		{
			// It's an accelerate-only or decelerate-only move.
			// Due to rounding errors and babystepping adjustments, we may have to adjust the acceleration slightly.
			if (startSpeed < endSpeed)
			{
				// This would ideally never happen, but might because of rounding errors
				accelDistance = totalDistance;
				decelDistance = 0.0;
				topSpeed = endSpeed;
				acceleration = (fsquare(endSpeed) - fsquare(startSpeed))/(2.0f * totalDistance);
			}
			else
			{
				accelDistance = 0.0;
				decelDistance = totalDistance;
				topSpeed = startSpeed;
				acceleration = (fsquare(startSpeed) - fsquare(endSpeed))/(2.0f * totalDistance);
			}
		}
	}
	else
	{
		topSpeed = requestedSpeed;
	}

	if (canPauseAfter && endSpeed != 0.0)
	{
		const Platform& p = reprap.GetPlatform();
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			if (pddm[drive] != nullptr && pddm[drive]->state == DMState::moving && endSpeed * fabsf(directionVector[drive]) > p.ActualInstantDv(drive))
			{
				canPauseAfter = false;
				break;
			}
		}
	}
#ifdef POLYPRINTER
	recalculate_flag = false;		// no longer needs recalculation
#endif
}

#ifdef POLYPRINTER
#else
// Decide what speed we would really like this move to end at.
// On entry, endSpeed is our proposed ending speed and targetNextSpeed is the proposed starting speed of the next move
// On return, targetNextSpeed is the speed we would like the next move to start at, and endSpeed is the corresponding end speed of this move.
void DDA::CalcNewSpeeds()
{
	// We may have to make multiple passes, because reducing one of the speeds may solve some problems but actually make matters worse on another axis.
	bool limited;
	do
	{
//		debugPrintf("  Pass, start=%f end=%f\n", targetStartSpeed, endSpeed);
		limited = false;
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			if (   (pddm[drive] != nullptr && pddm[drive]->state == DMState::moving)
				|| (next->pddm[drive] != nullptr && next->pddm[drive]->state == DMState::moving)
			   )
			{
				const float thisMoveFraction = directionVector[drive];
				const float nextMoveFraction = next->directionVector[drive];
				const float thisMoveSpeed = endSpeed * thisMoveFraction;
				const float nextMoveSpeed = targetNextSpeed * nextMoveFraction;
				const float idealDeltaV = fabsf(thisMoveSpeed - nextMoveSpeed);
				float maxDeltaV = reprap.GetPlatform().ActualInstantDv(drive);
				if (idealDeltaV > maxDeltaV)
				{
					// This drive can't change speed fast enough, so reduce the start and/or end speeds
					// This algorithm sometimes converges very slowly, requiring many passes.
					// To ensure it converges at all, and to speed up convergence, we over-adjust the speed to achieve an even lower deltaV.
					maxDeltaV *= 0.8;
					if ((thisMoveFraction >= 0.0) == (nextMoveFraction >= 0.0))
					{
						// Drive moving in the same direction for this move and the next one, so we must reduce speed of the faster one
						if (fabsf(thisMoveSpeed) > fabsf(nextMoveSpeed))
						{
							endSpeed = (fabsf(nextMoveSpeed) + maxDeltaV)/fabsf(thisMoveFraction);
						}
						else
						{
							targetNextSpeed = (fabsf(thisMoveSpeed) + maxDeltaV)/fabsf(nextMoveFraction);
						}
					}
					else if (fabsf(thisMoveSpeed) * 2 < maxDeltaV)
					{
						targetNextSpeed = (maxDeltaV - fabsf(thisMoveSpeed))/fabsf(nextMoveFraction);
					}
					else if (fabsf(nextMoveSpeed) * 2 < maxDeltaV)
					{
						endSpeed = (maxDeltaV - fabsf(nextMoveSpeed))/fabsf(thisMoveFraction);
					}
					else
					{
						targetNextSpeed = maxDeltaV/(2 * fabsf(nextMoveFraction));
						endSpeed = maxDeltaV/(2 * fabsf(thisMoveFraction));
					}
					limited = true;
					// Most conflicts are between X and Y. So if we just did Y, start another pass immediately to save time.
					if (drive == 1)
					{
						break;
					}
				}
			}
		}
	} while (limited);
}
#endif


// This is called by Move::CurrentMoveCompleted to update the live coordinates from the move that has just finished
bool DDA::FetchEndPosition(volatile int32_t ep[DRIVES], volatile float endCoords[DRIVES])
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		ep[drive] = endPoint[drive];
	}
	if (endCoordinatesValid)
	{
		const size_t visibleAxes = reprap.GetGCodes().GetVisibleAxes();
		for (size_t axis = 0; axis < visibleAxes; ++axis)
		{
			endCoords[axis] = endCoordinates[axis];
		}
	}

	// Extrusion amounts are always valid
	for (size_t eDrive = reprap.GetGCodes().GetTotalAxes(); eDrive < DRIVES; ++eDrive)
	{
		endCoords[eDrive] += endCoordinates[eDrive];
	}

	return endCoordinatesValid;
}

void DDA::SetPositions(const float move[DRIVES], size_t numDrives)
{
	reprap.GetMove().EndPointToMachine(move, endPoint, numDrives);
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		endCoordinates[axis] = move[axis];
	}
	endCoordinatesValid = true;
}

// Get a Cartesian end coordinate from this move
float DDA::GetEndCoordinate(size_t drive, bool disableMotorMapping)
pre(disableDeltaMapping || drive < MaxAxes)
{
	if (disableMotorMapping)
	{
		return Move::MotorEndpointToPosition(endPoint[drive], drive);
	}
	else
	{
		const size_t visibleAxes = reprap.GetGCodes().GetVisibleAxes();
		if (drive < visibleAxes && !endCoordinatesValid)
		{
			reprap.GetMove().MotorStepsToCartesian(endPoint, visibleAxes, reprap.GetGCodes().GetTotalAxes(), endCoordinates);
			endCoordinatesValid = true;
		}
		return endCoordinates[drive];
	}
}

// Calculate the time needed for this move
float DDA::CalcTime() const
{
	return (float)clocksNeeded/stepClockRate;
}

// Prepare this DDA for execution.
// This must not be called with interrupts disabled, because it calls Platform::EnableDrive.
void DDA::Prepare(uint8_t simMode)
{
	PrepParams params;
	params.decelStartDistance = totalDistance - decelDistance;

	// Convert the accelerate/decelerate distances to times
	const float accelStopTime = (topSpeed - startSpeed)/acceleration;
	const float decelStartTime = accelStopTime + (params.decelStartDistance - accelDistance)/topSpeed;
	const float totalTime = decelStartTime + (topSpeed - endSpeed)/acceleration;

	clocksNeeded = (uint32_t)(totalTime * stepClockRate);

	if (simMode == 0)
	{
		if (isDeltaMovement)
		{
			// This code used to be in DDA::Init but we moved it here so that we can implement babystepping in it,
			// also it speeds up simulation because we no longer execute this code when simulating.
			// However, this code assumes that the previous move in the DDA ring is the previously-executed move, because it fetches the X and Y end coordinates from that move.
			// Therefore the Move code must not store a new move in that entry until this one has been prepared! (It took me ages to track this down.)
			// Ideally we would store the initial X any Y coordinates in the DDA, but we need to be economical with memory in the Duet 06/085 build.
			a2plusb2 = fsquare(directionVector[X_AXIS]) + fsquare(directionVector[Y_AXIS]);
			cKc = (int32_t)(directionVector[Z_AXIS] * DriveMovement::Kc);

			const float initialX = prev->GetEndCoordinate(X_AXIS, false);
			const float initialY = prev->GetEndCoordinate(Y_AXIS, false);
			const LinearDeltaKinematics& dparams = (const LinearDeltaKinematics&)reprap.GetMove().GetKinematics();
			const float diagonalSquared = dparams.GetDiagonalSquared();
			const float a2b2D2 = a2plusb2 * diagonalSquared;

			for (size_t drive = 0; drive < DELTA_AXES; ++drive)
			{
				const float A = initialX - dparams.GetTowerX(drive);
				const float B = initialY - dparams.GetTowerY(drive);
				const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
				DriveMovement* const pdm = pddm[drive];
				if (pdm != nullptr)
				{
					const float aAplusbB = A * directionVector[X_AXIS] + B * directionVector[Y_AXIS];
					const float dSquaredMinusAsquaredMinusBsquared = diagonalSquared - fsquare(A) - fsquare(B);
					const float h0MinusZ0 = sqrtf(dSquaredMinusAsquaredMinusBsquared);
					pdm->mp.delta.hmz0sK = (int32_t)(h0MinusZ0 * stepsPerMm * DriveMovement::K2);
					pdm->mp.delta.minusAaPlusBbTimesKs = -(int32_t)(aAplusbB * stepsPerMm * DriveMovement::K2);
					pdm->mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared =
							(int64_t)(dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm * DriveMovement::K2));

					// Calculate the distance at which we need to reverse direction.
					if (a2plusb2 <= 0.0)
					{
						// Pure Z movement. We can't use the main calculation because it divides by a2plusb2.
						pdm->direction = (directionVector[Z_AXIS] >= 0.0);
						pdm->reverseStartStep = pdm->totalSteps + 1;
					}
					else
					{
						// The distance to reversal is the solution to a quadratic equation. One root corresponds to the carriages being below the bed,
						// the other root corresponds to the carriages being above the bed.
						const float drev = ((directionVector[Z_AXIS] * sqrt(a2b2D2 - fsquare(A * directionVector[Y_AXIS] - B * directionVector[X_AXIS])))
											- aAplusbB)/a2plusb2;
						if (drev > 0.0 && drev < totalDistance)		// if the reversal point is within range
						{
							// Calculate how many steps we need to move up before reversing
							const float hrev = directionVector[Z_AXIS] * drev + sqrt(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - a2plusb2 * fsquare(drev));
							const int32_t numStepsUp = (int32_t)((hrev - h0MinusZ0) * stepsPerMm);

							// We may be almost at the peak height already, in which case we don't really have a reversal.
							if (numStepsUp < 1 || (pdm->direction && (uint32_t)numStepsUp <= pdm->totalSteps))
							{
								pdm->reverseStartStep = pdm->totalSteps + 1;
							}
							else
							{
								pdm->reverseStartStep = (uint32_t)numStepsUp + 1;

								// Correct the initial direction and the total number of steps
								if (pdm->direction)
								{
									// Net movement is up, so we will go up a bit and then down by a lesser amount
									pdm->totalSteps = (2 * numStepsUp) - pdm->totalSteps;
								}
								else
								{
									// Net movement is down, so we will go up first and then down by a greater amount
									pdm->direction = true;
									pdm->totalSteps = (2 * numStepsUp) + pdm->totalSteps;
								}
							}
						}
						else
						{
							pdm->reverseStartStep = pdm->totalSteps + 1;
						}
					}
				}
			}
		}

		params.startSpeedTimesCdivA = (uint32_t)((startSpeed * stepClockRate)/acceleration);
		params.topSpeedTimesCdivA = (uint32_t)((topSpeed * stepClockRate)/acceleration);
		params.decelStartClocks = (uint32_t)(decelStartTime * stepClockRate);
		params.topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivA + params.decelStartClocks;
		params.accelClocksMinusAccelDistanceTimesCdivTopSpeed = (uint32_t)((accelStopTime - (accelDistance/topSpeed)) * stepClockRate);
		params.compFactor = 1.0 - startSpeed/topSpeed;

		firstDM = nullptr;

		const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			DriveMovement* const pdm = pddm[drive];
			if (pddm != nullptr && pdm->state == DMState::moving)
			{
				if (isLeadscrewAdjustmentMove)
				{

					reprap.GetPlatform().EnableDrive(Z_AXIS);			// ensure all Z motors are enabled
					pdm->PrepareCartesianAxis(*this, params);

					// Check for sensible values, print them if they look dubious
					if (reprap.Debug(moduleDda) && pdm->totalSteps > 1000000)
					{
						DebugPrint();
					}
				}
				else
				{
					reprap.GetPlatform().EnableDrive(drive);
					if (drive >= numAxes)
					{
#ifdef POLYPRINTER
					if ( doZHomingVibration )
					{
						debugPrintf("Preparing extruder for Vibration\n");
						pdm->PrepareExtruderWithVibration(*this, params, zHomingParams );
					}
					else
					{
						pdm->PrepareExtruderWithLinearAdvance(*this, params, usePressureAdvance);
					}
#else
					dm.PrepareExtruder(*this, params, usePressureAdvance);
#endif

						// Check for sensible values, print them if they look dubious
						if (reprap.Debug(moduleDda)
							&& (   pdm->totalSteps > 1000000
								|| pdm->reverseStartStep < pdm->mp.cart.decelStartStep
								|| (pdm->reverseStartStep <= pdm->totalSteps
									&& pdm->mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA > (int64_t)(pdm->mp.cart.twoCsquaredTimesMmPerStepDivA * pdm->reverseStartStep))
							   )
						   )
						{
							DebugPrint();
						}
					}
					else if (isDeltaMovement && drive < DELTA_AXES)			// for now, additional axes are assumed to be not part of the delta mechanism
					{
						pdm->PrepareDeltaAxis(*this, params);

						// Check for sensible values, print them if they look dubious
						if (reprap.Debug(moduleDda) && pdm->totalSteps > 1000000)
						{
							DebugPrint();
						}
					}
					else
					{
						pdm->PrepareCartesianAxis(*this, params);

						// Check for sensible values, print them if they look dubious
						if (reprap.Debug(moduleDda) && pdm->totalSteps > 1000000)
						{
							DebugPrint();
						}
					}
				}

				// Prepare for the first step
				pdm->nextStep = 0;
				pdm->nextStepTime = 0;
				pdm->stepInterval = 999999;						// initialise to a large value so that we will calculate the time for just one step
				pdm->stepsTillRecalc = 0;							// so that we don't skip the calculation
				const bool stepsToDo = (isDeltaMovement && drive < numAxes)

										? pdm->CalcNextStepTimeDelta(*this, false)
#ifdef POLYPRINTER
									: ( ( doZHomingVibration && pdm->drive == E0_AXIS ) ? pdm->CalcNextStepTimeVibration( *this, false ):  pdm->CalcNextStepTimeCartesian(*this, false) );
#else
										: pdm->CalcNextStepTimeCartesian(*this, false);
#endif

				if (stepsToDo)
				{
					InsertDM(pdm);
				}
				else
				{
					pdm->state = DMState::idle;
				}
			}
		}

		if (reprap.Debug(moduleDda) && reprap.Debug(moduleMove))		// temp show the prepared DDA if debug enabled for both modules
		{
			DebugPrint();
		}

#if DDA_MOVE_DEBUG
		MoveParameters& m = savedMoves[savedMovePointer];
		m.accelDistance = accelDistance;
		m.decelDistance = decelDistance;
		m.steadyDistance = totalDistance - accelDistance - decelDistance;
		m.startSpeed = startSpeed;
		m.topSpeed = topSpeed;
		m.endSpeed = endSpeed;
		savedMovePointer = (savedMovePointer + 1) % NumSavedMoves;
#endif
	}

	state = frozen;					// must do this last so that the ISR doesn't start executing it before we have finished setting it up
}

// Take a unit positive-hyperquadrant vector, and return the factor needed to obtain
// length of the vector as projected to touch box[].
/*static*/ float DDA::VectorBoxIntersection(const float v[], const float box[], size_t dimensions)
{
	// Generate a vector length that is guaranteed to exceed the size of the box
	const float biggerThanBoxDiagonal = 2.0*Magnitude(box, dimensions);
	float magnitude = biggerThanBoxDiagonal;
	for (size_t d = 0; d < dimensions; d++)
	{
		if (biggerThanBoxDiagonal*v[d] > box[d])
		{
			const float a = box[d]/v[d];
			if (a < magnitude)
			{
				magnitude = a;
			}
		}
	}
	return magnitude;
}

// Normalise a vector with dim1 dimensions so that it is unit in the first dim2 dimensions, and also return its previous magnitude in dim2 dimensions
/*static*/ float DDA::Normalise(float v[], size_t dim1, size_t dim2)
{
	const float magnitude = Magnitude(v, dim2);
	if (magnitude <= 0.0)
	{
		return 0.0;
	}
	Scale(v, 1.0/magnitude, dim1);
	return magnitude;
}

// Make the direction vector unit-normal in XYZ and return the previous magnitude
float DDA::NormaliseXYZ()
{
	// First calculate the magnitude of the vector. If there is more than one X or Y axis, take an average of their movements (they should be equal).
	float xMagSquared = 0.0, yMagSquared = 0.0;
	unsigned int numXaxes = 0, numYaxes = 0;
	for (size_t d = 0; d < MaxAxes; ++d)
	{
		if (IsBitSet(xAxes, d))
		{
			xMagSquared += fsquare(directionVector[d]);
			++numXaxes;
		}
		if (IsBitSet(yAxes, d))
		{
			yMagSquared += fsquare(directionVector[d]);
			++numYaxes;
		}
	}
	if (numXaxes > 1)
	{
		xMagSquared /= numXaxes;
	}
	if (numYaxes > 1)
	{
		yMagSquared /= numYaxes;
	}
	const float magnitude = sqrtf(xMagSquared + yMagSquared + fsquare(directionVector[Z_AXIS]));
	if (magnitude <= 0.0)
	{
		return 0.0;
	}

	// Now normalise it
	Scale(directionVector, 1.0/magnitude, DRIVES);
	return magnitude;
}

// Return the magnitude of a vector
/*static*/ float DDA::Magnitude(const float v[], size_t dimensions)
{
	float magnitude = 0.0;
	for (size_t d = 0; d < dimensions; d++)
	{
		magnitude += v[d]*v[d];
	}
	return sqrtf(magnitude);
}

// Multiply a vector by a scalar
/*static*/ void DDA::Scale(float v[], float scale, size_t dimensions)
{
	for (size_t d = 0; d < dimensions; d++)
	{
		v[d] *= scale;
	}
}

// Move a vector into the positive hyperquadrant
/*static*/ void DDA::Absolute(float v[], size_t dimensions)
{
	for (size_t d = 0; d < dimensions; d++)
	{
		v[d] = fabsf(v[d]);
	}
}

void DDA::CheckEndstops(Platform& platform)
{
	if ((endStopsToCheck & ZProbeActive) != 0)						// if the Z probe is enabled in this move
	{
		// Check whether the Z probe has been triggered. On a delta at least, this must be done separately from endstop checks,
		// because we have both a high endstop and a Z probe, and the Z motor is not the same thing as the Z axis.
		switch (platform.GetZProbeResult())
		{
		case EndStopHit::lowHit:
			MoveAborted();											// set the state to completed and recalculate the endpoints
			reprap.GetGCodes().MoveStoppedByZProbe();
			break;

		case EndStopHit::lowNear:
			ReduceHomingSpeed();
			break;

		default:
			break;
		}
#ifdef POLYPRINTER
		// TODO: properly handle the case of Bed Contact.
		//       - e.g. identify the actual official endstop bit that we connect the bed contact to, and
		//         make sure it registers properly with HitLowStop(); and then check for that as a special case
		//if ( platform.GetBedContactExists() != EndStopHit::noStop || platform.GetNutSwitchActive() != EndStopHit::noStop )
		//{
		//	// there's another type of error - stop trying to use the probe
		//	//debugPrintf("Error while doing probe. Abandoning this probe\n");
		//	MoveAborted();
		//}
		// TODO: decide whether to involve the Z switch or not.
		// if it is, though, it should stop the move (the endstop check below doesn't stop the move just from Z endstop hitting, because ZProbeActive remains set)
		if ( ((endStopsToCheck & (1 << Z_AXIS)) != 0) && platform.Stopped(Z_AXIS) != EndStopHit::noStop )
		{
			// must have hit the Z switch (or nut switch wired to that pin)
			//debugPrintf("Hit Z switch while doing probe. Abandoning this probe\n");
			MoveAborted();
			reprap.GetMove().HitLowStop( 1 << Z_AXIS );  // doesn't zero anything
		}

#endif

	}

#if DDA_LOG_PROBE_CHANGES
	else if ((endStopsToCheck & LogProbeChanges) != 0)
	{
		switch (platform.GetZProbeResult())
		{
		case EndStopHit::lowHit:
			if (!probeTriggered)
			{
				probeTriggered = true;
				LogProbePosition();
			}
			break;

		case EndStopHit::lowNear:
		case EndStopHit::noStop:
			if (probeTriggered)
			{
				probeTriggered = false;
				LogProbePosition();
			}
			break;

		default:
			break;
		}
	}
#endif
	// if the bits are set, then a check resulting in an active state is supposed to stop the movement.
	// this particular check is not necessarily what's used to detect errors while printing But it could.
	if ( platform.GetBedContactExists() != EndStopHit::noStop )
	{
		// TODO: log this error as one needing some kind of behavior modification - depends on what was being done
		//       - we need to be able to set an error condition
		MoveAborted();
		hadBedContact = true;
		reprap.GetMove().HitLowStop( 1 << BED_CONTACT_ENDSTOP_NUM );  // doesn't zero anything
		return;
	}
	else if ( platform.GetNutSwitchActive() != EndStopHit::noStop )
	{
		MoveAborted();
		hadNutSwitch = true;
		reprap.GetMove().HitLowStop( 1 << NUT_SWITCH_ENDSTOP_NUM );  // doesn't zero anything
		return;
	}

	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t drive = 0; drive < numAxes; ++drive)
	{
		if (IsBitSet(endStopsToCheck, drive))
		{
			const EndStopHit esh = platform.Stopped(drive);
			switch (esh)
			{
			case EndStopHit::lowHit:
			case EndStopHit::highHit:
				{
					ClearBit(endStopsToCheck, drive);					// clear this check so that we can check for more
					const Kinematics& kin = reprap.GetMove().GetKinematics();
					if (endStopsToCheck == 0 || kin.QueryTerminateHomingMove(drive))
					{
						MoveAborted();									// no more endstops to check, or this axis uses shared motors, so stop the entire move
					}
					else
					{
						StopDrive(drive);								// we must stop the drive before we mess with its coordinates
					}
					if (drive < reprap.GetGCodes().GetTotalAxes() && IsHomingAxes())
					{
						kin.OnHomingSwitchTriggered(drive, esh == EndStopHit::highHit, reprap.GetPlatform().GetDriveStepsPerUnit(), *this);
						reprap.GetGCodes().SetAxisIsHomed(drive);
					}
				}
				break;

			case EndStopHit::lowNear:
				// Only reduce homing speed if there are no more axes to be homed.
				// This allows us to home X and Y simultaneously.
				if (endStopsToCheck == MakeBitmap<AxesBitmap>(drive))
				{
					ReduceHomingSpeed();
				}
				break;

			default:
				break;
			}
		}
	}

}

// The remaining functions are speed-critical, so use full optimisation
// The GCC optimize pragma appears to be broken, if we try to force O3 optimisation here then functions are never inlined

// Start executing this move, returning true if Step() needs to be called immediately. Must be called with interrupts disabled, to avoid a race condition.
// Returns true if the caller needs to call the step ISR immediately.
bool DDA::Start(uint32_t tim)
pre(state == frozen)
{
	moveStartTime = tim;
	state = executing;

#if DDA_LOG_PROBE_CHANGES
	if ((endStopsToCheck & LogProbeChanges) != 0)
	{
		numLoggedProbePositions = 0;
		probeTriggered = false;
	}
#endif

	if (firstDM != nullptr)
	{
		unsigned int extrusions = 0, retractions = 0;		// bitmaps of extruding and retracting drives
		const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
		for (size_t i = 0; i < DRIVES; ++i)
		{
			DriveMovement* const pdm = pddm[i];
			if (pdm != nullptr && pdm->state == DMState::moving)
			{
				const size_t drive = pdm->drive;
				reprap.GetPlatform().SetDirection(drive, pdm->direction);
				if (drive >= numAxes && drive < DRIVES
#ifdef POLYPRINTER
						&& !doZHomingVibration
#endif
													)

				{
					if (pdm->direction == FORWARDS)
					{
						extrusions |= (1 << (i - numAxes));
					}
					else
					{
						retractions |= (1 << (i - numAxes));
					}
				}
			}
		}

		bool extruding = false;
		if (extrusions != 0 || retractions != 0)
		{
			const unsigned int prohibitedMovements = reprap.GetProhibitedExtruderMovements(extrusions, retractions);
			for (DriveMovement **dmpp = &firstDM; *dmpp != nullptr; )
			{
				const size_t drive = (*dmpp)->drive;
				const bool thisDriveExtruding = drive >= numAxes && drive < DRIVES;
				if (thisDriveExtruding && (prohibitedMovements & (1 << (drive - numAxes))) != 0)
				{
					*dmpp = (*dmpp)->nextDM;
				}
				else
				{
					extruding = extruding || thisDriveExtruding;
					dmpp = &((*dmpp)->nextDM);
				}
			}
		}

		Platform& platform = reprap.GetPlatform();
		if (extruding)
		{
			platform.ExtrudeOn();
		}
		else
		{
			platform.ExtrudeOff();
		}

		if (firstDM != nullptr)
		{
			return platform.ScheduleStepInterrupt(firstDM->nextStepTime + moveStartTime);
		}
	}

	// No steps are pending. This should not happen, except perhaps for an extrude-only move when extrusion is prohibited
	return true;	// schedule another interrupt immediately
}

extern uint32_t maxReps;	// diagnostic

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
bool DDA::Step()
{
	Platform& platform = reprap.GetPlatform();
	uint32_t lastStepPulseTime = platform.GetInterruptClocks();
	bool repeat;
	uint32_t numReps = 0;
	do
	{
		// Keep this loop as fast as possible, in the case that there are no endstops to check!

		// 1. Check endstop switches and Z probe if asked. This is not speed critical because fast moves do not use endstops or the Z probe.
		if (endStopsToCheck != 0)										// if any homing switches or the Z probe is enabled in this move
		{
			CheckEndstops(platform);	// Call out to a separate function because this may help cache usage in the more common case where we don't call it
			if (state == completed)		// we may have completed the move due to triggering an endstop switch or Z probe
			{
				break;
			}
		}

		// 2. Determine which drivers are due for stepping, overdue, or will be due very shortly
		DriveMovement* dm = firstDM;
		const uint32_t elapsedTime = (Platform::GetInterruptClocks() - moveStartTime) + minInterruptInterval;
		uint32_t driversStepping = 0;
		while (dm != nullptr && elapsedTime >= dm->nextStepTime)		// if the next step is due
		{
			++numReps;
			driversStepping |= platform.GetDriversBitmap(dm->drive);
			dm = dm->nextDM;

//uint32_t t3 = Platform::GetInterruptClocks() - t2;
//if (t3 > maxCalcTime) maxCalcTime = t3;
//if (t3 < minCalcTime) minCalcTime = t3;
		}
		// dm should now be nullptr, or else point to the first driver that doesn't need to be stepped at this time

		// 3. Step the drivers
		if ((driversStepping & platform.GetSlowDrivers()) != 0)
		{
			// we have one or more very slow drivers that we need a delay for, to hold the Low state a bit longer before we raise it now.
			while (Platform::GetInterruptClocks() - lastStepPulseTime < platform.GetSlowDriverClocks()) {}
			Platform::StepDriversHigh(driversStepping);					// generate the steps
			lastStepPulseTime = Platform::GetInterruptClocks();
		}
		else
		{
			Platform::StepDriversHigh(driversStepping);					// generate the steps
		}

		// 4. Remove those drives from the list, calculate the next step times, update the direction pins where necessary,
		//    and re-insert them so as to keep the list in step-time order. We assume that meeting the direction pin hold time
		//    is not a problem for any driver type. This is not necessarily true.
		DriveMovement *dmToInsert = firstDM;							// head of the chain we need to re-insert
		firstDM = dm;													// remove the chain we just stepped, from the list
		while (dmToInsert != dm)										// note that both of these may be nullptr
		{
			const bool hasMoreSteps = (isDeltaMovement && dmToInsert->drive < DELTA_AXES)
					? dmToInsert->CalcNextStepTimeDelta(*this, true)
#ifdef POLYPRINTER
					// when doing the quick set of steps, this does no recalc, so we immediately repeat this whole loop
					: ( ( doZHomingVibration && dmToInsert->drive == E0_AXIS ) ? dmToInsert->CalcNextStepTimeVibration( *this, true ) : dmToInsert->CalcNextStepTimeCartesian(*this, true) );
#else
					: dmToInsert->CalcNextStepTimeCartesian(*this, true);
#endif
			DriveMovement * const nextToInsert = dmToInsert->nextDM;
			if (hasMoreSteps)
			{
				InsertDM(dmToInsert);
			}
			dmToInsert = nextToInsert;
		}
		// now the list has been reconstructed in order of timer firing

		// 5. Reset all step pins low
		if ((driversStepping & platform.GetSlowDrivers()) != 0)
		{
			// we have one or more very slow drivers that we need a delay for, to hold the High state a bit longer before we lower it now.
			while (Platform::GetInterruptClocks() - lastStepPulseTime < platform.GetSlowDriverClocks()) {}
			Platform::StepDriversLow();									// set all step pins low
			lastStepPulseTime = Platform::GetInterruptClocks();
		}
		else
		{
			Platform::StepDriversLow();									// set all step pins low
		}

		// 6. Check for move completed
		if (firstDM == nullptr)
		{
			state = completed;
			break;
		}

		// 7. Schedule next interrupt, or if it would be too soon, generate more steps immediately
		repeat = platform.ScheduleStepInterrupt(firstDM->nextStepTime + moveStartTime);
	} while (repeat);

	// Diagnostic - track high water
	if (numReps > maxReps)
	{
		maxReps = numReps;
	}
	// end Diagnostic

	if (state == completed)
	{
		// The following finish time is wrong if we aborted the move because of endstop or Z probe checks.
		// However, following a move that checks endstops or the Z probe, we always wait for the move to complete before we schedule another, so this doesn't matter.
		const uint32_t finishTime = moveStartTime + clocksNeeded;	// calculate how long this move should take
		Move& move = reprap.GetMove();
		move.CurrentMoveCompleted();							// tell Move that the current move is complete
		return move.TryStartNextMove(finishTime);				// schedule the next move
	}
	return false;
}

// Stop a drive and re-calculate the corresponding endpoint
void DDA::StopDrive(size_t drive)
{
	DriveMovement* const pdm = pddm[drive];
	if (pdm->state == DMState::moving)
	{
		endPoint[drive] -= pdm->GetNetStepsLeft();
		pdm->state = DMState::idle;
		if (drive < reprap.GetGCodes().GetTotalAxes())
		{
			endCoordinatesValid = false;			// the XYZ position is no longer valid
		}
		RemoveDM(drive);
		if (firstDM == nullptr)
		{
			state = completed;
		}
	}
}

// This is called when we abort a move because we have hit an endstop.
// It adjusts the end points of the current move to account for how far through the move we got.
void DDA::MoveAborted()
{
	if (state == executing)
	{
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			StopDrive(drive);
		}
	}
	state = completed;
}

// Reduce the speed of this move to the indicated speed.
// This is called from the ISR, so interrupts are disabled and nothing else can mess with us.
// As this is only called for homing moves and with very low speeds, we assume that we don't need acceleration or deceleration phases.
void DDA::ReduceHomingSpeed()
{
	if (!goingSlow)
	{
		goingSlow = true;
		const float factor = 3.0;				// the factor by which we are reducing the speed
		topSpeed /= factor;
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			DriveMovement* const pdm = pddm[drive];
			if (pdm->state == DMState::moving)
			{
				pdm->ReduceSpeed(*this, factor);
				RemoveDM(pdm->drive);
				InsertDM(pdm);
			}
		}

		// We also need to adjust the total clocks needed, to prevent step errors being recorded
		const uint32_t clocksSoFar = Platform::GetInterruptClocks() - moveStartTime;
		if (clocksSoFar < clocksNeeded)
		{
			const uint32_t clocksLeft = clocksNeeded - clocksSoFar;
			clocksNeeded += (uint32_t)(clocksLeft * (factor - 1.0));
		}
	}
}

bool DDA::HasStepError() const
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		const DriveMovement* const pdm = pddm[drive];
		if (pdm != nullptr && pdm->state == DMState::stepError)
		{
			return true;
		}
	}
	return false;
}

// Free up this DDA, returning true if the lookahead underrun flag was set
bool DDA::Free()
{
	ReleaseDMs();
	state = empty;
	return hadLookaheadUnderrun;
}

// Return the number of net steps already taken in this move by a particular drive
int32_t DDA::GetStepsTaken(size_t drive) const
{
	const DriveMovement * const dmp = pddm[drive];
	return (dmp != nullptr) ? dmp->GetNetStepsTaken() : 0;
}

// End
