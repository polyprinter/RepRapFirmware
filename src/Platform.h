/****************************************************************************************************

RepRapFirmware - Platform: RepRapPro Ormerod with Duet controller

Platform contains all the code and definitions to deal with machine-dependent things such as control
pins, bed area, number of extruders, tolerable accelerations and speeds and so on.

No definitions that are system-independent should go in here.  Put them in Configuration.h.  Note that
the lengths of arrays such as DRIVES (see below) are defined here, so any array initialiser that depends on those
lengths, for example:

#define DRIVES 4
.
.
.
#define DRIVE_RELATIVE_MODES {false, false, false, true}

also needs to go here.

-----------------------------------------------------------------------------------------------------

Version 0.3

28 August 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef PLATFORM_H
#define PLATFORM_H

// Language-specific includes

#include <Libraries/TemperatureSensor/TemperatureSensor.h>
#include <cctype>
#include <cstring>
#include <malloc.h>
#include <TemperatureError.h>
#include <cstdlib>
#include <climits>

// Platform-specific includes

#include "Types.h"
#include "Core.h"
#include "OutputMemory.h"
#include "ff.h"

#if !defined(DUET_NG) || defined(PROTOTYPE_1)
# include "MCP4461.h"
#endif

#include "MassStorage.h"
#include "FileStore.h"
#include "MessageType.h"
#include "Fan.h"

// Definitions needed by Pins.h

const bool FORWARDS = true;
const bool BACKWARDS = !FORWARDS;

#include "Pins.h"

/**************************************************************************************************/

// Some numbers...

#define TIME_TO_REPRAP 1.0e6 	// Convert seconds to the units used by the machine (usually microseconds)
#define TIME_FROM_REPRAP 1.0e-6 // Convert the units used by the machine (usually microseconds) to seconds

/**************************************************************************************************/

#ifdef DUET_NG
const int Z_PROBE_AD_VALUE = 500;						// Default for the Z probe - should be overwritten by experiment
const bool Z_PROBE_AXES[AXES] = { false, false, true };	// Axes for which the Z-probe is normally used
#else
const int Z_PROBE_AD_VALUE = 400;						// Default for the Z probe - should be overwritten by experiment
const bool Z_PROBE_AXES[AXES] = { true, false, true };	// Axes for which the Z-probe is normally used
#endif

const float Z_PROBE_STOP_HEIGHT = 0.7;					// Millimetres
const unsigned int Z_PROBE_AVERAGE_READINGS = 8;		// We average this number of readings with IR on, and the same number with IR off
const int ZProbeTypeDelta = 6;							// Z probe type for experimental delta probe

#if SUPPORT_INKJET

// Inkjet (if any - no inkjet is flagged by INKJET_BITS negative)

const int8_t INKJET_BITS = 12;							// How many nozzles? Set to -1 to disable this feature
const int INKJET_FIRE_MICROSECONDS = 5;					// How long to fire a nozzle
const int INKJET_DELAY_MICROSECONDS = 800;				// How long to wait before the next bit

#endif

const float MAX_FEEDRATES[DRIVES] = DRIVES_(100.0, 100.0, 3.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0);						// mm/sec
const float ACCELERATIONS[DRIVES] = DRIVES_(500.0, 500.0, 20.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0);				// mm/sec^2
const float DRIVE_STEPS_PER_UNIT[DRIVES] = DRIVES_(87.4890, 87.4890, 4000.0, 420.0, 420.0, 420.0, 420.0, 420.0, 420.0, 420.0);	// steps/mm
const float INSTANT_DVS[DRIVES] = DRIVES_(15.0, 15.0, 0.2, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0);								// mm/sec

// AXES

const size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E0_AXIS = 3;	// The indices of the Cartesian axes in drive arrays
const size_t A_AXIS = 0, B_AXIS = 1, C_AXIS = 2;				// The indices of the 3 tower motors of a delta printer in drive arrays

const float AXIS_MINIMA[AXES] = { 0.0, 0.0, 0.0 };				// mm
const float AXIS_MAXIMA[AXES] = { 230.0, 210.0, 200.0 };		// mm

const float defaultPrintRadius = 50;							// mm
const float defaultDeltaHomedHeight = 200;						// mm

// HEATERS - The bed is assumed to be the at index 0

// Bed thermistor: http://uk.farnell.com/epcos/b57863s103f040/sensor-miniature-ntc-10k/dp/1299930?Ntt=129-9930
// Hot end thermistor: http://www.digikey.co.uk/product-search/en?x=20&y=11&KeyWords=480-3137-ND
const float defaultThermistorBetas[HEATERS] = HEATERS_(BED_BETA, EXT_BETA, EXT_BETA, EXT_BETA, EXT_BETA, EXT_BETA, EXT_BETA, EXT_BETA); // Bed thermistor: B57861S104F40; Extruder thermistor: RS 198-961
const float defaultThermistorSeriesRs[HEATERS] = HEATERS_(THERMISTOR_SERIES_RS, THERMISTOR_SERIES_RS, THERMISTOR_SERIES_RS, THERMISTOR_SERIES_RS,
													THERMISTOR_SERIES_RS, THERMISTOR_SERIES_RS, THERMISTOR_SERIES_RS, THERMISTOR_SERIES_RS);
const float defaultThermistor25RS[HEATERS] = HEATERS_(BED_R25, EXT_R25, EXT_R25, EXT_R25, EXT_R25, EXT_R25, EXT_R25, EXT_R25); // Thermistor ohms at 25 C = 298.15 K

// Note on hot end PID parameters:
// The system is highly nonlinear because the heater power is limited to a maximum value and cannot go negative.
// If we try to run a traditional PID when there are large temperature errors, this causes the I-accumulator to go out of control,
// which causes a large amount of overshoot at lower temperatures. There are at least two ways of avoiding this:
//
// 1. Allow the PID to operate even with very large errors, but choose a very small I-term, just the right amount so that when heating up
//    from cold, the I-accumulator is approximately the value needed to maintain the correct power when the target temperature is reached.
//    This works well most of the time. However if the Duet board is reset when the extruder is hot and is then
//    commanded to heat up again before the extruder has cooled, the I-accumulator doesn't grow large enough, so the
//    temperature undershoots. The small value of the I-term then causes it to take a long time to reach the correct temperature.
//
// 2. Only allow the PID to operate when the temperature error is small enough for the PID to operate in the linear region.
//    So we set FULL_PID_BAND to a small value. It needs to be at least 15C because that is how much the temperature overshoots by
//    on an Ormerod when we turn the heater off from full power at about 180C. When we transition to PID, we set the I-term to the
//    value we expect to be needed to maintain the target temperature. We use an additional T parameter to allow this value to be
//    estimated.
//
// The default values use method (2).
//
// Note: a negative P, I or D value means do not use PID for this heater, use bang-bang control instead.
// This allows us to switch between PID and bang-bang using the M301 and M304 commands.

// We use method 2 (see above)
const float defaultPidKis[HEATERS] = HEATERS_(5.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2); 			// Integral PID constants
const float defaultPidKds[HEATERS] = HEATERS_(500.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0); // Derivative PID constants
const float defaultPidKps[HEATERS] = HEATERS_(-1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0);	// Proportional PID constants, negative values indicate use bang-bang instead of PID
const float defaultPidKts[HEATERS] = HEATERS_(2.7, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4);			// approximate PWM value needed to maintain temperature, per degC above room temperature
const float defaultPidKss[HEATERS] = HEATERS_(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);			// PWM scaling factor, to allow for variation in heater power and supply voltage
const float defaultFullBands[HEATERS] = HEATERS_(5.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0);	// errors larger than this cause heater to be on or off
const float defaultPidMins[HEATERS] = HEATERS_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);			// minimum value of I-term
const float defaultPidMaxes[HEATERS] = HEATERS_(255, 180, 180, 180, 180, 180, 180, 180);			// maximum value of I-term, must be high enough to reach 245C for ABS printing

const float STANDBY_TEMPERATURES[HEATERS] = HEATERS_(ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO); // We specify one for the bed, though it's not needed
const float ACTIVE_TEMPERATURES[HEATERS] = HEATERS_(ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO, ABS_ZERO);

// For the theory behind ADC oversampling, see http://www.atmel.com/Images/doc8003.pdf
const unsigned int AD_OVERSAMPLE_BITS = 1;		// Number of bits we oversample when reading temperatures

// Define the number of temperature readings we average for each thermistor. This should be a power of 2 and at least 4 ** AD_OVERSAMPLE_BITS.
// Keep THERMISTOR_AVERAGE_READINGS * NUM_HEATERS * 2ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
const unsigned int THERMISTOR_AVERAGE_READINGS = 32;
const unsigned int AD_RANGE_REAL = 4095;													// The ADC that measures temperatures gives an int this big as its max value
const unsigned int AD_RANGE_VIRTUAL = ((AD_RANGE_REAL + 1) << AD_OVERSAMPLE_BITS) - 1;		// The max value we can get using oversampling
const unsigned int AD_DISCONNECTED_REAL = AD_RANGE_REAL - 3;								// We consider an ADC reading at/above this value to indicate that the thermistor is disconnected
const unsigned int AD_DISCONNECTED_VIRTUAL = AD_DISCONNECTED_REAL << AD_OVERSAMPLE_BITS;

const uint32_t maxPidSpinDelay = 5000;			// Maximum elapsed time in milliseconds between successive temp samples by Pid::Spin() permitted for a temp sensor

const size_t BED_HEATER = 0;					// Index of the heated bed
const size_t E0_HEATER = 1;						// Index of the first extruder heater

/****************************************************************************************************/

// File handling

const size_t MAX_FILES = 10;					// Must be large enough to handle the max number of simultaneous web requests + files being printed

const size_t FILE_BUFFER_SIZE = 256;

/****************************************************************************************************/

enum class BoardType : uint8_t
{
	Auto = 0,
#ifdef DUET_NG
# ifdef PROTOTYPE_1
	DuetWiFi_06 = 1
# else
	DuetWiFi_10 = 1
# endif
#elif defined(__RADDS__)
	RADDS_15 = 1
#else
	Duet_06 = 1,
	Duet_07 = 2,
	Duet_085 = 3
#endif
};

enum class EndStopHit
{
  noStop = 0,		// no endstop hit
  lowHit = 1,		// low switch hit, or Z-probe in use and above threshold
  highHit = 2,		// high stop hit
  lowNear = 3		// approaching Z-probe threshold
};

// The values of the following enumeration must tally with the definitions for the M574 command
enum class EndStopType
{
	noEndStop = 0,
	lowEndStop = 1,
	highEndStop = 2
};

/***************************************************************************************************/

// Enumeration describing the reasons for a software reset.
// The spin state gets or'ed into this, so keep the lower ~4 bits unused.
enum class SoftwareResetReason : uint16_t
{
	user = 0,					// M999 command
	erase = 55,					// special M999 command to erase firmware and reset
	inAuxOutput = 0x0800,		// this bit is or'ed in if we were in aux output at the time
	stuckInSpin = 0x1000,		// we got stuck in a Spin() function for too long
	inLwipSpin = 0x2000,		// we got stuck in a call to LWIP for too long
	inUsbOutput = 0x4000		// this bit is or'ed in if we were in USB output at the time
};

// Enumeration to describe various tests we do in response to the M111 command
enum class DiagnosticTestType : int
{
	TestWatchdog = 1001,			// test that we get a watchdog reset if the tick interrupt stops
	TestSpinLockup = 1002,			// test that we get a software reset if a Spin() function takes too long
	TestSerialBlock = 1003			// test what happens when we write a blocking message via debugPrintf()
};

// Info returned by FindFirst/FindNext calls
class FileInfo
{
public:

	bool isDirectory;
	unsigned long size;
	uint8_t day;
	uint8_t month;
	uint16_t year;
	char fileName[FILENAME_LENGTH];
};

/***************************************************************************************************************/

// Struct for holding Z probe parameters

struct ZProbeParameters
{
	int adcValue;					// the target ADC value
	float xOffset, yOffset;			// the offset of the probe relative to the print head
	float height;					// the nozzle height at which the target ADC value is returned
	float calibTemperature;			// the temperature at which we did the calibration
	float temperatureCoefficient;	// the variation of height with bed temperature
	float diveHeight;				// the dive height we use when probing
	float probeSpeed;				// the initial speed of probing
	float travelSpeed;				// the speed at which we travel to the probe point
	float param1, param2;			// extra parameters used by some types of probe e.g. Delta probe

	void Init(float h)
	{
		adcValue = Z_PROBE_AD_VALUE;
		xOffset = yOffset = 0.0;
		height = h;
		calibTemperature = 20.0;
		temperatureCoefficient = 0.0;	// no default temperature correction
		diveHeight = DEFAULT_Z_DIVE;
		probeSpeed = DEFAULT_PROBE_SPEED;
		travelSpeed = DEFAULT_TRAVEL_SPEED;
		param1 = param2 = 0.0;
	}

	float GetStopHeight(float temperature) const
	{
		return ((temperature - calibTemperature) * temperatureCoefficient) + height;
	}

	bool operator==(const ZProbeParameters& other) const
	{
		return adcValue == other.adcValue
				&& height == other.height
				&& xOffset == other.xOffset
				&& yOffset == other.yOffset
				&& calibTemperature == other.calibTemperature
				&& temperatureCoefficient == other.temperatureCoefficient
				&& diveHeight == other.diveHeight
				&& probeSpeed == other.probeSpeed
				&& travelSpeed == other.travelSpeed
				&& param1 == other.param1
				&& param2 == other.param2;
	}

	bool operator!=(const ZProbeParameters& other) const
	{
		return !operator==(other);
	}
};

class PidParameters
{
	// If you add any more variables to this class, don't forget to change the definition of operator== in Platform.cpp!
private:
	float thermistorBeta, thermistorInfR;				// private because these must be changed together

public:
	float kI, kD, kP, kT, kS;
	float fullBand, pidMin, pidMax;
	float thermistorSeriesR;
	float adcLowOffset, adcHighOffset;

	float GetBeta() const { return thermistorBeta; }
	float GetRInf() const { return thermistorInfR; }

	bool UsePID() const;
	float GetThermistorR25() const;
	void SetThermistorR25AndBeta(float r25, float beta);

	bool operator==(const PidParameters& other) const;
	bool operator!=(const PidParameters& other) const
	{
		return !operator==(other);
	}
};

// Class to perform averaging of values read from the ADC
// numAveraged should be a power of 2 for best efficiency

template<size_t numAveraged> class AveragingFilter
{
public:
	AveragingFilter()
	{
		Init(0);
	}

	void Init(uint16_t val) volatile
	{
		irqflags_t flags = cpu_irq_save();
		sum = (uint32_t)val * (uint32_t)numAveraged;
		index = 0;
		isValid = false;
		for (size_t i = 0; i < numAveraged; ++i)
		{
			readings[i] = val;
		}
		cpu_irq_restore(flags);
	}

	// Call this to put a new reading into the filter
	// This is only called by the ISR, so it not declared volatile to make it faster
	void ProcessReading(uint16_t r)
	{
		sum = sum - readings[index] + r;
		readings[index] = r;
		++index;
		if (index == numAveraged)
		{
			index = 0;
			isValid = true;
		}
	}

	// Return the raw sum
	uint32_t GetSum() const volatile
	{
		return sum;
	}

	// Return true if we have a valid average
	bool IsValid() const volatile
	{
		return isValid;
	}

private:
	uint16_t readings[numAveraged];
	size_t index;
	uint32_t sum;
	bool isValid;
	//invariant(sum == + over readings)
	//invariant(index < numAveraged)
};

typedef AveragingFilter<THERMISTOR_AVERAGE_READINGS> ThermistorAveragingFilter;
typedef AveragingFilter<Z_PROBE_AVERAGE_READINGS> ZProbeAveragingFilter;

// Enumeration of error condition bits
enum class ErrorCode : uint32_t
{
	BadTemp = 1 << 0,
	BadMove = 1 << 1,
	OutputStarvation = 1 << 2,
	OutputStackOverflow = 1 << 3
};

// Different types of hardware-related input-output
enum class SerialSource
{
	USB,
	AUX,
	AUX2
};

struct AxisDriversConfig
{
	size_t numDrivers;								// Number of drivers assigned to each axis
	uint8_t driverNumbers[MaxDriversPerAxis];		// The driver numbers assigned - only the first numDrivers are meaningful
};

// The main class that defines the RepRap machine for the benefit of the other classes
class Platform
{   
public:
	// Enumeration to describe the status of a drive
	enum class DriverStatus : uint8_t { disabled, idle, enabled };
  
	Platform();
  
//-------------------------------------------------------------------------------------------------------------

	// These are the functions that form the interface between Platform and the rest of the firmware.

	void Init();									// Set the machine up after a restart.  If called subsequently this should set the machine up as if
													// it has just been restarted; it can do this by executing an actual restart if you like, but beware the loop of death...
	void Spin();									// This gets called in the main loop and should do any housekeeping needed
	void Exit();									// Shut down tidily. Calling Init after calling this should reset to the beginning

	static void EnableWatchdog();
	static void KickWatchdog();						// kick the watchdog

	Compatibility Emulating() const;
	void SetEmulating(Compatibility c);
	void Diagnostics(MessageType mtype);
	void DiagnosticTest(int d);
	void ClassReport(float &lastTime);  			// Called on Spin() return to check everything's live.
	void LogError(ErrorCode e) { errorCodeBits |= (uint32_t)e; }

	void SoftwareReset(uint16_t reason);
	bool AtxPower() const;
	void SetAtxPower(bool on);
	void SetBoardType(BoardType bt);
	const char* GetElectronicsString() const;

	// Timing
  
	float Time();									// Returns elapsed seconds since some arbitrary time
	static uint32_t GetInterruptClocks();			// Get the interrupt clock count
	static bool ScheduleInterrupt(uint32_t tim);	// Schedule an interrupt at the specified clock count, or return true if it has passed already
	static void DisableStepInterrupt();				// Make sure we get no step interrupts
	void Tick();
  
  	// Communications and data storage
  
	bool GCodeAvailable(const SerialSource source) const;
	char ReadFromSource(const SerialSource source);

	void SetIPAddress(uint8_t ip[]);
	const uint8_t* IPAddress() const;
	void SetNetMask(uint8_t nm[]);
	const uint8_t* NetMask() const;
	void SetGateWay(uint8_t gw[]);
	const uint8_t* GateWay() const;
	void SetMACAddress(uint8_t mac[]);
	const uint8_t* MACAddress() const;
	void SetBaudRate(size_t chan, uint32_t br);
	uint32_t GetBaudRate(size_t chan) const;
	void SetCommsProperties(size_t chan, uint32_t cp);
	uint32_t GetCommsProperties(size_t chan) const;

	friend class FileStore;

	MassStorage* GetMassStorage() const;
	FileStore* GetFileStore(const char* directory, const char* fileName, bool write);
	const char* GetWebDir() const; 					// Where the htm etc files are
	const char* GetGCodeDir() const; 				// Where the gcodes are
	const char* GetSysDir() const;  				// Where the system files are
	const char* GetMacroDir() const;				// Where the user-defined macros are
	const char* GetConfigFile() const; 				// Where the configuration is stored (in the system dir).
	const char* GetDefaultFile() const;				// Where the default configuration is stored (in the system dir).
	void InvalidateFiles();							// Called to invalidate files when the SD card is removed

	// Message output (see MessageType for further details)

	void Message(const MessageType type, const char *message);
	void Message(const MessageType type, OutputBuffer *buffer);
	void MessageF(const MessageType type, const char *fmt, ...);
	void MessageF(const MessageType type, const char *fmt, va_list vargs);
	bool FlushMessages();							// Flush messages to USB and aux, returning true if there is more to send

	// Movement

	void EmergencyStop();
	void SetPhysicalDrives(size_t drive, uint32_t physicalDrives);
	uint32_t GetPhysicalDrives(size_t drive) const;
	void SetDirection(size_t drive, bool direction);
	void SetDirectionValue(size_t drive, bool dVal);
	bool GetDirectionValue(size_t drive) const;
	void SetEnableValue(size_t drive, bool eVal);
	bool GetEnableValue(size_t drive) const;
	void EnableDriver(size_t driver);
	void DisableDriver(size_t driver);
	void EnableDrive(size_t drive);
	void DisableDrive(size_t drive);
	void SetDriversIdle();
	void SetMotorCurrent(size_t drive, float current, bool isPercent);
	float GetMotorCurrent(size_t drive, bool isPercent) const;
	void SetIdleCurrentFactor(float f);
	float GetIdleCurrentFactor() const
		{ return idleCurrentFactor; }
	bool SetDriverMicrostepping(size_t driver, int microsteps, int mode);
	unsigned int GetDriverMicrostepping(size_t drive, bool& interpolation) const;
	bool SetMicrostepping(size_t drive, int microsteps, int mode);
	unsigned int GetMicrostepping(size_t drive, bool& interpolation) const;
	void SetDriverStepTiming(size_t driver, float microseconds);
	float GetDriverStepTiming(size_t driver) const;
	float DriveStepsPerUnit(size_t drive) const;
	const float *GetDriveStepsPerUnit() const
		{ return driveStepsPerUnit; }
	void SetDriveStepsPerUnit(size_t drive, float value);
	float Acceleration(size_t drive) const;
	const float* Accelerations() const;
	void SetAcceleration(size_t drive, float value);
	const float GetMaxAverageAcceleration() const
		{ return maxAverageAcceleration; }
	void SetMaxAverageAcceleration(float f)
		{ maxAverageAcceleration = f; }
	float MaxFeedrate(size_t drive) const;
	const float* MaxFeedrates() const;
	void SetMaxFeedrate(size_t drive, float value);
	float ConfiguredInstantDv(size_t drive) const;
	float ActualInstantDv(size_t drive) const;
	void SetInstantDv(size_t drive, float value);
	EndStopHit Stopped(size_t drive) const;
	float AxisMaximum(size_t axis) const;
	void SetAxisMaximum(size_t axis, float value);
	float AxisMinimum(size_t axis) const;
	void SetAxisMinimum(size_t axis, float value);
	float AxisTotalLength(size_t axis) const;
	float GetElasticComp(size_t drive) const;
	void SetElasticComp(size_t extruder, float factor);
	void SetEndStopConfiguration(size_t axis, EndStopType endstopType, bool logicLevel);
	void GetEndStopConfiguration(size_t axis, EndStopType& endstopType, bool& logicLevel) const;
	uint32_t GetAllEndstopStates() const;
	void SetAxisDriversConfig(size_t drive, const AxisDriversConfig& config);
	const AxisDriversConfig& GetAxisDriversConfig(size_t drive) const
		{ return axisDrivers[drive]; }
	void SetExtruderDriver(size_t extruder, uint8_t driver);
	uint8_t GetExtruderDriver(size_t extruder) const
		{ return extruderDrivers[extruder]; }
	uint32_t GetDriversBitmap(size_t drive) const			// get the bitmap of driver step bits for this axis or extruder
		{ return driveDriverBits[drive]; }
	static void StepDriversLow();							// set all step pins low
	static void StepDriversHigh(uint32_t driverMap);		// set the specified step pins high
	uint32_t GetSlowDrivers() const { return slowDrivers; }
	uint32_t GetSlowDriverClocks() const { return slowDriverStepPulseClocks; }

	// Z probe

	float ZProbeStopHeight();
	float GetZProbeDiveHeight() const;
	float GetZProbeTravelSpeed() const;
	int ZProbe() const;
	EndStopHit GetZProbeResult() const;
	int GetZProbeSecondaryValues(int& v1, int& v2);
	void SetZProbeType(int iZ);
	int GetZProbeType() const;
	void SetZProbeAxes(const bool axes[AXES]);
	void GetZProbeAxes(bool (&axes)[AXES]);
	const ZProbeParameters& GetZProbeParameters() const;
	bool SetZProbeParameters(const struct ZProbeParameters& params);
	bool MustHomeXYBeforeZ() const;

	void SetExtrusionAncilliaryPWM(float v);
	float GetExtrusionAncilliaryPWM() const;
	void ExtrudeOn();
	void ExtrudeOff();

	// Heat and temperature

	float GetTemperature(size_t heater, TemperatureError* err = nullptr); // Result is in degrees Celsius
	void SetHeater(size_t heater, float power);				// power is a fraction in [0,1]
	float HeatSampleTime() const;
	void SetHeatSampleTime(float st);
	void SetPidParameters(size_t heater, const PidParameters& params);
	const PidParameters& GetPidParameters(size_t heater) const;
	float TimeToHot() const;
	void SetTimeToHot(float t);
	void SetThermistorNumber(size_t heater, size_t thermistor);
	int GetThermistorNumber(size_t heater) const;
	bool IsThermistorChannel(uint8_t heater) const;
	bool IsThermocoupleChannel(uint8_t heater) const;
	bool IsRtdChannel(uint8_t heater) const;
	void SetTemperatureLimit(float t);
	float GetTemperatureLimit() const { return temperatureLimit; }
	void UpdateConfiguredHeaters();
	bool AnyHeaterHot(uint16_t heaters, float t);			// called to see if we need to turn on the hot end fan

	// Fans

	float GetFanValue(size_t fan) const;					// Result is returned in percent
	void SetFanValue(size_t fan, float speed);				// Accepts values between 0..1 and 1..255
	bool GetCoolingInverted(size_t fan) const;
	void SetCoolingInverted(size_t fan, bool inv);
	float GetFanPwmFrequency(size_t fan) const;
	void SetFanPwmFrequency(size_t fan, float freq);
	float GetTriggerTemperature(size_t fan) const;
	void SetTriggerTemperature(size_t fan, float t);
	uint16_t GetHeatersMonitored(size_t fan) const;
	void SetHeatersMonitored(size_t fan, uint16_t h);
	float GetFanRPM();

	// Flash operations
	void ResetNvData();
	void ReadNvData();
	void WriteNvData();
	void SetAutoSave(bool enabled);

	void UpdateFirmware();
	bool CheckFirmwareUpdatePrerequisites();

	// AUX device
	void Beep(int freq, int ms);
	void SendMessage(const char* msg);

	// Hotend configuration
	float GetFilamentWidth() const;
	void SetFilamentWidth(float width);
	float GetNozzleDiameter() const;
	void SetNozzleDiameter(float diameter);

	// Fire the inkjet (if any) in the given pattern
	// If there is no inkjet false is returned; if there is one this returns true
	// So you can test for inkjet presence with if(platform->Inkjet(0))
	bool Inkjet(int bitPattern);

	// Direct pin operations
	bool SetPin(int pin, int level);

	// MCU temperature
	void GetMcuTemperatures(float& minT, float& currT, float& maxT) const;
	void SetMcuTemperatureAdjust(float v) { mcuTemperatureAdjust = v; }
	float GetMcuTemperatureAdjust() const { return mcuTemperatureAdjust; }

#ifdef DUET_NG
	// Power in voltage
	void GetPowerVoltages(float& minV, float& currV, float& maxV) const;
#endif

//-------------------------------------------------------------------------------------------------------
  
private:
	void ResetChannel(size_t chan);					// re-initialise a serial channel
	float AdcReadingToCpuTemperature(uint16_t reading) const;

#ifdef DUET_NG
	static float AdcReadingToPowerVoltage(uint16_t reading);
#endif

	// These are the structures used to hold out non-volatile data.
	// The SAM3X doesn't have EEPROM so we save the data to flash. This unfortunately means that it gets cleared
	// every time we reprogram the firmware via bossa, but it can be retained when firmware updates are performed
	// via the web interface. That's why it's a good idea to implement versioning here - increase these values
	// whenever the fields of the following structs have changed.

	struct SoftwareResetData
	{
		static const uint16_t magicValue = 0x7C5F;	// value we use to recognise that all the flash data has been written
		static const uint16_t versionValue = 1;		// increment this whenever this struct changes
		static const uint32_t nvAddress = 0;		// must be 4-byte aligned

		uint16_t magic;
		uint16_t version;

		uint16_t resetReason;						// this records why we did a software reset, for diagnostic purposes
		uint16_t dummy;								// padding to align the next field (should happen automatically I think)
		size_t neverUsedRam;						// the amount of never used RAM at the last abnormal software reset
	};

	struct FlashData
	{
		static const uint16_t magicValue = 0xE6C4;	// value we use to recognise that the flash data has been written
		static const uint16_t versionValue = 1;		// increment this whenever this struct changes
		static const uint32_t nvAddress = (SoftwareResetData::nvAddress + sizeof(SoftwareResetData) + 3) & (~3);

		uint16_t magic;
		uint16_t version;

		// The remaining data could alternatively be saved to SD card.
		// Note however that if we save them as G codes, we need to provide a way of saving IR and ultrasonic G31 parameters separately.
		ZProbeParameters switchZProbeParameters;	// Z probe values for the switch Z-probe
		ZProbeParameters irZProbeParameters;		// Z probe values for the IR sensor
		ZProbeParameters alternateZProbeParameters;	// Z probe values for the alternate sensor
		int zProbeType;								// the type of Z probe we are currently using
		bool zProbeAxes[AXES];						// Z probe is used for these axes
		PidParameters pidParams[HEATERS];
		byte ipAddress[4];
		byte netMask[4];
		byte gateWay[4];
		uint8_t macAddress[6];
		Compatibility compatibility;
	};

	FlashData nvData;
	bool autoSaveEnabled;

	BoardType board;

	float lastTime;
	float longWait;
	float addToTime;
	unsigned long lastTimeCall;

	bool active;
	uint32_t errorCodeBits;

	void InitialiseInterrupts();
	void GetStackUsage(size_t* currentStack, size_t* maxStack, size_t* neverUsed) const;

	// DRIVES

	void SetDriverCurrent(size_t driver, float current, bool isPercent);
	void UpdateMotorCurrent(size_t driver);
	void SetDriverDirection(uint8_t driver, bool direction);
	static uint32_t CalcDriverBitmap(size_t driver);			// calculate the step bit for this driver

	volatile DriverStatus driverState[DRIVES];
	bool directions[DRIVES];
	bool enableValues[DRIVES];
	Pin endStopPins[DRIVES];
	float maxFeedrates[DRIVES];
	float accelerations[DRIVES];
	float driveStepsPerUnit[DRIVES];
	float instantDvs[DRIVES];
	float elasticComp[DRIVES - AXES];
	float motorCurrents[DRIVES];					// the normal motor current for each stepper driver
	float motorCurrentFraction[DRIVES];				// the percentages of normal motor current that each driver is set to
	AxisDriversConfig axisDrivers[AXES];			// the driver numbers assigned to each axis
	uint8_t extruderDrivers[DRIVES - AXES];			// the driver number assigned to each extruder
	uint32_t driveDriverBits[DRIVES];				// the bitmap of driver port bits for each axis or extruder
	uint32_t slowDriverStepPulseClocks;				// minimum high and low step pulse widths, in processor clocks
	uint32_t slowDrivers;							// bitmap of driver port bits that need extended step pulse timing
	float idleCurrentFactor;
	float maxAverageAcceleration;

	// Digipots

#if defined(DUET_NG) && !defined(PROTOTYPE_1)
	size_t numTMC2660Drivers;						// the number of TMC2660 drivers we have, the remaining are simple enable/step/dir drivers
#else
	MCP4461 mcpDuet;
	MCP4461 mcpExpansion;
	Pin potWipes[8];								// we have only 8 digipots, on the Duet 0.8.5 we use the DAC for the 9th
	float senseResistor;
	float maxStepperDigipotVoltage;
	float stepperDacVoltageRange, stepperDacVoltageOffset;
#endif

	// Z probe

	Pin zProbePin;
	Pin zProbeModulationPin;
	volatile ZProbeAveragingFilter zProbeOnFilter;					// Z probe readings we took with the IR turned on
	volatile ZProbeAveragingFilter zProbeOffFilter;					// Z probe readings we took with the IR turned off
	volatile ThermistorAveragingFilter thermistorFilters[HEATERS];	// bed and extruder thermistor readings

	float extrusionAncilliaryPWM;

	void InitZProbe();
	uint16_t GetRawZProbeReading() const;
	void UpdateNetworkAddress(byte dst[4], const byte src[4]);

	// Axes and endstops

	float axisMaxima[AXES];
	float axisMinima[AXES];
	EndStopType endStopType[AXES+1];
	bool endStopLogicLevel[AXES+1];
  
  // Heaters - bed is assumed to be the first

	int GetRawThermistorTemperature(size_t heater) const;
	void SetHeaterPwm(size_t heater, uint8_t pwm);

	Pin tempSensePins[HEATERS];
	Pin heatOnPins[HEATERS];
	TemperatureSensor SpiTempSensors[MaxSpiTempSensors];
	Pin spiTempSenseCsPins[MaxSpiTempSensors];
	uint32_t configuredHeaters;										// bitmask of all heaters in use
	float heatSampleTime;
	float timeToHot;
	float temperatureLimit;

	// Fans

	Fan fans[NUM_FANS];
	Pin coolingFanRpmPin;											// we currently support only one fan RPM input
	float lastRpmResetTime;
	void InitFans();

  	// Serial/USB

	uint32_t baudRates[NUM_SERIAL_CHANNELS];
	uint8_t commsParams[NUM_SERIAL_CHANNELS];
	OutputStack *auxOutput;
	OutputStack *aux2Output;
	OutputStack *usbOutput;

	// Files

	MassStorage* massStorage;
	FileStore* files[MAX_FILES];
	bool fileStructureInitialised;
	const char* webDir;
	const char* gcodeDir;
	const char* sysDir;
	const char* macroDir;
	const char* configFile;
	const char* defaultFile;
  
	// Data used by the tick interrupt handler

	// Heater #n, 0 <= n < HEATERS, uses "temperature channel" tc given by
	//
	//     tc = heaterTempChannels[n]
	//
	// Temperature channels follow a convention of
	//
	//     if (0 <= tc < HEATERS) then
	//        The temperature channel is a thermistor read using ADC.
	//        The actual ADC to read for tc is
	//
	//            thermistorAdcChannel[tc]
	//
	//        which, is equivalent to
	//
	//            PinToAdcChannel(tempSensePins[tc])
	//
	//     if (100 <= tc < 100 + (MaxSpiTempSensors - 1)) then
	//        The temperature channel is a thermocouple attached to a MAX31855 chip
	//        The MAX31855 object corresponding to the specific MAX31855 chip is
	//
	//            Max31855Devices[tc - 100]
	//
	//       Note that the MAX31855 objects, although statically declared, are not
	//       initialized until configured via a "M305 Pn X10m" command with 0 <= n < HEATERS
	//       and 0 <= m < MaxSpiTempSensors.
	//
	// NOTE BENE: When a M305 command is processed, the onus is on the gcode processor,
	// GCodes.cpp, to range check the value of the X parameter.  Code consuming the results
	// of the M305 command (e.g., SetThermistorNumber() and array lookups assume range
	// checking has already been performed.

	unsigned int heaterTempChannels[HEATERS];
	AnalogChannelNumber thermistorAdcChannels[HEATERS];
	AnalogChannelNumber zProbeAdcChannel;
	uint32_t thermistorOverheatSums[HEATERS];
	uint8_t tickState;
	size_t currentHeater;
	int debugCode;

	// Hotend configuration
	float filamentWidth;
	float nozzleDiameter;

	// Temperature and power monitoring
	AnalogChannelNumber temperatureAdcChannel;
	uint16_t currentMcuTemperature, highestMcuTemperature, lowestMcuTemperature;
	uint16_t mcuAlarmTemperature;
	float mcuTemperatureAdjust;

#ifdef DUET_NG
	AnalogChannelNumber vInMonitorAdcChannel;
	volatile uint16_t currentVin, highestVin, lowestVin;
	uint16_t upperVinLimit, lowerVinLimit;
	bool driversPowered;
#endif

	// Direct pin manipulation
	static const uint8_t pinAccessAllowed[NUM_PINS_ALLOWED/8];
	uint8_t pinInitialised[NUM_PINS_ALLOWED/8];
};

// Small class to hold an open file and data relating to it.
// This is designed so that files are never left open and we never duplicate a file reference.
class FileData
{
public:
	FileData() : f(NULL) {}

	// Set this to refer to a newly-opened file
	void Set(FileStore* pfile)
	{
		Close();	// close any existing file
		f = pfile;
	}

	bool IsLive() const { return f != NULL; }

	bool Close()
	{
		if (f != NULL)
		{
			bool ok = f->Close();
			f = NULL;
			return ok;
		}
		return false;
	}

	bool Read(char& b)
	{
		return f->Read(b);
	}

	bool Write(char b)
	{
		return f->Write(b);
	}

	bool Write(const char *s, unsigned int len)
	{
		return f->Write(s, len);
	}

	bool Flush()
	{
		return f->Flush();
	}

	FilePosition GetPosition() const
	{
		return f->Position();
	}

	bool Seek(FilePosition position)
	{
		return f->Seek(position);
	}

	float FractionRead() const
	{
		return (f == NULL ? -1.0 : f->FractionRead());
	}

	FilePosition Length() const
	{
		return f->Length();
	}

	// Assignment operator
	void CopyFrom(const FileData& other)
	{
		Close();
		f = other.f;
		if (f != NULL)
		{
			f->Duplicate();
		}
	}

	// Move operator
	void MoveFrom(FileData& other)
	{
		Close();
		f = other.f;
		other.Init();
	}

private:
	FileStore *f;

	void Init()
	{
		f = NULL;
	}

	// Private assignment operator to prevent us assigning these objects
	FileData& operator=(const FileData&);

	// Private copy constructor to prevent us copying these objects
	FileData(const FileData&);
};

// Where the htm etc files are

inline const char* Platform::GetWebDir() const
{
  return webDir;
}

// Where the gcodes are

inline const char* Platform::GetGCodeDir() const
{
  return gcodeDir;
}

// Where the system files are

inline const char* Platform::GetSysDir() const
{
  return sysDir;
}

inline const char* Platform::GetMacroDir() const
{
	return macroDir;
}

inline const char* Platform::GetConfigFile() const
{
  return configFile;
}

inline const char* Platform::GetDefaultFile() const
{
  return defaultFile;
}


//*****************************************************************************************************************

// Drive the RepRap machine - Movement

inline float Platform::DriveStepsPerUnit(size_t drive) const
{
  return driveStepsPerUnit[drive]; 
}

inline void Platform::SetDriveStepsPerUnit(size_t drive, float value)
{
  driveStepsPerUnit[drive] = value;
}

inline float Platform::Acceleration(size_t drive) const
{
	return accelerations[drive];
}

inline const float* Platform::Accelerations() const
{
	return accelerations;
}

inline void Platform::SetAcceleration(size_t drive, float value)
{
	accelerations[drive] = value;
}

inline float Platform::MaxFeedrate(size_t drive) const
{
  return maxFeedrates[drive];
}

inline const float* Platform::MaxFeedrates() const
{
	return maxFeedrates;
}

inline void Platform::SetMaxFeedrate(size_t drive, float value)
{
	maxFeedrates[drive] = value;
}

inline float Platform::ConfiguredInstantDv(size_t drive) const
{
	return instantDvs[drive];
}

inline void Platform::SetInstantDv(size_t drive, float value)
{
	instantDvs[drive] = value;
}

inline void Platform::SetDirectionValue(size_t drive, bool dVal)
{
	directions[drive] = dVal;
}

inline bool Platform::GetDirectionValue(size_t drive) const
{
	return directions[drive];
}

inline void Platform::SetDriverDirection(uint8_t driver, bool direction)
{
	if (driver < DRIVES)
	{
		bool d = (direction == FORWARDS) ? directions[driver] : !directions[driver];
		digitalWrite(DIRECTION_PINS[driver], d);
	}
}

inline void Platform::SetEnableValue(size_t drive, bool eVal)
{
	enableValues[drive] = eVal;
}

inline bool Platform::GetEnableValue(size_t drive) const
{
	return enableValues[drive];
}

inline float Platform::AxisMaximum(size_t axis) const
{
	return axisMaxima[axis];
}

inline void Platform::SetAxisMaximum(size_t axis, float value)
{
	axisMaxima[axis] = value;
}

inline float Platform::AxisMinimum(size_t axis) const
{
	return axisMinima[axis];
}

inline void Platform::SetAxisMinimum(size_t axis, float value)
{
	axisMinima[axis] = value;
}

inline float Platform::AxisTotalLength(size_t axis) const
{
	return axisMaxima[axis] - axisMinima[axis];
}

inline void Platform::SetExtrusionAncilliaryPWM(float v)
{
	extrusionAncilliaryPWM = v;
}

inline float Platform::GetExtrusionAncilliaryPWM() const
{
	return extrusionAncilliaryPWM;
}

// For the Duet we use the fan output for this
// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOn()
{
	if (extrusionAncilliaryPWM > 0.0)
	{
		SetFanValue(0,extrusionAncilliaryPWM); //@TODO T3P3 currently only turns fan0 on
	}
}

// DC 2015-03-21: To allow users to control the cooling fan via gcodes generated by slic3r etc.,
// only turn the fan on/off if the extruder ancilliary PWM has been set nonzero.
// Caution: this is often called from an ISR, or with interrupts disabled!
inline void Platform::ExtrudeOff()
{
	if (extrusionAncilliaryPWM > 0.0)
	{
		SetFanValue(0,0.0); //@TODO T3P3 currently only turns fan0 off
	}
}

//********************************************************************************************************

// Drive the RepRap machine - Heat and temperature

inline int Platform::GetRawThermistorTemperature(size_t heater) const
{
  return (heater < HEATERS)
		  ? thermistorFilters[heater].GetSum()/(THERMISTOR_AVERAGE_READINGS >> AD_OVERSAMPLE_BITS)
		  : 0;
}

inline float Platform::HeatSampleTime() const
{
  return heatSampleTime;
}

inline void Platform::SetHeatSampleTime(float st)
{
	heatSampleTime = st;
}

inline float Platform::TimeToHot() const
{
	return timeToHot;
}

inline void Platform::SetTimeToHot(float t)
{
	timeToHot = t;
}

inline bool Platform::IsThermistorChannel(uint8_t heater) const
{
	return heaterTempChannels[heater] < HEATERS;
}

inline bool Platform::IsThermocoupleChannel(uint8_t heater) const
{
	return heaterTempChannels[heater] >= FirstThermocoupleChannel
			&& heaterTempChannels[heater] - FirstThermocoupleChannel < MaxSpiTempSensors;
}

inline bool Platform::IsRtdChannel(uint8_t heater) const
{
	return heaterTempChannels[heater] >= FirstRtdChannel
			&& heaterTempChannels[heater] - FirstRtdChannel < MaxSpiTempSensors;
}

inline const uint8_t* Platform::IPAddress() const
{
	return nvData.ipAddress;
}

inline const uint8_t* Platform::NetMask() const
{
	return nvData.netMask;
}

inline const uint8_t* Platform::GateWay() const
{
	return nvData.gateWay;
}

inline const uint8_t* Platform::MACAddress() const
{
	return nvData.macAddress;
}

inline float Platform::GetElasticComp(size_t extruder) const
{
	return (extruder < DRIVES - AXES) ? elasticComp[extruder] : 0.0;
}

inline void Platform::SetEndStopConfiguration(size_t axis, EndStopType esType, bool logicLevel)
//pre(axis < AXES)
{
	endStopType[axis] = esType;
	endStopLogicLevel[axis] = logicLevel;
}

inline void Platform::GetEndStopConfiguration(size_t axis, EndStopType& esType, bool& logicLevel) const
//pre(axis < AXES)
{
	esType = endStopType[axis];
	logicLevel = endStopLogicLevel[axis];
}

// Get the interrupt clock count
/*static*/ inline uint32_t Platform::GetInterruptClocks()
{
	return STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
}

// This is called by the tick ISR to get the raw Z probe reading to feed to the filter
inline uint16_t Platform::GetRawZProbeReading() const
{
	switch (nvData.zProbeType)
	{
	case 4:
		{
			bool b = digitalRead(endStopPins[E0_AXIS]);
			if (!endStopLogicLevel[AXES])
			{
				b = !b;
			}
			return (b) ? 4000 : 0;
		}

	case 5:
		return (digitalRead(zProbePin)) ? 4000 : 0;

	default:
		return AnalogInReadChannel(zProbeAdcChannel);
	}
}

inline float Platform::GetFilamentWidth() const
{
	return filamentWidth;
}

inline void Platform::SetFilamentWidth(float width)
{
	filamentWidth = width;
}

inline float Platform::GetNozzleDiameter() const
{
	return nozzleDiameter;
}

inline void Platform::SetNozzleDiameter(float diameter)
{
	nozzleDiameter = diameter;
}

inline MassStorage* Platform::GetMassStorage() const
{
	return massStorage;
}

/*static*/ inline void Platform::EnableWatchdog()
{
	watchdogEnable(1000);
}

/*static*/ inline void Platform::KickWatchdog()
{
	watchdogReset();
}

inline float Platform::AdcReadingToCpuTemperature(uint16_t adcVal) const
{
	float voltage = (float)adcVal * (3.3/4096.0);
#ifdef DUET_NG
	return (voltage - 1.44) * (1000.0/4.7) + 27.0 + mcuTemperatureAdjust;			// accuracy at 27C is +/-13C
#else
	return (voltage - 0.8) * (1000.0/2.65) + 27.0 + mcuTemperatureAdjust;			// accuracy at 27C is +/-45C
#endif
}

#ifdef DUET_NG
inline float Platform::AdcReadingToPowerVoltage(uint16_t adcVal)
{
	return adcVal * (PowerFailVoltageRange/4096.0);
}
#endif

// *** These next two functions must use the same bit assignments in the drivers bitmap ***
// The bitmaps are organised like this:
// Duet WiFi:
//	All step pins are on port D, so the bitmap is just the map of bits in port D.
// Duet 0.6 and 0.8.5:
//	Step pins are PA0, PC7,9,11,14,25,29 and PD0,3.
//	The PC and PD bit numbers don't overlap, so we use their actual positions.
//	PA0 clashes with PD0, so we use bit 1 to represent PA0.
// RADDS:
//	To be done

// Calculate the step bit for a driver. This doesn't need to be fast.
/*static*/ inline uint32_t Platform::CalcDriverBitmap(size_t driver)
{
	const PinDescription& pinDesc = g_APinDescription[STEP_PINS[driver]];
#if defined(DUET_NG)
	return pinDesc.ulPin;
#elif defined(__RADDS__)
# error needs writing
#else
	return (pinDesc.pPort == PIOA) ? pinDesc.ulPin << 1 : pinDesc.ulPin;
#endif
}

// Set the specified step pins high and all other step pins low
// This needs to be as fast as possible, so we do a parallel write to the port(s).
// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
/*static*/ inline void Platform::StepDriversHigh(uint32_t driverMap)
{
#if defined(DUET_NG)
	PIOD->PIO_ODSR = driverMap;				// on Duet WiFi all step pins are on port D
#elif defined(__RADDS__)
# error need to write this
#else	// Duet
	PIOD->PIO_ODSR = driverMap;
	PIOC->PIO_ODSR = driverMap;
	PIOA->PIO_ODSR = driverMap >> 1;		// do this last, it means the processor doesn't need to preserve the register containing driverMap
#endif
}

// Set all step pins low
// This needs to be as fast as possible, so we do a parallel write to the port(s).
// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
/*static*/ inline void Platform::StepDriversLow()
{
#if defined(DUET_NG)
	PIOD->PIO_ODSR = 0;						// on Duet WiFi all step pins are on port D
#elif defined(__RADDS__)
# error need to write this
#else	// Duet
	PIOD->PIO_ODSR = 0;
	PIOC->PIO_ODSR = 0;
	PIOA->PIO_ODSR = 0;
#endif
}

//***************************************************************************************

#endif
