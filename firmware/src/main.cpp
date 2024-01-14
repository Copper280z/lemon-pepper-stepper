#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

#include "can.h"
#include "dfu.h"
#include "utils.h"
#include "InlineCurrentSenseSync.h"
#include "lemon-pepper.h"
#include "old.h"
#include "sfoc_tools.h"


#define USBD_MANUFACTURER_STRING "matei repair lab"
#define USBD_PRODUCT_STRING_FS "lemon-pepper-stepper"

// board specific data
typedef struct
{
	uint16_t signature;
	Direction electricalDir;
	float electricalZero;
	uint16_t abzResolution;
	uint8_t encoderCalibrated;
	uint8_t canID;
} userData;

userData boardData;
uint8_t updateData = 0;

const uint16_t magicWord = 0xAF0C;

// canbus things
extern volatile uint8_t TxData[8];
extern volatile uint8_t RxData[8];

// simpleFOC things
#define POLEPAIRS 50
#define RPHASE 2.0
#define MOTORKV 40
#define ENC_PPR 16383 // max 16383 (zero index) -> *4 for CPR, -1 is done in init to prevent rollover on 16 bit timer

#define SERIALPORT SerialUSB

// HardwareSerial Serial3 = HardwareSerial(PB8, PB9);

/**
 * SPI clockdiv of 16 gives ~10.5MHz clock. May still be stable with lower divisor.
 * The HW encoder is configured using PPR, which is then *4 for CPR (full 12384 gives overflow on 16 bit timer.)
 */
SPISettings myMT6835SPISettings(168000000 / 10, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 sensor = MagneticSensorMT6835(ENC_CS, myMT6835SPISettings);
// SPISettings myMT6835SPISettings(168000000 / 10, MT6835_BITORDER, SPI_MODE3);
// MagneticSensorMT6835 sensor = MagneticSensorMT6835(ENC_CS);
STM32HWEncoder enc = STM32HWEncoder(ENC_PPR, ENC_A, ENC_B, ENC_Z);

/**
 * TODO: Change the current sense code to reflect the new inline current sense amplifier choice with sense resistor.
 */
// InlineCurrentSenseSync currentsense = InlineCurrentSenseSync(90, ISENSE_U, ISENSE_V, ISENSE_W);


StepperDriver4PWM driver = StepperDriver4PWM(PA0, PA9, PA1, PA10, PB12);
StepperMotor motor = StepperMotor(POLEPAIRS, RPHASE, MOTORKV, 0.0035);
StepDirListener step_dir = StepDirListener( PC11, PA8, _2PI/(200.0*16) );
void onStep() { step_dir.handle(); }

int time_prev;


SineSweep sine_test = SineSweep();
PRBS prbs_test = PRBS();

Commander commander = Commander(SERIALPORT);

uint16_t counter = 0;
float stepCounter;

DQCurrent_s foc_currents;
float electrical_angle;
PhaseCurrent_s phase_currents;

// Prototypes
uint8_t configureFOC(void);
uint8_t configureCAN(void);
void calibrateEncoder(char *c);
void userButton(void);
void onStep(void) {step_dir.handle();}

void sineExecute(char* c) {
	sine_test.Execute();
}

void prbsExecute(char* c) {
	prbs_test.Execute_args(c);
}

void setup()
{
	sine_test.amplitude = 0.1;
	pinMode(LED_GOOD, OUTPUT);
	pinMode(LED_FAULT, OUTPUT);
	pinMode(CAL_EN, OUTPUT);
	pinMode(MOT_EN, OUTPUT);
	// pinMode(USER_BUTTON, INPUT);
    SimpleFOC_CORDIC_Config();      // initialize the CORDIC

	// attachInterrupt(USER_BUTTON, userButton, RISING);


	SERIALPORT.begin(2000000);

	EEPROM.get(0, boardData);

	digitalWrite(MOT_EN, HIGH);
	digitalWrite(CAL_EN, LOW);

	uint8_t ret;
	// ret = configureCAN();
	// if (!ret){
	// 	SIMPLEFOC_DEBUG("CAN init failed.");
	// 	digitalWrite(LED_FAULT, HIGH);
	// }


	step_dir.init();
	step_dir.enableInterrupt(onStep);
    step_dir.attach(&motor.target, &motor.feed_forward_velocity);

	ret = configureFOC();
	if (!ret){
		SIMPLEFOC_DEBUG("FOC init failed.");
		digitalWrite(LED_FAULT, HIGH);
	}

	if (sensor.getABZResolution() != ENC_PPR) // Check that PPR of the encoder matches our expectation.
	{
		digitalWrite(LED_FAULT, HIGH);
		SIMPLEFOC_DEBUG("Encoder ABZ resolution unexpected.");
	}


	time_prev=micros();

	sine_test.linkMotor(&motor);
	sine_test.linkSerial(&SERIALPORT);
	
	prbs_test.linkMotor(&motor);
	prbs_test.linkSerial(&SERIALPORT);
	
	// commander.scalar(&test.max_frequency, (char*)'x');
	// commander.scalar(&test.min_frequency, (char*)'n');
	// commander.scalar(&test.amplitude, (char*)'a');
	// commander.scalar(&test.steps, (char*)'s');
	// commander.scalar(&test.cycles_per_step, (char*)'c');

	commander.add('X', &sineExecute);
	commander.add('P', &prbsExecute);
	commander.add('E', &calibrateEncoder);

	SERIALPORT.println("BMP Over TCP!!!");


}


void loop()
{

	step_dir.cleanLowVelocity();  
	motor.move();
	motor.loopFOC();
	commander.run();


	if(counter == 0xFFFF){
		digitalToggle(LED_GOOD);
		counter = 0;
	}

	counter++;

#ifdef HAS_MONITOR
	motor.monitor();
#endif
}

void doMotor(char *cmd)
{
	commander.motor(&motor, cmd);
}

uint8_t configureFOC(void)
{
	commander.add('M', doMotor, "motor");
	commander.verbose = VerboseMode::machine_readable;

#ifdef SIMPLEFOC_STM32_DEBUG
	SimpleFOCDebug::enable(&SERIALPORT);
#endif

	// Encoder initialization.
	// Ideally configuring the sensor over SPI then use STM32HWEncoder

	step_dir.init();
	step_dir.enableInterrupt(onStep);
	step_dir.attach(&stepCounter);

	enc.init();
	if (!enc.initialized)
		digitalWrite(LED_FAULT, HIGH);

	sensor.init();
	// sensor.setHysteresis(7);
	// Check if the encoder has loaded the right PPR, if not, update and then write to EEPROM.
	if (sensor.getABZResolution() != ENC_PPR)
	{
		delay(200);
		sensor.setABZResolution(ENC_PPR);
		sensor.writeEEPROM();

		digitalWrite(LED_GOOD, HIGH);
		digitalWrite(LED_FAULT, LOW);

		for (uint8_t i = 0; i < 60; i++)
		{ // Datasheet says we need to wait 6 seconds after writing EEPROM.
			digitalToggle(LED_GOOD);
			digitalToggle(LED_FAULT);
			delay(100);
		}

		digitalWrite(LED_GOOD, LOW);
		digitalWrite(LED_FAULT, LOW);
	}

	// Driver initialization.
	driver.pwm_frequency = 35000;
	driver.voltage_power_supply = 12.0f;
	driver.voltage_limit = driver.voltage_power_supply/2.0f;
	driver.init();

	// Motor PID parameters.
    motor.PID_velocity.P = 0.075f;
    motor.PID_velocity.I = 4.0f;
    motor.PID_velocity.D = 0.00f;
    motor.LPF_velocity.Tf = 0.0015f;
	motor.PID_velocity.output_ramp = 0.0f;
	motor.PID_velocity.limit = 500.0f;

	motor.P_angle.P = 400.0f;
    motor.P_angle.I = 5.0f;
    motor.P_angle.D = 1.0f;
	motor.LPF_angle.Tf = 0.00f; // try to avoid

	motor.motion_downsample = 1;

	// Motor initialization.
	motor.voltage_sensor_align = 2.0f;
	motor.current_limit = 2.0f;
	motor.velocity_limit = 500;
	motor.controller = MotionControlType::angle;
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

// Monitor initialization
#ifdef HAS_MONITOR
	motor.useMonitoring(SERIALPORT);
	motor.monitor_start_char = 'M';
	motor.monitor_end_char = 'M';
	motor.monitor_downsample = 250;
#endif

	motor.linkSensor(&sensor);
	motor.linkDriver(&driver);

	// currentsense.linkDriver(&driver);
	// int ret = currentsense.init();
	// SERIALPORT.printf("Current Sense init result: %i\n", ret);
	// motor.linkCurrentSense(&currentsense);



	// motor.zero_electric_angle = 0.24; // small motor
	motor.zero_electric_angle = 0.625; // big motor
	motor.sensor_direction = Direction::CCW;

	motor.init();
	motor.initFOC();


	// calibrateEncoder();
	sensor.update();
	float start_angle = motor.shaftAngle();
	SERIALPORT.printf("Setting target to: %.2f\n", start_angle);
	motor.target = start_angle;
	// if(boardData.signature != magicWord){
	// 	// If we have not initialized the EEPROM before.
	// 	motor.init();
	// 	motor.initFOC();

	// 	boardData.signature = magicWord;
	// 	boardData.electricalZero = motor.zero_electric_angle;
	// 	boardData.electricalDir = motor.sensor_direction;
	//  updateData = 1;
	// }
	// else{
	// 	motor.zero_electric_angle = boardData.electricalZero;
	// 	motor.sensor_direction = boardData.electricalDir;
	// 	motor.init();
	// 	motor.initFOC();
	// }

	return 1;
}

uint8_t configureCAN(void)
{
	FDCAN_Start(0x000);
	return 1;
}

void calibrateEncoder(char* c)
{
	MotionControlType orig_controller = motor.controller;
	motor.controller = MotionControlType::velocity_openloop;
	motor.enable();

	motor.target = 7; // roughly 150rpm -> need to write 0x5 to Reg. AUTOCAL_FREQ
	int revs = 64;
	MT6835Options4 currentSettings = sensor.getOptions4();
	currentSettings.autocal_freq = 0x6;
	sensor.setOptions4(currentSettings);

	uint32_t calTime = micros();
	while ((micros() - calTime) < ((revs/(motor.target/_2PI) + 1.2)*1e6))
	{
		motor.loopFOC();
		motor.move();

		if ((micros() -calTime) > 200000)
		{
			// after motor is spinning at constant speed, enable calibration.
			digitalWrite(LED_GOOD, HIGH);
			digitalWrite(CAL_EN, HIGH);
		}
	}
	motor.target = 0;
	digitalWrite(LED_GOOD, LOW);
	digitalWrite(CAL_EN, LOW);
	motor.controller = orig_controller;

	digitalWrite(LED_GOOD, HIGH);
	digitalWrite(LED_FAULT, LOW);

	for (uint8_t i = 0; i < 60; i++)
	{ // Datasheet says we need to wait 6 seconds after writing EEPROM.
		digitalToggle(LED_GOOD);
		digitalToggle(LED_FAULT);
		delay(100);
	}

	digitalWrite(LED_GOOD, LOW);
	digitalWrite(LED_FAULT, LOW);

	// return sensor.getCalibrationStatus();
	return;
}

void userButton(void)
{
	if(USB->DADDR != 0)
		jump_to_bootloader();
	else
		digitalToggle(LED_FAULT);
}

