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

#define SERIALPORT Serial3

HardwareSerial Serial3 = HardwareSerial(PB8, PB9);

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

// StepperDriver4PWM driver = StepperDriver4PWM(MOT_A1, MOT_A2, MOT_B1, MOT_B2);
StepperDriver4PWM driver = StepperDriver4PWM(PA0, PA9, PA1, PA10, PB12);
// StepperDriver4PWM driver = StepperDriver4PWM(MOT_A1, MOT_A2, MOT_B1, MOT_B2);
StepperMotor motor = StepperMotor(POLEPAIRS, RPHASE, MOTORKV, 0.0035);
StepDirListener step_dir = StepDirListener( PC11, PA8, _2PI/(200.0*16) );
void onStep() { step_dir.handle(); }

// StepperMotor motor = StepperMotor(POLEPAIRS, RPHASE);

// MotorPositionModel model(1.625e-5, 2.06e-4, 1.65e-2, 4.346, 5.24e-3);
// StateSpaceController<3, 1, 1, false> observer(model);

Observer observer = Observer(1.625e-5, 2.06e-4, 1.65e-2, 4.346, 5.24e-3);

// Matrix<1> y = {0};
// Matrix<1> u = {0};
int time_prev;

// Observer observer = Observer(9e-6, 3.5e-6, 0.11, RPHASE, 0.0045);

SineSweep test = SineSweep();


Commander commander = Commander(SERIALPORT);

uint16_t counter = 0;
extern volatile uint16_t adc1Result[3];
extern volatile uint16_t adc2Result[2];

DQCurrent_s foc_currents;
float electrical_angle;
PhaseCurrent_s phase_currents;

// Prototypes
uint8_t configureFOC(void);
uint8_t configureCAN(void);
void calibrateEncoder(char *c);
void userButton(void);

void testExecute(char* c) {
	test.Execute();
}

void setup()
{
	test.amplitude = 0.1;
	pinMode(LED_GOOD, OUTPUT);
	pinMode(LED_FAULT, OUTPUT);
	pinMode(CAL_EN, OUTPUT);
	pinMode(MOT_EN, OUTPUT);
	// pinMode(USER_BUTTON, INPUT);
    SimpleFOC_CORDIC_Config();      // initialize the CORDIC

	// attachInterrupt(USER_BUTTON, userButton, RISING);

	SERIALPORT.begin(115200);

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
	// observer.initialise();
	observer.L = {1, 0.0, 0.0};
	// observer.K = {99.26, 1.37, 1.539};

	time_prev=micros();

	test.linkMotor(&motor);
	test.linkSerial(&SERIALPORT);

	// commander.scalar(&test.max_frequency, (char*)'x');
	// commander.scalar(&test.min_frequency, (char*)'n');
	// commander.scalar(&test.amplitude, (char*)'a');
	// commander.scalar(&test.steps, (char*)'s');
	// commander.scalar(&test.cycles_per_step, (char*)'c');

	commander.add('X', &testExecute);
	commander.add('E', &calibrateEncoder);


}


// float est_angle=0;
// float dt=0;
// float disturbance=0;

// float dist_prev = 0;
// float filt_dist = 0;

// PIDController dist_PID = PIDController(100, 400, 1 , 0, motor.voltage_limit);
// LowPassFilter dist_filter = LowPassFilter(1.0f/1000.0f);
void loop()
{

	// observer.r(0) = motor.target;
    // disturbance=(motor.shaft_angle- est_angle);
	// y(0) = disturbance;
    // u(0)=motor.feedforward_voltage.q;
	// observer.u=u;


	// int t_now = micros();
	// dt = (t_now-time_prev)*1e-6;
	// time_prev=t_now;

	// observer.x_hat = observer.estimate_state<false>(y,dt);
	// observer.update(y,dt);


	// est_angle=dist_filter(observer.x_hat(0));
	

	// filt_dist = dist_prev*0.3 + 0.7*disturbance;
	// filt_dist = dist_filter(disturbance);
	// dist_prev = disturbance;
	
	// observer.update(motor.feedforward_voltage.q, motor.shaft_angle, dt);
	// est_angle=dist_filter(observer.getSensorValue());
	// motor.pid_voltage.q = -dist_PID(disturbance);
	step_dir.cleanLowVelocity();  
	motor.move();
	motor.loopFOC();
	commander.run();

	// digitalWrite(LED_FAULT, digitalRead(PC11));

	if(counter == 0xFFFF){
		digitalToggle(LED_GOOD);
		// Serial.println(adc1Result[0]);
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
	driver.voltage_power_supply = 12;
	driver.voltage_limit = driver.voltage_power_supply/2;
	driver.init();

	// Motor PID parameters.
    motor.PID_velocity.P = 0.1;
    motor.PID_velocity.I = 4;
    motor.PID_velocity.D = 0.00;
    motor.LPF_velocity.Tf = 0.01;
	motor.PID_velocity.output_ramp = 0;
	motor.PID_velocity.limit = 500;

	motor.P_angle.P = 1500;
    motor.P_angle.I = 5;
    motor.P_angle.D = 8.0;
	motor.LPF_angle.Tf = 0.00; // try to avoid

	motor.motion_downsample = 20;

	// Motor initialization.
	motor.voltage_sensor_align = 2;
	motor.current_limit = 2;
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
	motor.target = 2.0;
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
}
