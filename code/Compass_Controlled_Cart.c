#include "engr2350_msp432.h"
#include <stdlib.h>
#include <math.h>

void GPIOInit();
void TimerInit();
void ADCInit();
void Encoder_ISR();
void T2_100ms_ISR();

void I2CInit();
uint16_t readPitch();
uint16_t readRoll();
uint16_t readHeading();

Timer_A_UpModeConfig TA0cfg; // PWM timer
Timer_A_UpModeConfig TA2cfg; // 100 ms timer
Timer_A_ContinuousModeConfig TA3cfg; // Encoder timer
Timer_A_CompareModeConfig TA0_ccr3; // PWM Right
Timer_A_CompareModeConfig TA0_ccr4; // PWM Left
Timer_A_CaptureModeConfig TA3_ccr0; // Encoder Right
Timer_A_CaptureModeConfig TA3_ccr1; // Encoder Left

// Encoder total events
uint32_t enc_total_L,enc_total_R;
int32_t enc_turn_L, enc_turn_R;
// Speed measurement variables
// Note that "Tach" stands for "Tachometer," or a device used to measure rotational speed
int32_t TachL_count,TachL,TachL_sum,TachL_sum_count,TachL_avg; // Left wheel
int32_t TachR_count,TachR,TachR_sum,TachR_sum_count,TachR_avg; // Right wheel
    // TachL,TachR are equivalent to enc_counts from Activity 10/Lab 3
    // TachL/R_avg is the averaged TachL/R value after every 12 encoder measurements
    // The rest are the intermediate variables used to assemble TachL/R_avg

uint8_t runControl = 0; // Flag to denote that 100ms has passed and control should be run.

float speedControl;
float turnControl;
int32_t turn;
int32_t desiredSpeed, desiredSpeedL, desiredSpeedR;
float correctedSpeedL, correctedSpeedR;
float measuredSpeedL, measuredSpeedR;
int16_t diffSpeed;
float max_diff;
int32_t speedErrorL, speedErrorR, intErrorL, intErrorR;
float kI = 0.02, kP = 0.2, kD = 0.1;  //kI = 0.02, kP = 0.1
float minRad = 0.0675, maxRad = 1.0, rad;

int32_t compareValL, compareValR;
uint16_t maxPWM = 864, minPWM = 96;
uint16_t minSpeed = 20, maxSpeed = 480;

int32_t measuredHeading;
int32_t desiredHeading, headingError, prev_headingError;
uint8_t fL, fR;
float deltaT;

int main() {    /** Main Function ****/
    sysInit();
    GPIOInit();
    TimerInit();

    I2CInit();

    __delay_cycles(24e6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN7);

    while( 1 ) {
        if(runControl){ // If 100 ms has passed
            runControl = 0; // Reset the 100 ms flag

            //convert pitch and roll measurements to be -128 to 128 instead of 0 to 255
            if (readPitch() <= 128) { speedControl = -(readPitch()); }
            else { speedControl = (256 - readPitch()); }
            desiredSpeed = (480 * (speedControl))/128;
            //desiredSpeed = 0;


            //determine turn control method
            if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN0)) {       //Part A roll turn
                kI = 0.02;
                TA2cfg.timerPeriod = 37499;
                if (readRoll() <= 128) { turnControl = (readRoll()); }
                else { turnControl = -(256 - readRoll()); }
                turn = (480.0 * (turnControl ))/128;
                //turn
                if (abs(turn) < 13) {
                    rad = 0;
                    desiredSpeedL = desiredSpeed;
                    desiredSpeedR = desiredSpeed;
                } else if (turn < -13) { rad = maxRad - (maxRad - minRad) * (abs(turn)) / 480.0; }
                else if (turn > 13) { rad = maxRad - (maxRad - minRad) * (abs(turn)) / 480.0; }
                if (rad < minRad) { rad = minRad; }
                max_diff = (0.5 * maxSpeed * 0.135)/0.0675;
                diffSpeed = (desiredSpeed * 0.5 * 0.135) / rad;
                if(diffSpeed > max_diff) { diffSpeed = max_diff; }
                else if (diffSpeed < -(max_diff)) { diffSpeed = -(max_diff); }

                //turning control
                if (turn < -20) {
                    desiredSpeedL = desiredSpeed - diffSpeed;
                    desiredSpeedR = desiredSpeed + diffSpeed;
                } else if( turn > 20) {
                    desiredSpeedL = desiredSpeed + diffSpeed;
                    desiredSpeedR = desiredSpeed - diffSpeed;
                }
            }else if (!GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN0)){         //Part B angle turn
                kI = 0.00;
                TA2cfg.timerPeriod = 18749;
                if (readHeading() <= 1800) { desiredHeading = -(1800 - readHeading()); }
                else { desiredHeading = readHeading() - 1800; }

                deltaT = (enc_turn_L - enc_turn_R) * ((2 * 0.035) / (360.0 * 0.135)) * 1800.0;
                enc_turn_L = 0;
                enc_turn_R = 0;

                measuredHeading += deltaT;

                headingError = desiredHeading - measuredHeading;
                while (headingError > 1800) { headingError -= 3600; }
                while (headingError < -1800) { headingError += 3600; }

                diffSpeed = (kP * headingError) + (kD * (headingError - prev_headingError));
                prev_headingError = headingError;

                if (diffSpeed > 192) {
                    diffSpeed = 192;
                }

                if (abs(headingError) < 10) {
                    desiredSpeedL = desiredSpeed;
                    desiredSpeedR = desiredSpeed;
                }
                else if (abs(headingError) > 10 ) {
                    desiredSpeedL = desiredSpeed + diffSpeed;
                    desiredSpeedR = desiredSpeed - diffSpeed;
                }
            }

            //wheel control
            //left
            if (abs(desiredSpeedL) < minSpeed){
                desiredSpeedL = 0;
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4 , desiredSpeedL);
            }
            else if(abs(desiredSpeedL) > maxSpeed){
                desiredSpeedL = maxSpeed;
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4 , desiredSpeedL);
            }
            else{
                if (desiredSpeedL < 0) {
                    fL = 0;
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN4);
                    desiredSpeedL = abs(desiredSpeedL);
                }
                else{ GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4);fL = 1; }
                measuredSpeedL = 15000000/TachL_avg;
                speedErrorL = desiredSpeedL - measuredSpeedL;
                intErrorL += speedErrorL;
                correctedSpeedL = desiredSpeedL + kI * intErrorL;
                if(correctedSpeedL >= 0) {
                    if (correctedSpeedL < minPWM) { correctedSpeedL = minPWM; }
                    else if (correctedSpeedL > maxPWM) { correctedSpeedL = maxPWM; }
                }
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4 , correctedSpeedL);
            }

            //right
            if (abs(desiredSpeedR) < minSpeed){
                desiredSpeedR = 0;
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3 , desiredSpeedR);
            }
            else if(abs(desiredSpeedR) > maxSpeed){
                desiredSpeedR = maxSpeed;
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3 , desiredSpeedR);
            }
            else{
                if (desiredSpeedR < 0) {
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN5);
                    desiredSpeedR = abs(desiredSpeedR);
                    fR = 0;
                }
                else { GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN5);fR = 1; }
                measuredSpeedR = 15000000/TachR_avg;
                speedErrorR = desiredSpeedR - measuredSpeedR;
                intErrorR += speedErrorR;
                correctedSpeedR = desiredSpeedR + kI * intErrorR;
                if(correctedSpeedR >= 0) {
                    if (correctedSpeedR < minPWM) { correctedSpeedR = minPWM; }
                    if (correctedSpeedR > maxPWM) { correctedSpeedR = maxPWM; }
                }
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3 , correctedSpeedR);
            }
            printf("DiffSpeed: %d, Theta: %f, desiredL: %d, Err: %d, M: %d, D: %d\t\r", diffSpeed, deltaT, desiredSpeedL, headingError, measuredHeading, desiredHeading);
        }
    }
}   /** End Main Function ****/

void GPIOInit(){
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motor direction pins
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motor enable pins
        // Motor PWM pins
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6|GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
        // Motor Encoder pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10,GPIO_PIN4|GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motors set to forward
    GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motors are OFF

    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P4, GPIO_PIN1 , GPIO_TERTIARY_MODULE_FUNCTION );     //A12
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P4, GPIO_PIN4 , GPIO_TERTIARY_MODULE_FUNCTION );     //A9

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION);   //SDA
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);   //SCL
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN0);    //SS input
}

void TimerInit(){
    // Configure PWM timer for 24 kHz
    TA0cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA0cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA0cfg.timerPeriod = 959;
    Timer_A_configureUpMode(TIMER_A0_BASE,&TA0cfg);
    // Configure TA0.CCR3 for PWM output, Right Motor
    TA0_ccr3.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    TA0_ccr3.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr3.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr3);
    // Configure TA0.CCR4 for PWM output, Left Motor
    TA0_ccr4.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    TA0_ccr4.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr4.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr4);
    // Configure Encoder timer in continuous mode
    TA3cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA3cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA3cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureContinuousMode(TIMER_A3_BASE,&TA3cfg);
    // Configure TA3.CCR0 for Encoder measurement, Right Encoder
    TA3_ccr0.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    TA3_ccr0.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr0.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr0.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr0.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr0);
    // Configure TA3.CCR1 for Encoder measurement, Left Encoder
    TA3_ccr1.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    TA3_ccr1.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr1.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr1.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr1.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr1);
    // Register the Encoder interrupt
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCR0_INTERRUPT,Encoder_ISR);
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,Encoder_ISR);
    // Configure 10 Hz timer
    TA2cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA2cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    TA2cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    TA2cfg.timerPeriod = 37499; //change to 50ms
    Timer_A_configureUpMode(TIMER_A2_BASE,&TA2cfg);
    Timer_A_registerInterrupt(TIMER_A2_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,T2_100ms_ISR);
    // Start all the timers
    Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A2_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A3_BASE,TIMER_A_CONTINUOUS_MODE);
}

void I2CInit() {
    eUSCI_I2C_MasterConfig config;
    config.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    config.i2cClk = 24000000;
    config.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    config.byteCounterThreshold = 0;
    I2C_initMaster(EUSCI_B3_BASE, &config);
    I2C_enableModule(EUSCI_B3_BASE);
}

uint16_t readPitch() {      //pitch 0 - 255
    uint8_t data[2];
    I2C_readData(EUSCI_B3_BASE, 0x60 , 4, data, 2);
    uint16_t pitch = data[0];
    return pitch;
}

uint16_t readRoll() {       //roll 0 - 255
    uint8_t data[2];
    I2C_readData(EUSCI_B3_BASE, 0x60 , 4, data, 2);
    uint16_t roll = data[1];
    return roll;
}

uint16_t readHeading() {        //2 byte heading
    uint8_t data[2];
    I2C_readData(EUSCI_B3_BASE, 0x60, 2, data, 2);
    //uint16_t heading = data[0];
    uint16_t heading = data[1] + (data[0] << 8);
    return heading;
}

// Add interrupt functions last so they are easy to find
void Encoder_ISR(){
    // If encoder timer has overflowed...
    if(Timer_A_getEnabledInterruptStatus(TIMER_A3_BASE) == TIMER_A_INTERRUPT_PENDING){
        Timer_A_clearInterruptFlag(TIMER_A3_BASE);
        TachR_count += 65536;
        if(TachR_count >= 1e6){ // Enforce a maximum count to TachR so stopped can be detected
            TachR_count = 1e6;
            TachR = 1e6;
        }
        TachL_count += 65536;
        if(TachL_count >= 1e6){ // Enforce a maximum count to TachL so stopped can be detected
            TachL_count = 1e6;
            TachL = 1e6;
        }
    // Otherwise if the Left Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN0)) {
            enc_total_R++;
        } else if (!GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN0)) {
            if (fR) {   //right wheel direction
                enc_turn_R++;
            } else if (!fR) {
                enc_turn_R--;
            }
        }

        // Calculate and track the encoder count values
        TachR = TachR_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        TachR_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        // Sum values for averaging
        TachR_sum_count++;
        TachR_sum += TachR;
        // If 6 values have been received, average them.
        if(TachR_sum_count == 6){
            TachR_avg = TachR_sum/6;
            TachR_sum_count = 0;
            TachR_sum = 0;
        }
    // Otherwise if the Right Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN0)) {
            enc_total_L++;
        } else if (!GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN0)) {
            if (fL) {   //right wheel direction
                enc_total_L++;
            } else if (!fL) {
                enc_total_L--;
            }
        }
        TachL = TachL_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        TachL_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        TachL_sum_count++;
        TachL_sum += TachL;
        if(TachL_sum_count == 6){
            TachL_avg = TachL_sum/6;
            TachL_sum_count = 0;
            TachL_sum = 0;
        }
    }
}

void T2_100ms_ISR(){
    Timer_A_clearInterruptFlag(TIMER_A2_BASE);
    runControl = 1;
}