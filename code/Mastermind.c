#include "engr2350_msp432.h"

#include <string.h>
#include <stdio.h>

#include <stdint.h>
#include <stdlib.h>
#include <time.h>

// Add function prototypes here, as needed.
void GPIOInit();
void timerInit();
int8_t readBumpers();
void setRGB(int8_t color);
uint8_t checkGuess(int8_t *sol,int8_t *guess,int8_t *result);
void printResult(int8_t *guess,int8_t *result);

void createSol();
void Timer_ISR();
void UpdateTime(uint8_t * time_arry);

// Add global variables here, as needed.
const uint8_t Lpattern = 4; // The pattern length. Could be changed if desired.

uint8_t PB1 = 0, BMP0 = 0, BMP1 = 0, BMP2 = 0, BMP3 = 0, BMP4 = 0, BMP5 = 0;
int8_t color = -1;      //color starts off
uint8_t start = 0;      //timer start/stop
uint8_t counter = 0;    //overflow count
uint8_t third = 0;
uint8_t reset = 0;
uint8_t timeout = 0;
uint8_t guess_status = 0;
uint8_t guess_num;
uint8_t win = 0;

int8_t sol[4];
int8_t guess[4];
int8_t result[4];

uint8_t time_arry[4];   //array for total time
uint8_t round_num = 0;      //track when failed
uint8_t led_status = 1;
uint8_t start_led = 0;

int main() {    //// Main Function ////
    
    sysInit();
    GPIOInit();
    timerInit();

    // Place initialization code (or run-once) code here
    printf("\n\rColordle Started\n\rPress the button to begin\n\r");
    while(1){
        Timer_A_stopTimer(TIMER_A0_BASE);
        while(!start){
            //setRGB(readBumpers());
            readBumpers();
            __delay_cycles(240e3);
        }
        if(start){
            start_led = 1;
            round_num = 0;
            win = 0;
            memset(guess, 0, sizeof(guess));
            memset(result, 0, sizeof(result));
            memset(sol, 0, sizeof(sol));
            memset(time_arry, 0, sizeof(time_arry));
            printf("\rNew Game: \n\r");
            //srand(time(NULL));
            createSol();            //create random solution
            Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
            while(start_led){       //indicate start of game
                setRGB(6);
            }setRGB(-1);
            Timer_A_stopTimer(TIMER_A0_BASE);
            while(round_num < 5 && !win){        //5 total guesses allowed
                start = 1;
                guess_status = 1;
                round_num ++;
                guess_num = 0;
                printf("Round #%u:",round_num);
                Timer_A_clearTimer(TIMER_A0_BASE);
                Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
                while(!timeout & guess_num != 4 ){      //take guesses when not timed out and less than 4 guesses
                    if(reset){
                        reset = 0;
                        memset(guess, 0, sizeof(guess));        //reset the guesses
                        guess_num = 0;
                    }
                    setRGB(readBumpers());
                    __delay_cycles(240e3);
                }
                guess_status = 0;
                Timer_A_stopTimer(TIMER_A0_BASE);           //stop timer
                if(timeout){
                    timeout = 0;
                    uint8_t i = 0;
                    for(i = 0;i<4;i++){
                        result[i] = 'R';
                    }
                    printf("\n\rTimeout!");

                }else{
                    checkGuess(sol,guess,result);
                    if(checkGuess(sol,guess,result) == 4){
                        win = 1;
                    }else{
                        win = 0;
                    }
                }
                setRGB(-1);
                start = 0;
                __delay_cycles(24e6);
                Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
                uint8_t i;
                counter = 0;
                for(i = 0; i<4;i++){            //display results
                    while(led_status){
                        setRGB(result[i]);
                    }while(!led_status){
                        setRGB(-1);
                    }
                }
                Timer_A_stopTimer(TIMER_A0_BASE);
                printf("\n\r");
                for(i = 0; i<4;i++){
                    printf("%c",guess[i]);
                }
                printf(" | ");
                printResult(guess,result);
            }if(win){
                printf("\n\rWin! Took %u tries. Time: %u mins %u seconds\n\r",round_num,time_arry[2],time_arry[1]);
                Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
                counter = 0;
                while(readBumpers() != 6){          //display results
                    if(led_status){                 //while?
                        setRGB(1);
                    }else if(!led_status){
                        setRGB(-1);
                    }
                }
            }else{
                printf("\n\rLost. Total Time: %u mins %u seconds\n\r",time_arry[2],time_arry[1]);
                Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
                counter = 0;
                while(readBumpers() != 6){          //display results
                    if(led_status){
                        setRGB(0);
                    }else if(!led_status){
                        setRGB(-1);
                    }
                }
            }
        }
    }
}   //// Main Function ////

void GPIOInit() {
    // Complete for Part B
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN0|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);
}

void timerInit() {
    // Complete for Part B. Also add interrupt function
    Timer_A_UpModeConfig config = {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_32,
        37500,                                      //50ms
        TIMER_A_TAIE_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR,
    };
    Timer_A_configureUpMode(TIMER_A0_BASE,&config);
    Timer_A_registerInterrupt(TIMER_A0_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,Timer_ISR);
}

int8_t readBumpers() {
    // Complete for Part B
    PB1 = GPIO_getInputPinValue(GPIO_PORT_P6,GPIO_PIN0);
    BMP0 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN0);
    BMP1 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN2);
    BMP2 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN3);
    BMP3 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN5);
    BMP4 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN6);
    BMP5 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN7);




    //if(!BMP0||!BMP1||!BMP2||!BMP3||!BMP4||!BMP5){
     //   printf("PRESSED");
        if(!BMP0){
            while(!BMP0){
                BMP0 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN0);
                setRGB(0);
            }
            guess[guess_num] = 'R';
            //printf("guess num %u", guess_num);
            guess_num ++;
            return 0;   //red
        }else if(!BMP1){
            while(!BMP1){
                BMP1 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN2);
                setRGB(1);
            }
            guess[guess_num] = 'G';
            //printf("guess num %u", guess_num);
            guess_num ++;
            return 1;   //green
        }else if(!BMP2){
            while(!BMP2){
                BMP2 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN3);
                setRGB(2);
            }
            guess[guess_num] = 'B';
            guess_num ++;
            //printf("gues num %u", guess_num);
            return 2;   //blue
        }else if(!BMP3){
            while(!BMP3){
            BMP3 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN5);
            setRGB(3);
            }
            guess[guess_num] = 'Y';
            guess_num ++;
            return 3;   //yellow
        }else if(!BMP4){
            while(!BMP4){
                BMP4 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN6);
                setRGB(4);
            }
            guess[guess_num] = 'M';
            guess_num ++;
            return 4;   //magenta
        }else if(!BMP5){
            while(!BMP5){
                BMP5 = GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN7);
                setRGB(5);
            }
            guess[guess_num] = 'C';
            guess_num ++;
            return 5;   //cyan
        }
    else if(PB1){
        if(start){
            reset = 1;
        }else{
        start = 1;
        }
        setRGB(6);
        return 6;
        //}
    }else{
        setRGB(-1);
        return -1;
    }
}

void setRGB(int8_t color) {
    // Complete for Part B
    if(color == 0){
        //printf("red");
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1|GPIO_PIN2);
    }else if(color == 1){
        //printf("green");
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN2);
    }else if(color == 2){
        //printf("blue");
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1);
    }else if(color == 3){
        //printf("yellow");
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);
    }else if(color == 4){
        //printf("magenta");
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);
    }else if(color == 5){
        //printf("cyan");
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1|GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);
    }else if(color == 6){
        //printf("white");
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);
    }else if(color == -1){
        //printf("off");
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);
    }
}

void createSol(){
    int8_t colors[6] = {'R', 'G', 'B', 'Y', 'M', 'C'};
    uint8_t i = 0;
    for(i = 0;i < 4;i++){
            sol[i] = colors[rand() % 6];
    }
}

/**
 * checkGuess is used to check the player's guess against the solution
 * and produce the associated correct positions, incorrect positions, and
 * incorrect colors.
 *
 *  !!! WARNING !!! All of these inputs are expected to be pointers. Arrays are
 *              !!! technically pointers already! They should not have an & in
 *              !!! front of them when passed into the function.
 * Input Parameters:
 *      int8_t * sol: A 4-element array that stores the game solution (input)
 *      int8_t * guess: A 4-element array that stores the player's guess (input)
 *      int8_t * result: A 4-element array that stores the guess correctness result
 *                  This array is really an output of this function. It is
 *                  modified within the function, with the changes persistent
 *                  after the function is complete.
 *                  This array will only have the values of:
 *                      0: Red - Incorrect color
 *                      1: Green - Correct color and position
 *                      3: Yellow - Correct color, incorrect position
 * Outputs:
 *      uint8_t - the number of correct positions. This may be used to determine
 *              is the guess was correct.
 */

uint8_t checkGuess(int8_t *sol,int8_t *guess,int8_t *result) {
    uint8_t _i,_j; // Loop variables. underscores added to avoid conflict with possible globals.
    uint8_t matched[4]; // Array to store if a color in the answer has been matched yet or not
    for(_i=0;_i<Lpattern;_i++){ // set default values of arrays
        result[_i] = 0; // Answer is incorrect (RED)
        matched[_i] = 0; // Guess position is not used yet
    }
    uint8_t Ncorrect = 0; // Number of positions correct.
    // Fist loop through and find corrects
    for(_i=0;_i<Lpattern;_i++){
        if(sol[_i] == guess[_i]){ // If the guess and answer match...
            Ncorrect++; // Increment number of correct guesses
            result[_i] = 1; // 1 for green
            matched[_i] = 1; // 1 for used (can't compare this position again)
        }
    }
    // Now check for correct color, incorrect position
    for(_i=0;_i<Lpattern;_i++){ // Loop through guess positions
        if(result[_i] == 1) continue; // If this position is marked correct, skip it
        for(_j=0;_j<Lpattern;_j++){ // Loop through answer positions, looking for the same color
        // if(i==j) continue; // If checking the same position, skip. This isn't necessary as it would correspond
                            // the correct case and would be skipped by the "checked" array anyway
            if(matched[_j]) continue; // If this answer color is already taken by a correct or close, skip it
            if(guess[_i] == sol[_j]){ // If the colors are the same (correct color, incorrect position)
                result[_i] = 3; // 3 for yellow
                matched[_j] = 1; // 1 for used (can't compare this position again)
            }
        }
    }
    return Ncorrect; // return number of correct positions
}

/*
 * printResult will take the players guess and the checked result and print them
 * in the necessary format on the terminal. The colors in the player's guess will be
 * printed first, using the first letter of each color. Afterwards the result of
 * the guess is printed using the characters:
 *              $ - correct color and position (Green result)
 *              O - correct color, incorrect position (Yellow result)
 *              X - incorrect color (Red result)
 *
 *  !!! WARNING !!! Both of these inputs are expected to be pointers. Arrays are
 *                  technically pointers already! They should not have an & in
 *                  front of them when passed into the function.
 * Input Parameters:
 *      int8_t * guess: A 4-element array that stores the player's guess (input)
 *      int8_t * result: A 4-element array that stores the guess correctness result
 */

void printResult(int8_t *guess,int8_t *result){
    uint8_t _i = 0; // loop variable
    for(_i=0;_i<Lpattern;_i++){
        switch(guess[_i]){
        case 0: putchar('R'); break;
        case 1: putchar('G'); break;
        case 2: putchar('B'); break;
        case 3: putchar('Y'); break;
        case 4: putchar('M'); break;
        case 5: putchar('C'); break;
        }
    }
    putchar(' '); // put a space in
    for(_i=0;_i<Lpattern;_i++){
        switch(result[_i]){
        case 0: putchar('X'); break;
        case 3: putchar('O'); break;
        case 1: putchar('$'); break;
        }
    }
    putchar('\r');putchar('\n');
}

void UpdateTime(uint8_t * time_arry){
    time_arry[0]++;  // Increment tenths of seconds
    if(time_arry[0] == 10){  // If a whole second has passed...
        time_arry[0] = 0;   // Reset tenths of seconds
        time_arry[1]++;     // And increment seconds
        if(time_arry[1] == 60){ // If a minute has passed...
            time_arry[1] = 0;   // Reset seconds
            time_arry[2]++;     // Increment minutes
            if(time_arry[2] == 60){  // and so on...
                time_arry[2] = 0;
                time_arry[3]++;
                if(time_arry[3] == 24){
                    time_arry[3] = 0;
                }
            }
        }
    }
}

// Add interrupt functions last so they are easy to find

void Timer_ISR(){
    Timer_A_clearInterruptFlag(TIMER_A0_BASE);
    counter++;

    if(start){
        if(guess_status){
            if(counter % 2 == 0){               //accurate to 100ms
                UpdateTime(time_arry);
            }
            if(counter == 200){     //guessing timer for 20
                third ++;
                counter = 0;
            }if(third == 3){
                timeout = 1;
                third = 0;
            }
        }else{
            if(counter == 20){          //1s for starting led
                start_led = 0;
                counter = 0;
            }
        }
    }else{
        if(counter == 10){          //500 ms    toggling/displaying led results
            led_status = !led_status;
            counter = 0;
        }
    }
}

