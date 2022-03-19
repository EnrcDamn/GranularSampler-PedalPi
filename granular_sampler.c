// Granular delay / micro-sampler for Raspberry Pedal Pi
// https://github.com/EnrcDamn/granular-sampler-pedal-pi


// Include libraries
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>

// Define input pins
#define PUSH1 	        RPI_GPIO_P1_08      //GPIO14
#define PUSH2           RPI_V2_GPIO_P1_38  	//GPIO20 
#define TOGGLE_SWITCH 	RPI_V2_GPIO_P1_32 	//GPIO12
#define FOOT_SWITCH     RPI_GPIO_P1_10 		//GPIO15
#define LED             RPI_V2_GPIO_P1_36 	//GPIO16

#define TRUE 1
#define FALSE 0
#define PRESSED 0
#define LOOP_MAX 250000 // 5 seconds approx.
#define DELAY_MAX 50000 // 1 second
#define PI 3.14159265359

// Global variables
uint32_t microseconds_delay = 20; // 20 microseconds
uint32_t sample_rate;
float sample_duration;
uint8_t mosi[10] = { 0x01, 0x00, 0x00 }; // 12 bit ADC read 0x08 ch0, - 0c for ch1 
uint8_t miso[10] = { 0 };

// Structures
typedef struct {
    uint32_t read_timer;
    uint8_t FOOT_SWITCH_val;
    uint8_t TOGGLE_SWITCH_val;
    uint8_t PUSH1_val;
    uint8_t is_PUSH1_pressed;
    uint8_t PUSH2_val;
    uint8_t LED_blinking;
    uint8_t is_LED_on;
    long LED_timer;
} BoardStatus;

typedef struct {
    uint32_t* Loop_Buffer;
    float pitch;
    uint8_t recording;
    uint8_t playback_mode;
    uint32_t starting_sample;
    uint32_t ending_sample;
    float sample_index;
    uint8_t is_buffer_maxed_out;
    uint32_t record_length;
} BufferStatus;

typedef struct {
    float phase;
    float phase_stride; // phase increment for every time step
    float rate;
} Oscillator;

typedef struct {
    uint32_t index;
    uint32_t factor;
    uint32_t min_size;
    uint32_t size;
    uint32_t position;
    uint8_t is_reversed;
} Grain;

typedef struct {
    uint32_t* Echo_Buffer;
    uint32_t index;
} Delay;


int init(
    BoardStatus* board,
    BufferStatus* buff,
    Grain* grain,
    Delay* delay,
    Oscillator* lfo_1,
    Oscillator* lfo_2)
{
    // Start the BCM2835 Library to access GPIO.
    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }
	// Start the SPI BUS.
	if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      return 1;
    }
 
    // Define PWM	
    bcm2835_gpio_fsel(18, BCM2835_GPIO_FSEL_ALT5 ); // PWM0 signal on GPIO18    
    bcm2835_gpio_fsel(13, BCM2835_GPIO_FSEL_ALT0 ); // PWM1 signal on GPIO13    
    bcm2835_pwm_set_clock(2);       // Max clk frequency (19.2MHz/2 = 9.6MHz)
    bcm2835_pwm_set_mode(0, 1, 1);  // channel 0, markspace mode, PWM enabled. 
    bcm2835_pwm_set_range(0, 64);   // channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
    bcm2835_pwm_set_mode(1, 1, 1);  // channel 1, markspace mode, PWM enabled.
    bcm2835_pwm_set_range(1, 64);   // channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
 
    // Define SPI bus configuration
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);    // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                 // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); 	// 4MHz clock with _64 
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                    // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);    // The default

    // Define GPIO pins configuration
    bcm2835_gpio_fsel(PUSH1, BCM2835_GPIO_FSEL_INPT);           // PUSH1 button as input
    bcm2835_gpio_fsel(PUSH2, BCM2835_GPIO_FSEL_INPT);           // PUSH2 button as input
    bcm2835_gpio_fsel(TOGGLE_SWITCH, BCM2835_GPIO_FSEL_INPT);   // TOGGLE_SWITCH as input
    bcm2835_gpio_fsel(FOOT_SWITCH, BCM2835_GPIO_FSEL_INPT);     // FOOT_SWITCH as input
    bcm2835_gpio_fsel(LED, BCM2835_GPIO_FSEL_OUTP);             // LED as output

    bcm2835_gpio_set_pud(PUSH1, BCM2835_GPIO_PUD_UP);           // PUSH1 pull-up enabled   
    bcm2835_gpio_set_pud(PUSH2, BCM2835_GPIO_PUD_UP);           // PUSH2 pull-up enabled 
    bcm2835_gpio_set_pud(TOGGLE_SWITCH, BCM2835_GPIO_PUD_UP);   // TOGGLE_SWITCH pull-up enabled 
    bcm2835_gpio_set_pud(FOOT_SWITCH, BCM2835_GPIO_PUD_UP);     // FOOT_SWITCH pull-up enabled

    // Define variables
    board->PUSH1_val = FALSE;
    board->PUSH2_val = FALSE;
    board->TOGGLE_SWITCH_val = FALSE;
    board->FOOT_SWITCH_val = FALSE;
    board->read_timer = 0;
    board->is_PUSH1_pressed = FALSE;
    board->LED_blinking = FALSE;
    board->is_LED_on = FALSE;
    board->LED_timer = 0;

    buff->Loop_Buffer = (uint32_t*)calloc(LOOP_MAX, sizeof(uint32_t));
    buff->starting_sample = 0;
    buff->ending_sample = LOOP_MAX - 1;
    buff->recording = FALSE;
    buff->playback_mode = 0;
    buff->is_buffer_maxed_out = FALSE;
    buff->sample_index = 0;
    buff->record_length = 0;
    buff->pitch = 1.0f;

    grain->index = 0;
    grain->factor = 10000; // 0.5 seconds
    grain->min_size = 0;
    grain->size = 0;
    grain->position = 0;
    grain->is_reversed = FALSE; // randomly choose TRUE or FALSE

    delay->Echo_Buffer = (uint32_t*)calloc(DELAY_MAX, sizeof(uint32_t));
    delay->index = 0;

    lfo_1->phase = 0.0f;
    lfo_1->rate = 0.5f;
    lfo_1->phase_stride = lfo_1->rate * sample_duration;

    lfo_2->phase = 0.0f;
    lfo_2->rate = 0.25f;
    lfo_2->phase_stride = lfo_2->rate * sample_duration;
    return 0;
}


void readInputSignal(uint32_t* input_signal)
{
    // Read 12 bits ADC
    bcm2835_spi_transfernb(mosi, miso, 3);
    *input_signal = miso[2] + ((miso[1] & 0x0F) << 8);
}


void readControls(BoardStatus* board)
{   
    // Update controls values
    board->FOOT_SWITCH_val = bcm2835_gpio_lev(FOOT_SWITCH);
    if (board->FOOT_SWITCH_val == PRESSED)
    {
        board->PUSH1_val = bcm2835_gpio_lev(PUSH1);
        board->PUSH2_val = bcm2835_gpio_lev(PUSH2);
        board->TOGGLE_SWITCH_val = bcm2835_gpio_lev(TOGGLE_SWITCH);
    }
}


void setControls( 
    BoardStatus* board, 
    BufferStatus* buff) 
{    
    // Read the controls approx. every 0.2 seconds to save resources
    
    board->read_timer++;
    if (board->read_timer < sample_rate / 5) 
        return;
    board->read_timer = 0;

    readControls(board);

    // Light the effect when the footswitch is activated 
    bcm2835_gpio_write(LED, !board->FOOT_SWITCH_val); 

    // When TOGGLE_SWITCH is in UP position
    if (board->TOGGLE_SWITCH_val == 0) 
    {   
        // When the button is pressed
        if (board->PUSH1_val == PRESSED) 
            board->is_PUSH1_pressed = TRUE;
        // When the button is released
        if (board->PUSH1_val == !PRESSED && board->is_PUSH1_pressed == TRUE) 
        {   
            board->is_PUSH1_pressed = FALSE; 
            // Stopping recording...
            if (buff->recording == TRUE) 
            {    
                buff->pitch = 1.0f; 
                buff->recording = FALSE;
                buff->ending_sample = buff->sample_index; 
                if (buff->sample_index >= LOOP_MAX - 1) 
                    buff->starting_sample = 0; 
                else 
                { 
                    if (buff->is_buffer_maxed_out) 
                        buff->starting_sample = buff->ending_sample + 1; 
                    else 
                        buff->starting_sample = 0; 
                } 
                buff->is_buffer_maxed_out = FALSE; 
                buff->sample_index = buff->starting_sample; 
            } 
            else
            // Starting recording...
            {   
                buff->pitch = 1.0f; 
                buff->recording = TRUE;
                buff->sample_index = 0;
                buff->is_buffer_maxed_out = FALSE;
                buff->record_length = 0;
            }
        } 
        if (board->PUSH2_val == PRESSED) 
        {    
            bcm2835_delay(100); // 100ms delay for buttons debouncing 
            buff->playback_mode++; 
            if (buff->playback_mode > 2) 
                buff->playback_mode = 0;
        } 
        return; 
    } 

    // When TOGGLE_SWITCH is in DOWN position
    if (board->PUSH1_val == PRESSED) 
    { 
        bcm2835_delay(100); // 100ms delay for buttons debouncing 
        if (buff->pitch > 0.5) 
            buff->pitch -= 0.25; 
    } 
    if (board->PUSH2_val == PRESSED)  
    {   
        bcm2835_delay(100); // 100ms delay for buttons debouncing 
        if (buff->pitch < 2.0) 
            buff->pitch += 0.25; 
    } 
}


void updateOsc(Oscillator* osc)
{   
    osc->phase += osc->phase_stride;
    if (osc->phase >= 1.0f) // 1 circle rotation: 0 -> 1 
        osc->phase = 0.0f;
}


void _recordSignal(
    uint32_t* input_signal,
    uint32_t* output_signal,
    BoardStatus* board,
    BufferStatus* buff)
{   
    uint32_t* Loop_Buffer = buff->Loop_Buffer;
    // Start recording
    board->LED_blinking = TRUE;
    board->LED_timer--;     
    Loop_Buffer[(uint32_t)buff->sample_index] = *input_signal;
    buff->sample_index++;
    buff->record_length++;
    *output_signal = *input_signal;
    if (buff->sample_index >= LOOP_MAX - 1) 
    {
        buff->is_buffer_maxed_out = TRUE;
        buff->sample_index = 0;
    }
    if (buff->record_length >= LOOP_MAX - 1)
        buff->record_length = LOOP_MAX - 1;
    
    // Led in blinking mode while recording
    if (board->LED_blinking && board->LED_timer < 0)
    {   
        board->is_LED_on = !board->is_LED_on;
        bcm2835_gpio_write(LED, board->is_LED_on);
        board->LED_timer = sample_rate / 5; // 0.2 seconds
    }
}


void _normalPlayback(
    uint32_t* input_signal,
    uint32_t* output_signal,
    BufferStatus* buff)
{
    uint32_t* Loop_Buffer = buff->Loop_Buffer;
    // NORMAL PLAYBACK
    *output_signal = (Loop_Buffer[(uint32_t)buff->sample_index] + *input_signal) >> 1;
    // Pitch shifting
    buff->sample_index += buff->pitch;
    if (buff->sample_index == buff->ending_sample)
        buff->sample_index = buff->starting_sample;
    else 
        if (buff->sample_index >= buff->record_length)
            buff->sample_index = 0;
}


void _granularPlayback(
    uint32_t* input_signal,
    uint32_t* output_signal,
    BufferStatus* buff,
    Grain* grain,
    Delay* delay,
    Oscillator* lfo_1,
    Oscillator* lfo_2)
{   
    uint32_t* Loop_Buffer = buff->Loop_Buffer;
    // GRANULAR MICRO-DELAY
    // 1) Generating audio grains
    updateOsc(lfo_1);
    updateOsc(lfo_2);
    grain->index++;
    if (grain->index >= grain->size) 
    {   
        // LFO 1 modulating grain size
        grain->min_size = (rand() % 1000) + 500; // rand 500 (0.02 sec) -> 1500
        grain->size = grain->min_size + (uint32_t)((1.0f + sinf(2.0f * PI * lfo_1->phase)) * grain->factor);
        grain->index = 0;
        grain->is_reversed = rand() % 2;
        // LFO 2 (slower) modulating grain position
        grain->position = (uint32_t)((1.0f + sinf(2.0f * PI * lfo_2->phase)) * ((buff->record_length - 1) / 2));
        buff->sample_index = grain->position;
    }

    // 2) Randomly reversing grains
    if (grain->is_reversed)
    {
        *output_signal = (Loop_Buffer[(uint32_t)buff->sample_index] + *input_signal);
        // Pitch shifting
        buff->sample_index -= buff->pitch;
        if (buff->sample_index < 0)
            buff->sample_index = buff->record_length - 1;
    }
    else
    {
        *output_signal = (Loop_Buffer[(uint32_t)buff->sample_index] + *input_signal);
        // Pitch shifting
        buff->sample_index += buff->pitch;
        if (buff->sample_index > buff->record_length - 1)
            buff->sample_index = 0;
    }
    
    // 3) Delay effect
    delay->Echo_Buffer[delay->index] = (*output_signal + delay->Echo_Buffer[delay->index]) >> 2;
    delay->index++;
    if(delay->index >= DELAY_MAX)
        delay->index = 0; 
    *output_signal = (*output_signal + (delay->Echo_Buffer[delay->index])) >> 1;
}


void _reversedPlayback(
    uint32_t* input_signal,
    uint32_t* output_signal,
    BufferStatus* buff)
{
    uint32_t* Loop_Buffer = buff->Loop_Buffer;
    // REVERSE ALL SIGNAL
    *output_signal = (Loop_Buffer[(uint32_t)buff->sample_index] + *input_signal) >> 1;
    // Pitch shifting
    buff->sample_index -= buff->pitch;
    if (buff->sample_index == buff->starting_sample)
        buff->sample_index = buff->ending_sample;
    else
        if (buff->sample_index <= 0)
            buff->sample_index = buff->record_length - 1;
}


void processSignal(
    uint32_t* input_signal,
    uint32_t* output_signal,
    BoardStatus* board,
    BufferStatus* buff,
    Grain* grain,
    Delay* delay,
    Oscillator* lfo_1,
    Oscillator* lfo_2)
{
    // LOOP ACTIVATE - DEACTIVATE //
    if (buff->recording == TRUE)
    {
        _recordSignal(input_signal, output_signal, board, buff);
    }
    // PLAYBACK MODES //
    else
    {
        board->LED_blinking == FALSE;

        if (buff->playback_mode == 0)
            _normalPlayback(input_signal, output_signal, buff);

        else if (buff->playback_mode == 1)
            _granularPlayback(input_signal, output_signal, buff, grain, delay, lfo_1, lfo_2);

        else
            _reversedPlayback(input_signal, output_signal, buff);
    }
}


int main(int argc, char **argv)
{   
    uint32_t input_signal = 0;
    uint32_t output_signal = 0;
    sample_rate = 1000000 / microseconds_delay; // 50 kHz
    sample_duration = (1.0f / sample_rate);

    BoardStatus board; // Hardware status
    BufferStatus buff; // Buffer structure
    Grain grain; // Granular parameters
    Delay delay; // Delay parameters
    Oscillator lfo_1; // Sine wave LFO
    Oscillator lfo_2; // Sine wave LFO

    init(
        &board,
        &buff,
        &grain,
        &delay,
        &lfo_1,
        &lfo_2);

    // Main Loop
    while(TRUE)
	{   
        setControls(&board, &buff);
        
        // Read the FOOT_SWITCH value
        if (board.FOOT_SWITCH_val == PRESSED)
        {   
            readInputSignal(&input_signal);

            processSignal(
                &input_signal,
                &output_signal,
                &board,
                &buff,
                &grain,
                &delay,
                &lfo_1,
                &lfo_2);

            // Add a delay of 20 microseconds (run freq = 50kHz)
            bcm2835_delayMicroseconds(microseconds_delay);

            // Generate output PWM signal (2 x 6 bits) depending on frequency rate
            bcm2835_pwm_set_data(1, output_signal & 0x3F);
            bcm2835_pwm_set_data(0, output_signal >> 6);
        }
        else
            // when FOOT_SWITCH deactivated
            board.FOOT_SWITCH_val = bcm2835_gpio_lev(FOOT_SWITCH);
    }
    // Close all and exit
    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}