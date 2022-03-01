#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>

// Define Input Pins
#define PUSH1 	        RPI_GPIO_P1_08      //GPIO14
#define PUSH2           RPI_V2_GPIO_P1_38  	//GPIO20 
#define TOGGLE_SWITCH 	RPI_V2_GPIO_P1_32 	//GPIO12
#define FOOT_SWITCH     RPI_GPIO_P1_10 		//GPIO15
#define LED             RPI_V2_GPIO_P1_36 	//GPIO16

#define TRUE 1
#define FALSE 0
#define PRESSED 0


// Define Delay Effect parameters: 50000 is 1 second approx.
#define DELAY_MAX 250000 //5 seconds
#define DELAY_MIN 0

long current_sample = 0;
uint32_t Delay_Buffer[DELAY_MAX];
uint32_t sample_is_randomized = FALSE; // Randomize flag
uint32_t grain_counter = 0;
uint32_t max_grain_size = 25000; // 0.5 seconds
uint32_t min_grain_size = 500;
uint32_t grain_size = 0;
uint32_t random_start = 0;

uint32_t input_signal = 0;
uint32_t speed = 20;
uint32_t up = 1;
uint32_t code_delay = 0;
uint32_t output_signal = 0;
uint32_t read_timer, delay;
uint32_t recording = FALSE;
uint32_t playback_mode = 0;
uint32_t record_lenght = 100;
 
uint8_t FOOT_SWITCH_val;
uint8_t TOGGLE_SWITCH_val;
uint8_t PUSH1_val;
uint8_t is_PUSH1_pressed;
uint8_t PUSH2_val;
uint8_t is_PUSH2_pressed;
uint8_t is_reversed = FALSE; // randomly choose TRUE or FALSE
 
int main(int argc, char **argv)
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
    bcm2835_gpio_fsel(18,BCM2835_GPIO_FSEL_ALT5 ); // PWM0 signal on GPIO18    
    bcm2835_gpio_fsel(13,BCM2835_GPIO_FSEL_ALT0 ); // PWM1 signal on GPIO13    
	bcm2835_pwm_set_clock(2); // Max clk frequency (19.2MHz/2 = 9.6MHz)
    bcm2835_pwm_set_mode(0,1 , 1); // channel 0, markspace mode, PWM enabled. 
	bcm2835_pwm_set_range(0,64);   // channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
    bcm2835_pwm_set_mode(1, 1, 1); // channel 1, markspace mode, PWM enabled.
	bcm2835_pwm_set_range(1,64);   // channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
 
	// Define SPI bus configuration
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); 	  // 4MHz clock with _64 
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
 
   	uint8_t mosi[10] = { 0x01, 0x00, 0x00 }; // 12 bit ADC read 0x08 ch0, - 0c for ch1 
	uint8_t miso[10] = { 0 };
 
    // Define GPIO pins configuration
    bcm2835_gpio_fsel(PUSH1, BCM2835_GPIO_FSEL_INPT); 			// PUSH1 button as input
	bcm2835_gpio_fsel(PUSH2, BCM2835_GPIO_FSEL_INPT); 			// PUSH2 button as input
	bcm2835_gpio_fsel(TOGGLE_SWITCH, BCM2835_GPIO_FSEL_INPT);	// TOGGLE_SWITCH as input
	bcm2835_gpio_fsel(FOOT_SWITCH, BCM2835_GPIO_FSEL_INPT); 	// FOOT_SWITCH as input
	bcm2835_gpio_fsel(LED, BCM2835_GPIO_FSEL_OUTP);				// LED as output
 
    bcm2835_gpio_set_pud(PUSH1, BCM2835_GPIO_PUD_UP);           // PUSH1 pull-up enabled   
	bcm2835_gpio_set_pud(PUSH2, BCM2835_GPIO_PUD_UP);           // PUSH2 pull-up enabled 
	bcm2835_gpio_set_pud(TOGGLE_SWITCH, BCM2835_GPIO_PUD_UP);   // TOGGLE_SWITCH pull-up enabled 
	bcm2835_gpio_set_pud(FOOT_SWITCH, BCM2835_GPIO_PUD_UP);     // FOOT_SWITCH pull-up enabled 
 
    // Main Loop
    while(1)
	{
        // Read 12 bits ADC
        bcm2835_spi_transfernb(mosi, miso, 3);
        input_signal = miso[2] + ((miso[1] & 0x0F) << 8); 
    
        // Read the PUSH buttons every 50000 times (0.25s) to save resources.
        read_timer++;
        if (read_timer==50000)
        {
            read_timer=0;
            uint8_t PUSH1_val = bcm2835_gpio_lev(PUSH1);
            uint8_t PUSH2_val = bcm2835_gpio_lev(PUSH2);
            TOGGLE_SWITCH_val = bcm2835_gpio_lev(TOGGLE_SWITCH);
            uint8_t FOOT_SWITCH_val = bcm2835_gpio_lev(FOOT_SWITCH);
            bcm2835_gpio_write(LED,!FOOT_SWITCH_val); // Light the effect when the footswitch is activated.

            if (TOGGLE_SWITCH_val == 0)
            {
                if (PUSH1_val == PRESSED)
                {          
                    is_PUSH1_pressed = TRUE;
                }

                if (PUSH1_val == !PRESSED && is_PUSH1_pressed)
                { 	
                    is_PUSH1_pressed = FALSE;
                    if (recording == TRUE)
                    {
                        recording = FALSE;
                        printf("Stopped recording.\n");
                        record_lenght = current_sample;
                        current_sample = 0;
                    }
                    else 
                    {
                        recording = TRUE;
                        printf("Recording...\n");
                        current_sample = 0;
                    }
                }
                if (PUSH2_val == PRESSED)
                {   
                    bcm2835_delay(100); // 100ms delay for buttons debouncing.
                    playback_mode++;
                    if (playback_mode > 2) playback_mode = 0;

                    if (playback_mode==0) printf("Normal playback (%d)\n", playback_mode);
                    else if (playback_mode==1) printf("Random granular playback (%d)\n", playback_mode);
                    else printf("Reversed playback (%d)\n", playback_mode);
                }
            }
        


            else
            {
                if (PUSH2_val==0)
                {
                    bcm2835_delay(100); // 100ms delay for buttons debouncing. 
                    speed+=5;
                    printf("Pitching up\n");
                }
                if (PUSH1_val==0) 
                { 	
                    bcm2835_delay(100); // 100ms delay for buttons debouncing. 
                    speed-=5;
                    printf("Pitching down\n");
                }
            }
        }

        //**** TRIANGULAR SIGNAL GENERATOR (for testing purposes) ***///
        // The input_signal goes directly to the PWM output.
    
        code_delay++;
        if (1)
        {
        code_delay=0;
        if (up==1)
        input_signal=input_signal+speed;
        else 
        input_signal=input_signal-speed;
    
        if (input_signal>4096-speed) up=0;
        if (input_signal<0+ speed) up=1;
        }
        


        //**** MODE 1: SAMPLER ***///

        // LOOP ACTIVATE - DEACTIVATE

        if (recording == TRUE)
        {   
            // Start recording
            Delay_Buffer[current_sample] = input_signal;
            current_sample++;
            output_signal = input_signal;
            //bcm2835_gpio_write(LED, 1);
            if(current_sample >= DELAY_MAX) current_sample = 0; 
        }
        else 
        {   
            // PLAYBACK MODES (normal, granular, reversed)

            if (playback_mode == 0)
            {   
                // Normal playback
                output_signal = (Delay_Buffer[current_sample]+input_signal)>>1;
                current_sample++;
                if (current_sample > record_lenght) current_sample=0;
            }
            else if (playback_mode == 1)
            {   
                // Random granular delay
                grain_counter++;
                if (!sample_is_randomized)
                {   
                    random_start = (rand() % (250 - 0 + 1) + 0) * 1000;
                    grain_size = rand() % (max_grain_size - min_grain_size + 1) + min_grain_size;
                    is_reversed = rand() % 2;
                    current_sample = random_start;
                    sample_is_randomized = TRUE;
                }
                if (grain_counter >= grain_size) 
                {
                    grain_size = rand() % (max_grain_size - min_grain_size + 1) + min_grain_size;
                    grain_counter = 0;
                    is_reversed = rand() % 2;
                    random_start = (rand() % (250 - 0 + 1) + 0) * 1000;
                    current_sample = random_start;
                }

                if (is_reversed)
                {
                    output_signal = (Delay_Buffer[current_sample] + input_signal)>>1;
                    current_sample--;
                    if (current_sample < 0) current_sample = record_lenght-1;
                }
                else
                {
                    output_signal = (Delay_Buffer[current_sample] + input_signal)>>1;
                    current_sample++;
                    if (current_sample > record_lenght) current_sample = 0;
                }
                
            }
            else
            {
                // Reversed playback
                sample_is_randomized = FALSE; // Reset randomized flag
                if (current_sample < 0) current_sample = record_lenght-1;
                output_signal = (Delay_Buffer[current_sample]+input_signal)>>1;
                current_sample--;
            }
        }


        //**** MODE 2: EFFECTS ***///

        // ADJUST DELAY

        // TIME STRETCHING

        

        // Add a delay of 20 microseconds (sampling frequency = 50kHz)
        bcm2835_delayMicroseconds(20);
        //usleep(1);

        // Generate output PWM signal 6 bits
        bcm2835_pwm_set_data(1,output_signal & 0x3F);
        bcm2835_pwm_set_data(0,output_signal >> 6);
    }
 
	// Close all and exit
	bcm2835_spi_end();
    bcm2835_close();
    return 0;
}