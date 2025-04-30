#include "stm32f0xx.h"
#include <stdint.h>

// Commutation step structure
typedef struct {
    uint8_t ah, bh, ch;
    uint8_t al, bl, cl;
} CommutationStep;

// 6-step commutation table (index = hall state)
const CommutationStep comm_table[8] = {
    {0,0,0,0,0,0}, // 000 - invalid
    {1,0,0,0,1,0}, // 001 - AH+, BL-
    {0,0,1,1,0,0}, // 010 - CH+, AL-
    {1,0,0,0,0,1}, // 011 - AH+, CL-
    {0,1,0,0,0,1}, // 100 - BH+, CL-
    {0,0,1,0,1,0}, // 101 - CH+, BL-
    {0,1,0,1,0,0}, // 110 - BH+, AL-
    {0,0,0,0,0,0}, // 111 - invalid
};

void adc_init(void);
uint16_t read_adc_pa2(void);
void soft_delay(volatile uint32_t count);

#define TM1628_CLK_GPIO_Port GPIOB
#define TM1628_CLK_Pin       GPIO_PIN_3

#define TM1628_DIO_GPIO_Port GPIOB
#define TM1628_DIO_Pin       GPIO_PIN_8

#define TM1628_STB_GPIO_Port GPIOB
#define TM1628_STB_Pin       GPIO_PIN_9

// Control macros
#define TM1628_CLK(x) HAL_GPIO_WritePin(TM1628_CLK_GPIO_Port, TM1628_CLK_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define TM1628_DIO(x) HAL_GPIO_WritePin(TM1628_DIO_GPIO_Port, TM1628_DIO_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define TM1628_STB(x) HAL_GPIO_WritePin(TM1628_STB_GPIO_Port, TM1628_STB_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

void TM1628_sendByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        TM1628_CLK(0);
        TM1628_DIO(data & 0x01);
        data >>= 1;
        TM1628_CLK(1);
    }
}

void TM1628_init(void) {
    // Set display control: Display ON, brightness medium
    TM1628_STB(0);
    TM1628_sendByte(0x8F);  // 0x88 to 0x8F → brightness levels
    TM1628_STB(1);
}

void TM1628_clearDisplay(void) {
    TM1628_STB(0);
    TM1628_sendByte(0x40); // Auto-increment mode
    TM1628_STB(1);

    TM1628_STB(0);
    TM1628_sendByte(0xC0); // Start address
    for (uint8_t i = 0; i < 16; i++) {
        TM1628_sendByte(0x00);
    }
    TM1628_STB(1);
}

void TM1628_displaySecondDigit(uint8_t data, uint8_t address) {
    TM1628_STB(0);
    TM1628_sendByte(0x40);         // Fixed address mode
    TM1628_STB(1);

    TM1628_STB(0);
    TM1628_sendByte(address);// Address for second digit (usually 0xC2)
    TM1628_sendByte(data);// Segment data
    TM1628_STB(1);
}

// Set phase pin (PA8–10 or PB13–15)
void set_phase_output(uint8_t pin, uint8_t state) {
    if (pin <= 10) {
        if (state) GPIOA->BSRR = (1 << pin);
        else       GPIOA->BRR  = (1 << pin);
    } else {
        if (state) GPIOB->BSRR = (1 << pin);
        else       GPIOB->BRR  = (1 << pin);
    }
}

// Apply commutation step based on Hall state
void commutate(uint8_t hall_state) {
    const CommutationStep* step = &comm_table[hall_state & 0x07];

    set_phase_output(13, step->ah);  // PB13 = AH
    set_phase_output(14, step->bh);  // PB14 = BH
    set_phase_output(15, step->ch);  // PB15 = CH

    set_phase_output(8, step->al);   // PA8  = AL
    set_phase_output(9, step->bl);   // PA9  = BL
    set_phase_output(10, step->cl);  // PA10 = CL
}

// Read 3-bit Hall sensor state from PA0, PA1, PA2
uint8_t read_hall_state(void) {
    uint8_t h1 = (GPIOA->IDR & GPIO_IDR_6) ? 1 : 0;  // PA6 = H1
    uint8_t h2 = (GPIOA->IDR & GPIO_IDR_7) ? 1 : 0;  // PA7 = H2
    uint8_t h3 = (GPIOB->IDR & GPIO_IDR_0) ? 1 : 0;  // PB0 = H3

    return (h1 << 2) | (h2 << 1) | h3;  // Combined 3-bit Hall state: H1H2H3
}


// GPIO init for PA8-10 and PB13-15 as output, PA0–2 as input
void gpio_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // PA5 as output
    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |=  (1 << (5 * 2));  // Output mode


    // PA8–PA10 as output
    for (int i = 8; i <= 10; i++) {
        GPIOA->MODER &= ~(3 << (i * 2));
        GPIOA->MODER |=  (1 << (i * 2)); // Output mode
    }

    // PB13–PB15 as output
    for (int i = 13; i <= 15; i++) {
        GPIOB->MODER &= ~(3 << (i * 2));
        GPIOB->MODER |=  (1 << (i * 2));
    }

    // PA0–PA2 as input for Hall sensors
    GPIOA->MODER &= ~((3 << 0) | (3 << 2) | (3 << 4));
}

// TIM1 init for periodic interrupts
void tim1_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC = 48000 - 1;     // 48MHz / 48000 = 1kHz base
    TIM1->ARR = 100 - 1;       // 1kHz / 100 = 10Hz commutation rate
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}

// TIM1 ISR — called every 100ms (10Hz)
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    if (TIM1->SR & TIM_SR_UIF) {
        TIM1->SR &= ~TIM_SR_UIF;

        uint8_t hall = read_hall_state();
        commutate(hall);
    }
}

int main(void) {
    gpio_init();
    tim1_init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct_PA2 = {0};
            GPIO_InitStruct_PA2.Pin = GPIO_PIN_2;
            GPIO_InitStruct_PA2.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct_PA2.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_PA2);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = TM1628_CLK_Pin | TM1628_DIO_Pin | TM1628_STB_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        TM1628_init();
        TM1628_clearDisplay();

        const uint8_t segments_forward[] = {
            0x40,  // A
            0x10,  // B
            0x04,  // C
            0x01,
    		0x00// D
        };
        const uint8_t segments_defa_forward[] = {
            0x01,  // D
            0x80,  // E
            0x20,  // F
            0x40,
    		0x00// A
        };


    GPIOA->BSRR = (1 << 5);
    adc_init();


    while (1) {
        __WFI(); // Wait for interrupt (low power)
        TM1628_clearDisplay();
        uint16_t adc_val = read_adc_pa2();
            if (adc_val <= 6)
            {
                // Show E6 (no clutch)
                TM1628_displaySecondDigit(0xE9, 0xC0); // E
                TM1628_displaySecondDigit(0xED, 0xC2); // 6
            }
            else
            {
                 //Show animation (clutch present)
            	for (int i = 0; i < 5; i++){
            	    			if (i == 4){
            	    				soft_delay(5000);  // Tune values as needed

            	    			}
            	    			else{
            	    				soft_delay(150000);  // Tune values as needed

            	    			}

            	    		    		TM1628_displaySecondDigit(segments_forward[i], 0xc2);
            	    		    	}
            	    		for (int i = 0; i < 5; i++){
            	    			if (i == 4){
            	    				soft_delay(5000);  // Tune values as needed

            	    			    			}
            	    			    			else{

            	    			    				soft_delay( 150000);  // Tune values as needed
            	    			    			}
            	    			TM1628_displaySecondDigit(segments_defa_forward[i], 0xc0);
            	    		    	    	}
            }

    }
}

void adc_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CHSELR = ADC_CHSELR_CHSEL2; // Select channel 2 (PA2)
    ADC1->CFGR1 &= ~ADC_CFGR1_CONT;   // Single conversion mode
    ADC1->CR |= ADC_CR_ADEN;          // Enable ADC
    // Wait until ADC is ready
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

uint16_t read_adc_pa2(void) {
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC));  // Wait for end of conversion
    return ADC1->DR;
}

void soft_delay(volatile uint32_t count) {
    while (count--);
}
