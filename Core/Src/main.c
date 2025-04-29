#include "stm32f0xx_hal.h"

// Define TM1628 Pins
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

int main(void) {
    HAL_Init();
    __HAL_RCC_GPIOB_CLK_ENABLE();

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

    // All segments ON: A | B | C | D | E | F | G = 0x7F (or 0xFF with dot)
      // without dot
    // TM1628_displaySecondDigit(0xFF);  // with dot

    while (1) {
        // Idle loop
    		for (int i = 0; i < 5; i++){
    			if (i == 4){
    				HAL_Delay(120);
    			}
    			else{
    				HAL_Delay(200);
    			}

    		    		TM1628_displaySecondDigit(segments_forward[i], 0xc2);
    		    	}
    		for (int i = 0; i < 5; i++){
    			if (i == 4){
    			    				HAL_Delay(120);
    			    			}
    			    			else{
    			    				HAL_Delay(200);
    			    			}
    		    		TM1628_displaySecondDigit(segments_defa_forward[i], 0xc0);
    		    	    	}

    }
}
