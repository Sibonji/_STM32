#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"

#include "xprintf.h"
#include "oled_driver.h"

#define x_max 3
#define y_max 3

static void rcc_config()
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                LL_RCC_PLL_MUL_12);

    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    SystemCoreClock = 48000000;
}

static void gpio_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    return;
}

static void printf_config(void)
{
    xdev_out(oled_putc);
    return;
}

char arr[y_max][x_max] = {'\0'};

uint8_t x = 0;
uint8_t y = 0;

uint8_t sum_x (uint8_t j)
{
    return ((x + 1) * 2 + x * 16 + 34 + j);
}

uint8_t sum_y (uint8_t i)
{
    return ((y + 1) * 2 + y * 16 + 2 + i);
}


void print_krest ()
{
    for (uint8_t i = 2; i < 16; i++)
        for (uint8_t j = 2; j < 16; j++)
	    {
	        if ( i == j || i == 17 - j)
		    {
			oled_set_pix (sum_x (j), sum_y (i), clWhite);
		    }
	    }
}

void print_krug ()
{
    for (uint8_t i = 2; i < 16; i++)
    {
	if (i == 2 || 17 - i == 2)
	    for (uint8_t j = 7; j < 11; j++)
		oled_set_pix (sum_x (j), sum_y (i), clWhite);
	if (i == 3 || 17 - i == 3)
	{
	    for (uint8_t j = 5; j < 7; j++)
		oled_set_pix (sum_x (j), sum_y (i), clWhite);
	    for (uint8_t j = 11; j < 13; j++)
                oled_set_pix (sum_x (j), sum_y (i), clWhite);
        }
	if (i == 4 || 17 - i == 4)
	{
	    uint8_t j = i;
                oled_set_pix (sum_x (j), sum_y (i), clWhite);
	    j = 17 - i;
	        oled_set_pix (sum_x (j), sum_y (i), clWhite);
	}
    }

    for (uint8_t j = 2; j < 16; j++)
    {
        if (j == 2 || 17 - j == 2)
            for (uint8_t i = 7; i < 11; i++)
                oled_set_pix (sum_x (j), sum_y (i), clWhite);
        if (j == 3 || 17 - j == 3)
        {
            for (uint8_t i = 5; i < 7; i++)
                oled_set_pix (sum_x (j), sum_y (i), clWhite);
            for (uint8_t i = 11; i < 13; i++)
                oled_set_pix (sum_x (j), sum_y (i), clWhite);
        }
    }

}

int main(void)
{
   for (int i = 0; i < y_max; i++)
       for (int j = 0; j < x_max; j++)
           arr[i][j] = '\0';

    rcc_config();
    gpio_config();
    oled_config();
    printf_config();

    
    for (uint8_t i = 3; i < 59; i++)
        for (uint8_t j = 35; j < 91; j++)
            {
                if ((((i - 3) % 18) == 0) || (((i - 4) % 18) == 0 ))
		    oled_set_pix (j, i, clWhite);
		if ((((j - 35) % 18) == 0) || (((j - 36) % 18) == 0 ))
		    oled_set_pix (j, i, clWhite);
            }

    print_krug ();
    print_krest ();
    oled_update();

    while (1);
    return 0;
}
