#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
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

/*
 * Configure timer to encoder mode
 */
static void timers_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_1, LL_GPIO_AF_2);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_2);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    /* (1) Configure TI1FP1 on TI1 (CC1S = 01)
         configure TI1FP2 on TI2 (CC2S = 01) */
    /* (2) Configure TI1FP1 and TI2FP2 non inverted (CC1P = CC2P = 0, reset value) */
    /* (3) Configure both inputs are active on both rising and falling edges
        (SMS = 011) */ 
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI); // 1
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI); // 1
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ETR_POLARITY_NONINVERTED);  // 2
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ETR_POLARITY_NONINVERTED);  // 2
    LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12); // 3
    //LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV16_N5);
    //LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV16_N5);
    LL_TIM_SetAutoReload(TIM2, 0xFFFF);
    LL_TIM_EnableCounter(TIM2);
    //
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

void print_grid ()
{
    for (uint8_t i = 3; i < 59; i++)
        for (uint8_t j = 35; j < 91; j++)
            {
                if ((((i - 3) % 18) == 0) || (((i - 4) % 18) == 0 ))
                    oled_set_pix (j, i, clWhite);
                if ((((j - 35) % 18) == 0) || (((j - 36) % 18) == 0 ))
                    oled_set_pix (j, i, clWhite);
            }
}

void make_zero ()
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            arr[i][j] = '\0';
}

void find_empty ()
{
    for (y = 0; y < 3; y++)
        for (x = 0; x < 3; x++)
            if (arr[y][x] == '\0')
		return;
}

void draw ()
{
    oled_clr (clBlack);
    print_grid ();

    for (y = 0; y < 3; y++)
        for (x = 0; x < 3; x++)
        {
            if (arr[y][x] == 'x')
                print_krest ();
            else if (arr[y][x] == 'o')
                print_krug ();
        }

    oled_update ();
}

void move_cursor_x (uint8_t dir)
{
    while (1)
    {
	x = (x + dir) % 3;

        if (arr[y][x] == '\0')
	{
            print_krug ();
	    print_krest ();
	    return;
	}
    }
}

void print_win (char winner)
{
    oled_clr (clBlack);
    oled_set_cursor (0, 0);
    xprintf ("Player %c had won the game\n", winner);
}

int check_win ()
{
    uint8_t j = 0;
    uint8_t i = 0;

    for (i = 0; i < 3; i++)
    {
        if (arr[i][j] == arr[i][j + 1] && arr[i][j] == arr[i][j + 2])
	{
            print_win (arr[i][j]);
	    return 1;
	}
    }

    i = 0;
    for (j = 0; j < 3; j++)
    {
        if (arr[i][j] == arr[i + 1][j] && arr[i][j] == arr[i + 2][j])
	{
            print_win (arr[i][j]);
	    return 1;
	}
    }

    if (arr[0][0] == arr[1][1] && arr[0][0] == arr[2][2])
    {
        print_win (arr[0][0]);
	return 1;
    }

    if (arr[0][2] == arr[1][1] && arr[1][1] == arr[2][0])
    {
        print_win (arr[1][1]);
	return 1;
    }

    return 0;
}

int main(void)
{
    rcc_config();
    gpio_config();
    oled_config();
    timers_config();
    printf_config();

    make_zero ();

    arr[0][0] = 'x';

    int check = 0;

    while (1) 
    {
        check = check_win ();

	if (check == 0)    
	    draw ();

	if (check == 0)
            find_empty ();

        if (LL_TIM_GetCounterMode(TIM2) == LL_TIM_COUNTERMODE_UP) {
            LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);
            LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
	    move_cursor_x (1);
	    check = 1;
        }
	else if (LL_TIM_GetCounterMode(TIM2) == LL_TIM_COUNTERMODE_DOWN) {
            LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
            LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
	    move_cursor_x (-1);
	    check = 1;
        }
    }

    return 0;
}
