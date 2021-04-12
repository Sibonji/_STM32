#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"

#define A    LL_GPIO_PIN_0
#define B    LL_GPIO_PIN_1
#define C    LL_GPIO_PIN_2
#define D    LL_GPIO_PIN_3
#define E    LL_GPIO_PIN_4
#define F    LL_GPIO_PIN_5
#define G    LL_GPIO_PIN_6
#define PD   LL_GPIO_PIN_7
#define POS0 LL_GPIO_PIN_8
#define POS1 LL_GPIO_PIN_9
#define POS2 LL_GPIO_PIN_10
#define POS3 LL_GPIO_PIN_11

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

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_DOWN);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);
    
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
    
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    return;
}

static void exti_config(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE1);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);

    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_1);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_1);

    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
}

int button_status, debouncer_clk = 0;
int GPIOA_0_Button_Status()
{ 
    if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) 
    {
        button_status = 1;
        debouncer_clk = 0;
    }
    
    if (button_status)
    {
        debouncer_clk++;
    }
    
    if (debouncer_clk >= 5)
    {
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
        button_status = 0;
        debouncer_clk = 0;
    }    

    return button_status;
}

void dyn_display(uint16_t number, int digit_num)
{

    static uint32_t mask = A | B | C | D | E | F | G | PD | POS0 | POS1 | POS2 | POS3;


    uint32_t decoder[16] = {
    A | B | C | D | E | F,     //0
    B | C,                     //1
    A | B | D | E | G,         //2
    A | B | C | D | G,         //3
    B | C | F | G,             //4
    A | C | D | F | G,         //5 
    A | C | D | E | F | G,     //6
    A | B | C,                 //7
    A | B | C | D | E | F | G, //8
    A | B | C | D | F | G,     //9
    A | B | C | E | F | G,     //A
    A | B | C | D | E | F | G, //B
    A | D | E | F,             //C
    A | B | C | D | E | F,     //D
    A | D | E | F | G,         //E
    A | E | F | G,             //F
   
    };

    uint32_t out = 0;

    uint32_t position[4] = {  
           POS1 | POS2 | POS3,//first indicator
    POS0 |        POS2 | POS3,//second indicator
    POS0 | POS1 |        POS3,//third indicator
    POS0 | POS1 | POS2,       //fourth indicator 
    };
    
    uint32_t num[4] = {      
    decoder[number % 10],
    decoder[(number % 100)   / 10],
    decoder[(number % 1000)  / 100],
    decoder[(number % 10000) / 1000],
    };
    
    out = num[digit_num % 4] | position[digit_num % 4];

    uint32_t port_state = 0;

    port_state = LL_GPIO_ReadOutputPort(GPIOB);
    port_state = (port_state & ~mask) | out;
    LL_GPIO_WriteOutputPort(GPIOB, port_state);

    digit_num = (digit_num + 1) % 4;

    return;
}

static void systick_config(void)
{
    LL_InitTick(48000000, 1000);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 0);
    return;
}

static int counter_top = 1000;
int digit_num = 0;
int numdisp, newnumdisp = 0;

void SysTick_Handler(void)
{
    static int counter = 0;
    counter = (counter + 1) % counter_top;
    
    dyn_display(newnumdisp, digit_num);
    digit_num ++;
           
    a0_button_status = GPIOA_0_Button_Status(); 
        
    if(a0_button_status > 0)
        newnumdisp = numdisp + 1;
       
    if(a0_button_status == 0)
        numdisp = newnumdisp;       
    
    if (!counter)
      LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
}

int main(void)
{
    rcc_config();
    gpio_config();
    exti_config();
    systick_config();

    while (1);
    return 0;
}
