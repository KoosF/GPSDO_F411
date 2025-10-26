
#include "stm32f4xx_hal.h"
#include "clock_custom.h"

// Frozen: HSE 10 MHz → PLL 96 MHz (M=5, N=192, P=4, Q=4)
// PA8 = 10 MHz (MCO1 = HSE)
// PA9 = 8 MHz (TIM1_CH2, UP-mode)
// PA1 = 1 kHz (TIM2_CH2)
// PA0 = 50 kHz PWM (TIM5_CH1)
// 500 ms warm-up

void clock_custom(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    GPIO_InitTypeDef gi = {0};

    // Warm-up / HAL init
    HAL_Init();
    HAL_Delay(500);

    // HSE + PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 5;     // 10/5 = 2 MHz
    RCC_OscInitStruct.PLL.PLLN       = 192;   // 2 * 192 = 384 MHz
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4; // 384 / 4 = 96 MHz
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;     // 96 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;       // 48 MHz (TIM2/5 → x2 = 96 MHz)
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;       // 96 MHz (TIM1 = 96 MHz)
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

    // PA8: MCO1 = HSE 10 MHz
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gi.Pin = GPIO_PIN_8;
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &gi);
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);

    // TIM5 CH1 → PA0 = 50 kHz PWM
    __HAL_RCC_TIM5_CLK_ENABLE();
    gi.Pin = GPIO_PIN_0;
    gi.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gi);

    TIM_HandleTypeDef htim5{};
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = (96000000 / 50000) - 1; // 1919
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim5);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = htim5.Init.Period / 2; // 50%
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim5, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

    // TIM2 CH2 → PA1 = 1 kHz blok
    __HAL_RCC_TIM2_CLK_ENABLE();
    gi.Pin = GPIO_PIN_1;
    gi.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gi);

    TIM_HandleTypeDef htim2{};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (96000000 / 1000) - 1; // 95999
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = htim2.Init.Period / 2;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // TIM1 CH2 → PA9 = 8 MHz blok (UP mode)
    __HAL_RCC_TIM1_CLK_ENABLE();
    gi.Pin = GPIO_PIN_9;
    gi.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gi);

    TIM_HandleTypeDef htim1{};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = (96000000 / 8000000) - 1; // 11 → 8 MHz
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim1);

    // Advanced timer: MOE aanzetten
    TIM_BreakDeadTimeConfigTypeDef bdtr = {0};
    bdtr.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdtr);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = htim1.Init.Period / 2;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);

    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    // LED: 3 korte flitsen
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gi.Pin = GPIO_PIN_13;
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gi);
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(80);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(80);
    }
}
