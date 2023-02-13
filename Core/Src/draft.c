/*
 * draft.c
 *
 *  Created on: Jan 26, 2023
 *      Author: inflamer
 */


  /* USER CODE BEGIN 1 */
	/*

 for(uint8_t is=0;is<=sizeof(init_seq);is++){
	 tbuffer[is] = init_seq[is];
 }

 for (uint32_t i=0;i<=200;i++){
	 tbuffer[i] = 0;
 }
	counter = 0;

	*/
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */
  //LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM14_STOP);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableIT_UPDATE(TIM14);
  LL_TIM_EnableCounter(TIM14);
  LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);

  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);

  LL_TIM_EnableIT_UPDATE(TIM3);
  LL_TIM_EnableIT_CC1(TIM3);
  LL_TIM_EnableIT_CC2(TIM3);
  //LL_TIM_EnableDMAReq_TRIG(TIM3);

  LL_TIM_EnableCounter(TIM3);

  LL_I2C_Enable(I2C2);

  //LL_USART_EnableDMAReq_TX

  LL_I2C_EnableDMAReq_TX(I2C2);

  //LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, buffer, I2C2_BASE, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  //LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&buffer);
  //LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_USART_DMA_GetRegAddr(I2C2,LL_I2C_DMA_REG_DATA_TRANSMIT));

  //LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 5);


  //LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  //LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);


  //LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);


  //LL_I2C_HandleTransfer(I2C2, 0x78, LL_I2C_ADDRSLAVE_7BIT , 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
  //LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);

  LL_I2C_SetSlaveAddr(I2C2, 0x78);
  //LL_I2C_EnableAutoEndMode(I2C2);
  LL_I2C_EnableReloadMode(I2C2);

  LL_I2C_GenerateStartCondition(I2C2);
  //LL_I2C_DMA_GetRegAddr(I2C2, Direction);
  LL_I2C_ClearFlag_STOP(I2C2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //LL_PWR_EnterLowPowerRunMode();


  /* Set DMA transfer addresses of source and destination */
    LL_DMA_ConfigAddresses(DMA1,
                           LL_DMA_CHANNEL_2,
                           (uint32_t)&aSRC_Const_Buffer,
                           (uint32_t)&aDST_Buffer,
                           LL_DMA_DIRECTION_MEMORY_TO_MEMORY);

    /* Set DMA transfer size */
    LL_DMA_SetDataLength(DMA1,
                         LL_DMA_CHANNEL_2,
                         10);

    /* Enable DMA transfer complete/error interrupts */
    //LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    //LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

    /* Start the DMA transfer Flash to Memory */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_I2C2_TX);

    LL_DMA_ConfigAddresses(DMA1,
                              LL_DMA_CHANNEL_1,
                              (uint32_t)&aSRC_Const_Buffer,
                              (uint32_t)&I2C2->TXDR,
                              LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

       /* Set DMA transfer size */
       LL_DMA_SetDataLength(DMA1,
                            LL_DMA_CHANNEL_1,
                            10);
       LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);


       LL_I2C_SetTransferSize(I2C2, 2);

       LL_I2C_EnableAutoEndMode(I2C2);
       LL_I2C_DisableReloadMode(I2C2);

       LL_I2C_GenerateStartCondition(I2C2);

       LL_I2C_GenerateStartCondition(I2C2);

       LL_I2C_GenerateStartCondition(I2C2);

       LL_I2C_GenerateStartCondition(I2C2);

       LL_I2C_GenerateStartCondition(I2C2);




       /*
        set addr && start
		(SADD 0x78, RELOAD=1,NBYTES=1)
		check TXIS=1
		TXDR = I2C_MEM_ADD_LSB(MemAddress);
		check TCR=1
		(SADD 0x78, AUTOEND=1,NBYTES=1)
		check TXIS=1
		TXDR = *hi2c->pBuffPtr;
		wait until stop flag is reset
		Clear STOPF


		update screen: i=0..8
        	ssd1306_WriteCommand(hi2c, 0xB0 + i);
        	ssd1306_WriteCommand(hi2c, 0x00);
        	ssd1306_WriteCommand(hi2c, 0x10);
 	 	 i2c memwrite (0x80 bytes)
        */

  while (1)
  {
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
