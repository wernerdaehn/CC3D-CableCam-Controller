void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
   uint32_t isrflags   = READ_REG(huart->Instance->SR);
   uint32_t cr1its     = READ_REG(huart->Instance->CR1);
   uint32_t cr3its     = READ_REG(huart->Instance->CR3);
   uint32_t errorflags = 0x00U;
   uint32_t dmarequest = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  if(errorflags == RESET)
  {
    if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) // DEal with idle flag
    {
      __HAL_UART_CLEAR_IDLEFLAG(huart);
      HAL_UART_RxCpltCallback(huart);
    }
    /* UART in mode Receiver -------------------------------------------------*/
    if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
      UART_Receive_IT(huart);
      return;
    }
  }









HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;

  /* Check the parameters */
  assert_param(IS_DMA_BUFFER_SIZE(DataLength));

  /* Process locked */
  __HAL_LOCK(hdma);

  if(HAL_DMA_STATE_READY == hdma->State)
  {
/* Change DMA peripheral state */
hdma->State = HAL_DMA_STATE_BUSY;

/* Initialize the error code */
hdma->ErrorCode = HAL_DMA_ERROR_NONE;

/* Configure the source, destination address and the data length */
DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

/* Clear all interrupt flags at correct offset within the register */
regs->IFCR = 0x3FU << hdma->StreamIndex;

/* Enable Common interrupts*/
hdma->Instance->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
**// hdma->Instance->FCR |= DMA_IT_FE; // Disable FIFO error**



