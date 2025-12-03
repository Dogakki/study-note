# 通信协议

## CAN通信

### 原理       

 CAN总线上可以挂载多个通讯节点，节点之间的信号经过总线传输，实现节点间通讯。由于CAN通讯协议不对节点进行地址编码，而是对数据内容进行编码，所以网络中的节点个数理论上不受限制，只要总线的负载足够即可，可以通过中继器增强负载。CAN通讯节点由一个CAN控制器及CAN收发器组成，控制器与收发器之间通过CAN_Tx及CAN_Rx信号线相连，收发器与CAN总线之间使用。           CAN_High及CAN_Low信号线相连。其中CAN_Tx及CAN_Rx使用普通的类似TTL逻辑信号，而CAN_High及CAN_Low是一对差分信号线，使用比较特别的差分信号。 

​       当CAN节点需要发送数据时，控制器把要发送的二进制编码通过CAN_Tx线发送到收发器，然后由收发器把这个普通的逻辑电平信号转化成差分信号，通过差分线CAN_High和CAN_Low线输出到CAN总线网络。而通过收发器接收总线上的数据到控制器时，则是相反的过程，收发器把总线上收到的CAN_High及CAN_Low信号转化成普通的逻辑电平信号，通过CAN_Rx输出到控制器中。 

### 配置



## LIN通信

###  关键技术特点

- **低成本**：采用单主多从架构，无需 CAN 总线的专用芯片，从节点可使用通用 MCU 的 UART 接口，硬件成本低。
- **低速率**：通信速率固定为 20kbps（LIN 1.0/1.1）或最高 20kbps（LIN 2.x），仅支持短距离传输（通常车内几米范围内）。
- **单主多从**：一个 LIN 网络中只有 1 个主节点（如车身 ECU），最多可连接 16 个从节点（如车窗电机、灯光传感器），主节点负责发起通信、同步从节点。
- **简单帧结构**：每帧数据包含 “同步段 + 识别段 + 数据段（最多 8 字节）+ 校验段”，结构简单，易实现、易解析。

#### 如何使用

### **电路连接**：

- STM32 的 UART_TX、UART_RX 引脚通过 LIN 收发器（如 TJA1020）连接到 LIN 总线。
- 收发器需外接 12V 电源（总线电压）和终端电阻（通常 1kΩ），确保信号完整性。

### 软件实现核心步骤

#### 1. 初始化 UART（模拟 LIN 物理层）

LIN 通信基于 UART，需配置为：

- 波特率：固定 20kbps（LIN 协议规定）。
- 数据格式：1 个起始位，8 个数据位，1 个停止位，无校验（LIN 用字节校验替代硬件校验）。
- 重点：需支持 “Break 信号” 发送（LIN 同步段的核心，由至少 13 个连续低电平位组成）。

```c++
void LIN_UART_Init(void) {
  // 使能USART和GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  // 配置TX引脚（推挽输出）、RX引脚（浮空输入）
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2; // USART2_TX
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3; // USART2_RX
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  // 配置USART：20kbps，8位数据，1停止位，无校验
  USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.USART_BaudRate = 20000;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART2, &USART_InitStruct);
  
  USART_Cmd(USART2, ENABLE);
}
```

#### 2. 实现 LIN 帧发送（主节点）

LIN 帧结构（主节点发送）：

1. **Break 段**：至少 13 个低电平位（通过将 TX 引脚强制拉低实现）。
2. **同步段**：0x55（二进制 01010101，用于从节点同步波特率）。
3. **标识符段**：包含帧 ID 和校验位（保护 ID 不被干扰）。
4. **数据段**：1~8 字节数据。
5. **校验段**：对数据段的累加和校验（LIN 1.x 用经典校验，LIN 2.x 支持增强校验）。

```c++
// 发送Break信号（至少13个低电平位，约1ms@20kbps）
void LIN_SendBreak(void) {
  // 暂时将TX引脚配置为推挽输出并拉低
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_ResetBits(GPIOA, GPIO_Pin_2);
  // 延时至少1ms（确保>13位低电平）
  delay_ms(2);
  // 恢复TX为复用功能（UART）
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// 发送LIN帧（主节点）
void LIN_SendFrame(uint8_t id, uint8_t* data, uint8_t len) {
  uint8_t checksum = 0;
  
  // 1. 发送Break
  LIN_SendBreak();
  
  // 2. 发送同步段（0x55）
  USART_SendData(USART2, 0x55);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  
  // 3. 发送标识符段（ID + 校验位，简化示例）
  USART_SendData(USART2, id);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  
  // 4. 发送数据段并计算校验和
  for(uint8_t i=0; i<len; i++) {
    USART_SendData(USART2, data[i]);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    checksum += data[i];
  }
  
  // 5. 发送校验段（取反或直接累加和，按协议选择）
  USART_SendData(USART2, ~checksum);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}
```

#### 3. 实现 LIN 帧接收（从节点）

从节点需先检测 Break 信号，再同步波特率，最后解析帧结构：

```c++
// 检测Break信号（从节点）
uint8_t LIN_DetectBreak(void) {
  // 监测RX引脚低电平持续时间是否>13位（约1ms）
  uint32_t start = TIM_GetCounter(TIM2); // 用定时器计时
  while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 0) {
    if(TIM_GetCounter(TIM2) - start > 2000) { // 假设TIM2计数频率1MHz，2000=2ms
      return 1; // 检测到Break
    }
  }
  return 0;
}

// 接收LIN帧（从节点）
uint8_t LIN_ReceiveFrame(uint8_t* id, uint8_t* data, uint8_t* len) {
  // 1. 等待Break信号
  if(!LIN_DetectBreak()) return 0;
  
  // 2. 验证同步段（0x55）
  while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
  if(USART_ReceiveData(USART2) != 0x55) return 0;
  
  // 3. 接收标识符
  while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
  *id = USART_ReceiveData(USART2);
  
  // 4. 接收数据段（假设固定8字节，实际按协议）
  *len = 8;
  uint8_t checksum = 0;
  for(uint8_t i=0; i<*len; i++) {
    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
    data[i] = USART_ReceiveData(USART2);
    checksum += data[i];
  }
  
  // 5. 验证校验段
  while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
  if(USART_ReceiveData(USART2) != ~checksum) return 0;
  
  return 1; // 接收成功
}
```

## I2C通信
