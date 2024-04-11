Error: L6218E: Undefined symbol sysTick_Config (referred from SysTick.o).
Not enough information to list image symbols.



<img src="E:\typora\Project\STM32报错汇总.assets\image-20240104233426079.png" alt="image-20240104233426079" style="zoom:150%;" />

这个错误是由于在使用ARM Cortex核心的ARM数学库（CMSIS-DSP库）时未正确定义所使用的Cortex核心引起的。

要解决这个错误，你需要根据实际使用的Cortex核心，在代码中正确定义它。根据错误信息提供的选项，你可以选择以下之一：

- 如果你使用的是ARM Cortex-M7核心，需要添加以下定义： `#define ARM_MATH_CM7`
- 如果你使用的是ARM Cortex-M4核心，需要添加以下定义： `#define ARM_MATH_CM4`
- 如果你使用的是ARM Cortex-M3核心，需要添加以下定义： `#define ARM_MATH_CM3`
- 如果你使用的是ARM Cortex-M0+核心，需要添加以下定义： `#define ARM_MATH_CM0PLUS`
- 如果你使用的是ARM Cortex-M0核心，需要添加以下定义： `#define ARM_MATH_CM0`

请根据你实际使用的Cortex核心，在你的代码中添加适当的定义，以解决这个错误。