# Keil AC6 + FreeRTOS + CubeMX ����˵��

���ĵ�˵��������Ϊ����Ҫ `tools/` �µĺ����ű����Լ��ճ������뻻����ʱ�Ĺ̶����̡�**���� `tools/README.md` һ���ύ git��**

---

## ������Ϊʲô CubeMX ���ɵĹ����� Keil ��಻��

| ��Ŀ | CubeMX Ĭ�ϣ�MDK-ARM V5.32�� | ������ʵ��ʹ�� |
|------|------------------------------|----------------|
| ������ | ���� AC5 ϰ�� | **Keil AC6**��ArmClang V6.x�� |
| FreeRTOS �˿� | `portable/RVDS/ARM_CM4F` | ���� **`portable/GCC/ARM_CM4F`** |
| `portmacro.h` | ʹ�� `__forceinline`��`__asm` �� AC5 �﷨ | AC6 ��ʶ�� �� �� `__forceinline` �ȴ��� |
| `SystemCoreClock` | д�� `#if defined(__GNUC__)` �� | **AC6 ������ `__GNUC__`** �� `port.c` ��δ���� |
| FPU | `configENABLE_FPU 0` | GCC CM4F �˿���Ҫ **`configENABLE_FPU 1`** |

CubeMX **ÿ�� Generate** ���᣺

- �� Keil ����ָ�� **RVDS**
- **ɾ��** `Middlewares/.../GCC/`��ֻ���� RVDS��
- ������ղ��� USER CODE���ű��Ჹ�عؼ��

��˲���������������һ�ξ�������ȷ������Ҫ **Generate ���Զ��ܽű�**��

---

## �ճ����̣��̶�������

```
1. �ر� Keil
2. STM32CubeMX �� GENERATE CODE
3. �� Keil �� Rebuild�������� Clean Targets��
```

CubeMX �� **Project Manager �� Code Generator �� After Code Generation** ��Ӧ����Ϊ��

```
tools\fix_keil_ac6_freertos.bat
```

�ű��� Generate **������**�Զ�ִ�У������ָ� `uvprojx` �� `FreeRTOSConfig.h`��

---

## �ű���ʲô��`tools/fix_keil_ac6_freertos.ps1`��

| ���� | ���� |
|------|------|
| 1 | �� `tools/freertos_port/GCC/ARM_CM4F/` ���� `port.c`��`portmacro.h` �� `Middlewares/.../GCC/ARM_CM4F/` |
| 2 | �޸� `MDK-ARM/STM32G474RET6_MOTOR.uvprojx`��include ·���� `port.c` �� **RVDS �� GCC** |
| 3 | ȥ�� `uvprojx` �� **UTF-8 BOM**������ Keil �� *Cannot read project file*�� |
| 4 | �� `Core/Inc/FreeRTOSConfig.h` �� **USER CODE** ��д�� `extern SystemCoreClock`��`configENABLE_FPU 1`����ȱʧ�� |
| 5 | �� Define ���Ƴ� `__CC_ARM`�������ڣ� |

**�ű����ⲻ������**�������� CubeMX ��ͻ����

- ���޸� Keil Before Make ����
- ���� `bringup`��CMSIS-DSP ���Զ��幤����
- ����� `.mxproject`

---

## ���������봦��

### CubeMX ������`MDK-ARM V5.32 project generation have a problem`

- **����**��C Դ�������ɣ��� CubeMX **û�������ϲ�** Keil �� `.uvprojx`��
- **����**���� OK ������ȷ�� After �ű����ܣ�Keil **Rebuild**���� **0 Error** Ϊ׼������׷�󵯴���ʧ��

### Keil��`Cannot read project file ... uvprojx`

- **����ԭ��**��`uvprojx` �� UTF-8 BOM���ɰ�ű��򹤾�д�룩��
- **����**��˫�� `tools\fix_keil_ac6_freertos.bat`���ٿ� Keil��

### ���룺`unknown type name '__forceinline'` / ·���� `RVDS`

- **ԭ��**���ű�δ�ܻ� After ·��δ���á�
- **����**��˫�� `tools\fix_keil_ac6_freertos.bat` �� Rebuild����� CubeMX After �Ƿ�ָ������ bat��

### ���룺`use of undeclared identifier 'SystemCoreClock'`���� `port.c`��

- **ԭ��**��`FreeRTOSConfig.h` USER CODE Includes ȱ��������AC6 ���� `__GNUC__` ��֧����
- **����**������һ�νű������� USER CODE Includes �Ƿ��� `extern uint32_t SystemCoreClock;`��

### `main.c`��`redefinition of HAL_TIM_PeriodElapsedCallback`

- **ԭ��**���� `USER CODE 4` ��д��һ�ݣ�CubeMX ��������һ�ݡ�
- **�淶**��FOC ���߼�ֻ���� CubeMX ���ɺ����ڵ� **`USER CODE BEGIN Callback 1`**��`USER CODE 4` ������ ADC �������ص���

---

## �¹��� / �µ���Ǩ��

1. �����ֿ� **`tools/` ����Ŀ¼** ���Ƶ��¹��̸�Ŀ¼���� `.ioc` ͬ������
2. CubeMX **After Code Generation** ��Ϊ `tools\fix_keil_ac6_freertos.bat`������ `.ioc`��
3. ȷ�� Keil ʹ�� **Compiler V6��AC6��**��C/C++ Define �� **��** `__CC_ARM`��
4. ������������ `STM32G474RET6_MOTOR`����Ľű��� `uvprojx` ·�����������Ϊ�Զ����� `MDK-ARM\*.uvprojx`����
5. �����ճ����̡�Generate �� Rebuild ��֤��

**�����ύ git �����ݣ�**

- `tools/`���� `freertos_port/`��
- `STM32G474RET6_MOTOR.ioc`���� UAScriptAfterPath��
- ���ĵ��� `tools/README.md`

---

## ��ʱ��Ҫ CubeMX Generate

| ��Ҫ Generate | ����Ҫ Generate |
|---------------|-----------------|
| �����š�ʱ�ӡ��������� | ֻ�� `main.c`��FOC��ң�⡢ͨ���߼� |
| �� FreeRTOS ���� / �ж����ȼ� | ֻ�� `bringup/` ��Ӧ�ô��� |
| �� DMA / UART �� CubeMX ���� | ���Բ�����VOFA ͨ���� |

Ӧ�ò㿪����**ֱ�� Keil Rebuild** ���ɡ�

---

## ����ļ�����

| ·�� | ˵�� |
|------|------|
| `tools/fix_keil_ac6_freertos.bat` | CubeMX After ��� |
| `tools/fix_keil_ac6_freertos.ps1` | ʵ���޲��߼� |
| `tools/freertos_port/GCC/ARM_CM4F/` | GCC �˿����ñ��� |
| `MDK-ARM/STM32G474RET6_MOTOR.uvprojx` | Keil ���̣�Generate ���ɽű���Ϊ GCC�� |
| `Core/Inc/FreeRTOSConfig.h` | USER CODE �ɽű�ά�� AC6 ����� |
| `Core/Src/main.c` | TIM �ص���FOC �� Callback 1 |

---

## �޶���¼

| ���� | ˵�� |
|------|------|
| 2026-06-07 | ���棺RVDS��GCC �ű���BOM �޸���FreeRTOSConfig USER CODE �Զ����� |
