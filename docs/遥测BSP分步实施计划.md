# ң�� BSP �ֲ�ʵʩ�ƻ���Bringup �׶Σ�

**�汾**: v1.0  
**����**: 2026-06-07  
**���ù���**: `STM32G474RET6_MOTOR`����ǰΪ bringup���������ֲ�ܹ���  
**�����ĵ�**: [����DMAң��ϵͳ����ĵ�.md](./����DMAң��ϵͳ����ĵ�.md)��Э�����հ�ܹ���  
**��λ��**: VOFA+��JustFloat��6Mbps��

---

## һ��������Ŀ��

��ǰ������ `bringup/` Ϊ������δ��ؼܹ��ĵ��е� `service/debug`��`driver/comm` Ŀ¼�����ƻ��� **���ı�����ܹ���ǰ����**��

1. �� `bringup/` ��� **BSP + ң����Сģ��**���� C������ C++��
2. �ֲ�ʵ�֡�**ÿ���ɶ�����֤**������ / VOFA+ / ������ / �����
3. Ϊ���� CLI Ԥ�� **����ʱ�ɱ�** �ӿڣ�decimation��k��ͨ��ָ�롢��ͣ�������� `const` ��������
4. �հ���·����`TIM1` 20kHz ISR д���壻��·����RTOS ����򣨿�ѡʵ�飩ISR �Է� DMA

**Step 0 �����**��Keil ����ͨ����LPUART1 6Mbps + `HAL_UART_Transmit_DMA` �շ�����֤��

---

## ����Ϊʲô�� C������ C++

| ԭ�� | ˵�� |
|------|------|
| ��·���� ISR | TIM1 20kHz �ص���C ������Ԥ�� |
| ������״ | `bringup/`��`FOC_CAL`��`HAL` �ص���Ϊ C |
| ���Ӷ� | ˫���� + ״̬�� + `memcpy`��struct �㹻 |
| C++ ���� | ISR �б��⹹��˳���쳣���� HAL ����� `extern "C"` |

**Լ��**��

- **Bringup / MVP**��`bsp_telem_tx`��`bsp_telemetry` ȫ�� **�� C**
- **�Ժ�**��CLI��ע������� C++����ͨ�� `extern "C"` ���� `telem_set_decimation()` ����·�� API��**ISR ��ֻ�� C �� `telem_tick()`**

---

## ����Ŀ¼��ֲ㣨Bringup �ڣ�

```text
bringup/
  bsp_telem_tx.h / bsp_telem_tx.c    # BSP��LPUART1 DMA ����æ�ж�
  bsp_telemetry.h / bsp_telemetry.c  # Э�顢˫���塢����ʱ����

Core/Src/main.c                      # ���ɣ�init / TIM1 �ص� / TxCplt
Core/Src/app_freertos.c              # Step 4 ��UART_DMA_DEBUG_TASK �� DMA
```

```text
��������������������������������������������������������������������������������������
��  main.c / app_freertos.c�����ɣ�4 ���ߣ�   ��
���������������������������������������Щ���������������������������������������������
                   �� telem_init / tick / try_send / on_dma_done
��������������������������������������������������������������������������������������
��  bsp_telemetry.c   �� HAL include       ��
��  JustFloat + ˫���� + g_telem_rt        ��
���������������������������������������Щ���������������������������������������������
                   �� bsp_telem_tx_dma / busy
��������������������������������������������������������������������������������������
��  bsp_telem_tx.c    HAL hlpuart1 + DMA   ��
��������������������������������������������������������������������������������������
```

CubeMX ������ **һ�㲻��**��`usart.c`��6Mbps����`dma.c`��DMA1_Ch1����`stm32g4xx_it.c`��

���ڱ�������ң�Ⲣ��ά����`tools/`��`docs/`��`bringup/` �ڵ�����Դ�ļ���

---

## �ġ��� vs ����ʱ����

### �ʺϱ����ں꣨CLI ���ģ�

| �� | Ĭ�� | ���� |
|----|------|------|
| `TELEMETRY_ENABLE` | 1 | 0 = ��ģ�鲻������� |
| `TELEMETRY_BUFFER_BYTES` | 4096 | ������ SRAM ��С����̬���� |

Э��֡β `{0x00,0x00,0x80,0x7f}` Ϊ JustFloat ������д���� `bsp_telemetry.c`��

### ��������ʱ���� + setter������ CLI��

```c
typedef struct {
    uint8_t  running;         /* telem on/off */
    uint8_t  decimation;      /* D��Ĭ�� 1 */
    uint8_t  channel_count;   /* k��Ĭ�� 4 */
    float   *channels[16];    /* ע���ָ�� */
} telem_runtime_t;

extern telem_runtime_t g_telem_rt;
```

| API����·���� | ��; |
|---------------|------|
| `telem_init_defaults(void)` | Ĭ��ͨ���� |
| `telem_start()` / `telem_stop()` | ��ͣ���� |
| `telem_set_decimation(uint8_t d)` | �Ľ����� |
| `telem_set_channel_count(uint8_t k)` | ��ͨ���� |
| `telem_set_channel(uint8_t idx, float *ptr)` | ��ĳ·����Դ |

**��Ҫ�� `const` ��ס�������ýṹ��**��ͨ����Ŀ¼�ɾ�ֻ̬����**��ǰѡ����Щ `float*` ����ɱ�**��

### ISR ��·�� API

| ���� | ������ |
|------|--------|
| `telem_tick(void)` | TIM1 20kHz �ص� |
| `telem_try_send(void)` | Step 4��RTOS ����Step 5����ѡ����ISR |
| `telem_on_dma_done(void)` | `HAL_UART_TxCpltCallback`��`hlpuart1`�� |

---

## �塢������Լ��4 ���ߣ�

| λ�� | ���� |
|------|------|
| `main` USER CODE 2 | `telem_init_defaults()`��**����**�ٱ�����ң���ͻ�Ĳ��� `HAL_UART_Transmit_DMA` |
| TIM1 `HAL_TIM_PeriodElapsedCallback` Callback 1 | FOC �߼��� `telem_tick()`����Step 5 ��ѡ��`telem_try_send()` |
| `UART_DMA_DEBUG_TASK` | Step 4 �� `telem_try_send()` + `osDelay(1)` |
| `HAL_UART_TxCpltCallback` | `huart == &hlpuart1` �� `telem_on_dma_done()` |

---

## �����ֲ�ʵʩ�����ձ�׼

### ����

```text
Step 0  ���� / ȥ��ͻ          ? ����ɣ�6Mbps HAL TX ����֤��
Step 1  BSP �㷢               ? ʵ����ɣ��ɲ��� Step 2 ��ʽ��װ��
Step 2  ��֡ JustFloat��������  ? ��ǰ��һ��
Step 3  ISR ֻд���壨��Ƶ��
Step 4  ˫���� + ���� DMA
Step 5  ����ѡ��ISR �Է� DMA
Step 6  decimation=1 ���� 20kHz
Step 7  �����������ʵͨ����
Step 8  ����ʱ API��CLI Ԥ����
```

---

### Step 0���������ͻ���� ?

**����**��

- Keil Rebuild 0 Error
- LPUART1 6Mbps��DMA1_Channel1 ���� CubeMX ����
- ȷ�ϲ���ң���� DMA��`main.c` ��һ���Բ��� `HAL_UART_Transmit_DMA` Ӧ�� Step 2 ǰɾ����ע�ͣ�

**����**������ͨ����6Mbps �շ���ʵ����á�

---

### Step 1��BSP �㷢 ?������ʽ��װ��

**����**��

- `bsp_telem_tx.c/h`��`bsp_telem_tx_busy()`��`bsp_telem_tx_dma(buf, len)`
- �ڲ�ʹ�� `hlpuart1`��`HAL_UART_Transmit_DMA`

**����**���� Step 0 �ȼ۵� DMA �����Ϊ������ Step 2 һ���ύ����

---

### Step 2����֡ JustFloat��main ������? ��ǰ

**����**��

- �½� `bsp_telemetry.c/h`����С�棩����һ֡ = `seq(4B) + k��float + ֡β`
- �� `main` �� `StartDefaultTask` ÿ **100ms** ��һ֡����ֵ���� 1.0, 2.0, 3.0, 4.0��
- ʵ�� `HAL_UART_TxCpltCallback` �� �����ɣ�������Ϊ `telem_on_dma_done`��

**����**��ISR��`g_telem_rt` ȫ�ס�˫���塢decimation��

**VOFA+ ����**��

| �� | ֵ |
|----|-----|
| ������ | 6000000 |
| Э�� | JustFloat |
| CH_COUNT | **5**��k=4 ʱ���� seq�� |
| ͨ�� 0 | ���� |

**ͨ����׼**����·���Բ����ȶ���VOFA+ ���ͻ��塢�̼���������

**ʧ���Ų�**��

- �����ݵ��� �� float С�� / ֡β���ֽ�
- ������ �� �ز� BSP / RS485 / ������
- VOFA+ ���� �� ֡β���� `0x7F800000` С��

**Keil**��bringup ����� `bsp_telem_tx.c`��`bsp_telemetry.c`��`../bringup` ���� Include Path����

---

### Step 3��ISR ֻд���壨���� DMA��

**����**��

- `telem_tick()` д�뻺�壨���ȵ����壬��˫���壩
- ���� `g_telem_rt`��`telem_init_defaults()`��Ĭ�ϰ� `&as5047_spi1.get`��`&uq` �� **ȫ��/��̬** ��ַ
- TIM1 Callback 1 ĩβ���� `telem_tick()`
- **`decimation = 200`**��Լ 100Hz�������͵���ѹ��
- **���� ISR ���� DMA**���õ������۲� `seq`��`write_idx` ����

**ͨ����׼**�������Ϊ���䣻`seq` ����������CPU ���ؿɽ��ܡ�

---

### Step 4��˫���� + RTOS ���� DMA���Ƽ��հ淢��·����

**����**��

- ״̬����`UNLOCKED` �� `LOCKED` �� `READY` �� `SENDING` �� `UNLOCKED`
- `telem_tick()` д�� �� `READY`
- **`UART_DMA_DEBUG_TASK`** ѭ������ `telem_try_send()`��`osDelay(1)`
- `HAL_UART_TxCpltCallback` �� `telem_on_dma_done()`
- DMA `Size` �� **`used_bytes`**������ 4096

**ͨ����׼**��VOFA+ �������Σ�decim=200 ʱԼ 100Hz������ʱ�䲻������˫����״̬�ֻ�������

**ʧ���Ų�**��

- ֻ��һ�� �� `on_dma_done` δ���߻� `used_bytes` ����
- �ϵ� �� �������ȼ� / `osDelay` ����

---

### Step 5������ѡ��ISR �� `telem_try_send`

**����**��

- TIM1 �ص���`telem_tick()` ����� `telem_try_send()`
- ������ `telem_try_send()` **�ر�**������˫����� DMA
- �Ա��� `decimation=200` ����

**ͨ����׼**���� Step 4 �൱�� VOFA+ ���֣��� HardFault��������쳣��

**����ͨ��**��**���˵� Step 4**��ISR ֻд������ DMA����Ϊ��ʽ��������ʧ�ܡ�

---

### Step 6�����٣�decimation = 1��

**����**��

- `g_telem_rt.decimation = 1`��k=4
- ����ĵ�Ŀ�꣺Լ 80% ���������ʣ�seq ����

**ͨ����׼**����·����������seq ż�����ſɽ��ܣ�����������������������Զ񻯡�

---

### Step 7���������

**����**��

- `BRINGUP_ADC_TEST = 0`��PWM + `setPhaseVoltage` ����
- ͨ������ʵ�����Ƕȡ�Uq��������`cnt` ��

**ͨ����׼**�����ת + VOFA+ �������������� `uq` ���θ���仯��

---

### Step 8������ʱ API��CLI Ԥ�����ȵ�������֤��

**����**��

- ʵ�� `telem_start/stop`��`telem_set_decimation`��`telem_set_channel`��`telem_set_channel_count`
- �ݲ��� CLI������������ʱ����� `g_telem_rt` ��֤

**ͨ����׼**��

- `telem_stop()` �� VOFA+ ͣ��
- `telem_set_decimation(10)` ��Ƶ��ԼΪԭ���� 1/10
- ��ͨ��ָ����Ӧ���߱仯

---

## �ߡ�VOFA+ ����嵥��Step 2 ��ÿ�����ã�

1. ������ **6000000**
2. Э�� **JustFloat**���� Raw / FireWater��
3. **CH_COUNT = k + 1**��k=4 �� 5��
4. ͨ�� 0 ����ʾ��seq��
5. USB ת RS485 ��֧�� 6M������֧�֣����õͲ�������֤���ߺ����� 6M

---

## �ˡ����·����ʱ���ʱ��

```text
Step 0 ? �� Step 2��VOFA+ ��֡���� Step 4������ + ˫���壩�� Step 6�����٣��� Step 7�������
```

Step 5��ISR �� DMA��Ϊ **��ѡʵ��**�����ȶ���̶�ʹ�� Step 4��

---

## �š����հ�ܹ���Ǩ��

| Bringup ��״ | �Ժ� |
|--------------|------|
| `bringup/bsp_telemetry` | ��Ǩ�� `service/debug/telemetry_stream` |
| `bringup/bsp_telem_tx` | ��Ǩ�� `driver/comm/lpuart_telemetry_tx` |
| `g_telem_rt` + setter | ֱ�ӹ� CLI ʹ�� |
| ISR �� `try_send`���������� | ��Ϊ�� RTOS �������ͬһ���� |

��·����������`telem_tick` / `telem_try_send` / `telem_on_dma_done`�����鱣�ֲ��䣬���� `main.c` �Ķ���

�� HAL �� LL / ��Ĵ�����**ֻ�滻 `bsp_telem_tx.c`**��`bsp_telemetry.c` ������

---

## ʮ������ļ�����

| ���� | ·�� |
|------|------|
| Э������� | `docs/����DMAң��ϵͳ����ĵ�.md` |
| ������ | `docs/Keil_AC6_FreeRTOS_CubeMX����˵��.md`��`tools/` |
| LPUART / DMA | `Core/Src/usart.c`��`Core/Src/dma.c` |
| TIM1 FOC ISR | `Core/Src/main.c` Callback 1 |
| RTOS ���� | `Core/Src/app_freertos.c`��`UART_DMA_DEBUG_TASK`�� |
| ���½� | `bringup/bsp_telem_tx.c/h`��`bringup/bsp_telemetry.c/h` |
| Keil ���� | `MDK-ARM/STM32G474RET6_MOTOR.uvprojx`��bringup ���Դ�ļ��� |

---

## ʮһ���ճ���������

```text
�� Keil �� CubeMX Generate��After �ű��Զ����� Keil Rebuild
```

`bringup/` �������ļ����ᱻ CubeMX ���ǣ�Generate ��ȷ�� Keil ������ telemetry Դ�ļ����ڡ�

---

## ʮ�����޶���¼

| �汾 | ���� | ˵�� |
|------|------|------|
| v1.0 | 2026-06-07 | ���棺Bringup �ֲ��ƻ���C-only������ʱ API Լ����Step 0/1 ����� |
