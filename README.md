
# 📡 nRF24L01 Driver — Public API Overview

This project provides a non-blocking, state-machine–driven nRF24L01 driver designed for STM32 microcontrollers using HAL + DMA.
All SPI communication, IRQ handling, configuration, TX/RX state transitions, and auto-recovery are handled internally by the driver.
The user only needs to initialize the module, call the main processing routine, and provide a callback for received data.

---

## 🔧 **Callback Prototype**

```c
typedef void (*NRF_ReceiceCallback_t)(uint8_t *receiveData , uint8_t pipe);
```

The driver calls this user-defined callback whenever a valid RX payload is received.

* `receiveData` — pointer to the received payload buffer
* `pipe` — the RX pipe number (0–5) that delivered the packet

> The callback is executed inside the driver’s internal state machine.
> Keep it short and non-blocking (e.g., copy the data and return).

---

## 🚀 **Exported Functions**

### 1. `void NRF_Init(SPI_HandleTypeDef * hSpi, NRF_ReceiceCallback_t recCallback);`

Initializes the nRF24L01 driver.

* `hSpi` — pointer to the SPI handler used for communication
* `recCallback` — the receive callback function
* Performs a soft reset
* Configures the module
* Enters receive mode automatically

Call this once at startup.

---

### 2. `void NRF_FinishSpiTransaction();`

Must be called from the SPI DMA complete callback:

```c
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    NRF_FinishSpiTransaction();
}
```

This function finalizes every SPI transaction, releases CSN, and notifies the internal state machine that the SPI operation is complete.

---

### 3. `void NRF_IrqInterruptRoutin(void);`

Must be called from the EXTI IRQ handler connected to the nRF24L01 `IRQ` pin:

```c
void EXTIx_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(NRF_IRQ_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == NRF_IRQ_Pin)
        NRF_IrqInterruptRoutin();
}
```

This informs the driver that the nRF24L01 triggered an interrupt (TX_DS, RX_DR, MAX_RT).

---

### 4. `void NRF_TransmitPacket(uint8_t *packet , uint8_t length, uint8_t pipe);`

Schedules a packet for transmission.

* `packet` — pointer to user data
* `length` — payload size (must match configured payload length)
* `pipe` — TX pipe index (0–5)

Transmission is **asynchronous**; the driver handles:

* switching to TX mode
* writing TX_ADDR
* loading the payload
* pulsing CE
* returning to RX mode

---

### 5. `bool NRF_CheckTransmittingAvailability(void);`

Returns `true` if the driver is ready to send a new packet.

Transmission is allowed only when:

* the module is idle
* previous operation has completed
* no pending TX_DS or RX_DR interrupts exist

Use this before calling `NRF_TransmitPacket()`:

```c
if (NRF_CheckTransmittingAvailability())
    NRF_TransmitPacket(data, len, 0);
```

---

### 6. `void NRF_Process();`

This is the **main state machine** and must be called frequently inside the main loop:

```c
while (1)
{
    NRF_Process();
    // Your code...
}
```

`NRF_Process()` handles:

* configuration state
* TX state transitions
* RX state transitions
* IRQ processing
* timeouts / auto-recovery
* idle management

The entire communication logic relies on this function being called continuously.

---

# 📝 **Minimum Setup Example**

```c
void ReceiveCallback(uint8_t *data, uint8_t pipe)
{
    // Process received data
}

int main(void)
{
    HAL_Init();
    MX_SPI1_Init();
    MX_GPIO_Init();

    NRF_Init(&hspi1, ReceiveCallback);

    while (1)
    {
        NRF_Process();

        if (NRF_CheckTransmittingAvailability())
        {
            uint8_t msg[] = {0x11, 0x22, 0x33};
            NRF_TransmitPacket(msg, sizeof(msg), 0);
        }
    }
}
```

---


