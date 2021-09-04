# segger_uart
Adaption of segger_uart.c for using SystemView together with a NUCLEO-H7 Board

1.)
Add following lines to SEGGER_SYSVIEW_Conf.h:


#define SEGGER_UART_REC 1

#if (SEGGER_UART_REC == 1)
	extern void HIF_UART_EnableTXEInterrupt(void);
    #define SEGGER_SYSVIEW_ON_EVENT_RECORDED(x)  HIF_UART_EnableTXEInterrupt()
#endif

2.)
add segger_uart_h7.c to your list of source files

3.)
in main.c, add SEGGER_UART_init(baudrate) right before the call to SEGGER_SYSVIEW_Conf() and remove SEGGER_SYSVIEW_Start():


  SEGGER_UART_init(500000);

  SEGGER_SYSVIEW_Conf();

  //SEGGER_SYSVIEW_Start();

4.)
In SystemView, klick on Target->Recorder Configuration, select UART, COM port and speed
Then klick on Target->Start Recording and watch the events live
