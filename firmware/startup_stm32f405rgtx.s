.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global g_pfnVectors
.global Default_Handler
.global Reset_Handler
.type g_pfnVectors, %object
.type Reset_Handler, %function
.extern SystemInit
.extern __libc_init_array
.extern main

.section .isr_vector, "a", %progbits
g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .rept 98
  .word Default_Handler
  .endr
.size g_pfnVectors, .-g_pfnVectors

.section .text.Reset_Handler, "ax", %progbits
.thumb_func
Reset_Handler:
  ldr r0, =_estack
  mov sp, r0
  bl SystemInit

  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b 2f
1:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4
2:
  adds r4, r0, r3
  cmp r4, r1
  bcc 1b

  ldr r0, =_sbss
  ldr r1, =_ebss
  movs r2, #0
  b 4f
3:
  str r2, [r0]
  adds r0, r0, #4
4:
  cmp r0, r1
  bcc 3b

  bl __libc_init_array
  bl main
5:
  b 5b
.size Reset_Handler, .-Reset_Handler

.section .text.Default_Handler, "ax", %progbits
.thumb_func
Default_Handler:
  b Default_Handler

.macro WEAK_DEFAULT handler
  .weak \handler
  .set \handler, Default_Handler
.endm

WEAK_DEFAULT NMI_Handler
WEAK_DEFAULT HardFault_Handler
WEAK_DEFAULT MemManage_Handler
WEAK_DEFAULT BusFault_Handler
WEAK_DEFAULT UsageFault_Handler
WEAK_DEFAULT SVC_Handler
WEAK_DEFAULT DebugMon_Handler
WEAK_DEFAULT PendSV_Handler
WEAK_DEFAULT SysTick_Handler
