set pagination off
set print pretty on
set print elements 64
target extended-remote 127.0.0.1:3333
monitor reset init
break main
break Error_Handler
break vApplicationStackOverflowHook
