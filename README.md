# XTR Cansat main code
***freeRTOS*** application running on the ***ESP32*** inside our Cansat.
Program is divided into ***Sensors*** and ***Queues***. Sensors send data to assigned Queues which then store or send all data at once.
Everything runs in asynchronous tasks on both ESP32 cores.
