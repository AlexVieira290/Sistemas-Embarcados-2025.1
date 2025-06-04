# Protótipo Blink controlado via UART

Este protótipo tem como objetivo replicar algumas características que serão necessárias no projeto principal.

---
## Características desenvolvidas

- Comandos GPIO (LED blink);
	- digitalWrite(pin: integer, value: integer)
	- operation pinMode(pin: integer, mode: integer)
- Comunicação UART;
	- operation UART_config()
	- operation UART_print(mensagem: string)
	- operation UART_read() : string
- Temporização de ações;
	- sc_timer_service.cpp (Gerado utilizando ITEMIS)
	- sc_timer_service.h
- Geração de código ITEMIS (C++).

## Projeto prototipo
 ![ITEMIS](ITEMIS_PROT.PNG)
```mermaid
graph LR
A[Setups] -- 1 s  --> B((Iddle))
B -- "1"  --> C((pisca_rapido))
B -- "2"  --> D((pisca_normal))
B -- "3"  --> E((pisca_Lento))
C -- "STOP"  --> B
D -- "STOP"  --> B
E -- "STOP"  --> B
