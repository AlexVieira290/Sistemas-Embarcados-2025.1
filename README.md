# Sistemas-Embarcados-2025.1

Desenvolvimento de um sistema embarcado para o controle de curvas de temperatura para o processo de criação de cerveja.

---
## Requisitos mínimos

- Interfaces: 
	- Interface i2C;
	- Interface GPIO;
	- Interface PWM;
- Funcionais: 
	- Curva padrão: o sistema deve possuir uma carva padrão pré-configurada composta por 3 etapas de aquecimento T0 67°C, T1 78°C e T3 temperatura max (fervura);
	- Interface de comando: o sistema deve possuir interface de comando via computador;
	- Tratativa de erros: o sistema deve possuir métricas que permitam identificar, sinalizar e atuar em casos de erros no sistema;
	- Controle de temperatura: o sistema deve regular a temperatura através de um controle PID ou controle simples por histerese;

- Organizacionais:
	- Documentação de Hardware: deve possuir documentação que descreva o hardware implementado;
	- Statecharts: o sistema deve ser implementado utilizando a statecharts para descrever o seu funcionamento;
	- Github: deve possui documentação compartilhada em repositório github;

Anotações/Sugestões discutidas em sala:
- Doxygen, aplicação permite gerar um documentação automatizada dos códigos gerados durante o projeto;  
