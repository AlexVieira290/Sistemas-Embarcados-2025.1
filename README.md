
# Sistemas-Embarcados-2025.1

Desenvolvimento de um sistema embarcado para o controle de curvas de temperatura para o processo de criação de cerveja.

---
## Requisitos mínimos

- Interfaces: 
	- Interface i2C [não alcançado];
	- Interface GPIO [implementado];
	- Interface PWM [implementado];
	- interface UART [implementado];
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

## Requisitos Adicionais

- Interfaces de comandos Raspbery: 
	- Interface gráfica [implementado];
	- interface de comandos via UART [implementado];
	- Leitor de PWM;
	- Simulador de sensor de temperatura i2C [não alcançado];
			- Migrado para resposta UART [implementado];
	- Aplicar resposta da leitura PWM na temperatura simulada [implementado]
- Configuração de curvas adicionais personalizadas[implementado];

## Etapa 1 - Implementação de fluxograma Itemis

Durante esta etapa foi desenvolvido a estrutura inicial que determina o funcionamento do sistema, esta foi implementada através de um statechart criado utilizando o software Itemis.
O primeiro bloco implementado foi o fluxo de inicialização e de seleção de operação;
#
![MAIN](Imagens/ITEMIS_MAIN.PNG)
> As funções mostradas na imagem foram sendo adicionadas ao longo da execução projeto e serão detalhadas mais a diante. 
#
A imagem acima mostra o fluxo de operação imaginado para o projeto:
- Setup: Inicia as variáveis e serviços necessários para que o sistema opere corretamente;
- Idle: Aguarda o usuário interagir com o sistema;
- Configuration: aciona as funções que permitirão ao usuário adicionar ou editar curvas de temperatura ao sistema;
- Operation: permite ao usuário iniciar a execução das curvas configuradas;
#
![Configuration](Imagens/ITEMIS_MAIN.PNG)
#
O bloco de configuração foi dividido de forma a permitir o usuário:
- Add: Adicionar uma nova curva customizada ao sistema;
- Edit: Inicialmente pensado em para permitir ao usuário editar curvas salvas, posteriormente alterado para permitir que o usuário seleciona qual curva customizada será enviada para o sistema, esta alteração ocorreu devido a mudança de local de armazenamento das curvas(inicialmente ESP, posteriormente rasp junto a aplicação de controle);
Devido a estratégia de operação via interface gráfica, os estados ADD e EDIT ficaram semelhantes e representam apenas um ponto de espera onde o ESP aguarda os dados tratados pela interface com usuário;
#
![ADD](Imagens/ITEMIS_CFG_ADD.PNG)
![EDIT](Imagens/ITEMIS_CFG_EDIT.PNG)
#
O bloco de operação foi dividido de forma que o usuário possa escolher executar a curva pré estabelecida no sistema ou executar uma das curvas criadas posteriormente.
#
![OPERATION](Imagens/ITEMIS_OPR.PNG)
#
Ao executar a curva pré configurada o processo ocorre conforme a seguir:
- Inicializa uma variável para navegar na curva default;
- Inicializa a leitura dos sensores de temperatura i2C, alterado posteriormente para UART;
- Inicia a etapa de controle/aquecimento, para chegar a temperatura desejada;
- Inicia a etapa para manter a temperatura pelo tempo determinado após alcançar a temperatura desejada
- Verificar se existe uma nova temperatura:
	- Caso exista, reinicia o processo com o novo alvo;
	- Caso não exista, encerra o processo e retorna ao menu principal;
- Tambem foi adicionado um estado de erro, que em caso de problemas pode ser acionada por todas as etapas principais;
 #
![OPERATION](Imagens/ITEMIS_OPR_DEFAULT.PNG)
#
Ao executar curvas customizadas, o processo foi desenhado de forma muito semelhante, com apenas um passo a mais, uma verificação se a curva personalizada possui pelo menos um degrau.
 #
![OPERATION](Imagens/ITEMIS_OPR_CUSTOM.PNG)
#

