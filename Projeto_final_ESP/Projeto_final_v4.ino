#include "src-gen/Statechart.h"
#include "src-gen/Statechart.cpp"
#include "src/sc_timer_service.h"
#include "CallbackController.h"
#include <Wire.h>

using namespace sc;
using namespace sc::timer;

// Configurações da máquina de estados
constexpr size_t MAX_TIMER_TASKS = 10;
TimerTask timerTasks[MAX_TIMER_TASKS];
TimerService timerService(timerTasks, MAX_TIMER_TASKS);

//Statechart statechart;
CallbackController callbackControl;

// Task que substitui o loop()
void statechartTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        timerService.proceed(10);  // Atualiza timers
        callbackControl.getStateMachine()->triggerWithoutEvent();  // Ciclo da máquina de estados
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10)); // Delay fixo
    }
}

void setup() {

    callbackControl.initMutex();
    //pinMode(21, INPUT_PULLUP);
    //pinMode(22, INPUT_PULLUP);
    //Wire.setClock(100000);
    //Wire.begin(21, 22); // SDA = GPIO21, SCL = GPIO22
    //callbackControl.init();
    // Inicialização da máquina de estados
    //statechart.setTimerService(&timerService);
    callbackControl.getStateMachine()->setOperationCallback(&callbackControl);

    callbackControl.getStateMachine()->enter();

    // Criação da task do FreeRTOS
    xTaskCreate(
        statechartTask,     // Função
        "StatechartTask",   // Nome
        4096,               // Stack size (ajuste se necessário)
        NULL,               // Parametros
        1,                  // Prioridade
        NULL                // Handle
    );

    xTaskCreate(
        uartListenerTask,
        "UARTListener",
        2048,
        NULL,
        1,
        NULL
);
}

void uartListenerTask(void *pvParameters) {
    while (true) {
        //Serial.println("DEBUG: ");  //adicionado para debug do erro panick
        sc_string cmd = callbackControl.UART_read();
        String strCmd = String(cmd);

        if (strCmd.length() > 0) {
            if (xSemaphoreTake(commandMutex, pdMS_TO_TICKS(50))) {
                receivedCommand = strCmd;
                xSemaphoreGive(commandMutex);
            }
        }

        free(cmd); // Libera memória do strdup
        //Serial.print("Heap disponível: "); //debug, mostra a quantidade de heap
        //Serial.println(ESP.getFreeHeap());
        vTaskDelay(pdMS_TO_TICKS(500)); //REVISAR TEMPO, POSSIVELMENTE PODE SER ATÉ 500
    }
}

void loop() {
  // Nada aqui - o FreeRTOS assume o controle
}