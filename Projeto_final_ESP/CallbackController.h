#ifndef CALLBACKCONTROLLER_H
#define CALLBACKCONTROLLER_H

#include "src-gen/Statechart.h"
#include <Arduino.h>
#include <vector>
#include "freertos/semphr.h"
//======= struct para curvas
struct CurvaPonto {
  float temperatura;
  String tempo;
};

struct Curva {
  std::vector<CurvaPonto> pontos;
};
//=================================

void carregarCurva(const String& jsonStr, Curva& curvaDestino);

class CallbackController : public Statechart::OperationCallback {
public:
    // Métodos para controle de GPIO
    void digitalWrite(sc_integer pin, sc_integer value) override;
    void pinMode(sc_integer pin, sc_integer mode) override;

    // Métodos para controle de UART
    void UART_config() override;
    void UART_print(sc_string mensagem) override;
    sc_string UART_read() override;
    //sc_integer UART_read_int() override;
    sc_integer stringParaInteiro(sc_string mensagem) override;
    sc_real stringParaReal(sc_string mensagem) override;
    void initMutex();  // Inicializa o mutex, deve ser chamado no setup
    sc_string getCommand() override;
    void carregarCurvaDefault() override;   // Operation para carregar curva
    sc_string carregarCurvaTemperatura(sc_string comando) override;  // Operation para carregar curva
    void imprimirCurvaPersonalizada() override;                 // Debug da carga
    //static float readTemperatureFromSensor(int sensorId);
    float readTemperatureFromSensor(int sensorId);
    static void temperaturaTask(void* pvParameters); // função ler temperatura UART
    void iniciarLeituraTemperatura() override;
    void pararLeituraTemperatura() override;
    
    static constexpr int VEC_SIZE = 5;
    static constexpr int TEMP_TASK_DELAY_MS = 5000;
    float calcMedia(float* vetor);
    static sc_string getCommand2();

    void setupPWM() override;
    void iniciarTaskDegrau(sc_integer curvaId) override;
    void iniciarTaskManutencao(sc_integer curvaId) override;
    void encerrarTaskDegrau() override;
    void encerrarTaskManutencao() override;
    void verificarProximoDegrau(sc_integer curvaId) override;
    Statechart* getStateMachine();
    static void taskDegrauPID(void* pvParameters);
    static void taskManutencao(void* pvParameters);

private:
    static String uartBuffer; // Buffer estático para leitura UART
    static SemaphoreHandle_t uartMutex;

    static float temp1[VEC_SIZE];
    static float temp2[VEC_SIZE];
    static int index;
    static TaskHandle_t tempTaskHandle;
    static TaskHandle_t taskDegrauHandle;
    static TaskHandle_t taskManutencaoHandle;
    Statechart statechart;
    static int contaerros; // Contador de erros de Leitura
    static float ultimatemperatura;
    static int contaerrosdegrau;
};

//======= struct para selecionar curvas
struct TaskParamsID {
    CallbackController* controller;
    int curvaId;  // 0 = default, 1 = personalizada
};
//===================================================

extern String receivedCommand;
extern SemaphoreHandle_t commandMutex;

#endif // CALLBACKCONTROLLER_H
