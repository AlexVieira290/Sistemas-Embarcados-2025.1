#include "CallbackController.h"
#include <cstdlib>  // Para std::atoi e std::atof
#include <ArduinoJson.h>
#include "freertos/semphr.h"
#include <Arduino.h>
#include <stdio.h>
#include "src-gen/Statechart.h"
#include <PID_v1_bc.h>


// Inicialização do buffer estático
String CallbackController::uartBuffer = "";
SemaphoreHandle_t CallbackController::uartMutex = nullptr;
String receivedCommand = "";
SemaphoreHandle_t commandMutex = xSemaphoreCreateMutex();

Curva curvaDefault;
Curva curvaPersonalizada;

float CallbackController::temp1[VEC_SIZE] = {0};
float CallbackController::temp2[VEC_SIZE] = {0};
int CallbackController::index = 0;
TaskHandle_t CallbackController::tempTaskHandle = nullptr;
TaskHandle_t CallbackController::taskDegrauHandle = nullptr;
TaskHandle_t CallbackController::taskManutencaoHandle = nullptr;
int CallbackController::contaerros = 0; // Inicinaliza contador de erros
float CallbackController::ultimatemperatura = 0.0; // buffer de temperatura para checagem de erro do degrau
int CallbackController::contaerrosdegrau = 0; // Inicinaliza contador de erros do degrau

// ======= DEFINIÇÕES E ESTRUTURAS =======
#define PINO_PWM_AQUECEDOR 18  // GPIO usado para PWM

int pontoIndex = 0; // Agora global e controlado pelo firmware
extern Curva curvaDefault; // Mantido como referência externa
extern int tempoParaSegundos(String tempoStr);

// =======

#define SDA_PIN 21
#define SCL_PIN 22
#define PINO_BOMBA 2   // bomba
#define PINO_MIXER 4   // mixer

void CallbackController::digitalWrite(sc_integer pin, sc_integer value) {
    //Serial.print(millis());
    //Serial.print(" ms - digitalWrite chamado: ");
    //Serial.print(pin);
    //Serial.print(" = ");
    //Serial.println(value);
    ::digitalWrite(pin, value);
}

void CallbackController::pinMode(sc_integer pin, sc_integer mode) {
    Serial.print("pinMode chamado: ");
    Serial.print(pin);
    Serial.print(" = ");
    Serial.println(mode);
    
    if (mode > 0) {
        ::pinMode(pin, OUTPUT);
    } else {
        ::pinMode(pin, INPUT);
    }
}

void CallbackController::UART_config() {
    Serial.begin(115200);  // Inicializa a UART com 115200 bps
    while (!Serial) {
        ; // Aguarda a inicialização
    }
    Serial.println("UART inicializada com sucesso!");
}

void CallbackController::UART_print(sc_string mensagem) {
    if (uartMutex && xSemaphoreTakeRecursive(uartMutex, portMAX_DELAY)) {
        Serial.println(mensagem);
        xSemaphoreGiveRecursive(uartMutex);
    }
}

sc_string CallbackController::UART_read() {
    if (uartMutex && xSemaphoreTakeRecursive(uartMutex, portMAX_DELAY)) {
        while (Serial.available()) {
            char c = Serial.read();
            if (c == '\n') {
                String result = uartBuffer;
                uartBuffer = "";
                result.trim();
                Serial.println(result.c_str());
                xSemaphoreGiveRecursive(uartMutex);
                return strdup(result.c_str());
            } else {
                uartBuffer += c;
            }
        }
        xSemaphoreGiveRecursive(uartMutex);
    }
    return strdup("");
}

sc_integer CallbackController::stringParaInteiro(sc_string mensagem) {
    if (mensagem == nullptr) {
        return 0; // Valor padrão em caso de ponteiro nulo
    }
    return static_cast<sc_integer>(std::atoi(mensagem));
}

sc_real CallbackController::stringParaReal(sc_string mensagem) {
    if (mensagem == nullptr) {
        return 0.0; // Valor padrão em caso de ponteiro nulo
    }
    return static_cast<sc_real>(std::atof(mensagem));
}

void CallbackController::initMutex() {
    if (uartMutex == nullptr) {
        uartMutex = xSemaphoreCreateRecursiveMutex();
    }
    if (commandMutex == nullptr) {
        commandMutex = xSemaphoreCreateMutex();
    }
}

sc_string CallbackController::getCommand() {
    static char buffer[512];
    strncpy(buffer, receivedCommand.c_str(), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    receivedCommand = "";
    return buffer;
}

sc_string CallbackController::getCommand2() {
    static char buffer[512];
    strncpy(buffer, receivedCommand.c_str(), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    receivedCommand = "";
    return buffer;
}

void CallbackController::carregarCurvaDefault() {
    curvaDefault.pontos.clear();
    curvaDefault.pontos.push_back({67.0, "01:00"});     //valor acordado como default 67
    curvaDefault.pontos.push_back({72.0, "03:00"});     //valor acordado como default 78
    curvaDefault.pontos.push_back({77.0, "03:00"});    //valor acordado como default 100
    Serial.println("Curva default carregada via código.");
}

void carregarCurva(const String& jsonStr, Curva& curvaDestino) {
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) {
        Serial.print(F("Erro ao fazer parse do JSON: "));
        Serial.println(error.c_str());
        return;
    }
    std::vector<CurvaPonto>().swap(curvaDestino.pontos);
    JsonArray array = doc["curva_temperatura"];
    for (JsonObject ponto : array) {
        CurvaPonto novoPonto;
        novoPonto.temperatura = ponto["temperatura"];
        novoPonto.tempo = ponto["tempo"].as<String>();
        curvaDestino.pontos.push_back(novoPonto);
    }
    Serial.println(F("Curva carregada com sucesso."));
}

sc_string CallbackController::carregarCurvaTemperatura(sc_string comando) {
    String jsonStr(comando);
    
    if (!jsonStr.startsWith("{") || !jsonStr.endsWith("}")) {
        return "";
    }
    
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) {
        Serial.print("Erro ao fazer parse do JSON: ");
        Serial.println(error.c_str());
        return "";
    }
    
    if (!doc.containsKey("curva_temperatura") || !doc["curva_temperatura"].is<JsonArray>()) {
        Serial.println("JSON inválido: chave 'curva_temperatura' ausente ou não é array.");
        return "";
    }
    
    JsonArray curvaArray = doc["curva_temperatura"];
    for (JsonObject ponto : curvaArray) {
        if (!ponto.containsKey("temperatura") || !ponto.containsKey("tempo")) {
            Serial.println("JSON inválido: um ou mais pontos estão incompletos.");
            return "";
        }
    }
    
    carregarCurva(jsonStr, curvaPersonalizada);
    return "criado";
}

void CallbackController::imprimirCurvaPersonalizada() {
    Serial.println("Pontos da curva personalizada:");
    for (const auto& ponto : curvaPersonalizada.pontos) {
        Serial.print("Tempo: ");
        Serial.print(ponto.tempo);
        Serial.print(" - Temperatura: ");
        Serial.println(ponto.temperatura);
    }
}

float CallbackController::readTemperatureFromSensor(int sensorId) {
    const char* cmd = (sensorId == 1) ? "TEMPERATURA1\n" : "TEMPERATURA2\n";
    Serial.println(cmd);
    unsigned long start = millis();

    while (millis() - start < 1000) {
        sc_string resposta = UART_read();
        if (resposta && strlen(resposta) > 0) {
            return atof(resposta);
        }
        delay(10);
    }
    Serial.println("Timeout na leitura do sensor!");
    return NAN;
}

float CallbackController::calcMedia(float* vetor) {
    float soma = 0;
    for (int i = 0; i < VEC_SIZE; i++) {
        soma += vetor[i];
    }
    return soma / VEC_SIZE;
}

void CallbackController::temperaturaTask(void* pvParameters) {
    CallbackController* self = static_cast<CallbackController*>(pvParameters);
    Statechart* sm = self->getStateMachine(); // Get state machine pointer

    while (true) {
        float t1 = self->readTemperatureFromSensor(1);
        delay(100);
        float t2 = self->readTemperatureFromSensor(2);
        if (t1 > 0.1 && t2 > 0.1) {  // Considera válidas apenas temperaturas acima de 0.1 °C
            temp1[index] = t1;
            temp2[index] = t2;
            index = (index + 1) % VEC_SIZE;
            self->contaerros = 0; // resetando contador de erros
        } else {
            Serial.println("Leituras descartadas por erro de comunicação.");
            self->contaerros++; // Increment counter
            if (self->contaerros >= 10) {
                Serial.println("10 leituras descartadas consecutivamente!");
                if (sm) {
                    sm->raiseErrodetectado(); // Raise the event
                }
                self->contaerros = 0; // Reset counter after raising event
            }
        }

        float media1 = self->calcMedia(temp1);
        float media2 = self->calcMedia(temp2);
        float diff = fabs(media1 - media2);

        //Serial.printf("Média T1: %.2f°C | T2: %.2f°C | Diferença: %.2f°C\n", media1, media2, diff);

        if (diff > 2.0) {
            self->digitalWrite(PINO_BOMBA, HIGH);   // Liga bomba
            self->digitalWrite(PINO_MIXER, HIGH);   // Liga mixer
        } else {
            self->digitalWrite(PINO_BOMBA, LOW);    // Desliga bomba
            self->digitalWrite(PINO_MIXER, LOW);    // Desliga mixer
        }

        vTaskDelay(pdMS_TO_TICKS(TEMP_TASK_DELAY_MS));
    }
}

void CallbackController::iniciarLeituraTemperatura() {
    if (tempTaskHandle == nullptr) {
        index = 0;
        xTaskCreatePinnedToCore(
            temperaturaTask,
            "TempTask",
            4096,
            this,
            1,
            &tempTaskHandle,
            APP_CPU_NUM
        );
        Serial.println("Task de temperatura iniciada");
    }
}

void CallbackController::pararLeituraTemperatura() {
    if (tempTaskHandle != nullptr) {
        vTaskDelete(tempTaskHandle);
        tempTaskHandle = nullptr;
        Serial.println("Task de temperatura encerrada");
    }
}

// ======= FUNÇÕES DE CONTROLE PID =======

double setpoint;          // Temperatura alvo
double input;             // Média das temperaturas
double output;            // Saída PWM

//double Kp = 2.0, Ki = 0.5, Kd = 1.0;
double Kp = 10.0, Ki = 0.8, Kd = 1.0;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ======= TASK DE CONTROLE DO DEGRAU =======

void CallbackController::taskDegrauPID(void* pvParameters) {
    TaskParamsID* params = static_cast<TaskParamsID*>(pvParameters);
    CallbackController* self = params->controller;
    int curvaId = params->curvaId;

    Curva* curva = (curvaId == 1) ? &curvaPersonalizada : &curvaDefault;
    setpoint = curva->pontos[pontoIndex].temperatura;
    
    delete params;
    // Reset do PID
    pid.SetMode(MANUAL);
    pid.SetOutputLimits(0, 255);
    output = 0;

    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255);

    Statechart* sm = self->getStateMachine();
    Serial.printf("Aquecimento iniciado - Target: %.2f °C\n", setpoint);

    self->ultimatemperatura = (self->calcMedia(temp1) + self->calcMedia(temp2)) / 2.0; // Initialize with current temp
    self->contaerrosdegrau = 0;

    while (true) {
        // Atualiza a variável de entrada com a média dos sensores
        input = (self->calcMedia(temp1) + self->calcMedia(temp2)) / 2.0;
        Serial.printf("Valor atual: %.2f °C\n", input); // apenas para DEBUG

        pid.Compute();  // Calcula o novo valor de saída

        ledcWrite(PINO_PWM_AQUECEDOR, (int)output);  // Aplica o valor PWM

        // Verificar erro de aquecimento
        if (output > (0.01 * 255) && input < self->ultimatemperatura) {
            self->contaerrosdegrau++;
            if (self->contaerrosdegrau >= 24) { // verificar 2 minutos
                Serial.println("Erro: Possivel erro na resistencia.");
                if (sm) sm->raiseErrodetectado();
                self->contaerrosdegrau = 0; // Resetando o contador
                self->taskDegrauHandle = nullptr;
                vTaskDelete(NULL);
            }
        } else {
            self->contaerrosdegrau = 0; // Resetando o contador
        }
        
        self->ultimatemperatura = input; // atualizando o buffer

        if (fabs(setpoint - input) < 0.5) {
            Serial.println("Setpoint alcançado.");
            if (sm) sm->raiseDegrau_ok();
            self->taskDegrauHandle = nullptr;
            vTaskDelete(NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(TEMP_TASK_DELAY_MS)); // intervalo padronizado com a leitura
    }
}

// ======= TASK DE MANUTENÇÃO DO DEGRAU =======

void CallbackController::taskManutencao(void* pvParameters) {
    TaskParamsID* params = static_cast<TaskParamsID*>(pvParameters);
    CallbackController* self = params->controller;
    int curvaId = params->curvaId;

    Curva* curva = (curvaId == 1) ? &curvaPersonalizada : &curvaDefault;
    setpoint = curva->pontos[pontoIndex].temperatura;
    String tempoStr = curva->pontos[pontoIndex].tempo;
    int tempoMs = tempoParaSegundos(tempoStr) * 1000;

    delete params;

    Statechart* sm = self->getStateMachine();

    // Reset do PID
    pid.SetMode(MANUAL);
    pid.SetOutputLimits(0, 255);
    output = 0;

    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255);

    unsigned long inicio = millis();
    int printCounter = 0;

    self->ultimatemperatura = (self->calcMedia(temp1) + self->calcMedia(temp2)) / 2.0; // Initialize with current temp
    self->contaerrosdegrau = 0;

    while (true) {
        input = (self->calcMedia(temp1) + self->calcMedia(temp2)) / 2.0;
        Serial.printf("Valor atual: %.2f °C\n", input); // apenas para DEBUG
        pid.Compute();
        ledcWrite(PINO_PWM_AQUECEDOR, (int)output);

        // Verificar erro de aquecimento
        if (output > (0.01 * 255) && input < self->ultimatemperatura) {
            self->contaerrosdegrau++;
            if (self->contaerrosdegrau >= 24) { // verificar 2 minutos
                Serial.println("Erro: Possivel erro na resistencia.");
                if (sm) sm->raiseErrodetectado();
                self->contaerrosdegrau = 0; // Resetando o contador
                self->taskManutencaoHandle = nullptr;
                vTaskDelete(NULL);
            }
        } else {
            self->contaerrosdegrau = 0; // Resetando o contador
        }
        
        self->ultimatemperatura = input; // atualizando o buffer

        // Tempo decorrido e total
        unsigned long agora = millis();
        int decorrido = (agora - inicio) / 1000;
        int total = tempoMs / 1000;

        // Exibe tempo a cada ciclo
        if (printCounter++ % 4 == 0) {
            Serial.printf("Tempo: %ds de %ds (%.1f%%)\n", decorrido, total, (decorrido * 100.0) / total);
        }

        if (agora - inicio >= tempoMs) {
            Serial.println("Tempo de manutenção encerrado.");
            if (sm) sm->raiseManutencao_ok();
            self->taskManutencaoHandle = nullptr;
            vTaskDelete(NULL); // Encerra a task
        }

        vTaskDelay(pdMS_TO_TICKS(TEMP_TASK_DELAY_MS)); // intervalo padronizado com a leitura
    }
}

// ======= CHAMADA PARA INICIAR AS TASKS DIRETAMENTE DO CALLBACK =======

void CallbackController::iniciarTaskDegrau(sc_integer curvaId) {
    if (taskDegrauHandle == nullptr) {
        TaskParamsID* params = new TaskParamsID{this, curvaId};
        xTaskCreatePinnedToCore(
            CallbackController::taskDegrauPID,
            "TaskDegrauPID",
            4096,
            params,
            1,
            &taskDegrauHandle,
            APP_CPU_NUM
        );
    }
}

void CallbackController::iniciarTaskManutencao(sc_integer curvaId) {
    if (taskManutencaoHandle == nullptr) {
        TaskParamsID* params = new TaskParamsID{this, curvaId};
        xTaskCreatePinnedToCore(
            CallbackController::taskManutencao,
            "TaskManutencao",
            4096,
            params,
            1,
            &taskManutencaoHandle,
            APP_CPU_NUM
        );
    }
}

// ======= FUNÇÕES PARA ENCERRAR TASKS EM CASO DE PROBLEMAS =======

void CallbackController::encerrarTaskDegrau() {
    if (taskDegrauHandle != nullptr) {
        vTaskDelete(taskDegrauHandle);
        taskDegrauHandle = nullptr;
        Serial.println("Task Degrau encerrada.");
    }
}

void CallbackController::encerrarTaskManutencao() {
    if (taskManutencaoHandle != nullptr) {
        vTaskDelete(taskManutencaoHandle);
        taskManutencaoHandle = nullptr;
        Serial.println("Task Manutenção encerrada.");
    }
}

// ======= PWM SETUP (Adicionar no setup do sistema) =======

void CallbackController::setupPWM() {
    ledcAttach(PINO_PWM_AQUECEDOR, 5000, 8);  // Pino, frequência, resolução
}


// ======= pegar ponteiro da maquina de estados =======
Statechart* CallbackController::getStateMachine() {
    return &statechart;
}

// ======= comversor simples para segundos =======
int tempoParaSegundos(String tempo) {
    int minutos = 0, segundos = 0;
    sscanf(tempo.c_str(), "%d:%d", &minutos, &segundos);
    return minutos * 60 + segundos;
}

//função para percorrer as curvas duranta a operação
void CallbackController::verificarProximoDegrau(sc_integer curvaId) {
    Curva* curva = (curvaId == 1) ? &curvaPersonalizada : &curvaDefault;
    Statechart* sm = getStateMachine();

    if (pontoIndex + 1 < static_cast<int>(curva->pontos.size())) {
        pontoIndex++;
        Serial.printf("Novo degrau: pontoIndex = %d\n", pontoIndex);
        if (sm) sm->raiseNovo_degrau();
    } else {
        Serial.println("Último degrau alcançado.");
        if (sm) sm->raiseUltimo_degrau();
    }
}

