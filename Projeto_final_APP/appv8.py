import tkinter as tk
from tkinter.scrolledtext import ScrolledText
from tkinter import filedialog
from tkinter import ttk
import threading
import platform
import time
import serial
import json
import csv
import os
from tkinter import messagebox

# START - ADICIONADO PARA TESTE DE LEITURA PWM
import pigpio


class PwmReader:
    def __init__(self, gpio_pin):
        self.pi = pigpio.pi()
        self.gpio_pin = gpio_pin
        self.high_time = 0
        self.low_time = 0
        self.last_tick = 0
        self.level = 0
        self.duty_cycle = 0
        self.last_update = time.time()

        self.cb = self.pi.callback(self.gpio_pin, pigpio.EITHER_EDGE, self._cb)

    def _cb(self, gpio, level, tick):
        if self.last_tick == 0:
            self.last_tick = tick
            self.level = level
            return

        dt = pigpio.tickDiff(self.last_tick, tick)
        self.last_tick = tick

        if self.level == 1:
            self.high_time = dt
        else:
            self.low_time = dt

        self.level = level

        total = self.high_time + self.low_time
        if total > 0:
            self.duty_cycle = (self.high_time / total) * 100
            self.last_update = time.time()

    def get_duty_cycle(self):
        # timeout de 1 segundo para sinal perdido
        if time.time() - self.last_update > 1:
            if self.pi.read(self.gpio_pin) == 1:
                return 100.0  # PWM travado em nível alto
            else:
                return 0.0  # PWM travado em nível baixo (0%)
        return round(self.duty_cycle, 2)

    def stop(self):
        self.cb.cancel()
        self.pi.stop()


class SimuladorTemperatura(threading.Thread):
    def __init__(self, pwm_reader):
        super().__init__(daemon=True)
        self.executando = True
        self.pwm_reader = pwm_reader
        self.temp_ambiente = 64
        self.temp_cpu = 64

    def parar(self):
        self.executando = False
        self.pwm_reader.stop()

    def run(self):
        while self.executando:
            duty = self.pwm_reader.get_duty_cycle()
            pwm_ativo = duty > 0

            bomba_on = bomba.is_active
            misturador_on = misturador.is_active

            if pwm_ativo:
                diff = self.temp_ambiente - self.temp_cpu
                if bomba_on and misturador_on:
                    # Cria diferença artificial: temp1 aumenta mais que temp2
                    delta1 = (duty / 100) * 1.2
                    delta2 = ((duty / 100) * 0.4) + diff * 0.05
                elif not bomba_on and not misturador_on:
                    # Convergência: reduz diferença entre temp1 e temp2
                    ajuste = (duty / 100) * 0.6

                    if diff > 0:
                        delta1 = -ajuste
                        delta2 = ajuste
                    elif diff < 0:
                        delta1 = ajuste
                        delta2 = -ajuste
                    else:
                        delta1 = delta2 = (duty / 100) * 0.5
                else:
                    # Caso intermediário: sobe normalmente
                    delta1 = delta2 = (duty / 100) * 0.6
            else:
                # PWM = 0 → Esfriamento padrão
                ambient_floor = 20.0  # mínimo realista
                delta1 = -0.2 if self.temp_ambiente > ambient_floor else 0
                if self.temp_cpu > self.temp_ambiente:
                    delta2 = -0.2
                else:
                    delta2 = -0.05

            self.temp_ambiente = max(0, min(100, self.temp_ambiente + delta1 * 0.1))
            self.temp_cpu = max(0, min(100, self.temp_cpu + delta2 * 0.1))

            global temperatura_simulada_1, temperatura_simulada_2
            temperatura_simulada_1 = self.temp_ambiente
            temperatura_simulada_2 = self.temp_cpu

            root.after(0, lambda: pwm_progressbar.config(value=duty))
            root.after(0, lambda: pwm_percentage_label.config(text=f"{duty:.1f}%"))

            root.after(0, lambda: label_temp_ambiente.config(text=f"Sensor Panela - Fundo: {self.temp_ambiente:.1f} °C"))
            root.after(0, lambda: label_temp_cpu.config(text=f"Sensor Panela - Superior: {self.temp_cpu:.1f} °C"))

            time.sleep(1)


simulador = None

try:
    from gpiozero import DigitalInputDevice

    raspberry_pi = True
except ImportError:
    raspberry_pi = False


    class DigitalInputDevice:
        def __init__(self, pin):
            self.state = False

        def is_active(self):
            return self.state

        def toggle(self):
            self.state = not self.state

# Inicializa UART
try:
    uart = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
except serial.SerialException:
    uart = None
    print("Erro ao abrir porta UART. Verifique se o dispositivo está conectado.")

PIN_BOMBA = 23
PIN_MISTURADOR = 24
PIN_ALERTA = 25

bomba = DigitalInputDevice(PIN_BOMBA)
misturador = DigitalInputDevice(PIN_MISTURADOR)
alerta = DigitalInputDevice(PIN_ALERTA)

root = tk.Tk()
root.title("Painel de Controle")
root.geometry("400x600")

simulador_var = tk.BooleanVar()
ocultar_logs_var = tk.BooleanVar()

style = ttk.Style()
style.configure("ProgressoVermelho.Horizontal.TProgressbar",
                troughcolor='lightgray',
                background='#E57373',
                bordercolor='gray',
                lightcolor='#FFCDD2',
                darkcolor='#D32F2F')


def toggle_simulador():
    global simulador
    if simulador_var.get():
        pwm_reader = PwmReader(gpio_pin=18)
        simulador = SimuladorTemperatura(pwm_reader)
        simulador.start()
    else:
        if simulador:
            simulador.parar()
            simulador = None


status_widgets = {}

# Define a consistent width for the "LED" indicators and the progress bar
LED_INDICATOR_WIDTH_PIXELS = 250
LED_INDICATOR_WIDTH_CHARS = 35

# --- FRAME PARA STATUS E AQUECEDOR ---
frame_status_e_aquecedor = tk.Frame(root)
# Posiciona o frame_status_e_aquecedor na janela principal
frame_status_e_aquecedor.grid(row=0, column=0, padx=10, pady=5, sticky="nw")

# Configura as colunas dentro de frame_status_e_aquecedor
# Coluna 0 para os nomes, coluna 1 para os indicadores/barra
frame_status_e_aquecedor.grid_columnconfigure(0, weight=1)  # Permite que a coluna 0 se expanda se necessário
frame_status_e_aquecedor.grid_columnconfigure(1, weight=0)  # Mantém a coluna 1 com largura fixa dos elementos


def criar_led_virtual(nome, row):
    # Label for the description (e.g., "Bomba:") - placed in column 0 of its FRAME, right-aligned
    desc_label = tk.Label(frame_status_e_aquecedor, text=f"{nome}:")
    desc_label.grid(row=row, column=0, pady=5, sticky="w")

    # Label for the ON/OFF indicator (the colored box) - placed in column 1 of its FRAME, left-aligned
    indicator_label = tk.Label(frame_status_e_aquecedor, text="OFF", bg="gray", fg="white",
                               width=LED_INDICATOR_WIDTH_CHARS,
                               relief="solid", borderwidth=1, anchor="center")
    indicator_label.grid(row=row, column=1, padx=(10, 0), pady=5, sticky="w")  # padx for separation

    status_widgets[nome] = {"desc_label": desc_label, "indicator_label": indicator_label}


# Criando LEDs virtuais DENTRO do frame_status_e_aquecedor
criar_led_virtual("Bomba", 0)
criar_led_virtual("Misturador", 1)
criar_led_virtual("Alerta", 2)

# PWM (Aquecedor) DENTRO do frame_status_e_aquecedor
tk.Label(frame_status_e_aquecedor, text="Aquecedor:").grid(row=3, column=0, pady=5, sticky="e")

# Frame para agrupar a barra de progresso e o percentual, DENTRO do frame_status_e_aquecedor
frame_pwm_indicator = tk.Frame(frame_status_e_aquecedor)
frame_pwm_indicator.grid(row=3, column=1, padx=(10, 0), pady=5, sticky="w")

pwm_progressbar = ttk.Progressbar(frame_pwm_indicator, orient="horizontal",
                                  length=LED_INDICATOR_WIDTH_PIXELS,
                                  mode="determinate",
                                  style="ProgressoVermelho.Horizontal.TProgressbar")
pwm_progressbar.pack(side="left")

pwm_percentage_label = tk.Label(frame_pwm_indicator, text="-- %")
pwm_percentage_label.pack(side="left", padx=(5, 0))


def atualizar_status():
    bomba_estado = bomba.is_active
    misturador_estado = misturador.is_active
    alerta_estado = alerta.is_active

    status_widgets["Bomba"]["indicator_label"].config(
        text='ON' if bomba_estado else 'OFF',
        bg="green" if bomba_estado else "gray"
    )
    status_widgets["Misturador"]["indicator_label"].config(
        text='ON' if misturador_estado else 'OFF',
        bg="blue" if misturador_estado else "gray"
    )
    status_widgets["Alerta"]["indicator_label"].config(
        text='ON' if alerta_estado else 'OFF',
        bg="red" if alerta_estado else "gray"
    )

    if not simulador_var.get():
        pwm_progressbar.config(value=0)
        pwm_percentage_label.config(text="-- %")

        label_temp_ambiente.config(text="Sensor Panela - Fundo: -- °C")
        label_temp_cpu.config(text="Sensor Panela - Superior: -- °C")

    root.after(1000, atualizar_status)


def enviar_uart(comando):
    if uart and uart.is_open:
        uart.write(f"{comando}\n".encode())
        print(f"Enviado: {comando}")
    else:
        print("UART não está disponível.")


# Os elementos abaixo ainda estão diretamente no root, mas suas linhas serão separadas do frame_status_e_aquecedor
check_simulador = tk.Checkbutton(root, text="Ativar Simulador I2C", variable=simulador_var, command=toggle_simulador)
check_simulador.grid(row=5, column=0, columnspan=2, pady=5, sticky="w", padx=10)

label_temp_ambiente = tk.Label(root, text="Sensor Panela - Fundo: -- °C")
label_temp_ambiente.grid(row=6, column=0, columnspan=2, pady=2, sticky="w", padx=10)

label_temp_cpu = tk.Label(root, text="Sensor Panela - Superior: -- °C")
label_temp_cpu.grid(row=7, column=0, columnspan=2, pady=2, sticky="w", padx=10)


def abrir_popup_curva():
    popup = tk.Toplevel(root)
    popup.title("Editar Curva de Temperatura")
    popup.geometry("400x500")

    pontos = []

    lista = tk.Listbox(popup, width=40, height=10)
    lista.pack(pady=5)

    frame_sliders = tk.Frame(popup)
    frame_sliders.pack(pady=10)

    tk.Label(frame_sliders, text="Temperatura (°C)").grid(row=0, column=0, columnspan=2)
    temp_slider = tk.Scale(frame_sliders, from_=0, to=100, orient="horizontal", length=300, resolution=1)
    temp_slider.set(25)
    temp_slider.grid(row=1, column=0, columnspan=2, padx=5, pady=5)

    tk.Label(frame_sliders, text="Minutos").grid(row=2, column=0)
    tk.Label(frame_sliders, text="Segundos").grid(row=2, column=1)

    min_slider = tk.Scale(frame_sliders, from_=1, to=60, orient="horizontal", length=144)
    min_slider.grid(row=3, column=0, padx=5)

    sec_slider = tk.Scale(frame_sliders, from_=0, to=59, orient="horizontal", length=140)
    sec_slider.grid(row=3, column=1, padx=5)

    def adicionar_ponto():
        if not pontos:
            min_slider.set(0)
            sec_slider.set(0)

        if len(pontos) >= 8:
            messagebox.showwarning("Limite atingido", "Você só pode adicionar até 8 pontos.")
            return

        temp = temp_slider.get()
        minutos = min_slider.get()
        segundos = sec_slider.get()

        if pontos and (minutos == 0 and segundos == 0):
            messagebox.showwarning("Tempo Inválido", "Para pontos subsequentes, o tempo não pode ser 00:00.")
            return

        tempo_formatado = f"{minutos:02d}:{segundos:02d}"
        ponto = {"temperatura": temp, "tempo": tempo_formatado}
        pontos.append(ponto)
        lista.insert(tk.END, f"{temp}°C por {tempo_formatado}")

    def enviar_curva():
        if not pontos:
            messagebox.showinfo("Aviso", "Nenhum ponto adicionado.")
            return
        curva = {"curva_temperatura": pontos}
        comando_json = json.dumps(curva)
        enviar_uart(comando_json)
        popup.destroy()

    def salvar_csv():
        if not pontos:
            messagebox.showinfo("Aviso", "Nenhum ponto para salvar.")
            return

        pasta_curvas = os.path.join(os.getcwd(), "curvas")
        os.makedirs(pasta_curvas, exist_ok=True)

        caminho = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialdir=pasta_curvas,
            title="Salvar Curva de Temperatura"
        )

        if caminho:
            with open(caminho, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Temperatura", "Tempo"])
                for ponto in pontos:
                    writer.writerow([ponto["temperatura"], ponto["tempo"]])
            messagebox.showinfo("Salvo", f"Curva salva com sucesso em:\n{caminho}")

    btns = tk.Frame(popup)
    btns.pack(pady=10)

    tk.Button(btns, text="Adicionar Ponto", command=adicionar_ponto, width=18).grid(row=0, column=0, padx=5, pady=5)
    tk.Button(btns, text="Enviar Curva", command=enviar_curva, width=18).grid(row=1, column=0, padx=5, pady=5)
    tk.Button(btns, text="Salvar CSV", command=salvar_csv, width=18).grid(row=2, column=0, padx=5, pady=5)


def abrir_popup_importar_curva():
    popup = tk.Toplevel(root)
    popup.title("Importar Curva de Temperatura")
    popup.geometry("450x400")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    pasta_curvas = os.path.join(script_dir, "curvas")
    os.makedirs(pasta_curvas, exist_ok=True)

    arquivos = [f for f in os.listdir(pasta_curvas) if f.endswith('.csv')]

    tk.Label(popup, text="Selecione uma curva:").pack(pady=5)
    lista_arquivos = tk.Listbox(popup, width=40, height=8)
    lista_arquivos.pack(pady=5)

    for nome in arquivos:
        lista_arquivos.insert(tk.END, nome)

    txt_exibicao = ScrolledText(popup, width=50, height=10, state='disabled')
    txt_exibicao.pack(pady=10)

    curva_carregada = []

    def visualizar():
        nonlocal curva_carregada
        curva_carregada = []

        selecionado = lista_arquivos.curselection()
        if not selecionado:
            messagebox.showinfo("Aviso", "Nenhum arquivo selecionado.")
            return

        nome_arquivo = lista_arquivos.get(selecionado[0])
        caminho = os.path.join(pasta_curvas, nome_arquivo)

        try:
            with open(caminho, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                txt_exibicao.configure(state='normal')
                txt_exibicao.delete(1.0, tk.END)

                for row in reader:
                    temp = float(row["Temperatura"])
                    tempo = row["Tempo"]
                    curva_carregada.append({"temperatura": temp, "tempo": tempo})
                    txt_exibicao.insert(tk.END, f"{temp}°C por {tempo}\n")

                txt_exibicao.configure(state='disabled')

        except Exception as e:
            messagebox.showerror("Erro", f"Erro ao ler arquivo: {e}")

    def enviar():
        if not curva_carregada:
            messagebox.showinfo("Aviso", "Nenhuma curva carregada.")
            return
        curva = {"curva_temperatura": curva_carregada}
        comando_json = json.dumps(curva)
        enviar_uart(comando_json)
        popup.destroy()

    frame_botoes = tk.Frame(popup)
    frame_botoes.pack(pady=5)

    tk.Button(frame_botoes, text="Visualizar", command=visualizar, width=20).grid(row=0, column=0, padx=5)
    tk.Button(frame_botoes, text="Enviar Curva", command=enviar, width=20).grid(row=0, column=1, padx=5)


frame_configuracao = tk.LabelFrame(root, text="Configuração")
frame_configuracao.grid(row=8, column=0, columnspan=2, padx=10, pady=10, sticky="w")


def abrir_popup_configurar():
    enviar_uart('CFG')
    popup_cfg = tk.Toplevel(root)
    popup_cfg.title("Configurar Curva")
    popup_cfg.geometry("300x150")

    def add_curva_action():
        enviar_uart('ADD')
        popup_cfg.destroy()
        abrir_popup_curva()

    def importar_curva_action():
        enviar_uart('EDIT')
        popup_cfg.destroy()
        abrir_popup_importar_curva()

    def voltar_action():
        enviar_uart('STOP')
        popup_cfg.destroy()

    tk.Button(popup_cfg, text="ADD. CURVA", width=15, command=add_curva_action).pack(pady=5)
    tk.Button(popup_cfg, text="IMPORTAR", width=15, command=importar_curva_action).pack(pady=5)
    tk.Button(popup_cfg, text="VOLTAR", width=15, command=voltar_action).pack(pady=5)


def abrir_popup_iniciar():
    enviar_uart('OPR')
    popup_iniciar = tk.Toplevel(root)
    popup_iniciar.title("Iniciar Operação")
    popup_iniciar.geometry("300x150")

    def padrao_action():
        enviar_uart('DEFAULT')
        popup_iniciar.destroy()

    def customizado_action():
        enviar_uart('CUSTOM')
        popup_iniciar.destroy()

    def voltar_action():
        enviar_uart('STOP')
        popup_iniciar.destroy()

    tk.Button(popup_iniciar, text="PADRÃO", width=15, command=padrao_action).pack(pady=5)
    tk.Button(popup_iniciar, text="CUSTOMIZADO", width=15, command=customizado_action).pack(pady=5)
    tk.Button(popup_iniciar, text="VOLTAR", width=15, command=voltar_action).pack(pady=5)


btn_configurar = tk.Button(frame_configuracao, text='CONFIGURAR', width=12, command=abrir_popup_configurar)
btn_configurar.grid(row=0, column=0, padx=5, pady=5)

btn_iniciar = tk.Button(frame_configuracao, text='Iniciar', width=12, command=abrir_popup_iniciar)
btn_iniciar.grid(row=0, column=1, padx=5, pady=5)

# --- NOVO FRAME PARA OS LOGS UART ---
frame_logs = tk.Frame(root)
# Posiciona o frame_logs na janela principal (em uma linha diferente)
frame_logs.grid(row=10, column=0, columnspan=2, padx=10, pady=10,
                sticky="ew")  # sticky="ew" para que o frame preencha a largura disponível

uart_output = ScrolledText(frame_logs, width=45, height=10, state='disabled', wrap='word')
uart_output.pack(pady=0, fill="both", expand=True)  # Usar pack dentro do frame_logs

btn_limpar = tk.Button(frame_logs, text="Limpar",
                       command=lambda: uart_output.configure(state='normal') or uart_output.delete(1.0,
                                                                                                   tk.END) or uart_output.configure(
                           state='disabled'))
btn_limpar.pack(pady=(5, 0), anchor="w")  # Usar pack dentro do frame_logs, ancorado à esquerda


def toggle_logs():
    if ocultar_logs_var.get():
        frame_logs.grid_remove()  # Remove o frame inteiro
    else:
        frame_logs.grid()  # Mostra o frame inteiro


check_ocultar_logs = tk.Checkbutton(root, text="Ocultar Logs", variable=ocultar_logs_var, command=toggle_logs)
check_ocultar_logs.grid(row=9, column=0, columnspan=2, pady=5, sticky="w", padx=10)


def mostrar_mensagem_uart(msg):
    uart_output.configure(state='normal')
    uart_output.insert(tk.END, msg + '\n')
    uart_output.see(tk.END)
    uart_output.configure(state='disabled')


def escutar_uart():
    while True:
        if uart and uart.is_open:
            try:
                linha = uart.readline().decode().strip()
                if linha:
                    print(f"Recebido: {linha}")
                    mostrar_mensagem_uart(linha)

                    if linha.upper() == "TEMPERATURA1":
                        uart.write(f"{temperatura_simulada_1:.1f}\n".encode())
                    elif linha.upper() == "TEMPERATURA2":
                        uart.write(f"{temperatura_simulada_2:.1f}\n".encode())
            except Exception as e:
                mostrar_mensagem_uart(f"Erro leitura UART: {e}")
        time.sleep(0.1)


atualizar_status()
threading.Thread(target=escutar_uart, daemon=True).start()

root.mainloop()
