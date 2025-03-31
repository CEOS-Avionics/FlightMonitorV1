import serial
import time
import csv
import matplotlib.pyplot as plt

# Recebe os dados printados na serial pela ESP32 por meio do código lerSD.ino.
def receber_dados():

    porta = 'COM18'
    baud_rate = 9600
    flag = 'r'

    biases = []
    t = []
    state = []
    samples = []

    # Inicia a comunicacao serial criando uma variavel pra representar ela.
    conexao_serial = serial.Serial(porta, baud_rate, timeout=1)
    print(f"conectando em {porta}")
    time.sleep(1)

    # Envia a flag para a ESP32, sinalizando que pode iniciar a leitura do cartão SD.
    while conexao_serial.in_waiting == 0:
        conexao_serial.write(flag.encode('utf-8'))
        print("aguardando dados")
        time.sleep(1)

    # Pega a primeira linha do arquivo que contém os biases.
    if conexao_serial.in_waiting != 0:

        biasesstr = conexao_serial.readline().decode('utf-8').strip().split(',')
        biases = [float(i) for i in biasesstr]

    while conexao_serial.in_waiting != 0:

        timestr = conexao_serial.readline().decode('utf-8').strip().split(',')
        statestr = conexao_serial.readline().decode('utf-8').strip().split(',')
        samplestr = conexao_serial.readline().decode('utf-8').strip().split(',')

        t.append(float(timestr))
        state.append([float(i) for i in statestr])
        samples.append([float(i) for i in samplestr])

    return biases, t, state, samples

# A inteção é receber state e samples, mas organiza-los em matrizes de forma que cada coluna corresponde a uma grandeza.
def organizar_em_mmatriz(state, samples):
    Mstate = []
    Msamples = []

    # Divide o vetor state em seções de 28 elementos e adiciona como linhas em Mstate
    for i in range(0, len(state), 28):
        Mstate.append(state[i:i + 28])

    # Divide o vetor samples em seções de 15 elementos e adiciona como linhas em Msamples
    for i in range(0, len(samples), 15):
        Msamples.append(samples[i:i + 15])

    return Mstate, Msamples

def salvar_em_csv(biases, t, Mstate, Msamples, nome_arquivo):
    with open(nome_arquivo, mode='w', newline='') as arquivo_csv:
        escritor = csv.writer(arquivo_csv)

        # Escreve os biases na primeira linha
        escritor.writerow(biases)

        # Itera sobre os elementos de t, Mstate e Msamples
        for i in range(len(t)):
            # Escreve o elemento de t
            escritor.writerow([t[i]])

            # Escreve a seção correspondente de Mstate
            if i < len(Mstate):
                escritor.writerow(Mstate[i])

            # Escreve a seção correspondente de Msamples
            if i < len(Msamples):
                escritor.writerow(Msamples[i])

def visualizar_dados(t, Mstate, Msamples):

    plt.figure(figsize=(12, 8))

    # Colunas de Mstate, descomente para exibir no grafico.
    # plt.plot(t, [row[0] for row in Mstate], label='Q0 - Quaternion parte 1')  # Quaternion parte 1
    # plt.plot(t, [row[1] for row in Mstate], label='Q1 - Quaternion parte 2')  # Quaternion parte 2
    # plt.plot(t, [row[2] for row in Mstate], label='Q2 - Quaternion parte 3')  # Quaternion parte 3
    # plt.plot(t, [row[3] for row in Mstate], label='Q3 - Quaternion parte 4')  # Quaternion parte 4
    # plt.plot(t, [row[4] for row in Mstate], label='WX - Velocidade angular X')  # Velocidade angular X
    # plt.plot(t, [row[5] for row in Mstate], label='WY - Velocidade angular Y')  # Velocidade angular Y
    # plt.plot(t, [row[6] for row in Mstate], label='WZ - Velocidade angular Z')  # Velocidade angular Z
    # plt.plot(t, [row[7] for row in Mstate], label='PX - Posição X (NED)')  # Posição X (NED)
    # plt.plot(t, [row[8] for row in Mstate], label='PY - Posição Y (NED)')  # Posição Y (NED)
    # plt.plot(t, [row[9] for row in Mstate], label='PZ - Posição Z (NED)')  # Posição Z (NED)
    # plt.plot(t, [row[10] for row in Mstate], label='VX - Velocidade X (NED)')  # Velocidade X (NED)
    # plt.plot(t, [row[11] for row in Mstate], label='VY - Velocidade Y (NED)')  # Velocidade Y (NED)
    # plt.plot(t, [row[12] for row in Mstate], label='VZ - Velocidade Z (NED)')  # Velocidade Z (NED)
    # plt.plot(t, [row[13] for row in Mstate], label='AX - Aceleração X (NED)')  # Aceleração X (NED)
    # plt.plot(t, [row[14] for row in Mstate], label='AY - Aceleração Y (NED)')  # Aceleração Y (NED)
    # plt.plot(t, [row[15] for row in Mstate], label='AZ - Aceleração Z (NED)')  # Aceleração Z (NED)
    # plt.plot(t, [row[16] for row in Mstate], label='ABX - Bias acelerômetro X')  # Bias acelerômetro X
    # plt.plot(t, [row[17] for row in Mstate], label='ABY - Bias acelerômetro Y')  # Bias acelerômetro Y
    # plt.plot(t, [row[18] for row in Mstate], label='ABZ - Bias acelerômetro Z')  # Bias acelerômetro Z
    # plt.plot(t, [row[19] for row in Mstate], label='GBX - Bias giroscópio X')  # Bias giroscópio X
    # plt.plot(t, [row[20] for row in Mstate], label='GBY - Bias giroscópio Y')  # Bias giroscópio Y
    # plt.plot(t, [row[21] for row in Mstate], label='GBZ - Bias giroscópio Z')  # Bias giroscópio Z
    # plt.plot(t, [row[22] for row in Mstate], label='MX - Campo geomagnético X (NED)')  # Campo geomagnético X (NED)
    # plt.plot(t, [row[23] for row in Mstate], label='MY - Campo geomagnético Y (NED)')  # Campo geomagnético Y (NED)
    # plt.plot(t, [row[24] for row in Mstate], label='MZ - Campo geomagnético Z (NED)')  # Campo geomagnético Z (NED)
    # plt.plot(t, [row[25] for row in Mstate], label='MBX - Bias magnetômetro X')  # Bias magnetômetro X
    # plt.plot(t, [row[26] for row in Mstate], label='MBY - Bias magnetômetro Y')  # Bias magnetômetro Y
    # plt.plot(t, [row[27] for row in Mstate], label='MBZ - Bias magnetômetro Z')  # Bias magnetômetro Z

    # Colunas de Msamples, descomente para exibir no grafico.
    # plt.plot(t, [row[0] for row in Msamples], label='Altitude BMP')  # Altitude BMP
    # plt.plot(t, [row[1] for row in Msamples], label='Posição X (NED)')  # Posição X (NED)
    # plt.plot(t, [row[2] for row in Msamples], label='Posição Y (NED)')  # Posição Y (NED)
    # plt.plot(t, [row[3] for row in Msamples], label='Altitude GPS')  # Altitude GPS
    # plt.plot(t, [row[4] for row in Msamples], label='Heading (Orientação do movimento horizontal)')  # Heading
    # plt.plot(t, [row[5] for row in Msamples], label='Velocidade horizontal')  # Velocidade horizontal
    # plt.plot(t, [row[6] for row in Msamples], label='AX - Aceleração X (referencial XYZ)')  # Accelerometer X
    # plt.plot(t, [row[7] for row in Msamples], label='AY - Aceleração Y (referencial XYZ)')  # Accelerometer Y
    # plt.plot(t, [row[8] for row in Msamples], label='AZ - Aceleração Z (referencial XYZ)')  # Accelerometer Z
    # plt.plot(t, [row[9] for row in Msamples], label='WX - Velocidade angular X (referencial XYZ)')  # Gyroscope X
    # plt.plot(t, [row[10] for row in Msamples], label='WY - Velocidade angular Y (referencial XYZ)')  # Gyroscope Y
    # plt.plot(t, [row[11] for row in Msamples], label='WZ - Velocidade angular Z (referencial XYZ)')  # Gyroscope Z
    # plt.plot(t, [row[12] for row in Msamples], label='MX - Campo magnético X (referencial XYZ)')  # Magnetometer X
    # plt.plot(t, [row[13] for row in Msamples], label='MY - Campo magnético Y (referencial XYZ)')  # Magnetometer Y
    # plt.plot(t, [row[14] for row in Msamples], label='MZ - Campo magnético Z (referencial XYZ)')  # Magnetometer Z

    # Configurações do gráfico
    plt.xlabel('Tempo (t)')
    plt.ylabel('Valores')
    plt.title('Visualização das Colunas de Mstate')
    plt.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))  # Ajusta a posição da legenda
    plt.grid()
    plt.tight_layout()
    plt.show()

biases, t, state, samples = receber_dados()
Mstate, Msamples = organizar_em_mmatriz(state, samples)
salvar_em_csv(biases, t, Mstate, Msamples, 'kalman.csv')
visualizar_dados(t, Mstate, Msamples)

