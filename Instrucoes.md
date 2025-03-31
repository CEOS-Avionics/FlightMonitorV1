
### Dependências:
- As seguintes libs são usadas no projeto, porém não foi possível realizar o dowload diretamente pela Arduino IDE:
- **[SparkFun_ICM-20948_ArduinoLibrary](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)**
- **[kalman](https://github.com/mherb/kalman)**
- Eigen 3.7 (a versão 3.4.0 não foi compatível com a lib kalman)
### Calibragem do ICM-20948:
- Faça upload do código de exemplo `Example11_DMP_Bias_Save_Restore_ESP32`
- Siga as instruções que irão aparecer no monitor serial. Os biases do icm serão coletados em forma de valores brutos e armazenados na EEPROM da ESP32.
- É esperado que os biases do sensor se mantenham constantes durante alguns minutos, por isso é recomendado calibrá-lo um pouco antes da decolagem.
- Os biases estão incluídos no vetor de estados do filtro de Kalman,  portanto é esperado que se ajustem se sofrerem alguma mudança.
### Arquivo main:
- Altere as linhas 12, 13, 14, 15 e 16 para que incluam os caminhos que os respectivos arquivos têm no seu computador. 
- Altere também as linhas 66, 67 e 68 para que especifiquem a sua rede WiFi, a senha da rede e o endereço IP do computador que está se comunicando com a ESP32, respectivamente. Para achar este último basta fornecer o comando ifconfig no terminal do Linux (ou ipconfig no Windows).
- Faça o upload do código no controlador.

### Mensagens UDP:
- Em um terminal Linux forneça o comando ``nc -u -l <usdPort>`` para escutar mensagens enviadas pelo controlador de voo. usdPort é a mesma definida na linha 69 de main.ino. 
- Abra outra janela do terminal Linux. O mesmo comando deverá ser fornecido duas vezes. A primeira, quando o foguete se encontrar no local de lançamento, devendo este permanecer completamente estático até o lançamento. A segunda é logo antes do lançamento, após o recebimento da mensagem "Foguete pronto para o lançamento!" no primeiro terminal. 
- O comando é o seguinte: ``echo "y" | nc -u <ip_da_ESP32> <usdPort>``. usdPort continua a mesma, ip_da_ESP32 é fornecido no primeiro terminal assim que o controlador se conecta à rede WiFi.  Ele envia a flag 'y' para o controlador.
- A primeira flag diz ao controlador que ele deve determinar o referencial absoluto, a segunda inicia filtro de Kalman. 

### Ler e visualizar dados:
- Para visualização dos dados guardados no cartão SD é preciso conectar o controlador de voo ao PC por uma porta USB.
- Faça o upload do arquivo lerSD.
- Em seguida execute o código visualizeKalman.py no VSCode. Um arquivo csv será gerado na mesma pasta em que se encontra salva visualizeKalman.py. Se quiser visualizar gráficos basta descomentar linhas no código. 