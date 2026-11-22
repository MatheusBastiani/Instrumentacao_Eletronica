# Sistema de Instrumentação Analógica para Monitoramento de CE, pH e Temperatura

> **Projeto de Instrumentação Eletrônica (EEL 7300)**  
> **Autores:** Carlos Henrique Braatz Lautert e Luis Felipe Candido Sola  
> **Plataforma:** ESP32 (ESP-IDF v6.0) + ADS1115 + Cadeia Analógica Discreta

---

## Sobre o Projeto

Este repositório contém o firmware e a documentação de operação de um instrumento eletrônico desenvolvido para medir **Condutividade Elétrica (CE)**, **pH** e **Temperatura** em amostras da solução nutritiva aplicada e do drenado no cultivo semi-hidropônico de morango.

O projeto prioriza uma **cadeia analógica discreta completa** para o condicionamento do canal de condutividade elétrica (excitação alternada sem componente CC, amplificador de transimpedância, ganho selecionável, retificador de precisão de onda completa e filtro passa-baixas), integrada a um conversor A/D de 16 bits (`ADS1115`) e a sensores auxiliares para compensação térmica em tempo real.

---

## Arquitetura do Sistema

O instrumento opera com alimentação simples a partir da porta USB (5 V), dividida entre o domínio digital (3,3 V) e o domínio analógico (`AVDD`, com terra virtual `VREF`):

1. **Canal de Condutividade Elétrica (CE — Cadeia Analógica Discreta):**
   * **Oscilador Astável:** Disparador Schmitt (`MCP6004`) gerando onda quadrada a **2,38 kHz** (R₄ = 19,1 kΩ, C₅ = 10 nF).
   * **Driver e Acoplamento AC:** Atenuador para **0,300 Vpp** em torno de \(V_{\text{REF}}\), seguidor de tensão (`MCP6002`), jumper de habilitação (`J1` / `JP2`) e capacitor de bloqueio CC (C₆ = 100 μF) para evitar polarização eletroquímica nos eletrodos.
   * **Amplificador de Transimpedância (TIA):** Converte a corrente da célula (15 μA a 750 μA) em tensão com resistor de realimentação R₉ = 1,000 kΩ e compensação C₇ = 220 pF.
   * **Ganho Selecionável (`J2` / `JP1`):**
     * **Faixa `LOW` (`JP1` fechado):** Ganho \(G_A = 5,02\) para soluções diluídas (0,10 a 1,00 mS/cm).
     * **Faixa `HIGH` (`JP1` aberto):** Ganho \(G_A = 1,00\) para soluções concentradas (1,00 a 5,00 mS/cm).
   * **Retificador de Precisão e Filtro:** Retificador de onda completa (`LMC6482` + diodos `1N4148`) seguido de filtro RC passa-baixas (R₁₇ = 15 kΩ, C₈ = 10 μF, \(f_c \approx 1,06\,\text{Hz}\)).
   * **Aquisição Diferencial:** Leitura diferencial `AIN2 - AIN3` (\(V_{\text{FILT}} - V_{\text{REF}}\)) no `ADS1115` em 16 bits.

2. **Canal de pH:**
   * Eletrodo combinado conectado ao módulo condicionador `PH-4502C` (alimentado em 5 V), com saída \(P_o\) reduzida por divisor resistivo de precisão 2:1 (10 kΩ + 10 kΩ) antes da leitura no canal `AIN0` do `ADS1115`.
   * Compensação automática da inclinação de Nernst (S(T) = 0,1984 ⋅ T mV/pH) utilizando a temperatura medida em tempo real.

3. **Canal de Temperatura:**
   * Sensor digital à prova d'água `DS18B20` operando no barramento **1-Wire** com resolução de 12 bits (0,0625 °C) e resistor de *pull-up* externo de 4,7 kΩ.
   * Utilizado tanto para a inclinação de Nernst no pH quanto para referenciar a condutividade elétrica a 25 °C (\(\kappa_{25} = \frac{\kappa_T}{1 + \alpha(T - 25)}\), com α = 0,019 °C⁻¹).

---

## Pinagem e Conexões (ESP32 WROOM-32)

| Periférico / Sinal | Pino no Módulo | GPIO no ESP32 | Observações |
| :--- | :--- | :--- | :--- |
| **ADS1115 — SDA** | `SDA` | `GPIO 21` (`D21`) | Barramento I2C (100 kHz) |
| **ADS1115 — SCL** | `SCL` | `GPIO 22` (`D22`) | Barramento I2C (100 kHz) |
| **ADS1115 — Endereço** | `ADDR` | `GND` | Endereço I2C fixado em `0x48` |
| **ADS1115 — Canal pH** | `A0` | — | Saída `PH_DIV` (após divisor 10 kΩ / 10 kΩ) |
| **ADS1115 — Canal CE (+)** | `A2` | — | Saída filtrada do retificador (`CE_FILT`) |
| **ADS1115 — Canal CE (-)** | `A3` | — | Referência analógica (`VREF`) |
| **DS18B20 — Dados** | Fio Amarelo (`DQ`) | `GPIO 4` (`D4`) | Pull-up de 4,7 kΩ ligado ao `3V3` |
| **DS18B20 — Alimentação** | Fio Vermelho / Preto | `3V3` / `GND` | Domínio digital 3,3 V |
| **Módulo pH (`PH-4502C`)** | `V+` / `G` | `VIN` (`5V`) / `GND` | Alimentado em 5 V |

---

## Como Compilar e Gravar (ESP-IDF v6.0)

### 1. Pré-requisitos
* **ESP-IDF v6.0** instalado e configurado no ambiente (terminal ESP-IDF ou CLion / VS Code).

### 2. Arquivo `main/CMakeLists.txt`
Certifique-se de que o `CMakeLists.txt` da pasta `main` declara as dependências do driver I2C, GPIO e Timer:

```cmake
idf_component_register(SRCS "generic_esp32.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_timer)
```

### 3. Compilação e Flash
No terminal do ESP-IDF, execute:

```bash
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```
*(Substitua `COMx` pela porta serial do seu ESP32, ex.: `COM3` ou `/dev/ttyUSB0`).*

---

## Procedimento de Operação e Calibração via Serial

Para evitar interferência elétrica entre a excitação AC do canal de condutividade e a entrada de alta impedância do eletrodo de pH, **realize as medições sequencialmente (uma sonda por vez na amostra)**.

### Comandos Disponíveis no Monitor Serial (115200 bps)

| Tecla | Função | Procedimento |
| :---: | :--- | :--- |
| **`7`** | Seleciona tampão **pH 7,00** | Define o valor alvo de calibração para `7.00`. |
| **`4`** | Seleciona tampão **pH 4,00** | Define o valor alvo de calibração para `4.00`. |
| **`9`** | Seleciona tampão **pH 9,18** | Define o valor alvo de calibração para `9.18`. |
| **`c`** | **Calibra pH** | Com o jumper `J1` aberto e a sonda estabilizada no tampão selecionado, grava a tensão atual (`V_mod`) como referência. |
| **`r`** | **Alterna faixa de CE** | Alterna o cálculo entre `[HIGH]` (`JP1` aberto, ganho 1) e `[LOW]` (`JP1` fechado, ganho 5,02). |
| **`z`** | **Zera CE no ar** | Com `J1` fechado e a célula **seca fora da água**, grava o deslocamento residual do retificador como `0.000 mS/cm`. |

### Roteiro Rápido de Bancada
1. **Calibração de pH:** Abra o jumper `J1` (`CE ENABLE`), enxágue a sonda de pH, mergulhe no tampão **pH 7,00**, aguarde ~20 s e envie `7` seguido de `c`.
2. **Zero de CE:** Retire a sonda de pH da amostra, conecte os eletrodos de CE (`CEL1` e `CEL2`), feche o jumper `J1` (`CE ENABLE`) e, com a célula seca no ar na faixa desejada, envie `z`.
3. **Leitura da Amostra:** Mergulhe a célula de CE cobrindo totalmente os eletrodos e aguarde ~5 s para acomodação do filtro passa-baixas (τ = 0,15 s).
4. **Calibração da Constante da Célula (\(K_{\text{ef}}\)):** Ao medir uma solução padrão certificada (ex.: 1413 μS/cm = 1,413 mS/cm), ajuste `k_cell_eff` no código pela relação:
   \[K_{\text{ef}} = \frac{\kappa_{\text{padrão}}}{\kappa_{25,\text{lido}}}\]

---

## Exemplo de Saída no Monitor Serial

```text
I (291) EEL7300_FULL: Sistema iniciado (Temp + pH + CE + VREF).
I (291) EEL7300_FULL: Comandos: '7'/'4'/'9' tampao | 'c' calibra pH | 'r' faixa CE | 'z' zera CE no ar.
TEMP: OK (25.06 C) | pH: 7.00 (V_mod=3.272 V) | V_ref: 2.020 V | CE_25: 0.000 mS/cm (V_diff=1.2320 V) [HIGH]
TEMP: OK (25.06 C) | pH: 7.00 (V_mod=3.272 V) | V_ref: 2.040 V | CE_25: 0.120 mS/cm (V_diff=1.2140 V) [HIGH]
```
