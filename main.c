/*
* Arquitetura e Aplicação de sistemas - exercicio 04 - Leitor de temperatura interna do MCU RP2040 Exibindo no display LCD
* Aluno: Victor Hugo de Toledo Nunes
* Prof.: Gustavo Ferreira Palma 
*/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"

// Definições para o LCD
#define LCD_LIMPA_TELA     0x01
#define LCD_INICIA         0x02
#define LCD_ENTRYMODESET   0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_FUNCTIONSET    0x20

#define LCD_INICIO_ESQUERDA 0x02
#define LCD_LIGA_DISPLAY    0x04
#define LCD_16x2            0x08
#define LCD_BACKLIGHT       0x08
#define LCD_ENABLE_BIT      0x04

// Endereço do display LCD no barramento I2C
#define BUS_ADDR 0x27

// Modos para lcd_envia_byte
#define LCD_CARACTER  1
#define LCD_COMANDO   0

#define DELAY_US 600

// Funções para controle do LCD
void lcd_envia_comando(uint8_t val) {
    i2c_write_blocking(i2c_default, BUS_ADDR, &val, 1, false);
}

void lcd_pulsa_enable(uint8_t val) {
    sleep_us(DELAY_US);
    lcd_envia_comando(val | LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
    lcd_envia_comando(val & ~LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
}

void lcd_envia_byte(uint8_t caractere, int modo) {
    uint8_t nible_high = modo | (caractere & 0xF0) | LCD_BACKLIGHT;
    uint8_t nible_low = modo | ((caractere << 4) & 0xF0) | LCD_BACKLIGHT;

    lcd_envia_comando(nible_high);
    lcd_pulsa_enable(nible_high);
    lcd_envia_comando(nible_low);
    lcd_pulsa_enable(nible_low);
}

void lcd_limpa_tela(void) {
    lcd_envia_byte(LCD_LIMPA_TELA, LCD_COMANDO);
}

void lcd_posiciona_cursor(int linha, int coluna) {
    int aux = (linha == 0) ? 0x80 + coluna : 0xC0 + coluna;
    lcd_envia_byte(aux, LCD_COMANDO);
}

void lcd_envia_string(const char *s) {
    while (*s) {
        lcd_envia_byte(*s++, LCD_CARACTER);
    }
}

void lcd_init() {
    lcd_envia_byte(LCD_INICIA, LCD_COMANDO);
    lcd_envia_byte(LCD_INICIA | LCD_LIMPA_TELA, LCD_COMANDO);
    lcd_envia_byte(LCD_ENTRYMODESET | LCD_INICIO_ESQUERDA, LCD_COMANDO);
    lcd_envia_byte(LCD_FUNCTIONSET | LCD_16x2, LCD_COMANDO);
    lcd_envia_byte(LCD_DISPLAYCONTROL | LCD_LIGA_DISPLAY, LCD_COMANDO);
    lcd_limpa_tela();
}

// Função para leitura da temperatura
float read_onboard_temperature() {
    const float conversionFactor = 3.3f / (1 << 12); // Conversão para ADC de 12 bits com ADC_VREF = 3.3V
    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;
    return tempC;
}

int main() {
    stdio_init_all();
    sleep_ms(5000); // Aguarda inicialização

    // Inicializa I2C e LCD
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    lcd_init();

    // Inicializa ADC para sensor de temperatura interno
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    char buffer[16]; // Buffer para exibição de texto
    while (true) {
        float temperature = read_onboard_temperature();
        snprintf(buffer, sizeof(buffer), "Temp: %.2f C", temperature); // Formata a string

        // Exibe a temperatura no display LCD
        lcd_limpa_tela();
        lcd_posiciona_cursor(0, 0);
        lcd_envia_string(buffer);

        sleep_ms(1000); // Atualiza a cada segundo
    }

    return 0;
}
