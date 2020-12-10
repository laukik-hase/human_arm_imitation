#include "i2c_utils.h"

static const char *TAG_I2C = "i2c_utils";

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    esp_err_t ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK)
        ESP_LOGE(TAG_I2C, "I2C Master Initialisation Failed!");
    return ret;
}

esp_err_t i2c_check_slave(uint8_t slave_addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
        ESP_LOGI(TAG_I2C, "Slave found: 0x%x", slave_addr);
    else
        ESP_LOGE(TAG_I2C, "Slave not found: 0x%x!", slave_addr);

    return ret;
}

int i2c_write_to_slave(uint8_t slave_addr, uint8_t reg_addr, size_t len, uint8_t *data_wr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
        return 0;
    else
        return -1;
}

int i2c_read_from_slave(uint8_t slave_addr, uint8_t reg_add, size_t len, uint8_t *data_rd)
{
    if (len == 0)
        return ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_add, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (len > 1)
        i2c_master_read(cmd, data_rd, len - 1, ACK_VAL);
    i2c_master_read_byte(cmd, data_rd + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
        return 0;
    else
        return -1;
}

void combine_msb_lsb_raw_data(uint8_t *buf_1, int16_t *buf_2)
{
    buf_2[0] = ((buf_1[0] << 8) + buf_1[1]);
    buf_2[1] = ((buf_1[2] << 8) + buf_1[3]);
    buf_2[2] = ((buf_1[4] << 8) + buf_1[5]);
}

int esp32_get_clock_ms(unsigned long *count)
{
    uint32_t time_us = esp_timer_get_time();
    *count = (float)(time_us) / 1000;
    return 0;
}

int esp32_delay_ms(unsigned long num_ms)
{
    vTaskDelay(num_ms / portTICK_RATE_MS);
    return 0;
}