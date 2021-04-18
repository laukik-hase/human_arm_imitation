#include "nvs_utils.h"

const char *key[36] = {"s_ax", "s_ay", "s_az", "s_gx", "s_gy", "s_gz", "s_mx", "s_my", "s_mz",
                       "b_ax", "b_ay", "b_az", "b_gx", "b_gy", "b_gz", "b_mx", "b_my", "b_mz",
                       "f_ax", "f_ay", "f_az", "f_gx", "f_gy", "f_gz", "f_mx", "f_my", "f_mz",
                       "p_ax", "p_ay", "p_az", "p_gx", "p_gy", "p_gz", "p_mx", "p_my", "p_mz"};

const int16_t value[36] = {68, -18, -37, -2, 0, -2, 18, -55, -31,
                           25, -38, -49, 0, -4, 0, -83, -97, 50,
                           -30, -23, -18, -6, -5, -2, 5, 123, 71,
                           7, -9, -19, -5, -3, -4, 189, -240, -137};


esp_err_t nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        IS_ESP_OK(nvs_flash_erase());
        IS_ESP_OK(nvs_flash_init());
    }

    return ESP_OK;
}

bool nvs_does_key_exist(const char *key)
{
    nvs_handle_t handle;

    bool ret = false;
    int16_t val = 0;

    if (nvs_open("storage", NVS_READONLY, &handle) == ESP_OK)
    {
        if (nvs_get_i16(handle, key, &val) == ESP_OK)
            ret = true;
    }

    return ret;
}

nvs_type_t nvs_get_key_type(const char *key)
{
    nvs_iterator_t it = nvs_entry_find("nvs", "storage", NVS_TYPE_ANY);
    nvs_type_t key_type = NVS_TYPE_ANY;

    while (it != NULL)
    {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        it = nvs_entry_next(it);
        if (!strcmp(key, info.key))
            key_type = info.type;
    }

    return key_type;
}

esp_err_t nvs_read_short(const char *key, int16_t *value)
{
    nvs_handle_t handle;

    IS_ESP_OK(nvs_open("storage", NVS_READONLY, &handle));
    IS_ESP_OK(nvs_get_i16(handle, key, value));

    nvs_close(handle);
    return ESP_OK;
}

esp_err_t nvs_write_short(const char *key, const int16_t *value)
{
    nvs_handle_t handle;

    IS_ESP_OK(nvs_open("storage", NVS_READWRITE, &handle));
    IS_ESP_OK(nvs_set_i16(handle, (char *)key, *value));
    IS_ESP_OK(nvs_commit(handle));

    nvs_close(handle);
    return ESP_OK;
}

esp_err_t nvs_load_calib_data(void)
{
    for (int i = 0; i < 36; i++)
        IS_ESP_OK(nvs_write_short(key[i], &value[i]));

    return ESP_OK;
}

esp_err_t nvs_delete_key(const char *key)
{
    nvs_handle_t handle;

    IS_ESP_OK(nvs_open("storage", NVS_READWRITE, &handle));
    IS_ESP_OK(nvs_erase_key(handle, key));
    IS_ESP_OK(nvs_commit(handle));

    nvs_close(handle);
    return ESP_OK;
}
