#include "nvs_utils.h"

const char *key[36] = {"s_ax", "s_ay", "s_az", "s_gx", "s_gy", "s_gz", "s_mx", "s_my", "s_mz",
                       "b_ax", "b_ay", "b_az", "b_gx", "b_gy", "b_gz", "b_mx", "b_my", "b_mz",
                       "e_ax", "e_ay", "e_az", "e_gx", "e_gy", "e_gz", "e_mx", "e_my", "e_mz",
                       "w_ax", "w_ay", "w_az", "w_gx", "w_gy", "w_gz", "w_mx", "w_my", "w_mz"};

const int16_t value[36] = {-4, -92, 953, -4, 5, 1, 334, 76, 3,
                           65, -92, 890, -2, 4, 0, 42, -146, -398,
                           47, 24, 996, -2, 0, 4, -126, 176, -385,
                           -75, -5, 941, 0, 1, 1, 347, -9, -167};

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
