#include "provisioning_server.h"
#include "ota_upgrade.h"
#include "esp_ota_ops.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "dap_core_service.h"
#include "device_identity_store.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"
#include "flash_user.h"
#include "network_tcp_server.h"
#include "wifi_manager.h"
#include "wifi_profile_store.h"
#include "wireless_config.h"
#include "bsp_pin_config.h"
#include "robot_link.h"
#include "robot_camera.h"

#define HTTP_BODY_MAX_SIZE 512U
#define AUTH_COOKIE_NAME "amphilink_auth"
#define AUTH_TOKEN_BYTES 16U
#define AUTH_TOKEN_HEX_SIZE (AUTH_TOKEN_BYTES * 2U)

static const char *TAG = "PROVISIONING_HTTP";
static httpd_handle_t s_server;
static provisioning_server_config_t s_config;
static char s_auth_token[AUTH_TOKEN_HEX_SIZE + 1U];

static const char *mode_name(wifi_manager_mode_t mode)
{
    switch (mode) {
    case WIFI_MANAGER_MODE_CONNECTING:
        return "connecting";
    case WIFI_MANAGER_MODE_STA:
        return "sta";
    case WIFI_MANAGER_MODE_AP:
        return "ap";
    case WIFI_MANAGER_MODE_STARTING:
    default:
        return "starting";
    }
}

static const char *operation_name(wifi_manager_operation_state_t state)
{
    switch (state) {
    case WIFI_MANAGER_OPERATION_RUNNING:
        return "running";
    case WIFI_MANAGER_OPERATION_SUCCEEDED:
        return "succeeded";
    case WIFI_MANAGER_OPERATION_FAILED:
        return "failed";
    case WIFI_MANAGER_OPERATION_IDLE:
    default:
        return "idle";
    }
}

static const char *auth_name(wifi_auth_mode_t authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        return "open";
    case WIFI_AUTH_WPA2_PSK:
        return "wpa2";
    case WIFI_AUTH_WPA3_PSK:
        return "wpa3";
    case WIFI_AUTH_WPA2_WPA3_PSK:
        return "wpa2_wpa3";
    case WIFI_AUTH_WPA_PSK:
    case WIFI_AUTH_WPA_WPA2_PSK:
        return "legacy_wpa";
    default:
        return "unsupported";
    }
}

static bool auth_supported(wifi_auth_mode_t authmode)
{
    return (authmode == WIFI_AUTH_OPEN) || (authmode == WIFI_AUTH_WPA_PSK) ||
           (authmode == WIFI_AUTH_WPA_WPA2_PSK) ||
           (authmode == WIFI_AUTH_WPA2_PSK) ||
           (authmode == WIFI_AUTH_WPA3_PSK) ||
           (authmode == WIFI_AUTH_WPA2_WPA3_PSK);
}

static esp_err_t send_json(httpd_req_t *request, const char *status,
                           cJSON *root)
{
    char *body = cJSON_PrintUnformatted(root);
    if (body == NULL) {
        return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_status(request, status);
    httpd_resp_set_type(request, "application/json; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send(request, body, HTTPD_RESP_USE_STRLEN);
    cJSON_free(body);
    return err;
}

static esp_err_t send_error(httpd_req_t *request, const char *status,
                            const char *code)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *error = cJSON_AddObjectToObject(root, "error");
    cJSON_AddStringToObject(error, "code", code);
    esp_err_t result = send_json(request, status, root);
    cJSON_Delete(root);
    return result;
}

static esp_err_t unauthorized(httpd_req_t *request)
{
    return send_error(request, "401 Unauthorized", "unauthorized");
}

static bool request_authenticated(httpd_req_t *request)
{
    size_t length = httpd_req_get_hdr_value_len(request, "Cookie");
    if ((length == 0U) || (length >= 128U)) {
        return false;
    }

    char cookie[128];
    if (httpd_req_get_hdr_value_str(request, "Cookie", cookie,
                                    sizeof(cookie)) != ESP_OK) {
        return false;
    }
    char marker[sizeof(AUTH_COOKIE_NAME) + 2U];
    snprintf(marker, sizeof(marker), "%s=", AUTH_COOKIE_NAME);
    const char *value = cookie;
    while ((value = strstr(value, marker)) != NULL) {
        if ((value == cookie) || (value[-1] == ';') || (value[-1] == ' ')) {
            value += strlen(marker);
            size_t token_length = strcspn(value, ";");
            return (token_length == AUTH_TOKEN_HEX_SIZE) &&
                   (memcmp(value, s_auth_token, AUTH_TOKEN_HEX_SIZE) == 0);
        }
        value += strlen(marker);
    }
    return false;
}

#define REQUIRE_AUTH(request) \
    do { \
        if (!request_authenticated(request)) { \
            return unauthorized(request); \
        } \
    } while (0)

static void generate_auth_token(void)
{
    static const char hex[] = "0123456789abcdef";
    uint8_t random_bytes[AUTH_TOKEN_BYTES];
    esp_fill_random(random_bytes, sizeof(random_bytes));
    for (size_t index = 0U; index < sizeof(random_bytes); ++index) {
        s_auth_token[index * 2U] = hex[random_bytes[index] >> 4U];
        s_auth_token[index * 2U + 1U] =
            hex[random_bytes[index] & 0x0FU];
    }
    s_auth_token[AUTH_TOKEN_HEX_SIZE] = '\0';
}

static esp_err_t send_store_error(httpd_req_t *request, esp_err_t error)
{
    if (error == ESP_ERR_NOT_FOUND) {
        return send_error(request, "404 Not Found", "not_found");
    }
    if ((error == ESP_ERR_INVALID_STATE) || (error == ESP_ERR_NO_MEM)) {
        return send_error(request, "409 Conflict",
                          (error == ESP_ERR_NO_MEM) ? "profile_limit"
                                                    : "revision_conflict");
    }
    if (error == ESP_ERR_INVALID_SIZE) {
        return send_error(request, "400 Bad Request", "invalid_size");
    }
    if (error == ESP_ERR_INVALID_ARG) {
        return send_error(request, "400 Bad Request", "invalid_profile");
    }
    return send_error(request, "500 Internal Server Error", "storage_error");
}

static cJSON *receive_json(httpd_req_t *request)
{
    if ((request->content_len <= 0) ||
        ((size_t)request->content_len > HTTP_BODY_MAX_SIZE)) {
        return NULL;
    }
    char body[HTTP_BODY_MAX_SIZE + 1U];
    size_t received = 0U;
    while (received < (size_t)request->content_len) {
        int result = httpd_req_recv(request, body + received,
                                    (size_t)request->content_len - received);
        if (result <= 0) {
            return NULL;
        }
        received += (size_t)result;
    }
    body[received] = '\0';
    return cJSON_ParseWithLength(body, received);
}

static bool json_uint32(cJSON *root, const char *name, uint32_t *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    if (!cJSON_IsNumber(item) || (item->valuedouble < 0.0) ||
        (item->valuedouble > 4294967295.0)) {
        return false;
    }
    uint32_t converted = (uint32_t)item->valuedouble;
    if ((double)converted != item->valuedouble) {
        return false;
    }
    *value = converted;
    return true;
}

static esp_err_t login_post(httpd_req_t *request)
{
    cJSON *body = receive_json(request);
    cJSON *password =
        (body == NULL)
            ? NULL
            : cJSON_GetObjectItemCaseSensitive(body, "password");
    if (!cJSON_IsString(password) ||
        (strcmp(password->valuestring, ADMIN_PASSWORD) != 0)) {
        cJSON_Delete(body);
        return send_error(request, "401 Unauthorized", "invalid_password");
    }
    cJSON_Delete(body);

    char cookie[96];
    snprintf(cookie, sizeof(cookie), "%s=%s; HttpOnly; SameSite=Strict; Path=/",
             AUTH_COOKIE_NAME, s_auth_token);
    httpd_resp_set_hdr(request, "Set-Cookie", cookie);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "authenticated", true);
    esp_err_t err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t logout_post(httpd_req_t *request)
{
    if (!request_authenticated(request)) return unauthorized(request);
    generate_auth_token();
    httpd_resp_set_hdr(request, "Set-Cookie", AUTH_COOKIE_NAME "=; Max-Age=0; HttpOnly; SameSite=Strict; Path=/");
    return httpd_resp_sendstr(request, "{}");
}

static bool json_profile(cJSON *root, const char **ssid, const char **password)
{
    cJSON *ssid_item = cJSON_GetObjectItemCaseSensitive(root, "ssid");
    cJSON *password_item = cJSON_GetObjectItemCaseSensitive(root, "password");
    if (!cJSON_IsString(ssid_item) || !cJSON_IsString(password_item)) {
        return false;
    }
    *ssid = ssid_item->valuestring;
    *password = password_item->valuestring;
    return true;
}

static void sync_manager(const wifi_profile_snapshot_t *snapshot)
{
    wifi_manager_credential_t credentials[WIFI_PROFILE_MAX_COUNT] = {0};
    for (size_t index = 0U; index < snapshot->count; ++index) {
        credentials[index].id = snapshot->profiles[index].id;
        snprintf(credentials[index].ssid, sizeof(credentials[index].ssid), "%s",
                 snapshot->profiles[index].ssid);
        snprintf(credentials[index].password,
                 sizeof(credentials[index].password), "%s",
                 snapshot->profiles[index].password);
    }
    (void)wifi_manager_set_credentials(credentials, snapshot->count);
}

static cJSON *snapshot_json(const wifi_profile_snapshot_t *snapshot)
{
    wifi_manager_status_t status = {0};
    (void)wifi_manager_get_status(&status);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "revision", snapshot->revision);
    cJSON *items = cJSON_AddArrayToObject(root, "items");
    for (size_t index = 0U; index < snapshot->count; ++index) {
        const wifi_profile_t *profile = &snapshot->profiles[index];
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "id", profile->id);
        cJSON_AddStringToObject(item, "ssid", profile->ssid);
        cJSON_AddStringToObject(item, "password", profile->password);
        cJSON_AddNumberToObject(item, "priority", index);
        cJSON_AddBoolToObject(item, "connected",
                              (status.mode == WIFI_MANAGER_MODE_STA) &&
                                  (strcmp(status.ssid, profile->ssid) == 0));
        cJSON_AddItemToArray(items, item);
    }
    return root;
}

static esp_err_t status_get(httpd_req_t *request)
{
    /* Read-only connection metadata is also used before management login.
     * Saved profiles, passwords and all mutations remain authenticated. */
    if (strcmp(request->uri, "/api/v1/health") != 0) {
        REQUIRE_AUTH(request);
    }
    wifi_manager_status_t status;
    if (wifi_manager_get_status(&status) != ESP_OK) {
        return send_error(request, "503 Service Unavailable", "not_ready");
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "mode", mode_name(status.mode));
    cJSON_AddStringToObject(root, "ssid", status.ssid);
    cJSON_AddStringToObject(root, "ip", status.ip);
    cJSON_AddStringToObject(root, "ap_ssid", status.ap_ssid);
    cJSON_AddBoolToObject(root, "scanning", status.scanning);
    cJSON_AddBoolToObject(root, "ap_enabled", status.ap_enabled);
    cJSON_AddStringToObject(root, "ap_ip", status.ap_enabled ? "192.168.4.1" : "");
    cJSON_AddBoolToObject(root, "sta_connected", status.mode == WIFI_MANAGER_MODE_STA);
    cJSON_AddStringToObject(root, "sta_ip", status.mode == WIFI_MANAGER_MODE_STA ? status.ip : "");
    cJSON_AddNumberToObject(root, "dap_port", 4441);
    cJSON_AddNumberToObject(root, "nrst_gpio", BSP_PIN_TARGET_RESET);
    cJSON_AddStringToObject(root, "firmware", "rlyoulink");
    cJSON_AddBoolToObject(root, "web_ui", false);
    cJSON_AddStringToObject(root,"version",esp_app_get_description()->version);
    cJSON_AddStringToObject(root,"hardware","xiaos3-8mb-nrst-d5");
    const esp_partition_t *running=esp_ota_get_running_partition();
    cJSON_AddStringToObject(root,"running_slot",running?running->label:"");
    esp_ota_img_states_t ota_state=ESP_OTA_IMG_UNDEFINED;
    if(running)esp_ota_get_state_partition(running,&ota_state);
    cJSON_AddBoolToObject(root,"ota_confirmed",ota_state==ESP_OTA_IMG_VALID);
    cJSON_AddBoolToObject(root, "dap_client", (network_tcp_server_has_client() || dap_core_service_busy()));
    esp_err_t err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static cJSON *device_identity_json(const device_identity_t *identity)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", identity->name);
    cJSON_AddNumberToObject(root, "id", identity->id);
    return root;
}

static esp_err_t device_get(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    device_identity_t identity;
    esp_err_t err = device_identity_store_get(&identity);
    if (err != ESP_OK) {
        return send_error(request, "503 Service Unavailable", "not_ready");
    }
    cJSON *root = device_identity_json(&identity);
    err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t device_put(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    cJSON *body = receive_json(request);
    cJSON *name_item =
        (body == NULL) ? NULL : cJSON_GetObjectItemCaseSensitive(body, "name");
    uint32_t id;
    if (!cJSON_IsString(name_item) || !json_uint32(body, "id", &id)) {
        cJSON_Delete(body);
        return send_error(request, "400 Bad Request", "invalid_json");
    }

    device_identity_t identity;
    esp_err_t err =
        device_identity_store_set(name_item->valuestring, id, &identity);
    cJSON_Delete(body);
    if (err == ESP_ERR_INVALID_ARG) {
        return send_error(request, "400 Bad Request", "invalid_identity");
    }
    if (err != ESP_OK) {
        return send_error(request, "500 Internal Server Error",
                          "storage_error");
    }

    if (s_config.identity_changed != NULL) {
        s_config.identity_changed(&identity, s_config.context);
    }
    cJSON *root = device_identity_json(&identity);
    err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t networks_get(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    wifi_profile_snapshot_t snapshot;
    esp_err_t err = wifi_profile_store_get(&snapshot);
    if (err != ESP_OK) {
        return send_store_error(request, err);
    }
    cJSON *root = snapshot_json(&snapshot);
    err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t networks_post(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    cJSON *body = receive_json(request);
    const char *ssid;
    const char *password;
    uint32_t revision;
    if ((body == NULL) || !json_uint32(body, "revision", &revision) ||
        !json_profile(body, &ssid, &password)) {
        cJSON_Delete(body);
        return send_error(request, "400 Bad Request", "invalid_json");
    }
    wifi_profile_snapshot_t snapshot;
    esp_err_t err = wifi_profile_store_add(revision, ssid, password, &snapshot);
    cJSON_Delete(body);
    if (err != ESP_OK) {
        return send_store_error(request, err);
    }
    sync_manager(&snapshot);
    cJSON *root = snapshot_json(&snapshot);
    err = send_json(request, "201 Created", root);
    cJSON_Delete(root);
    return err;
}

static bool uri_id(const char *uri, uint32_t *id)
{
    const char *separator = strrchr(uri, '/');
    if ((separator == NULL) || (separator[1] == '\0')) {
        return false;
    }
    char *end;
    unsigned long value = strtoul(separator + 1, &end, 10);
    if ((*end != '\0') || (value == 0UL) || (value > UINT32_MAX)) {
        return false;
    }
    *id = (uint32_t)value;
    return true;
}

static esp_err_t network_put(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    uint32_t id;
    cJSON *body = receive_json(request);
    const char *ssid;
    const char *password;
    uint32_t revision;
    if (!uri_id(request->uri, &id) || (body == NULL) ||
        !json_uint32(body, "revision", &revision) ||
        !json_profile(body, &ssid, &password)) {
        cJSON_Delete(body);
        return send_error(request, "400 Bad Request", "invalid_json");
    }
    wifi_profile_snapshot_t snapshot;
    esp_err_t err =
        wifi_profile_store_update(revision, id, ssid, password, &snapshot);
    cJSON_Delete(body);
    if (err != ESP_OK) {
        return send_store_error(request, err);
    }
    sync_manager(&snapshot);
    cJSON *root = snapshot_json(&snapshot);
    err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t network_delete(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    uint32_t id;
    cJSON *body = receive_json(request);
    uint32_t revision;
    if (!uri_id(request->uri, &id) || (body == NULL) ||
        !json_uint32(body, "revision", &revision)) {
        cJSON_Delete(body);
        return send_error(request, "400 Bad Request", "invalid_json");
    }
    wifi_profile_snapshot_t snapshot;
    esp_err_t err = wifi_profile_store_remove(revision, id, &snapshot);
    cJSON_Delete(body);
    if (err != ESP_OK) {
        return send_store_error(request, err);
    }
    sync_manager(&snapshot);
    cJSON *root = snapshot_json(&snapshot);
    err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t order_put(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    cJSON *body = receive_json(request);
    uint32_t revision;
    cJSON *ids =
        (body == NULL) ? NULL : cJSON_GetObjectItemCaseSensitive(body, "ids");
    if ((body == NULL) || !json_uint32(body, "revision", &revision) ||
        !cJSON_IsArray(ids) ||
        (cJSON_GetArraySize(ids) > WIFI_PROFILE_MAX_COUNT)) {
        cJSON_Delete(body);
        return send_error(request, "400 Bad Request", "invalid_json");
    }
    uint32_t ordered[WIFI_PROFILE_MAX_COUNT];
    size_t count = (size_t)cJSON_GetArraySize(ids);
    for (size_t index = 0U; index < count; ++index) {
        cJSON *item = cJSON_GetArrayItem(ids, (int)index);
        if (!cJSON_IsNumber(item) || (item->valuedouble <= 0.0) ||
            (item->valuedouble > UINT32_MAX)) {
            cJSON_Delete(body);
            return send_error(request, "400 Bad Request", "invalid_json");
        }
        ordered[index] = (uint32_t)item->valuedouble;
    }
    wifi_profile_snapshot_t snapshot;
    esp_err_t err =
        wifi_profile_store_reorder(revision, ordered, count, &snapshot);
    cJSON_Delete(body);
    if (err != ESP_OK) {
        return send_store_error(request, err);
    }
    sync_manager(&snapshot);
    cJSON *root = snapshot_json(&snapshot);
    err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t scan_post(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    if ((network_tcp_server_has_client() || dap_core_service_busy())) {
        return send_error(request, "409 Conflict", "dap_client_active");
    }
    uint32_t operation_id;
    if (!dap_core_service_network_begin()) return send_error(request, "409 Conflict", "dap_client_active");
    esp_err_t err = wifi_manager_scan_start(&operation_id);
    if (err != ESP_OK) {
        dap_core_service_network_end(NULL);
        return send_error(request, "409 Conflict", "operation_busy");
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "operation_id", operation_id);
    err = send_json(request, "202 Accepted", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t scan_get(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    uint32_t operation_id;
    if (!uri_id(request->uri, &operation_id)) {
        return send_error(request, "400 Bad Request", "invalid_id");
    }
    wifi_manager_scan_result_t result;
    if (wifi_manager_scan_get(operation_id, &result) != ESP_OK) {
        return send_error(request, "404 Not Found", "not_found");
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "operation_id", result.operation_id);
    cJSON_AddStringToObject(root, "state", operation_name(result.state));
    cJSON_AddNumberToObject(root, "error", result.error);
    cJSON *items = cJSON_AddArrayToObject(root, "items");
    wifi_profile_snapshot_t snapshot;
    (void)wifi_profile_store_get(&snapshot);
    for (size_t index = 0U; index < result.count; ++index) {
        const wifi_manager_scan_record_t *record = &result.records[index];
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "ssid", record->ssid);
        cJSON_AddNumberToObject(item, "rssi", record->rssi);
        cJSON_AddNumberToObject(item, "channel", record->channel);
        cJSON_AddStringToObject(item, "auth", auth_name(record->authmode));
        cJSON_AddBoolToObject(item, "supported",
                              auth_supported(record->authmode));
        bool saved = false;
        for (size_t profile = 0U; profile < snapshot.count; ++profile) {
            if (strcmp(snapshot.profiles[profile].ssid, record->ssid) == 0) {
                saved = true;
                break;
            }
        }
        cJSON_AddBoolToObject(item, "saved", saved);
        cJSON_AddItemToArray(items, item);
    }
    esp_err_t err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t connect_post(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    if (network_tcp_server_has_client() || dap_core_service_busy()) return send_error(request, "409 Conflict", "dap_client_active");
    cJSON *body = receive_json(request);
    const char *ssid;
    const char *password;
    if ((body == NULL) || !json_profile(body, &ssid, &password) ||
        (wifi_profile_validate(ssid, password) != ESP_OK)) {
        cJSON_Delete(body);
        return send_error(request, "400 Bad Request", "invalid_profile");
    }
    wifi_profile_snapshot_t snapshot;
    (void)wifi_profile_store_get(&snapshot);
    bool existing = false;
    for (size_t index = 0U; index < snapshot.count; ++index) {
        if (strcmp(snapshot.profiles[index].ssid, ssid) == 0) {
            existing = true;
            break;
        }
    }
    if (!existing && (snapshot.count >= WIFI_PROFILE_MAX_COUNT)) {
        cJSON_Delete(body);
        return send_error(request, "409 Conflict", "profile_limit");
    }
    wifi_manager_credential_t credential = {0};
    snprintf(credential.ssid, sizeof(credential.ssid), "%s", ssid);
    snprintf(credential.password, sizeof(credential.password), "%s", password);
    uint32_t operation_id;
    if (!dap_core_service_network_begin()) { cJSON_Delete(body); return send_error(request, "409 Conflict", "dap_client_active"); }
    esp_err_t err = wifi_manager_connect_start(&credential, &operation_id);
    cJSON_Delete(body);
    if (err != ESP_OK) {
        dap_core_service_network_end(NULL);
        return send_error(request, "409 Conflict", "operation_busy");
    }
    network_tcp_server_disconnect_client();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "operation_id", operation_id);
    err = send_json(request, "202 Accepted", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t connect_get(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    uint32_t operation_id;
    if (!uri_id(request->uri, &operation_id)) {
        return send_error(request, "400 Bad Request", "invalid_id");
    }
    wifi_manager_connect_result_t result;
    if (wifi_manager_connect_get(operation_id, &result) != ESP_OK) {
        return send_error(request, "404 Not Found", "not_found");
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "operation_id", result.operation_id);
    cJSON_AddStringToObject(root, "state", operation_name(result.state));
    cJSON_AddNumberToObject(root, "error", result.error);
    cJSON_AddStringToObject(root, "ssid", result.ssid);
    wifi_manager_status_t connected_status = {0};
    if (wifi_manager_get_status(&connected_status) == ESP_OK && result.state == WIFI_MANAGER_OPERATION_SUCCEEDED) cJSON_AddStringToObject(root, "ip", connected_status.ip);
    esp_err_t err = send_json(request, "200 OK", root);
    cJSON_Delete(root);
    return err;
}

static esp_err_t upgrade_post(httpd_req_t *request) {
    REQUIRE_AUTH(request);
    return ota_upgrade_receive(request);
}


static esp_err_t robot_status_get(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    robot_link_status_t state={0};
    (void)robot_link_snapshot(&state);
    cJSON *root=cJSON_CreateObject();
    cJSON_AddBoolToObject(root,"online",state.online);
    cJSON_AddBoolToObject(root,"ready",state.ready);
    cJSON_AddBoolToObject(root,"wifi_control_allowed",state.wifi_control_allowed);
    cJSON_AddBoolToObject(root,"battery_valid",state.battery_valid);
    cJSON_AddNumberToObject(root,"battery_state",state.online ? state.battery_state : RF_SPI_BATTERY_UNAVAILABLE);
    cJSON_AddNumberToObject(root,"battery_age_ms",state.battery_age_ms);
    if (state.battery_valid) cJSON_AddNumberToObject(root,"battery_voltage",state.battery_mv/1000.0);
    else cJSON_AddNullToObject(root,"battery_voltage");
    cJSON_AddBoolToObject(root,"camera",robot_camera_available());
    cJSON_AddNumberToObject(root,"sent",state.sent);
    cJSON_AddNumberToObject(root,"received",state.received);
    cJSON_AddNumberToObject(root,"invalid",state.invalid);
    cJSON_AddNumberToObject(root,"errors",state.errors);
    cJSON_AddNumberToObject(root,"peer_time_us",state.peer_time_us);
    cJSON_AddNumberToObject(root,"round_trip_us",state.round_trip_us);
    cJSON_AddNumberToObject(root,"clock_offset_us",state.clock_offset_us);
    cJSON_AddNumberToObject(root,"command_id",state.command_id);
    cJSON_AddNumberToObject(root,"command_ack",state.command_ack);
    cJSON_AddNumberToObject(root,"command_result",state.command_result);
    cJSON_AddNumberToObject(root,"mode",state.mode);
    cJSON_AddNumberToObject(root,"action",state.action);
    cJSON_AddNumberToObject(root,"power",state.power);
    cJSON_AddBoolToObject(root,"failsafe",state.failsafe);
    cJSON_AddNumberToObject(root,"frame_attempts",state.frame_attempts);
    cJSON_AddNumberToObject(root,"frame_failures",state.frame_failures);
    cJSON_AddNumberToObject(root,"tail1",state.tail1/100.0);
    cJSON_AddNumberToObject(root,"tail2",state.tail2/100.0);
    cJSON_AddNumberToObject(root,"left_fin",state.left_fin/100.0);
    cJSON_AddNumberToObject(root,"right_fin",state.right_fin/100.0);
    cJSON_AddNumberToObject(root,"left_esc",state.left_esc);
    cJSON_AddNumberToObject(root,"right_esc",state.right_esc);
    cJSON_AddNumberToObject(root,"pitch",state.pitch);
    esp_err_t result=send_json(request,"200 OK",root);
    cJSON_Delete(root); return result;
}
static esp_err_t robot_command_post(httpd_req_t *request)
{
    REQUIRE_AUTH(request);
    cJSON *root=receive_json(request);
    if (!root) return send_error(request,"400 Bad Request","invalid_json");
    rf_spi_command_t command={0};
    uint32_t type=0,a=0,b=0,c=0,d=0;
    bool valid=json_uint32(root,"type",&type) && type<=RF_SPI_CMD_VISION;
    command.type=(uint8_t)type;
    if (valid) switch(type) {
    case RF_SPI_CMD_MOTION:
        valid=json_uint32(root,"action",&a) && a<=4 &&
              json_uint32(root,"mode",&b) && b<=3 &&
              json_uint32(root,"power",&c) && c<=100 &&
              json_uint32(root,"strength",&d) && d<=100;
        command.action=(uint8_t)a;command.mode=(uint8_t)b;
        command.power=(uint8_t)c;command.strength=(uint8_t)d;
        break;
    case RF_SPI_CMD_LORA:
        valid=json_uint32(root,"code",&a) && a<=21;
        command.code=(uint8_t)a;
        if(cJSON_GetObjectItemCaseSensitive(root,"knob")) {
            valid=valid && json_uint32(root,"knob",&b) && b<=1000;
            command.has_knob=1;command.knob=(uint16_t)b;
        }
        break;
    case RF_SPI_CMD_PITCH:
        valid=json_uint32(root,"pitch",&a) && a<=180;
        command.pitch=(int16_t)a;
        break;
    case RF_SPI_CMD_VISION: {
        cJSON *steering=cJSON_GetObjectItemCaseSensitive(root,"steering");
        valid=json_uint32(root,"power",&a) && a<=100 &&
              cJSON_IsNumber(steering) && steering->valuedouble>=-100 &&
              steering->valuedouble<=100 &&
              steering->valuedouble==(double)steering->valueint;
        command.power=(uint8_t)a;
        if(valid) command.steering=(int16_t)steering->valueint;
        cJSON *brake=cJSON_GetObjectItemCaseSensitive(root,"brake");
        if(brake && !cJSON_IsBool(brake)) valid=false;
        command.brake=cJSON_IsTrue(brake);
        break;
    }
    default:valid=false;break;
    }
    cJSON_Delete(root);
    if (!valid || !rf_spi_command_valid(&command))
        return send_error(request,"400 Bad Request","invalid_command");
    uint32_t id=0;
    esp_err_t err=robot_link_submit(&command,&id);
    if(err!=ESP_OK)
        return send_error(request,"409 Conflict",err==ESP_ERR_NO_MEM?"queue_full":"controller_offline");
    root=cJSON_CreateObject();cJSON_AddNumberToObject(root,"id",id);
    cJSON_AddStringToObject(root,"state","queued");
    esp_err_t result=send_json(request,"202 Accepted",root);
    cJSON_Delete(root);return result;
}

static esp_err_t register_handler(const char *uri, httpd_method_t method,
                                  esp_err_t (*handler)(httpd_req_t *))
{
    const httpd_uri_t descriptor = {
        .uri = uri,
        .method = method,
        .handler = handler,
    };
    return httpd_register_uri_handler(s_server, &descriptor);
}

esp_err_t provisioning_server_start(const provisioning_server_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_server != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_config = *config;
    generate_auth_token();
    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.server_port = 80U;
    http_config.stack_size = 6144U;
    http_config.core_id = 0;
    http_config.max_uri_handlers = 32U;
    /* The camera stream has its own server; sdkconfig reserves 16 sockets. */
    http_config.max_open_sockets = 4U;
    http_config.lru_purge_enable = true;
    http_config.uri_match_fn = httpd_uri_match_wildcard;
    esp_err_t err = httpd_start(&s_server, &http_config);
    if (err != ESP_OK) {
        memset(&s_config, 0, sizeof(s_config));
        return err;
    }

    err = register_handler("/api/v1/robot",HTTP_GET,robot_status_get);
    if (err == ESP_OK) err = register_handler("/api/v1/robot/command",HTTP_POST,robot_command_post);
    if (err == ESP_OK) err = robot_camera_register(s_server);
    if (err == ESP_OK) err = register_handler("/api/v1/upgrade", HTTP_POST, upgrade_post);
    if (err == ESP_OK) err = register_handler("/api/v1/login", HTTP_POST, login_post);
    if (err == ESP_OK) err = register_handler("/api/v1/logout", HTTP_POST, logout_post);
    if (err == ESP_OK) {
        err = register_handler("/api/v1/status", HTTP_GET, status_get);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/health", HTTP_GET, status_get);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/device", HTTP_GET, device_get);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/device", HTTP_PUT, device_put);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/networks", HTTP_GET, networks_get);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/networks", HTTP_POST, networks_post);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/networks/order", HTTP_PUT, order_put);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/networks/*", HTTP_PUT, network_put);
    }
    if (err == ESP_OK) {
        err =
            register_handler("/api/v1/networks/*", HTTP_DELETE, network_delete);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/scan", HTTP_POST, scan_post);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/scan/*", HTTP_GET, scan_get);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/connect", HTTP_POST, connect_post);
    }
    if (err == ESP_OK) {
        err = register_handler("/api/v1/connect/*", HTTP_GET, connect_get);
    }
    if (err != ESP_OK) {
        httpd_stop(s_server);
        s_server = NULL;
        memset(&s_config, 0, sizeof(s_config));
        return err;
    }
    ESP_LOGI(TAG, "Provisioning portal listening on HTTP port 80");
    return ESP_OK;
}

esp_err_t provisioning_server_stop(void)
{
    if (s_server == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = httpd_stop(s_server);
    if (err == ESP_OK) {
        s_server = NULL;
        memset(&s_config, 0, sizeof(s_config));
    }
    return err;
}
