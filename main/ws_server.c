#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "drone_api.h"
#include "ws_server.h"

static const char *TAG = "ws";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static httpd_handle_t s_server = NULL;

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start);
}

static esp_err_t favicon_get_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static float json_num(cJSON *o, const char *k, float def) {
    cJSON *n = cJSON_GetObjectItemCaseSensitive(o, k);
    return cJSON_IsNumber(n) ? (float)n->valuedouble : def;
}

static void handle_command(const char *msg) {
    cJSON *root = cJSON_Parse(msg);
    if (!root) return;
    cJSON *c = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    if (!cJSON_IsString(c)) { cJSON_Delete(root); return; }
    const char *cmd = c->valuestring;

    if      (!strcmp(cmd, "stop"))     api_set_full_stop(true);
    else if (!strcmp(cmd, "start"))    api_set_full_stop(false);
    else if (!strcmp(cmd, "arm"))      api_set_armed(true);
    else if (!strcmp(cmd, "disarm"))   api_set_armed(false);
    else if (!strcmp(cmd, "throttle")) api_set_throttle((int)json_num(root, "value", 0));
    else if (!strcmp(cmd, "test_all")) api_test_all();
    else if (!strcmp(cmd, "test"))     api_test_motor((int)json_num(root, "motor", 0));
    else if (!strcmp(cmd, "calibrate")) api_calibrate_level();
    else if (!strcmp(cmd, "gains")) {
        pid_gains_t g;
        api_get_gains(&g);
        g.kp_pr  = json_num(root, "kp_pr",  g.kp_pr);
        g.ki_pr  = json_num(root, "ki_pr",  g.ki_pr);
        g.kd_pr  = json_num(root, "kd_pr",  g.kd_pr);
        g.kp_yaw = json_num(root, "kp_yaw", g.kp_yaw);
        api_set_gains(&g);
    }
    cJSON_Delete(root);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WS handshake fd=%d", httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    httpd_ws_frame_t frame = { .type = HTTPD_WS_TYPE_TEXT };
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;
    if (frame.len == 0)  return ESP_OK;
    if (frame.len > 512) return ESP_FAIL;

    uint8_t *buf = calloc(1, frame.len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    frame.payload = buf;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret == ESP_OK && frame.type == HTTPD_WS_TYPE_TEXT) {
        buf[frame.len] = 0;
        handle_command((char *)buf);
    }
    free(buf);
    return ret;
}

typedef struct {
    httpd_handle_t hd;
    int fd;
    char *payload;
} ws_send_ctx_t;

static void ws_async_send(void *arg) {
    ws_send_ctx_t *c = arg;
    httpd_ws_frame_t f = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)c->payload,
        .len     = strlen(c->payload),
    };
    httpd_ws_send_frame_async(c->hd, c->fd, &f);
    free(c->payload);
    free(c);
}

static void broadcast_telemetry(void) {
    if (!s_server) return;
    int fds[8];
    size_t n = 8;
    if (httpd_get_client_list(s_server, &n, fds) != ESP_OK) return;
    for (size_t i = 0; i < n; i++) {
        int fd = fds[i];
        if (httpd_ws_get_fd_info(s_server, fd) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
        char *payload = malloc(384);
        if (!payload) continue;
        api_telemetry_json(payload, 384);
        ws_send_ctx_t *ctx = malloc(sizeof(*ctx));
        if (!ctx) { free(payload); continue; }
        ctx->hd      = s_server;
        ctx->fd      = fd;
        ctx->payload = payload;
        if (httpd_queue_work(s_server, ws_async_send, ctx) != ESP_OK) {
            free(payload);
            free(ctx);
        }
    }
}

static void telemetry_task(void *p) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));
        broadcast_telemetry();
    }
}

void ws_server_start(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets  = 4;
    cfg.lru_purge_enable  = true;
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return;
    }
    httpd_uri_t root = { .uri = "/",            .method = HTTP_GET, .handler = root_get_handler };
    httpd_uri_t ws   = { .uri = "/ws",          .method = HTTP_GET, .handler = ws_handler, .is_websocket = true };
    httpd_uri_t fav  = { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get_handler };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &ws);
    httpd_register_uri_handler(s_server, &fav);
    xTaskCreate(telemetry_task, "tlm_ws", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "HTTP+WS server up on port 80");
}
