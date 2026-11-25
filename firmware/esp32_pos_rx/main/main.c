/*
 * ESP32-S3 坐标接收端
 *
 * 职责：连上树莓派开的 2.4GHz 热点 PiLink，主动连树莓派的 TCP 5000 端口，
 *       然后**收** 12 字节坐标帧（帧头/类型/序号/x/y/CRC16/帧尾），
 *       校验通过就交给 on_position() 去驱动执行机构。
 *
 * 本固件不发送任何数据。坐标由树莓派算、树莓派发，这里只当执行端。
 *
 * 帧格式（12 字节，小端）：
 *   [0]  0xAA         帧头
 *   [1]  0x55         帧头
 *   [2]  0x01         类型：坐标帧
 *   [3]  seq          序号 0~255 循环
 *   [4]  x & 0xFF     X 低字节
 *   [5]  x >> 8       X 高字节  (int16)
 *   [6]  y & 0xFF     Y 低字节
 *   [7]  y >> 8       Y 高字节  (int16)
 *   [8]  crc & 0xFF   CRC16-CCITT-FALSE 低字节，覆盖 [0]~[7]
 *   [9]  crc >> 8     CRC16-CCITT-FALSE 高字节
 *   [10] 0x0D         帧尾
 *   [11] 0x0A         帧尾
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ============ 需要你改的 4 个参数 ============ */
#define WIFI_SSID       "PiLink"        /* 树莓派热点名 */
#define WIFI_PASS       "raspberry123"  /* 树莓派热点密码 */
#define HOST_IP         "192.168.50.1"  /* 树莓派在热点里的固定地址 */
#define HOST_PORT       5000            /* 树莓派 pos_link 桥接端口 */
/* ============================================ */

#define RECV_TIMEOUT_S  3               /* 超过这么久没收到字节就告警一次 */

/* 帧定义：改这里必须同步改 src/comm/pos_link/pos_link/frame.py */
#define FRAME_LEN       12
#define FRAME_HEAD0     0xAA
#define FRAME_HEAD1     0x55
#define FRAME_TAIL0     0x0D
#define FRAME_TAIL1     0x0A
#define TYPE_POS        0x01

static const char *TAG = "pos_rx";

/* WiFi 是否已连上并拿到 IP，用事件组做同步 */
static EventGroupHandle_t s_wifi_evt;
#define WIFI_CONNECTED_BIT BIT0

/* WiFi 配置放全局：重连逻辑里要能改它（解除 BSSID 锁定） */
static wifi_config_t s_wc;
static bool s_bssid_locked = false;   /* 当前是否锁定了某个 AP 的 BSSID */
static int  s_fail_cnt      = 0;      /* 连续失败次数 */

/* 接收统计，排障时看这两个数 */
static uint32_t s_rx_frames = 0;
static uint32_t s_rx_bad    = 0;

/* ---------- CRC16-CCITT-FALSE：poly 0x1021, init 0xFFFF ---------- */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ---------- 校验并拆一帧，失败返回 false ---------- */
static bool parse_frame(const uint8_t *f, uint8_t *seq, int16_t *x, int16_t *y)
{
    if (f[0] != FRAME_HEAD0 || f[1] != FRAME_HEAD1) return false;
    if (f[2] != TYPE_POS)                           return false;
    if (f[10] != FRAME_TAIL0 || f[11] != FRAME_TAIL1) return false;

    uint16_t crc_rx = (uint16_t)(f[8] | ((uint16_t)f[9] << 8));
    if (crc16_ccitt(f, 8) != crc_rx) return false;

    *seq = f[3];
    *x   = (int16_t)((uint16_t)f[4] | ((uint16_t)f[5] << 8));
    *y   = (int16_t)((uint16_t)f[6] | ((uint16_t)f[7] << 8));
    return true;
}

/* ------------------------------------------------------------------ *
 *  ★★ 坐标到手的地方 —— 你要接执行机构就改这个函数 ★★
 *
 *  现在只打日志。实际用的时候把下面的内容换成你的动作：
 *    - 驱动电机/舵机：算差分、限幅、写 PWM
 *    - 转给另一颗 MCU：UART 转发、再打包
 *    - 存起来等条件触发（比如「x>300 才动」）
 *
 *  注意：这是 TCP 任务上下文，不是中断，可以调 vTaskDelay 之外的普通 API；
 *  但别在这里做耗时超过几十毫秒的事，否则会拖慢收帧（1 帧/秒，缓冲很浅）。
 * ------------------------------------------------------------------ */
static void on_position(uint8_t seq, int16_t x, int16_t y)
{
    ESP_LOGI(TAG, "收到 seq=%u x=%d y=%d", seq, x, y);
}

/* ---------- 读一个字节：1=读到, 0=超时, -1=连接断了 ---------- */
static int recv_byte(int sock, uint8_t *b)
{
    int n = recv(sock, b, 1, 0);
    if (n == 1) {
        return 1;
    }
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;          /* 超时，不是错误 */
        }
        return -1;
    }
    return -1;                 /* n == 0：对端关了连接 */
}

/* ------------------------------------------------------------------ *
 *  读满一帧。返回：
 *    1 = 好帧（已回调 on_position）
 *    2 = 帧读到了但校验失败（计坏帧，继续）
 *    0 = 一直没等到帧头（RECV_TIMEOUT_S 内无字节）
 *   -1 = 连接断开
 *
 *  为什么逐字节找帧头：TCP 是字节流，一次 recv 可能给半帧、也可能给三帧
 *  加半截垃圾。逐字节做状态机最笨也最不容易错，1 Hz × 12 字节的开销
 *  完全可以忽略。
 * ------------------------------------------------------------------ */
static int read_frame(int sock, uint8_t f[FRAME_LEN])
{
    int got = 0;

    while (got < FRAME_LEN) {
        uint8_t b;
        int r = recv_byte(sock, &b);
        if (r < 0) {
            return -1;
        }
        if (r == 0) {
            if (got == 0) {
                return 0;      /* 还没开始一帧：树莓派可能暂时没有坐标 */
            }
            got = 0;           /* 半帧断流：丢掉重来 */
            continue;
        }

        if (got == 0) {
            if (b != FRAME_HEAD0) {
                continue;
            }
        } else if (got == 1) {
            if (b != FRAME_HEAD1) {
                /* 不是 0x55：如果又是 0xAA，就当新的帧头起点 */
                if (b == FRAME_HEAD0) {
                    f[0] = FRAME_HEAD0;
                    got  = 1;
                } else {
                    got = 0;
                }
                continue;
            }
        }
        f[got++] = b;
    }

    uint8_t seq;
    int16_t x, y;
    if (!parse_frame(f, &seq, &x, &y)) {
        s_rx_bad++;
        ESP_LOGW(TAG, "帧校验失败（累计坏帧 %u），丢弃后重新找帧头", s_rx_bad);
        return 2;
    }

    s_rx_frames++;
    on_position(seq, x, y);
    return 1;
}

/* ---------- 建立 TCP 连接，失败返回 -1 ---------- */
static int tcp_connect(void)
{
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() 失败, errno=%d", errno);
        return -1;
    }

    /* 接收超时 3 秒：对端不发数据时 recv() 不会永久阻塞，
       这样"树莓派没坐标源"能变成一条告警而不是卡死 */
    struct timeval tv = { .tv_sec = RECV_TIMEOUT_S, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* 关掉 Nagle：对面每次就发 12 字节，攒包只会平白加延迟 */
    int nodelay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    struct sockaddr_in dest = { 0 };
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(HOST_PORT);
    dest.sin_addr.s_addr = inet_addr(HOST_IP);

    if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
        ESP_LOGW(TAG, "连接 %s:%d 失败, errno=%d（树莓派上的 bridge 起了吗？）",
                 HOST_IP, HOST_PORT, errno);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "TCP 已连接 %s:%d，开始等坐标帧", HOST_IP, HOST_PORT);
    return sock;
}

/* ---------- WiFi / IP 事件回调 ---------- */
static void event_handler(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        /* 注意：这里不直接 connect，等 app_main 扫描打印完再连 */
        ESP_LOGI(TAG, "WiFi 已启动");

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)data;
        xEventGroupClearBits(s_wifi_evt, WIFI_CONNECTED_BIT);
        s_fail_cnt++;
        ESP_LOGW(TAG, "WiFi 断开，reason=%d，ssid=\"%s\"，连续失败 %d 次，1 秒后重连",
                 d->reason, (char *)d->ssid, s_fail_cnt);

        /* 连续失败多次：多半是热点换了信道或重建了，解除锁定重新全网找 */
        if (s_bssid_locked && s_fail_cnt >= 5) {
            s_wc.sta.bssid_set = 0;
            memset(s_wc.sta.bssid, 0, 6);
            s_wc.sta.channel = 0;
            esp_wifi_set_config(WIFI_IF_STA, &s_wc);
            s_bssid_locked = false;
            s_fail_cnt = 0;
            ESP_LOGW(TAG, "连续失败 5 次，解除 BSSID 锁定，改为按 SSID 扫描");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));   /* 退避一下，避免疯狂重连 */
        esp_wifi_connect();

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "拿到 IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_fail_cnt = 0;
        xEventGroupSetBits(s_wifi_evt, WIFI_CONNECTED_BIT);
    }
}

/* ---------- 调试用：扫描并打印周围所有 AP ---------- */
static const char *authmode_name(wifi_auth_mode_t m)
{
    switch (m) {
    case WIFI_AUTH_OPEN:            return "OPEN";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2_WPA3_PSK";
    default:                        return "UNKNOWN";
    }
}

static bool find_target_ap(wifi_ap_record_t *out)
{
    wifi_scan_config_t sc = { 0 };
    sc.show_hidden = true;

    ESP_LOGI(TAG, "===== 开始扫描周围 AP =====");
    esp_err_t err = esp_wifi_scan_start(&sc, true);   /* true = 阻塞到扫完 */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "扫描失败: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t num = 0;
    esp_wifi_scan_get_ap_num(&num);
    if (num == 0) {
        ESP_LOGW(TAG, "一个 AP 都没扫到（射频没起来？天线没接？）");
        return false;
    }
    if (num > 20) num = 20;

    wifi_ap_record_t *recs = calloc(num, sizeof(wifi_ap_record_t));
    if (recs == NULL) {
        ESP_LOGE(TAG, "calloc 失败");
        return false;
    }
    esp_wifi_scan_get_ap_records(&num, recs);

    ESP_LOGI(TAG, "共扫到 %u 个 AP：", num);
    bool found = false;
    for (int i = 0; i < num; i++) {
        int mine = (strcmp((char *)recs[i].ssid, WIFI_SSID) == 0);
        ESP_LOGI(TAG, "  [%d]%s ssid=\"%s\" rssi=%d ch=%d auth=%d(%s) pair=%d group=%d",
                 i, mine ? " <==" : "",
                 (char *)recs[i].ssid, recs[i].rssi, recs[i].primary,
                 (int)recs[i].authmode, authmode_name(recs[i].authmode),
                 (int)recs[i].pairwise_cipher, (int)recs[i].group_cipher);
        if (mine && !found) {
            *out = recs[i];
            found = true;
        }
    }
    free(recs);
    ESP_LOGI(TAG, "===== 扫描结束，%s =====", found ? "找到目标" : "没有找到目标");
    return found;
}

/* ---------- 主流程 ---------- */
void app_main(void)
{
    /* 1. NVS：WiFi 驱动需要用它存少量校准数据 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. 网络协议栈 + 默认事件循环 + STA 网卡对象 */
    s_wifi_evt = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* 3. WiFi 驱动初始化 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    /* 4. 填 SSID / 密码 */
    memcpy(s_wc.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
    memcpy(s_wc.sta.password, WIFI_PASS, strlen(WIFI_PASS));
    /* 门槛说明（踩过的坑）：
       1) 写 WIFI_AUTH_OPEN 没用——密码 ≥8 位时 IDF 会自动把门槛提到 WPA2，
          日志会打 "authmode threshold changes from OPEN to WPA2"；
       2) 树莓派那个热点会被扫成 WPA_PSK(WPA1)，高于 WPA2 的门槛会把它筛掉，
          表现为 reason=211（NO_AP_FOUND_IN_AUTHMODE_THRESHOLD）。
       所以这里显式写 WPA_PSK，让 WPA1/WPA2 都能过。 */
    s_wc.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;

    /* 5. 设成 STA 模式并启动 */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &s_wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 6. 关省电模式：否则延迟会从几毫秒涨到几十上百毫秒 */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* 7. 先扫描一次：既打印周围 AP，也把目标的 BSSID/信道抓出来 */
    wifi_ap_record_t target;
    if (find_target_ap(&target)) {
        ESP_LOGI(TAG, "锁定目标: bssid=%02x:%02x:%02x:%02x:%02x:%02x ch=%d rssi=%d",
                 target.bssid[0], target.bssid[1], target.bssid[2],
                 target.bssid[3], target.bssid[4], target.bssid[5],
                 target.primary, target.rssi);
        /* 直接点名 BSSID：跳过扫描阶段的加密方式筛选，也省掉一次扫描 */
        s_wc.sta.bssid_set = 1;
        memcpy(s_wc.sta.bssid, target.bssid, 6);
        s_wc.sta.channel = target.primary;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &s_wc));
        s_bssid_locked = true;
    } else {
        ESP_LOGW(TAG, "扫描没找到 %s，仍按普通方式尝试连接", WIFI_SSID);
    }

    /* 8. 再发起连接 */
    ESP_LOGI(TAG, "开始连接 %s ...", WIFI_SSID);
    esp_wifi_connect();

    /* 9. 主循环：连上 → 收帧 → 断了重连。本固件不发任何数据。 */
    int sock = -1;
    uint8_t frame[FRAME_LEN];

    while (1) {
        if (sock < 0) {
            /* 等 WiFi 拿到 IP，最多无限等 */
            xEventGroupWaitBits(s_wifi_evt, WIFI_CONNECTED_BIT,
                                pdFALSE, pdTRUE, portMAX_DELAY);
            sock = tcp_connect();
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(2000));   /* 2 秒后重试 */
                continue;
            }
        }

        int r = read_frame(sock, frame);
        if (r < 0) {
            ESP_LOGW(TAG, "TCP 断开，1 秒后重连（累计好帧 %u / 坏帧 %u）",
                     s_rx_frames, s_rx_bad);
            close(sock);
            sock = -1;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (r == 0) {
            /* 3 秒一个字节都没来：树莓派那边可能还没坐标源，或 bridge 没起 */
            ESP_LOGW(TAG, "%d 秒没收到帧（累计好帧 %u / 坏帧 %u）",
                     RECV_TIMEOUT_S, s_rx_frames, s_rx_bad);
        }
    }
}
