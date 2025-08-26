#include <stdio.h>
#include "ble_ctl.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "sdkconfig.h"
#include "nvs_manager.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG                         "ble"

#define BLE_TX_TASK_PERIOD          250u
#define BLE_TX_TASK_PRIO            15u

// Define connection parameters for high throughput
#define MIN_CONN_INTERVAL 0x0006  // 7.5ms
#define MAX_CONN_INTERVAL 0x0006  // 7.5ms
#define SLAVE_LATENCY     0
#define SUPERVISION_TIMEOUT 0x0100 // 4 seconds

static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
static uint8_t own_addr_type;
int gatt_svr_register(void);
QueueHandle_t spp_common_uart_queue = NULL;
static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
static uint16_t ble_spp_svc_gatt_read_val_handle;

void ble_store_config_init(void);

static void set_ble_conn_status(bool status)
{
    //set_coil(MB_COILS_BLE_STATUS, status);
    uint16_t ble_con_status = BLE_STATUS_DISCONNECTED;
   // set_holding_regs(MB_HOLDING_REG_BLE_CONN_STATUS, &ble_con_status, 1U);
}
static void set_ble_pair_req(bool status)
{
    //set_coil(MB_COILS_BLE_PAIR_REQ, status);
}

static void set_pass_key_to_hmi(uint32_t passkey)
{
    #if 0
    uint16_t val = (passkey >> 16u) & 0xFFFF;
    set_holding_regs(MB_HOLDING_REG_BLE_PASS_MSB, &val, 1u);
    val = passkey & 0xFFFF;
    set_holding_regs(MB_HOLDING_REG_BLE_PASS_LSB, &val, 1u);
    #endif
}

static uint32_t generate_random_passkey() {
    uint32_t passkey;
    // Generate a random number and fit it into the range 100000-999999
    do {
        passkey = esp_random() % 1000000; // Generate a random number between 0 and 999999
    } while (passkey < 100000); // Ensure it's at least 100000
    return passkey;
}

static void print_addr(const void *addr)
{
	const uint8_t *u8p;

	u8p = addr;
	ESP_LOGV(TAG, "%02x:%02x:%02x:%02x:%02x:%02x",
		       u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

/**
 * Logs information about a connection to the console.
 */
static void
ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */

static void
ble_spp_server_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields  fields;
    const char               *name;
    int                       rc;

    memset(&fields, 0, sizeof fields);

    fields.flags                 = BLE_HS_ADV_F_DISC_GEN  |
                                   BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name                         = ble_svc_gap_device_name();
    fields.name                  = (uint8_t *)name;
    fields.name_len              = strlen(name);
    fields.name_is_complete      = 1;

    /* Advertise the empty service using 128-bit UUID */
    fields.uuids128 = (ble_uuid128_t[]) {
        BLE_UUID128_INIT(
            0xB4, 0x09, 0x8D, 0xE0, 0x61, 0x08, 0x66, 0xBA,
            0x61, 0x4B, 0x3C, 0xF1, 0x5A, 0xAD, 0x66, 0x99
        )
    };
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "advertising data set failed; rc=%d\n", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_spp_server_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "advertising start failed; rc=%d\n", rc);
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * ble_spp_server uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  ble_spp_server.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
ble_spp_server_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;
    uint16_t ble_con_status = 0U;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        MODLOG_DFLT(INFO, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            ble_spp_server_print_conn_desc(&desc);
            set_ble_conn_status(true);
            //ble_gap_security_initiate(event->connect.conn_handle);
        }
        MODLOG_DFLT(INFO, "\n");
        if (event->connect.status != 0 || CONFIG_BT_NIMBLE_MAX_CONNECTIONS > 1) {
            /* Connection failed or if multiple connection allowed; resume advertising. */
            ble_spp_server_advertise();
            set_ble_conn_status(false);
        }
        ble_con_status = BLE_STATUS_CONNECTED;
        //set_holding_regs(MB_HOLDING_REG_BLE_CONN_STATUS, &ble_con_status, 1U);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        ble_spp_server_print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");

        conn_handle_subs[event->disconnect.conn.conn_handle] = false;

        /* Connection terminated; resume advertising. */
        ble_spp_server_advertise();

        set_ble_pair_req(false);
        set_ble_conn_status(false);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        MODLOG_DFLT(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        ble_spp_server_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "advertise complete; reason=%d",
                    event->adv_complete.reason);
        ble_spp_server_advertise();
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                    "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        conn_handle_subs[event->subscribe.conn_handle] = true;
        return 0;
    case BLE_GAP_EVENT_PASSKEY_ACTION:
        MODLOG_DFLT(INFO, "PASSKEY_ACTION_EVENT started \n");
        struct ble_sm_io pkey = {0};
        int key = 0;

        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            pkey.action = event->passkey.params.action;
            pkey.passkey = generate_random_passkey(); // This is the passkey to be entered on peer
            set_pass_key_to_hmi(pkey.passkey);
            MODLOG_DFLT(INFO, "Enter passkey %" PRIu32 " on the peer side\n", pkey.passkey);
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            MODLOG_DFLT(INFO, "ble_sm_inject_io result: %d\n", rc);
            set_ble_pair_req(true);
        } else if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            MODLOG_DFLT(INFO, "Passkey on device's display: %" PRIu32 , event->passkey.params.numcmp);
            MODLOG_DFLT(INFO, "Accept or reject the passkey through console in this format -> key Y or key N");
            pkey.action = event->passkey.params.action;
            pkey.numcmp_accept = 0;
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            MODLOG_DFLT(INFO, "ble_sm_inject_io result: %d\n", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_OOB) {
            static uint8_t tem_oob[16] = {0};
            pkey.action = event->passkey.params.action;
            for (int i = 0; i < 16; i++) {
                pkey.oob[i] = tem_oob[i];
            }
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            MODLOG_DFLT(INFO, "ble_sm_inject_io result: %d\n", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_INPUT) {
            MODLOG_DFLT(INFO, "Enter the passkey through console in this format-> key 123456");
            pkey.action = event->passkey.params.action;
            pkey.passkey = 0;
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            MODLOG_DFLT(INFO, "ble_sm_inject_io result: %d\n", rc);
        }
        return 0;
    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        MODLOG_DFLT(INFO, "encryption change event; status=%d \n",
                    event->enc_change.status);
        set_ble_pair_req(false);
        if(0 != event->enc_change.status)
        {
            set_ble_conn_status(false);
        }
        return 0;
    default:
        return 0;
    }
}

static void
ble_spp_server_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
ble_spp_server_on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");
    /* Begin advertising. */
    ble_spp_server_advertise();
}

void ble_spp_server_host_task(void *param)
{
    MODLOG_DFLT(INFO, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

/* Callback function for custom service */
static int  ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{    
    static uint16_t buf_len = 0u;
    static uint16_t data_len = 0u;
    static uint8_t data[512u];
    switch (ctxt->op) 
    {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            MODLOG_DFLT(DEBUG, "Callback for read");
            int rc = 0;
            if(0 == memcmp(&data, "ver", sizeof("ver")))
            {
                //int rc = os_mbuf_append(ctxt->om, CONFIG_APP_PROJECT_VER, sizeof(CONFIG_APP_PROJECT_VER));
            }
            else
            {
                data_len = sizeof(data);
                read_property(ctxt->chr->uuid, &data[0U], &data_len);
                ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);
                rc = os_mbuf_append(ctxt->om, data, data_len);
            }
            
            ESP_LOGI(TAG, "RC %d", rc);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            MODLOG_DFLT(INFO, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);            
            //Save data for a read back
            buf_len = ctxt->om->om_len;
            data_len = ctxt->om->om_len;
            memcpy(data, ctxt->om->om_data, data_len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_DEBUG);
            write_property(ctxt->chr->uuid, data, data_len);       //Send to MB stack
            break;

        default:
            MODLOG_DFLT(INFO, "\nDefault Callback");
            break;
    }
    return 0;
}

/* Define new custom service */
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = SERVICE_UUID128(BLE_SVC_LEARNING_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid       = CHAR_UUID128(BLE_CHAR_LEARN_MODE_UUID16, BLE_SVC_LEARNING_UUID16),
                .access_cb  = ble_svc_gatt_handler,
                .val_handle = &learn_mode_handle,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            { 0 } /* No more characteristics */
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = SERVICE_UUID128(BLE_SVC_CONTROL_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_BUTTON_UP_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &button_up_handle,
              .flags      = BLE_GATT_CHR_F_WRITE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_BUTTON_DOWN_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &button_down_handle,
              .flags      = BLE_GATT_CHR_F_WRITE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_BUTTON_STOP_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &button_stop_handle,
              .flags      = BLE_GATT_CHR_F_WRITE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_BUTTON_AUX_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &button_aux_handle,
              .flags      = BLE_GATT_CHR_F_WRITE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_LEARN_MODE_CTRL_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &learn_mode_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_AUTO_MODE_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &auto_mode_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_DEVICE_NAME_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &device_name_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_MOTOR_STATUS_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &motor_status_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_LIMITS_REACHED_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &limits_reached_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_DEVICE_STATUS_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &device_status_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE },
            { .uuid = CONTROL_CHAR_UUID128(BLE_CHAR_STATUS_REASON_UUID16, BLE_SVC_CONTROL_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &status_reason_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE },
            { 0 }
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = SERVICE_UUID128(BLE_SVC_EXPANDED_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            { .uuid = CHAR_UUID128(BLE_CHAR_WIFI_SSID_UUID16, BLE_SVC_EXPANDED_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &wifi_ssid_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
            { .uuid = CHAR_UUID128(BLE_CHAR_WIFI_PWD_UUID16, BLE_SVC_EXPANDED_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &wifi_password_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
            { .uuid = CHAR_UUID128(BLE_CHAR_ERROR_TRIGGER_UUID16, BLE_SVC_EXPANDED_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &error_logs_trigger_handle,
              .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_INDICATE },
            { .uuid = CHAR_UUID128(BLE_CHAR_ERROR_DATA_UUID16, BLE_SVC_EXPANDED_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &error_logs_data_handle,
              .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE },
            { .uuid = CHAR_UUID128(BLE_CHAR_LEVELING_BTN_UUID16, BLE_SVC_EXPANDED_UUID16),
              .access_cb  = ble_svc_gatt_handler,
              .val_handle = &leveling_button_handle,
              .flags      = BLE_GATT_CHR_F_WRITE },
            { 0 }
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_SVC_EMPTY_SERVICE_UUID128,
        .characteristics = (struct ble_gatt_chr_def[]){ { 0 } },
    },
    { 0 } /* No more services. */
};

static void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

static int gatt_svr_init(void)
{
    int rc = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

void init_ble(void)
{
    int rc;

    /* Initialize NVS — it is used to store PHY calibration data already done before*/
    esp_err_t ret;

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }

    /* Initialize connection_handle array */
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        conn_handle_subs[i] = false;
    }

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = ble_spp_server_on_reset;
    ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_ONLY;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_our_key_dist = 1;
    ble_hs_cfg.sm_their_key_dist = 1;
    ble_hs_cfg.sm_bonding = 0;
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_bonding = 1;
#endif
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_our_key_dist = 1;
    ble_hs_cfg.sm_their_key_dist = 1;
#endif

    /* Register custom service */
    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name. */
    static char macStr[64u] = {'\0'};
    //Check if the device name is set
    char* name_from_storage = nvs_manager_get_str(DEVICENAME_NAME_SPACE, DEVICENAME_KEY);
    if((NULL != name_from_storage) && (strlen(name_from_storage) > 5U))
    {
        //Use the name from nvs
        strncpy(macStr, name_from_storage, sizeof(macStr) - 1U);
        free(name_from_storage);
    }
    else
    {
        //Create the name
        uint8_t macBuf[6];
        esp_efuse_mac_get_default((uint8_t*) &macBuf);
        memset(macStr, 0, sizeof(macStr));
        snprintf(macStr, sizeof(macStr), "GEM");
        //Save to nvs for new boot
        nvs_manager_set_str(DEVICENAME_NAME_SPACE, DEVICENAME_KEY, macStr);
    }
    //rc = ble_svc_gap_device_name_set(&macStr[0u]);
    rc = ble_svc_gap_device_name_set("GEM");
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(ble_spp_server_host_task);
}
