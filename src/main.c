#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/gap.h> /* needed for BT_LE_ADV_PARAM if available */

LOG_MODULE_REGISTER(main);

#define DHT22_NODE DT_ALIAS(dht22)

#if !DT_NODE_HAS_STATUS(DHT22_NODE, okay)
#error "DHT22 node is not okay"
#endif

static const struct gpio_dt_spec dht22 =
    GPIO_DT_SPEC_GET(DHT22_NODE, dio_gpios);

/* Last-measured values in tenths:
 * temperature: int16_t (tenths of °C, signed)
 * humidity: uint16_t (tenths of %RH)
 */
static int16_t last_temp_tenths = 0;
static uint16_t last_hum_tenths = 0;

static struct bt_conn *current_conn;
static bool temp_notify_enabled;
static bool hum_notify_enabled;

void dht22_start_signal(void) {
    gpio_pin_configure_dt(&dht22, GPIO_OUTPUT);
    gpio_pin_set_dt(&dht22, 0); // Pull low
    k_msleep(20);               // Hold low for at least 18ms

    gpio_pin_set_dt(&dht22, 1); // Pull high
    k_busy_wait(30);            // 20-40 us

    gpio_pin_configure_dt(&dht22, GPIO_INPUT); // Release & switch to input
}

int dht22_wait_for(int level, uint32_t timeout_us) {
    uint32_t count = 0;
    while (gpio_pin_get_dt(&dht22) != level) {
        if (count++ > timeout_us) {
            return -1;
        }
        k_busy_wait(1);
    }
    return 0;
}

int dht22_read_bit(void) {
    if (dht22_wait_for(1, 100))
        return -1;
    k_busy_wait(40); // wait 40us
    int bit = gpio_pin_get_dt(&dht22);
    if (dht22_wait_for(0, 100))
        return -1;
    return bit;
}

int dht22_read(uint8_t data[5]) {
    memset(data, 0, 5);
    dht22_start_signal();

    if (dht22_wait_for(0, 100))
        return -1; // DHT pulls low
    if (dht22_wait_for(1, 100))
        return -1; // DHT pulls high
    if (dht22_wait_for(0, 100))
        return -1; // DHT pulls low to start data

    for (int i = 0; i < 40; i++) {
        int bit = dht22_read_bit();
        if (bit < 0)
            return -1;

        data[i / 8] <<= 1;
        if (bit)
            data[i / 8] |= 1;
    }
    LOG_HEXDUMP_INF(data, sizeof(data), "Data Hexdump:");

    // Checksum
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4])
        return -2;

    return 0;
}

/* Helper: sample DHT22 and update last_* variables */
static int sample_dht22_and_update(void)
{
    uint8_t data[5];
    int rc = dht22_read(data);
    if (rc != 0) {
        LOG_ERR("DHT22 read failed (%d)", rc);
        return rc;
    }

    uint16_t raw_hum = (data[0] << 8) | data[1];
    uint16_t raw_temp = (data[2] << 8) | data[3];

    float hum = raw_hum * 0.1f;
    float temp = raw_temp * 0.1f;
    if (raw_temp & 0x8000) {
        temp = -temp;
    }

    last_hum_tenths = (uint16_t)(hum * 10.0f + 0.5f);
    last_temp_tenths = (int16_t)(temp * 10.0f + (temp >= 0 ? 0.5f : -0.5f));

    LOG_INF("Sampled Temp: %.1f C  Humidity: %.1f %%", (float)last_temp_tenths/10.0f, (float)last_hum_tenths/10.0f);
    return 0;
}

/* ===== BLE GATT service =====
 * Custom 128-bit UUIDs (replace if you want stable UUIDs)
 */
static struct bt_uuid_128 dht_svc_uuid = BT_UUID_INIT_128(
    0x23,0x01,0x45,0x67,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef);
static struct bt_uuid_128 temp_char_uuid = BT_UUID_INIT_128(
    0x23,0x02,0x45,0x67,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef);
static struct bt_uuid_128 hum_char_uuid = BT_UUID_INIT_128(
    0x23,0x03,0x45,0x67,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef);

/* Read handlers return the last-measured values (little-endian) */
static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    uint16_t val_le = sys_cpu_to_le16((uint16_t)last_temp_tenths);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val_le, sizeof(val_le));
}

static ssize_t read_hum(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    uint16_t val_le = sys_cpu_to_le16(last_hum_tenths);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val_le, sizeof(val_le));
}

/* CCC changed callbacks */
static void temp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    temp_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Temp notifications %s", temp_notify_enabled ? "enabled" : "disabled");
}

static void hum_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    hum_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Humidity notifications %s", hum_notify_enabled ? "enabled" : "disabled");
}

/* Service definition */
BT_GATT_SERVICE_DEFINE(dht_svc,
    BT_GATT_PRIMARY_SERVICE(&dht_svc_uuid),
    BT_GATT_CHARACTERISTIC(&temp_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, read_temp, NULL, NULL),
    BT_GATT_CCC(temp_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&hum_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, read_hum, NULL, NULL),
    BT_GATT_CCC(hum_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Connection callbacks */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    LOG_INF("Connected");
    if (current_conn) {
        bt_conn_unref(current_conn);
    }
    current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    /* clear notify enabled flags: centrals lost subscription */
    temp_notify_enabled = false;
    hum_notify_enabled = false;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

/* Helper to notify if enabled. We notify all subscribed connections (pass NULL). */
/* value is a pointer to 16-bit little-endian value. attr_index: 2 for temp val, 5 for hum val in our BT_GATT_SERVICE_DEFINE order */
static void notify_if_enabled(bool enabled, const void *value, size_t vlen, size_t attr_index)
{
    if (!enabled) {
        return;
    }

    /* The BT_GATT_SERVICE_DEFINE creates dht_svc.attrs[] array:
     * index mapping (service definition ordering):
     * 0: primary service
     * 1: temp characteristic declaration
     * 2: temp characteristic value  <-- notify this
     * 3: temp CCC
     * 4: hum characteristic declaration
     * 5: hum characteristic value   <-- notify this
     * 6: hum CCC
     */
    /* dht_svc.attrs is a pointer; use attr_count instead of ARRAY_SIZE */
    if (attr_index >= dht_svc.attr_count) {
         return;
     }
     const struct bt_gatt_attr *attr = &dht_svc.attrs[attr_index];
     int rc = bt_gatt_notify(current_conn, attr, value, vlen);
     if (rc) {
         LOG_ERR("bt_gatt_notify failed: %d", rc);
     }
}

int main(void) {
    if (!gpio_is_ready_dt(&dht22)) {
        LOG_ERR("GPIO not ready");
        return 0;
    }

    LOG_INF("NRF52 DK and DHT22 sensor interfacing (with BLE)");

    /* Initialize Bluetooth */
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (%d)", err);
    } else {
        LOG_INF("Bluetooth initialized");
    }

    bt_conn_cb_register(&conn_callbacks);

    /* Advertising data -- use runtime name length to avoid accidental oversized AD */
    const char *dev_name = bt_get_name();
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, dev_name, strlen(dev_name)),
    };

    
// #ifdef BT_LE_ADV_PARAM
//     /* Preferred: explicit adv param struct (non-deprecated). */
//     static const struct bt_le_adv_param adv_params = BT_LE_ADV_PARAM(
//         BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
//         BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);
//     err = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), NULL, 0);
// #else
//     /* Fallback for older SDKs where BT_LE_ADV_PARAM isn't present */
//     err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
// #endif

    err = bt_le_adv_start(
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY,
                    BT_GAP_ADV_FAST_INT_MIN_2,
                    BT_GAP_ADV_FAST_INT_MAX_2,
                    NULL),
                    ad, ARRAY_SIZE(ad), NULL, 0);

    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        /* EINVAL often means AD too big or invalid params — print name length & AD size */
        LOG_INF("dev_name='%s' len=%zu ad_entries=%zu", dev_name, strlen(dev_name), ARRAY_SIZE(ad));
    } else {
        LOG_INF("Advertising started");
    }

    /* warm sample */
    sample_dht22_and_update();

    while (1) {
        int rc = sample_dht22_and_update();
        if (rc == 0) {
            /* Prepare little-endian 16-bit values */
            uint16_t temp_le = sys_cpu_to_le16((uint16_t)last_temp_tenths);
            uint16_t hum_le = sys_cpu_to_le16(last_hum_tenths);

            /* Notify if subscribed:
             * temp value attr index = 2, hum value attr index = 5 (see comment above)
             */
            notify_if_enabled(temp_notify_enabled, &temp_le, sizeof(temp_le), 2);
            notify_if_enabled(hum_notify_enabled, &hum_le, sizeof(hum_le), 5);
        } else {
            LOG_WRN("Skipping notify due to sensor read error");
        }

        k_sleep(K_SECONDS(5));
    }

    return 0;
}
