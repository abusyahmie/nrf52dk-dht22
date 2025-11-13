#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <string.h>

LOG_MODULE_REGISTER(sensor_ble_template, LOG_LEVEL_INF);

/* -- Configuration defaults -- */
#define DEFAULT_SAMPLE_PERIOD_S 5   /* seconds */
#define MIN_SAMPLE_PERIOD_S     1
#define MAX_SAMPLE_PERIOD_S     3600

/* Last-measured values in tenths (user-chosen format) */
static int16_t last_temp_tenths = 0;
static uint16_t last_hum_tenths = 0;

/* Protect shared values */
static struct k_mutex data_lock;

/* Sampling period (seconds) */
static atomic_t sample_period_s;

/* Connection + subscription state */
static struct bt_conn *current_conn;
static bool temp_notify_enabled;
static bool hum_notify_enabled;

/* Forward declarations */
static void sample_and_notify(struct k_work *work);

/* Work item for periodic sampling */
static struct k_work_delayable sample_work;

/* ======= GATT UUIDs (custom example) ======= */
static struct bt_uuid_128 svc_uuid = BT_UUID_INIT_128(
    0x23,0x01,0x45,0x67,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef);
static struct bt_uuid_128 temp_uuid = BT_UUID_INIT_128(
    0x23,0x02,0x45,0x67,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef);
static struct bt_uuid_128 hum_uuid = BT_UUID_INIT_128(
    0x23,0x03,0x45,0x67,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef);
static struct bt_uuid_128 sampling_period_uuid = BT_UUID_INIT_128(
    0x23,0x04,0x45,0x67,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef);

/* ---------- GATT read handlers ---------- */
static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    uint16_t temp_le;
    k_mutex_lock(&data_lock, K_FOREVER);
    temp_le = sys_cpu_to_le16((uint16_t)last_temp_tenths);
    k_mutex_unlock(&data_lock);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_le, sizeof(temp_le));
}

static ssize_t read_hum(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    uint16_t hum_le;
    k_mutex_lock(&data_lock, K_FOREVER);
    hum_le = sys_cpu_to_le16(last_hum_tenths);
    k_mutex_unlock(&data_lock);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &hum_le, sizeof(hum_le));
}

/* Sampling period read/write handler (uint16_t seconds) */
static ssize_t read_sample_period(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
    uint16_t sp = (uint16_t)atomic_get(&sample_period_s);
    uint16_t le = sys_cpu_to_le16(sp);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &le, sizeof(le));
}

static ssize_t write_sample_period(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset != 0 || len != sizeof(uint16_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    uint16_t in_le;
    memcpy(&in_le, buf, sizeof(in_le));
    uint16_t new_sp = sys_le16_to_cpu(in_le);
    if (new_sp < MIN_SAMPLE_PERIOD_S || new_sp > MAX_SAMPLE_PERIOD_S) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    atomic_set(&sample_period_s, new_sp);
    LOG_INF("Sampling period set to %u s", new_sp);

    /* reschedule sampling work with new period */
    k_work_reschedule(&sample_work, K_SECONDS(new_sp));
    return len;
}

/* CCC changed callbacks */
static void temp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    temp_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Temp notify %s", temp_notify_enabled ? "on" : "off");
}

static void hum_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    hum_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Hum notify %s", hum_notify_enabled ? "on" : "off");
}

/* ---------- GATT service definition ---------- */
BT_GATT_SERVICE_DEFINE(sensor_svc,
    BT_GATT_PRIMARY_SERVICE(&svc_uuid),
    /* Temperature characteristic: uint16_t (tenths), read + notify */
    BT_GATT_CHARACTERISTIC(&temp_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, read_temp, NULL, NULL),
    BT_GATT_CCC(temp_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Humidity characteristic: uint16_t (tenths), read + notify */
    BT_GATT_CHARACTERISTIC(&hum_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, read_hum, NULL, NULL),
    BT_GATT_CCC(hum_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Sampling period: uint16_t seconds, read + write */
    BT_GATT_CHARACTERISTIC(&sampling_period_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_sample_period,
                           write_sample_period, NULL),
);

/* Attribute indexes in sensor_svc.attrs:
 * 0: primary service
 * 1: temp char decl
 * 2: temp value   <-- notify attr index = 2
 * 3: temp CCC
 * 4: hum char decl
 * 5: hum value    <-- notify attr index = 5
 * 6: hum CCC
 * 7: sampling period char decl
 * 8: sampling period value
 */

/* ---------- Connection callbacks ---------- */
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
    LOG_INF("Disconnected (0x%02x)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    temp_notify_enabled = false;
    hum_notify_enabled = false;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ---------- Sensor sampling API (plug your sensor code here) ---------- */
/* Implement your own sensor read. Return 0 on success.
 * temp_tenths: signed tenths of degree Celsius (e.g., 215 => 21.5C)
 * hum_tenths: unsigned tenths of %RH (e.g., 553 => 55.3%)
 */
static int sensor_read(int16_t *temp_tenths, uint16_t *hum_tenths)
{
    /* Placeholder implementation: replace with GPIO/I2C/SPI sensor read logic */
    static int16_t t = 200; /* 20.0C */
    static uint16_t h = 500; /* 50.0% */
    t += 1; if (t > 300) t = 200;
    h += 2; if (h > 800) h = 500;
    *temp_tenths = t;
    *hum_tenths = h;
    return 0;
}

/* ---------- Notify helper ---------- */
static void notify_if_enabled(bool enabled, size_t attr_index, const void *value, size_t vlen)
{
    if (!enabled || !current_conn) {
        return;
    }

    if (attr_index >= sensor_svc.attr_count) { /* attr_count is runtime */
        return;
    }

    const struct bt_gatt_attr *attr = &sensor_svc.attrs[attr_index];
    int rc = bt_gatt_notify(current_conn, attr, value, vlen);
    if (rc) {
        LOG_ERR("bt_gatt_notify failed: %d", rc);
    }
}

/* ---------- Sampling work handler ---------- */
static void sample_and_notify(struct k_work *work)
{
    ARG_UNUSED(work);

    int16_t temp;
    uint16_t hum;
    if (sensor_read(&temp, &hum) == 0) {
        k_mutex_lock(&data_lock, K_FOREVER);
        last_temp_tenths = temp;
        last_hum_tenths = hum;
        k_mutex_unlock(&data_lock);

        /* Prepare little-endian values */
        uint16_t temp_le = sys_cpu_to_le16((uint16_t)last_temp_tenths);
        uint16_t hum_le = sys_cpu_to_le16(last_hum_tenths);

        notify_if_enabled(temp_notify_enabled, 2, &temp_le, sizeof(temp_le)); /* temp attr idx = 2 */
        notify_if_enabled(hum_notify_enabled, 5, &hum_le, sizeof(hum_le));     /* hum attr idx = 5 */
    } else {
        LOG_WRN("Sensor read failed");
    }

    /* Reschedule according to sample_period_s */
    uint32_t sp = (uint32_t)atomic_get(&sample_period_s);
    k_work_reschedule(&sample_work, K_SECONDS(sp));
}

/* ---------- Initialization & advertising ---------- */
static void start_advertising(void)
{
    const char *name = bt_get_name();
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, name, strlen(name)),
    };

    int err = bt_le_adv_start(
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
                        BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL),
        ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
    } else {
        LOG_INF("Advertising started");
    }
}

int init_ble_sensor(void)
{
    int err;

    k_mutex_init(&data_lock);
    atomic_set(&sample_period_s, DEFAULT_SAMPLE_PERIOD_S);
    k_work_init_delayable(&sample_work, sample_and_notify);

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed: %d", err);
        return err;
    }
    LOG_INF("Bluetooth initialized");

    bt_conn_cb_register(&conn_callbacks);

    /* start advertising */
    start_advertising();

    /* initial immediate sample then schedule recurring */
    k_work_schedule(&sample_work, K_NO_WAIT);

    return 0;
}

/* Example app entry: call init_ble_sensor() from main() of your app */