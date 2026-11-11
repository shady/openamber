#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

static uint32_t now = 100;
static uint32_t millis() { return now; }
#define id(x) x
#define ESP_LOGI(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)
#define ESP_LOGW(...) ((void) 0)
#define ESP_LOGE(...) ((void) 0)
static float ble_temp_value, ble_hum_value;
static uint64_t ble_paired_models[3] = {};
static const int MALLOC_CAP_INTERNAL = 1, MALLOC_CAP_DMA = 2;
static size_t heap_caps_get_largest_free_block(int) { return 65536; }
static const int ESP_BT_CONTROLLER_STATUS_IDLE = 0, ESP_BT_CONTROLLER_STATUS_INITED = 1;
static const int ESP_BT_CONTROLLER_STATUS_ENABLED = 2, ESP_BT_MODE_BLE = 1, ESP_OK = 0;
using esp_err_t = int;
using esp_bt_controller_config_t = int;
#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() 0
static int esp_bt_controller_get_status() { return ESP_BT_CONTROLLER_STATUS_ENABLED; }
static int esp_bt_controller_init(int *) { return ESP_OK; }
static int esp_bt_controller_enable(int) { return ESP_OK; }
static const char *esp_err_to_name(int) { return "error"; }
static void vTaskDelay(int) {}

namespace esphome::esp32_ble
{
struct BLE
{
    bool active = false;
    void enable() { active = true; }
    void disable() { active = false; }
    bool is_failed() const { return false; }
    bool is_active() const { return active; }
};
static BLE ble;
static BLE *global_ble = &ble;
}
namespace esphome::esp32_ble_tracker
{
enum class ScannerState { IDLE, STARTING, RUNNING, STOPPING, FAILED };
struct Tracker
{
    bool active = false, continuous = true;
    ScannerState state = ScannerState::IDLE;
    unsigned stops = 0;
    bool get_scan_active() const { return active; }
    ScannerState get_scanner_state() const { return state; }
    void set_scan_active(bool value) { active = value; }
    void set_scan_continuous(bool value) { continuous = value; }
    void stop_scan()
    {
        continuous = false;
        if (state == ScannerState::IDLE || state == ScannerState::STOPPING)
            return;
        assert(state == ScannerState::RUNNING);
        state = ScannerState::STOPPING;
        stops++;
    }
    void settle()
    {
        if (state == ScannerState::STOPPING)
            state = ScannerState::IDLE;
        if (continuous)
            state = ScannerState::RUNNING;
    }
};
static Tracker tracker;
static Tracker *global_esp32_ble_tracker = &tracker;
}

#include "openamber/controllers/switchbot_ble.h"

using namespace esphome::esp32_ble_tracker;
static std::string macs[3] = {"AA:BB:CC:DD:EE:01", "", ""};
static const std::string *addresses[3] = {&macs[0], &macs[1], &macs[2]};
static const uint8_t address[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};
struct UUID
{
    uint16_t value;
    static UUID from_uint16(uint16_t value) { return {value}; }
    bool operator==(const UUID &other) const { return value == other.value; }
};
struct Data { UUID uuid; std::vector<uint8_t> data; };
using Datas = std::vector<Data>;
static const Datas manufacturer = {{UUID{0x0969}, {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01, 0, 0, 8, 0x93, 48}}};
static const Datas model = {{UUID{0xFD3D}, {0x77, 0xC0, 24}}};
static void update(bool enabled = true)
{
    uint64_t *models[3] = {&ble_paired_models[0], &ble_paired_models[1], &ble_paired_models[2]};
    switchbot_update(enabled, 0, addresses, models);
}
static void identify()
{
    switchbot_on_advertise(address, Datas{}, model);
    update();
    tracker.settle();
}

static void reboot()
{
    for (auto &slot : switchbot_slots)
        slot = {};
    for (auto &device : switchbot_devices)
        device = {};
    switchbot_discovering = false;
    switchbot_ble_requested = false;
    esphome::esp32_ble::ble.active = false;
    tracker = {};
    now += 1000;
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    std::string test = argv[1];
    switchbot_boot_ready = true;
    if (test == "rollover")
        now = UINT32_MAX - 30000;
    if (test == "zero")
        now = 0;
    update();
    tracker.settle();
    assert(tracker.active);

    if (test == "identification")
    {
        // Separate advertisement and response: provisional data is not lost.
        switchbot_on_advertise(address, manufacturer, Datas{});
        assert(!switchbot_reading_fresh());
        identify();
        assert(!tracker.active && switchbot_reading_fresh());
        assert(std::abs(ble_temp_value - 19.8f) < 0.01f);
        now += 700000;
        update();
        assert(!switchbot_reading_fresh() && switchbot_reading_missing());
        assert(!tracker.active);
        switchbot_on_advertise(address, manufacturer, Datas{});
        update();
        assert(switchbot_reading_fresh() && !switchbot_reading_missing());
        assert(!tracker.active);
    }
    else if (test == "discovery")
    {
        identify();
        switchbot_begin_discovery();
        update();
        tracker.settle();
        assert(tracker.active);
        const uint8_t second[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02};
        switchbot_on_advertise(second, Datas{}, model);
        switchbot_on_advertise(second, manufacturer, Datas{});
        macs[1] = "AA:BB:CC:DD:EE:02";
        update();
        assert(switchbot_slots[1].model == 0x77);
        now += SWITCHBOT_DISCOVERY_MS;
        update();
        tracker.settle();
        assert(!switchbot_is_discovering() && !tracker.active);
    }
    else if (test == "timeout" || test == "rollover")
    {
        now += SWITCHBOT_DISCOVERY_MS - 1;
        update();
        assert(tracker.active);
        now++;
        update();
        tracker.settle();
        assert(!tracker.active);
        // The expired window must not reopen at the next counter wrap.
        now = switchbot_slots[0].tracking_since_ms + 1;
        update();
        assert(!tracker.active);
        switchbot_begin_discovery();
        update();
        tracker.settle();
        assert(tracker.active);
    }
    else if (test == "replacement")
    {
        identify();
        macs[0] = "AA:BB:CC:DD:EE:02";
        update();
        tracker.settle();
        assert(tracker.active && switchbot_slots[0].model == 0);
        macs[0] = "invalid";
        update();
        tracker.settle();
        assert(!tracker.active && !switchbot_slots[0].valid);
    }
    else if (test == "disable")
    {
        switchbot_begin_discovery();
        update(false);
        assert(!esphome::esp32_ble::ble.active);
        assert(!switchbot_is_discovering() && !switchbot_slots[0].valid);
        tracker.state = ScannerState::IDLE;
        update();
        tracker.settle();
        identify();
        assert(!tracker.active);
    }
    else if (test == "transition")
    {
        tracker.state = ScannerState::STARTING;
        switchbot_on_advertise(address, Datas{}, model);
        update();
        assert(tracker.active && tracker.stops == 0);
        tracker.state = ScannerState::RUNNING;
        update();
        assert(!tracker.active && tracker.state == ScannerState::STOPPING);
        assert(tracker.continuous && tracker.stops == 1);
        switchbot_begin_discovery();
        update();
        assert(!tracker.active && tracker.stops == 1);
        tracker.settle();
        update();
        assert(tracker.active && tracker.stops == 2);
    }
    else if (test == "ota")
    {
        // OTA has stopped scanning. A policy update must not restart it.
        tracker.stop_scan();
        tracker.settle();
        identify();
        assert(!tracker.active && !tracker.continuous);
        assert(tracker.state == ScannerState::IDLE);
        // ESPHome resumes continuous scanning after a failed upload.
        tracker.set_scan_continuous(true);
        tracker.settle();
        assert(tracker.state == ScannerState::RUNNING && !tracker.active);
    }
    else if (test == "service_only")
    {
        const Datas meter = {{UUID{0x000D}, {0x54, 0, 100, 8, 0x93, 48}}};
        switchbot_on_advertise(address, Datas{}, meter);
        update();
        tracker.settle();
        assert(switchbot_reading_fresh() && !tracker.active);
        now += SWITCHBOT_READING_TIMEOUT_MS;
        update();
        assert(switchbot_reading_missing() && !tracker.active);
    }
    else if (test == "unidentified")
    {
        now += SWITCHBOT_READING_TIMEOUT_MS;
        switchbot_on_advertise(address, manufacturer, Datas{});
        update();
        assert(!switchbot_reading_fresh() && switchbot_reading_missing());
        assert(std::isnan(ble_temp_value) && !tracker.active);
    }
    else if (test == "mixed_slots")
    {
        macs[1] = "AA:BB:CC:DD:EE:02";
        update();
        identify();
        assert(tracker.active); // The second slot still needs its model.
        now += SWITCHBOT_DISCOVERY_MS;
        update();
        tracker.settle();
        assert(!tracker.active && switchbot_slots[0].model == 0x77);
        unsigned stops = tracker.stops;
        update();
        assert(tracker.stops == stops);
    }
    else if (test == "zero")
    {
        now = SWITCHBOT_DISCOVERY_MS - 1;
        update();
        assert(tracker.active);
        now = SWITCHBOT_DISCOVERY_MS + 1;
        update();
        assert(!tracker.active);
    }
    else if (test == "empty")
    {
        macs[0].clear();
        update();
        tracker.settle();
        assert(!tracker.active);
        switchbot_begin_discovery();
        update();
        tracker.settle();
        assert(tracker.active); // Discovery works before anything is paired.
    }
    else if (test == "restart")
    {
        identify();
        assert(ble_paired_models[0] == 0xAABBCCDDEE0177ULL);
        reboot();
        update();
        tracker.settle();
        assert(!tracker.active && switchbot_slots[0].model == 0x77);
        assert(!switchbot_reading_fresh()); // Never restore an old temperature.
        switchbot_on_advertise(address, manufacturer, Datas{});
        update();
        assert(switchbot_reading_fresh() && !tracker.active);
    }
    else if (test == "replace_saved")
    {
        identify();
        macs[0] = "AA:BB:CC:DD:EE:02";
        update();
        assert(ble_paired_models[0] == 0 && switchbot_slots[0].model == 0);
        macs[0] = "AA:BB:CC:DD:EE:01";
        update();
        tracker.settle();
        assert(tracker.active && switchbot_slots[0].model == 0);
    }
    else if (test == "unpair_disabled")
    {
        identify();
        update(false);
        assert(ble_paired_models[0] != 0); // Bluetooth off preserves pairing.
        macs[0].clear();
        update(false);
        assert(ble_paired_models[0] == 0);
        reboot();
        macs[0] = "AA:BB:CC:DD:EE:01";
        update();
        assert(tracker.active && switchbot_slots[0].model == 0);
    }
    else if (test == "invalid_saved")
    {
        ble_paired_models[0] = 0xAABBCCDDEE0142ULL;
        reboot();
        update();
        assert(ble_paired_models[0] == 0 && switchbot_slots[0].model == 0);
        ble_paired_models[0] = 0xAABBCCDDEE0277ULL;
        reboot();
        update();
        assert(ble_paired_models[0] == 0 && switchbot_slots[0].model == 0);
    }
    else if (test == "all_slots")
    {
        identify();
        macs[1] = "AA:BB:CC:DD:EE:02";
        macs[2] = "AA:BB:CC:DD:EE:03";
        update();
        const uint8_t second[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02};
        const uint8_t third[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03};
        switchbot_on_advertise(second, Datas{}, Datas{{UUID{0xFD3D}, {0x69}}});
        switchbot_on_advertise(third, Datas{}, Datas{{UUID{0x000D}, {0x54}}});
        update();
        reboot();
        update();
        assert(!tracker.active);
        assert(switchbot_slots[0].model == 0x77);
        assert(switchbot_slots[1].model == 0x69);
        assert(switchbot_slots[2].model == 0x54);
        macs[1].clear();
        update();
        assert(ble_paired_models[1] == 0);
        assert(ble_paired_models[0] == 0xAABBCCDDEE0177ULL);
        assert(ble_paired_models[2] == 0xAABBCCDDEE0354ULL);
    }
    else
        assert(false);
}
