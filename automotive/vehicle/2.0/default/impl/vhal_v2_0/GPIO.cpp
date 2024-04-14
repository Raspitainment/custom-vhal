
#define LOG_TAG "DefaultVehicleHal_v2_0_GPIO"

#include <android-base/chrono_utils.h>
#include <android/hardware/automotive/vehicle/2.0/types.h>
#include <assert.h>
#include <stdio.h>
#include <unordered_set>
#include <utils/Log.h>
#include <utils/SystemClock.h>
#include <vhal_v2_0/RecurrentTimer.h>
#include <vhal_v2_0/VehicleUtils.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "FakeObd2Frame.h"
#include "PropertyUtils.h"
#include "VehicleUtils.h"

#include "DefaultVehicleHal.h"

namespace android {
namespace hardware {
namespace automotive {
namespace vehicle {
namespace V2_0 {
namespace impl {

struct Pin {
    bool isInput;
    uint8_t pin;
    int32_t fileDescriptor;
    VehicleProperty property;
    VehiclePropertyType type;
    std::function<void(bool, VehiclePropValuePtr)> inputValue;
    std::function<bool(VehiclePropValuePtr)> outputValue;
};

static std::vector<Pin> PINS = {
    {
        true,
        26,
        -1,
        VehicleProperty::NIGHT_MODE,
        VehiclePropertyType::INT32,
        [](bool gpioValue, VehiclePropValuePtr propValue) { propValue->value.int32Values[0] = gpioValue ? 1 : 0; },
        nullptr,
    },
};

GPIO::GPIO() {
    ALOGI("GPIO constructor");

    for (const auto &pin : PINS) {
        // export GPIO pin
        FILE *exportDevice = fopen("/sys/class/gpio/export", "w");
        if (exportDevice == NULL) {
            ALOGE("Failed to open /sys/class/gpio/export");
            continue;
        }

        if (fprintf(exportDevice, "%d", pin.pin) < 0) {
            ALOGE("Failed to export GPIO");
            fclose(exportDevice);
            continue;
        }

        fclose(exportDevice);

        // set direction of GPIO pin
        char path[256];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin.pin);
        FILE *directionDevice = fopen(path, "w");
        if (directionDevice == NULL) {
            ALOGE("Failed to open %s", path);
            continue;
        }

        if (fprintf(directionDevice, "in") < 0) {
            ALOGE("Failed to set direction of GPIO");
            fclose(directionDevice);
            continue;
        }

        fclose(directionDevice);

        // open GPIO pin
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin.pin);
        pin.fileDescriptor = fopen(path, "r");
        if (pin.fileDescriptor == NULL) {
            ALOGE("Failed to open %s", path);
            continue;
        }
    }
}

GPIO::~GPIO() {
    ALOGI("GPIO destructor");
    // do stuff here
}

bool GPIO::isHandled(VehicleProperty prop) {
    for (const auto &pin : PINS) {
        if (pin.property == prop) {
            return true;
        }
    }

    return false;
}

VehiclePropValuePtr GPIO::get(uint8_t pin, VehiclePropValuePool *pool) {
    for (const auto &pin : PINS) {
        if (pin.pin == pin) {
            if (!pin.isInput) {
                ALOGE("Cannot read from output pin %d", pin);
                return nullptr;
            }

            char gpioValue = '\0';

            if (fread(&gpioValue, 1, 1, pin.fileDescriptor) < 0) {
                ALOGE("Failed to read GPIO value for pin %d", pin);
                return nullptr;
            }

            VehiclePropValuePtr v = pool.obtain(pin.type);
            v->prop = static_cast<int32_t>(pin.property);
            v->timestamp = elapsedRealtimeNano();

            pin.inputValue(gpioValue == '1', v);
        }
    }
}

void GPIO::set(uint8_t pin, bool value) { ALOGI("GPIO set pin %d to %d", pin, value); }

void GPIO::writeAll(VehiclePropValuePool *pool, VehicleHalClient *vehicleClient) {
    for (const auto &pin : PINS) {
        if (pin.isOutput) {
            continue;
        }

        VehiclePropValuePtr v = this->get(pin.pin, pool);
        if (v == nullptr) {
            continue;
        }

        mVehicleClient->setProperty(*v, /*updateStatus=*/false);
    }
}

} // namespace impl
} // namespace V2_0
} // namespace vehicle
} // namespace automotive
} // namespace hardware
} // namespace android
