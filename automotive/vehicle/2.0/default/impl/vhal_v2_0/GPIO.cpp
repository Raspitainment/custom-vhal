
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
    FILE *fileDescriptor;
    VehicleProperty property;
    VehiclePropertyType type;
    std::function<VehicleHal::VehiclePropValuePtr(bool, VehicleHal::VehiclePropValuePtr)> inputValue;
    std::function<bool(const VehiclePropValue &)> outputValue;
};

std::vector<Pin> initPins() {
    return std::vector<Pin>{
        (struct Pin){
            true,
            26,
            nullptr,
            VehicleProperty::NIGHT_MODE,
            VehiclePropertyType::INT32,
            [](bool gpioValue, VehicleHal::VehiclePropValuePtr propValue) {
                propValue->value.int32Values[0] = gpioValue ? 0 : 1;
                return propValue;
            },
            nullptr,
        },
        (struct Pin){
            false,
            19,
            nullptr,
            VehicleProperty::HVAC_AC_ON,
            VehiclePropertyType::INT32,
            nullptr,
            [](std::unique_ptr<VehiclePropValue> propValue) { return propValue->value.int32Values[0] == 1; },
        }};
}

std::vector<Pin> PINS = initPins();

GPIO::GPIO() {
    ALOGI("GPIO constructor");

    for (auto &pin : PINS) {
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

        const char *direction = pin.isInput ? "in" : "out";
        if (fprintf(directionDevice, "%s", direction) < 0) {
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

bool GPIO::isHandled(int prop) {
    ALOGI("GPIO isHandled %d", prop);
    for (const auto &pin : PINS) {
        if (static_cast<int32_t>(pin.property) == prop) {
            return pin.isInput;
        }
    }

    return false;
}

VehicleHal::VehiclePropValuePtr GPIO::get(uint8_t pin_number, VehiclePropValuePool *pool) {
    ALOGI("GPIO get pin %d", pin_number);
    for (const auto &pin : PINS) {
        if (pin.pin == pin_number) {
            if (!pin.isInput) {
                ALOGE("Cannot read from output pin %d", pin.pin);
                return nullptr;
            }

            rewind(pin.fileDescriptor);

            char gpioValue = '\0';

            if (fread(&gpioValue, 1, 1, pin.fileDescriptor) < 0) {
                ALOGE("Failed to read GPIO value for pin %d", pin.pin);
                return nullptr;
            }

            ALOGI("Read ascii value %d from pin %d", gpioValue, pin.pin);

            VehicleHal::VehiclePropValuePtr v = pool->obtain(pin.type);
            v->prop = static_cast<int32_t>(pin.property);
            v->timestamp = elapsedRealtimeNano();

            return pin.inputValue(gpioValue == '1', std::move(v));
        }
    }

    return nullptr;
}

void GPIO::writeAll(VehiclePropValuePool *pool, VehicleHalClient *vehicleClient) {
    ALOGI("GPIO writeAll");
    for (const auto &pin : PINS) {
        if (!pin.isInput) {
            continue;
        }
        ALOGI("Writing value from pin %d", pin.pin);

        VehicleHal::VehiclePropValuePtr v = this->get(pin.pin, pool);
        if (v == nullptr) {
            continue;
        }

        vehicleClient->setProperty(*v, /*updateStatus=*/false);
    }
}

void GPIO::read(const VehiclePropValue &propValue) {
    ALOGI("GPIO read %d", propValue.prop);
    for (const auto &pin : PINS) {
        if (pin.isInput || static_cast<int32_t>(pin.property) != propValue.prop) {
            continue;
        }

        bool gpioValue = pin.outputValue(propValue);
        ALOGI("Writing value %d to pin %d", gpioValue, pin.pin);

        rewind(pin.fileDescriptor);

        if (fprintf(pin.fileDescriptor, "%d", gpioValue) < 0) {
            ALOGE("Failed to write GPIO value for pin %d", pin.pin);
        }
    }
}

} // namespace impl
} // namespace V2_0
} // namespace vehicle
} // namespace automotive
} // namespace hardware
} // namespace android
