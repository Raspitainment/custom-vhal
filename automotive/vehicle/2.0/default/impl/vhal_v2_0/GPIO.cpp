#include <linux/gpio.h>

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

enum PIN {
    PHOTO_DIODE = 26,
    LED_GREEN_1 = 19,
    LED_YELLOW_1 = 13,
    LED_RED_1 = 11,
    SWITCH_1_A = 6,
    SWITCH_1_B = 5,
    SWITCH_1_C = 0,
    LED_BLUE_1 = 9,
    LED_BLUE_2 = 10,
    LED_BLUE_3 = 22,
    SWITCH_2_A = 27,
    SWITCH_2_B = 17,
    SWITCH_2_C = 4,
};

struct InputPin {
    std::vector<enum PIN> pins;
    std::vector<FILE *> fileDescriptors;
    VehicleProperty property;
    VehiclePropertyType type;
    std::function<VehicleHal::VehiclePropValuePtr(std::vector<bool>, VehicleHal::VehiclePropValuePtr)> inputValue;

    // read the specified GPIO pins and compute the value of the property
    const VehicleHal::VehiclePropValuePtr read(VehiclePropValuePool *pool) const {
        ALOGI("Reading value of property %d from GPIO", static_cast<int32_t>(property));

        std::vector<bool> gpioValues = {};

        for (unsigned long i = 0; i < pins.size(); i++) {
            FILE *fileDescriptor = fileDescriptors[i];

            rewind(fileDescriptor);

            char gpioValue = '\0';
            if (fread(&gpioValue, 1, 1, fileDescriptor) < 0) {
                ALOGE("Failed to read GPIO value for pin %d", pins[i]);
                return nullptr;
            }

            gpioValues.push_back(gpioValue == '1');
        }

        VehicleHal::VehiclePropValuePtr v = pool->obtain(type);
        if (v == nullptr) {
            ALOGE("Failed to obtain VehiclePropValuePtr");
            return nullptr;
        }

        v->prop = static_cast<int32_t>(property);
        v->timestamp = elapsedRealtimeNano();

        return inputValue(gpioValues, std::move(v));
    }
};

struct OutputPin {
    enum PIN pin;
    FILE *fileDescriptor;
    VehicleProperty property;
    VehiclePropertyType type;
    std::function<bool(const VehiclePropValue &)> outputValue;

    // write the value of the property to the specified GPIO pin
    void write(const VehiclePropValue &propValue) const {
        ALOGI("Writing value of property %d to GPIO", static_cast<int32_t>(property));

        char gpioValue = outputValue(propValue) ? '1' : '0';
        ALOGI("Writing value %c to pin %d", gpioValue, pin);

        rewind(fileDescriptor);

        if (fprintf(fileDescriptor, "%c", gpioValue) < 0) {
            ALOGE("Failed to write GPIO value %c for pin %d", gpioValue, pin);
        }
    }
};

struct Pin {
    InputPin inputPin;
    OutputPin outputPin;
    bool isInput;
};

std::vector<Pin> initPins() {
    return std::vector<Pin>{
        (struct Pin){.isInput = true,
                     .inputPin =
                         (struct InputPin){
                             .pins = {PIN::PHOTO_DIODE},
                             .property = VehicleProperty::NIGHT_MODE,
                             .type = VehiclePropertyType::INT32,
                             .inputValue =
                                 [](std::vector<bool> gpioValues, VehicleHal::VehiclePropValuePtr propValue) {
                                     propValue->value.int32Values[0] = gpioValues[0] ? 1 : 0;
                                     return propValue;
                                 },

                         }},
        (struct Pin){.isInput = false,
                     .outputPin =
                         (struct OutputPin){
                             .pin = PIN::LED_BLUE_1,
                             .property = VehicleProperty::HVAC_AC_ON,
                             .type = VehiclePropertyType::INT32,
                             .outputValue =
                                 [](const VehiclePropValue &propValue) { return propValue.value.int32Values[0] == 0; },
                         }},
        (struct Pin){.isInput = false,
                     .outputPin =
                         (struct OutputPin){
                             .pin = PIN::LED_BLUE_2,
                             .property = VehicleProperty::HVAC_DEFROSTER,
                             .type = VehiclePropertyType::INT32,
                             .outputValue =
                                 [](const VehiclePropValue &propValue) { return propValue.value.int32Values[0] == 0; },
                         }},
        (struct Pin){.isInput = false,
                     .outputPin =
                         (struct OutputPin){
                             .pin = PIN::LED_BLUE_3,
                             .property = VehicleProperty::HVAC_RECIRC_ON,
                             .type = VehiclePropertyType::INT32,
                             .outputValue =
                                 [](const VehiclePropValue &propValue) { return propValue.value.int32Values[0] == 0; },
                         }},
        (struct Pin){.isInput = true,
                     .inputPin =
                         (struct InputPin){
                             .pins = {PIN::SWITCH_1_A, PIN::SWITCH_1_B, PIN::SWITCH_1_C},
                             .property = VehicleProperty::HVAC_FAN_SPEED,
                             .type = VehiclePropertyType::INT32,
                             .inputValue =
                                 [](std::vector<bool> gpioValues, VehicleHal::VehiclePropValuePtr propValue) {
                                     propValue->areaId = HVAC_ALL;
                                     propValue->value.int32Values[0] = gpioValues[2]   ? 4
                                                                       : gpioValues[1] ? 3
                                                                       : gpioValues[0] ? 2
                                                                                       : 1;
                                     return propValue;
                                 },
                         }},
        (struct Pin){.isInput = true,
                     .inputPin =
                         (struct InputPin){
                             .pins = {PIN::SWITCH_2_A, PIN::SWITCH_2_B, PIN::SWITCH_2_C},
                             .property = VehicleProperty::HVAC_SEAT_TEMPERATURE,
                             .type = VehiclePropertyType::INT32,
                             .inputValue =
                                 [](std::vector<bool> gpioValues, VehicleHal::VehiclePropValuePtr propValue) {
                                     propValue->areaId = SEAT_1_RIGHT;
                                     propValue->value.int32Values[0] = gpioValues[2]   ? 1
                                                                       : gpioValues[1] ? 0
                                                                       : gpioValues[0] ? -1
                                                                                       : -2;
                                     return propValue;
                                 },
                         }},
        (struct Pin){
            .isInput = false,
            .outputPin =
                (struct OutputPin){
                    .pin = PIN::LED_GREEN_1,
                    .property = VehicleProperty::HVAC_TEMPERATURE_SET,
                    .type = VehiclePropertyType::FLOAT,
                    .outputValue =
                        [](const VehiclePropValue &propValue) { return propValue.value.floatValues[0] <= 20.0; },
                }},
        (struct Pin){
            .isInput = false,
            .outputPin =
                (struct OutputPin){
                    .pin = PIN::LED_YELLOW_1,
                    .property = VehicleProperty::HVAC_TEMPERATURE_SET,
                    .type = VehiclePropertyType::FLOAT,
                    .outputValue =
                        [](const VehiclePropValue &propValue) {
                            return propValue.value.floatValues[0] > 20.0 && propValue.value.floatValues[0] < 30.0;
                        },
                },
        },
        (struct Pin){
            .isInput = false,
            .outputPin =
                (struct OutputPin){
                    .pin = PIN::LED_RED_1,
                    .property = VehicleProperty::HVAC_TEMPERATURE_SET,
                    .type = VehiclePropertyType::FLOAT,
                    .outputValue =
                        [](const VehiclePropValue &propValue) { return propValue.value.floatValues[0] >= 30.0; },
                },
        },

    };
}

std::vector<Pin> PINS = initPins();

GPIO::GPIO() {
    ALOGI("GPIO constructor");

    for (auto &pin : PINS) {
        std::vector<enum PIN> pins = pin.isInput ? pin.inputPin.pins : std::vector<enum PIN>{pin.outputPin.pin};
        std::vector<FILE *> fileDescriptors{};

        for (const auto &pinNumber : pins) {
            FILE *exportDevice = fopen("/sys/class/gpio/export", "w");
            if (exportDevice == NULL) {
                ALOGE("Failed to open /sys/class/gpio/export");
                continue;
            }

            if (fprintf(exportDevice, "%d", pinNumber) < 0) {
                ALOGE("Failed to export GPIO");
                fclose(exportDevice);
                continue;
            }

            fclose(exportDevice);

            // set direction of GPIO pin
            char path[256];
            snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pinNumber);
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
            snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pinNumber);

            FILE *fileDescriptor;
            if (pin.isInput) {
                fileDescriptor = fopen(path, "r");
            } else {
                fileDescriptor = fopen(path, "w");
            }

            if (fileDescriptor == NULL) {
                ALOGE("Failed to open %s", path);
                continue;
            }

            fileDescriptors.push_back(fileDescriptor);
        }

        if (pin.isInput) {
            pin.inputPin.fileDescriptors = fileDescriptors;
        } else {
            pin.outputPin.fileDescriptor = fileDescriptors[0];
        }
    }
}

GPIO::~GPIO() {
    ALOGI("GPIO destructor");
    // do stuff here
}

void GPIO::readAll(VehiclePropValuePool *pool, VehicleHalClient *vehicleClient) {
    ALOGI("GPIO readAll");
    for (const auto &pin : PINS) {
        if (!pin.isInput) {
            continue;
        }

        ALOGI("Reading value of property %d into vehicleClient", static_cast<int32_t>(pin.inputPin.property));

        VehicleHal::VehiclePropValuePtr v = pin.inputPin.read(pool);
        if (v == nullptr) {
            ALOGE("Failed to read value of property %d from GPIO", static_cast<int32_t>(pin.inputPin.property));
            continue;
        }

        vehicleClient->setProperty(*v, /*updateStatus=*/false);
    }
}

void GPIO::write(const VehiclePropValue &propValue) {
    ALOGI("GPIO write %d", propValue.prop);
    for (const auto &pin : PINS) {
        if (pin.isInput) {
            continue;
        }

        if (static_cast<int32_t>(pin.outputPin.property) != propValue.prop) {
            continue;
        }

        pin.outputPin.write(propValue);
    }

    ALOGE("Failed to write value of property %d to GPIO", propValue.prop);
}

} // namespace impl
} // namespace V2_0
} // namespace vehicle
} // namespace automotive
} // namespace hardware
} // namespace android
