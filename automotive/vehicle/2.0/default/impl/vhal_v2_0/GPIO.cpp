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

#include <errno.h>
#include <linux/gpio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

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
    SWITCH_1_B = 0,
    SWITCH_1_C = 5,
    LED_BLUE_1 = 9,
    LED_BLUE_2 = 10,
    LED_BLUE_3 = 22,
    SWITCH_2_A = 27,
    SWITCH_2_B = 4,
    SWITCH_2_C = 17,
};

struct InputPin {
    std::vector<enum PIN> pins;
    int fd;
    VehicleProperty property;
    VehiclePropertyType type;
    std::function<VehicleHal::VehiclePropValuePtr(std::vector<bool>, VehicleHal::VehiclePropValuePtr)> inputValue;

    // read the specified GPIO pins and compute the value of the property
    const VehicleHal::VehiclePropValuePtr read(VehiclePropValuePool *pool) const {
        ALOGI("Reading value of property %d from GPIO", static_cast<int32_t>(property));

        struct gpio_v2_line_values line_values = {
            .mask = (uint64_t)((1 << pins.size()) - 1),
            .bits = 0,
        };

        int ret = ioctl(fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &line_values);
        if (ret < 0) {
            ALOGE("InputPin::read() ioctl failed with error %d (%s)", errno, strerror(errno));
            return nullptr;
        }

        std::vector<bool> gpioValues = {};
        for (unsigned long i = 0; i < pins.size(); i++) {
            gpioValues.push_back((line_values.bits & (1 << i)) != 0);
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
    int fd;
    VehicleProperty property;
    VehiclePropertyType type;
    std::function<bool(const VehiclePropValue &)> outputValue;

    // write the value of the property to the specified GPIO pin
    void write(const VehiclePropValue &propValue) const {
        ALOGI("Writing value of property %d to GPIO", static_cast<int32_t>(property));

        uint64_t bits = static_cast<uint64_t>(outputValue(propValue));
        struct gpio_v2_line_values line_values = {
            .bits = bits,
            .mask = 0b1,
        };

        int ret = ioctl(fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &line_values);
        if (ret < 0) {
            ALOGE("OutputPin::write() ioctl failed with error %d (%s)", errno, strerror(errno));
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
                                 [](const VehiclePropValue &propValue) { return propValue.value.int32Values[0] == 1; },
                         }},
        (struct Pin){.isInput = false,
                     .outputPin =
                         (struct OutputPin){
                             .pin = PIN::LED_BLUE_2,
                             .property = VehicleProperty::HVAC_DEFROSTER,
                             .type = VehiclePropertyType::INT32,
                             .outputValue =
                                 [](const VehiclePropValue &propValue) { return propValue.value.int32Values[0] == 1; },
                         }},
        (struct Pin){.isInput = false,
                     .outputPin =
                         (struct OutputPin){
                             .pin = PIN::LED_BLUE_3,
                             .property = VehicleProperty::HVAC_RECIRC_ON,
                             .type = VehiclePropertyType::INT32,
                             .outputValue =
                                 [](const VehiclePropValue &propValue) { return propValue.value.int32Values[0] == 1; },
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
                                     propValue->areaId = SEAT_1_LEFT;
                                     propValue->value.int32Values[0] = gpioValues[2]   ? 3
                                                                       : gpioValues[1] ? 2
                                                                       : gpioValues[0] ? 1
                                                                                       : 0;
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
                            return propValue.value.floatValues[0] > 20.0 && propValue.value.floatValues[0] < 24.0;
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
                        [](const VehiclePropValue &propValue) { return propValue.value.floatValues[0] >= 24.0; },
                },
        },

    };
}

std::vector<Pin> PINS = initPins();

GPIO::GPIO() {
    ALOGI("GPIO constructor");

    int chip_fd = open("/dev/gpiochip0", O_RDWR);
    if (chip_fd < 0) {
        ALOGE("Failed to open /dev/gpiochip0");
        return;
    }

    struct gpiochip_info chip_info;
    int ret = ioctl(chip_fd, GPIO_GET_CHIPINFO_IOCTL, &chip_info);
    if (ret < 0) {
        ALOGE("GPIO::GPIO() ioctl failed with error %d (%s)", errno, strerror(errno));
        return;
    }

    ALOGI("GPIO chip name: %s", chip_info.name);
    ALOGI("GPIO chip label: %s", chip_info.label);
    ALOGI("GPIO chip lines: %d", chip_info.lines);

    for (auto &pin : PINS) {
        std::vector<enum PIN> pins = pin.isInput ? pin.inputPin.pins : std::vector<enum PIN>{pin.outputPin.pin};

        struct gpio_v2_line_config config = {
            .flags = GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN |
                     (uint64_t)(pin.isInput ? GPIO_V2_LINE_FLAG_INPUT : GPIO_V2_LINE_FLAG_OUTPUT),
            .num_attrs = 0,
        };

        struct gpio_v2_line_request line_request = {
            .offsets = {0},
            .consumer = "raspitainment",
            .config = config,
            .num_lines = (uint32_t)pins.size(),
            .event_buffer_size = 0,
            .padding = {0},
            .fd = -1,
        };
        for (size_t i = 0; i < pins.size(); i++) {
            line_request.offsets[i] = pins[i];
        }

        int ret = ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &line_request);
        if (ret < 0) {
            ALOGE("GPIO::GPIO() ioctl failed with error %d (%s)", errno, strerror(errno));
            return;
        }

        if (line_request.fd < 0) {
            ALOGE("GPIO::GPIO() failed to get line fd");
            return;
        }

        if (pin.isInput) {
            pin.inputPin.fd = line_request.fd;
        } else {
            pin.outputPin.fd = line_request.fd;
        }
    }
}

GPIO::~GPIO() {
    ALOGI("GPIO destructor");
    // ideally we would do stuff here
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
}

} // namespace impl
} // namespace V2_0
} // namespace vehicle
} // namespace automotive
} // namespace hardware
} // namespace android
