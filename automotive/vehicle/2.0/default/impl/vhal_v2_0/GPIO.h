#ifndef android_hardware_automotive_vehicle_V2_0_impl_GPIO_H_
#define android_hardware_automotive_vehicle_V2_0_impl_GPIO_H_

#include <android-base/chrono_utils.h>
#include <android/hardware/automotive/vehicle/2.0/types.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <unordered_set>
#include <utils/Log.h>
#include <utils/SystemClock.h>
#include <vhal_v2_0/GPIO.h>
#include <vhal_v2_0/RecurrentTimer.h>
#include <vhal_v2_0/VehicleHal.h>
#include <vhal_v2_0/VehiclePropertyStore.h>
#include <vhal_v2_0/VehicleUtils.h>

#include "DefaultVehicleHal.h"
#include "FakeObd2Frame.h"
#include "FakeUserHal.h"
#include "PropertyUtils.h"
#include "VehicleHalClient.h"
#include "VehicleUtils.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

namespace android {
namespace hardware {
namespace automotive {
namespace vehicle {
namespace V2_0 {
namespace impl {

class GPIO {
  public:
    GPIO();

    ~GPIO();

    bool isHandled(int prop);

    VehicleHal::VehiclePropValuePtr get(uint8_t pin, VehiclePropValuePool *pool);

    void writeAll(VehiclePropValuePool *pool, VehicleHalClient *vehicleClient);

    void read(const VehiclePropertyValue &propValue);

} // namespace impl
} // namespace V2_0
} // namespace vehicle
} // namespace automotive
} // namespace hardware
} // namespace android

#endif // android_hardware_automotive_vehicle_V2_0_impl_GPIO_H_