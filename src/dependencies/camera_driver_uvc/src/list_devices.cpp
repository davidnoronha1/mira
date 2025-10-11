#include <libuvc/libuvc.h>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include "camera_driver_uvc/clog.hpp"

auto list_devices(uvc_context_t *ctx) -> int
{
    uvc_error_t res;
    uvc_device_t **dev_list;
    // Retrieve list of devices
    res = uvc_get_device_list(ctx, &dev_list);
    if (res != UVC_SUCCESS)
    {
        uvc_perror(res, "uvc_get_device_list");
        uvc_exit(ctx);
        return EXIT_FAILURE;
    }

    plog::info() << "Found devices:";
    for (int i = 0; dev_list[i] != NULL; ++i)
    {
        uvc_device_t *dev = dev_list[i];
        uvc_device_descriptor_t *desc;

        // Get device descriptor details
        res = uvc_get_device_descriptor(dev, &desc);
        if (res != UVC_SUCCESS)
        {
            uvc_perror(res, "uvc_get_device_descriptor");
            continue;
        }

        std::cout << " [" << i << "] Vendor ID: 0x"
                  << std::hex << std::setw(4) << std::setfill('0') << desc->idVendor
                  << ", Product ID: 0x" << std::setw(4) << std::setfill('0') << desc->idProduct
                  << ", Product Name: " << (desc->product ? desc->product : "(none)")
                  << ", Serial Number: " << (desc->serialNumber ? desc->serialNumber : "(none)")
                  << std::endl;

        uvc_free_device_descriptor(desc);
    }

    // Cleanup
    uvc_free_device_list(dev_list, 1); // unref_devices = 1
    return EXIT_SUCCESS;
}