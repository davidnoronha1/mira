#include <libuvc/libuvc.h>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>

auto list_devices(uvc_context_t *ctx, rclcpp::Node::SharedPtr nh) -> int
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

    RCLCPP_INFO(nh->get_logger(), "Found devices:");
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

        RCLCPP_INFO(nh->get_logger(), " [%d] Vendor ID: 0x%04x, Product ID: 0x%04x, Product Name: %s, Serial Number: %s",
               i, desc->idVendor, desc->idProduct, desc->product, desc->serialNumber);

        if (desc->serialNumber)
            RCLCPP_INFO(nh->get_logger(), ", Serial: %s", desc->serialNumber);

        uvc_free_device_descriptor(desc);
    }

    // Cleanup
    uvc_free_device_list(dev_list, 1); // unref_devices = 1
    return EXIT_SUCCESS;
}