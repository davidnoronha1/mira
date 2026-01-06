# ArduPilot and MAVProxy Patches

This document provides instructions on how to apply the provided patches for ArduPilot and MAVProxy.

## DNT_VTC Motor Configuration Patch for ArduPilot

The `dnt_vtc.patch` file adds a custom motor configuration for a "DNT_VTC" frame to the ArduPilot codebase.

### Applying the patch

1.  Navigate to the root directory of your ArduPilot repository.
2.  Apply the patch using the following command:

    ```bash
    git apply /path/to/dnt_vtc.patch
    ```

    Replace `/path/to/dnt_vtc.patch` with the actual path to the patch file. The `-p1` option is important as it strips the leading directory information from the file paths embedded in the patch file.

## MAVProxy rline Fix

The `mavproxy_rline_fix_generic.patch` file fixes an issue with the `rline` module in MAVProxy.

### Applying the patch

1.  Locate your MAVProxy installation directory. This can vary depending on your system and installation method. A common location is within your Python `site-packages` directory.
2.  Navigate to the root of the MAVProxy installation directory.
3.  Apply the patch using the following command:

    ```bash
    patch -p0 < /path/to/mavproxy_rline_fix_generic.patch
    ```

    Replace `/path/to/mavproxy_rline_fix_generic.patch` with the actual path to the patch file.

    If you encounter an error when running MavProxy this patch can help you fix it