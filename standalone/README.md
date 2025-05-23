# SR1XX Driver Standalone Test Application

This application is a **standalone test tool** for initializing and testing the SR1XX driver in various roles and configurations. It provides simple role-based setup and supports receiving data through a registered callback.

---

## Overview

The application initializes the SR1XX driver, configures it for a specific role, and starts it for ranging. It also includes a callback mechanism to handle received data.

This tool is designed for testing and validation purposes, demonstrating the usage of the SR1XX driver in different modes.

---

## Supported Roles

1. **Initiator (`i`)**: Configures the device as a Time-of-Flight ranging initiator.
2. **Responder (`r`)**: Sets the device as a responder.
3. **Tag (`t`)**: Configures the device as a tag in Time Difference of Arrival (TDoA) mode.
4. **Anchor Controller (`1`)**: Sets up the device as a controller managing multiple controlees.
5. **Anchor Controlee (`2`)**: Configures the device as a controlee in anchor mode.

---

## How to Use

### Build and Run
Compile the program and execute it with the desired role. You can also specify custom MAC addresses.


## How to Build

This project uses **CMake** for building. Follow the steps below:


1. **Create a Build Directory**:
   ```bash
   mkdir build
   cd build
   ```

2. **Configure the Project**:
   ```bash
   cmake ..
   ```

3. **Build the Application**:
   ```bash
   make
   ```

4. **Run the Application**:
   The compiled binary will be located in the `build` directory:
   ```bash
   ./sr1xx-standalone <role> [source_mac] [dest_mac] [...additional_dest_macs]
   ```

### Examples
1. **Default Initiator**:
   ```bash
   ./sr1xx-standalone i
   ```

2. **Responder with Custom MACs**:
   ```bash
   ./sr1xx-standalone r 0x2222 0x1111
   ```

3. **Anchor Controller with Multiple Controlees**:
   ```bash
   ./sr1xx-standalone 1 0x1111 0x2222 0x3333
   ```

---

## Example Callback

The callback simply prints "Callback" when data is received:
```c
void rx_callback(uint8_t *payload, size_t size, void *user_data) {
    printf("Callback\n");
}
```

---

## Notes

- The application runs in an infinite loop, continuously awaiting incoming data.
- Errors during driver initialization or startup are reported to the console.