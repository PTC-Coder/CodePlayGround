# Magpie_BLE_example
Based on Max32665 Bluetooth fitness device. Modified with ability to broadcast a custom manufacturer and productID. Showcases various characteristics.
Refer to [BLE_fit](../../../Libraries/Cordio/docs/Applications/BLE_fit.md) documentation in the Cordio Library.

## Software
Use any BLE discovery app on Android or iOS to find "MagpieRecorder" in the broadcast ID list.  Connect to the unit to further browse the characteristics.  User can also test data send and receive.

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

Note that custom project BLE folders are used to override the default Cardio BLE in MSDK.

### Required Connections
Use MAX32666FTHR board

### Project-Specific Build Notes
* Setting `TRACE=1` in [**project.mk**](project.mk) initializes the on-board USB-to-UART adapter for
viewing the trace messages and interacting with the application. Port uses settings:
    - Baud            : 115200  
    - Char size       : 8  
    - Parity          : None  
    - Stop bits       : 1  
    - HW Flow Control : No  
    - SW Flow Control : No  

