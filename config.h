#pragma once

#define USE_TEST_NETWORK 1

#if USE_TEST_NETWORK
    #define STA_SSID        "DeFord_5"
    #define STA_PASSWORD    "jaggedsky483"
#else
    #define STA_SSID        "DroneNetwork"
    #define STA_PASSWORD    NULL
#endif

