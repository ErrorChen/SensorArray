idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_timer
    PRIV_REQUIRES boardSupport powerCtrl tmuxSwitch ads126xAdc fdc2214Cap protocolWire transportWire bleTransport
)
