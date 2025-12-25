idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        boardSupport
        powerCtrl
        tmuxSwitch
        ads126xAdc
        fdc2214Cap
        protocolWire
        protocolUsb
        transportWire
        bleTransport
        matrixEngine
)
