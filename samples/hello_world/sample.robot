*** Test Cases ***
Should Print Hello Message
    Execute Command           $bin = ${ELF}
    Execute Command           include ${RESC}
    Create Terminal Tester    ${UART}
    Start Emulation
    Wait For Line On Uart     Hello World!
