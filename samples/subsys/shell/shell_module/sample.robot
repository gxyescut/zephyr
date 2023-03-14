*** Test Cases ***
Should Read Version From Shell
    Execute Command           $bin = ${ELF}
    Execute Command           include ${RESC}
    Create Terminal Tester    ${UART}
    Start Emulation
    Wait For Prompt On Uart   uart:~$
    Write Line To Uart        version
    Wait For Line On Uart     Zephyr version
