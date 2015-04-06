subst W: /D
subst W: "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility"
start /WAIT W:\ST-LINK_CLI.exe -c -ME -P %1\Release\%2.hex 0x8000000
subst W: /D