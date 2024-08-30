source [find interface/jlink.cfg]
transport select swd
set CHIPNAME &&_MTB_RECIPE__OPENOCD_CHIP&&
source [find target/&&_MTB_RECIPE__OPEN_OCD_FILE&&]
${_TARGETNAME} configure -rtos auto -rtos-wipe-on-reset-halt 1
