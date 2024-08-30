::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
:: an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
::
:: This software, including source code, documentation and related
:: materials ("Software") is owned by Cypress Semiconductor Corporation
:: or one of its affiliates ("Cypress") and is protected by and subject to
:: worldwide patent protection (United States and foreign),
:: United States copyright laws and international treaty provisions.
:: Therefore, you may use this Software only as provided in the license
:: agreement accompanying the software package from which you
:: obtained this Software ("EULA").
:: If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
:: non-transferable license to copy, modify, and compile the Software
:: source code solely for use in connection with Cypress's
:: integrated circuit products.  Any reproduction, modification, translation,
:: compilation, or representation of this Software except as specified
:: above is prohibited without the express written permission of Cypress.
::
:: Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
:: EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
:: WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
:: reserves the right to make changes to the Software without notice. Cypress
:: does not assume any liability arising out of the application or use of the
:: Software or any product or circuit described in the Software. Cypress does
:: not authorize its products for use in any products where a malfunction or
:: failure of the Cypress product may reasonably be expected to result in
:: significant property damage, injury or death ("High Risk Product"). By
:: including Cypress's product in a High Risk Product, the manufacturer
:: of such system or application assumes all risk of such use and in doing
:: so agrees to indemnify Cypress against all liability.
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

@echo off

chcp 65001
set auto_dir=%CD%
cd ../../mtb_shared/motor-ctrl-lib/r*/OperationalCode/

echo.
echo "   ____                              _               _____         _        "
echo "  |  _ \ ___  __ _ _ __ ___  ___ ___(_) ___  _ __   |_   _|__  ___| |_ ___  "
echo "  | |_) / _ \/ _` | '__/ _ \/ __/ __| |/ _ \| '_ \    | |/ _ \/ __| __/ __| "
echo "  |  _ <  __/ (_| | | |  __/\__ \__ \ | (_) | | | |   | |  __/\__ \ |_\__ \ "
echo "  |_| \_\___|\__, |_|  \___||___/___/_|\___/|_| |_|   |_|\___||___/\__|___/ "
echo "             |___/                                                          "

echo.
echo " ░█▀▄░█▀▄░█▀▀░█▀█ "
echo " ░█▀▄░█▀▄░█▀▀░█░█ "
echo " ░▀░▀░▀░▀░▀░░░▀▀▀ "
echo.
..\RegressionTests\Build\Win32\ReleaseRFO\RegressionTestsWin32ReleaseRFO -v

echo.
echo " ░█▀▄░█▀▀░█▀▀░█▀█ "
echo " ░█▀▄░▀▀█░█▀▀░█░█ "
echo " ░▀░▀░▀▀▀░▀░░░▀▀▀ "
echo.
..\RegressionTests\Build\Win32\ReleaseSFO\RegressionTestsWin32ReleaseSFO -v

echo.
echo " ░█▀▄░▀█▀░█▀▄░█▀▀ "
echo " ░█▀▄░░█░░█▀▄░█░░ "
echo " ░▀░▀░░▀░░▀▀░░▀▀▀ "
echo.
..\RegressionTests\Build\Win32\ReleaseTBC\RegressionTestsWin32ReleaseTBC -v

echo.
echo " ░█▀▄░█▀▄░█▀▀░█▀█ "
echo " ░█░█░█▀▄░█▀▀░█░█ "
echo " ░▀▀░░▀░▀░▀░░░▀▀▀ "
echo.
..\RegressionTests\Build\Win32\DebugRFO\RegressionTestsWin32DebugRFO -v

echo.
echo " ░█▀▄░█▀▀░█▀▀░█▀█ "
echo " ░█░█░▀▀█░█▀▀░█░█ "
echo " ░▀▀░░▀▀▀░▀░░░▀▀▀ "
echo.
..\RegressionTests\Build\Win32\DebugSFO\RegressionTestsWin32DebugSFO -v

echo.
echo " ░█▀▄░▀█▀░█▀▄░█▀▀ "
echo " ░█░█░░█░░█▀▄░█░░ "
echo " ░▀▀░░░▀░░▀▀░░▀▀▀ "
echo.
..\RegressionTests\Build\Win32\DebugTBC\RegressionTestsWin32DebugTBC -v

cd %auto_dir%
