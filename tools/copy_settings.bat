@echo off
rem Both programs, e33_mission and e33_calibrate, must use the same
rem calibration.h. Double-click this after you change either copy: it
rem copies the one you changed LAST over the other one.
rem (The messages come from comparing the files, not from what xcopy prints,
rem so they are the same in every Windows language.)
set HERE=%~dp0
set M=%HERE%..\e33_mission
set C=%HERE%..\e33_calibrate
if not exist "%M%\calibration.h" goto missing
if not exist "%C%\calibration.h" goto missing
fc /b "%M%\calibration.h" "%C%\calibration.h" >nul && goto same
rem xcopy /D copies only when the source is newer than the copy it replaces
xcopy /D /Y /Q "%C%\calibration.h" "%M%\" >nul
fc /b "%M%\calibration.h" "%C%\calibration.h" >nul && echo e33_calibrate\calibration.h was newer: copied into e33_mission.&& goto done
xcopy /D /Y /Q "%M%\calibration.h" "%C%\" >nul
fc /b "%M%\calibration.h" "%C%\calibration.h" >nul && echo e33_mission\calibration.h was newer: copied into e33_calibrate.&& goto done
echo NOT COPIED: the two calibration.h differ and neither is newer.
echo Copy the one you changed into the other folder by hand (choose "Replace").
goto end
:same
echo The two calibration.h were already the same: nothing to copy.
goto end
:done
echo The two calibration.h are the same now. Upload again to use the new numbers.
goto end
:missing
echo Could not find calibration.h. This file must be in the tools folder, next to e33_mission and e33_calibrate.
echo (If you opened the ZIP without unzipping it: right-click the ZIP, "Extract All", and use that folder.)
:end
pause
