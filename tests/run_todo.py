from pathlib import Path
import argparse,subprocess,shutil,os
p=argparse.ArgumentParser();p.add_argument('--cc',default=shutil.which('gcc') or 'gcc');a=p.parse_args()
root=Path(__file__).resolve().parents[1];fw=root/'firmware';build=root/'.host-build';build.mkdir(exist_ok=True)
cases=[('journal',[fw/'services/parameters/param_journal.c',root/'tests/journal_tests.c'])]
if (fw/'services/chassis_service.c').exists():
    cases.append(('parameters',[fw/'services/parameters/chassis_parameters.c',fw/'services/protocol/star_protocol.c',fw/'services/protocol/star_dispatch.c',root/'tests/parameter_tests.c']))
else:
    cases.append(('gyro_calibration',[fw/'algorithms/calibration/gyro_calibration.c',root/'tests/gyro_calibration_tests.c']))
    cases.append(('flight',[fw/'app/flight_machine.c',fw/'services/parameters/calibration_record.c',fw/'services/parameters/mag_record.c',fw/'algorithms/calibration/mag_calibration.c',fw/'services/parameters/accel_record.c',fw/'algorithms/calibration/accel_calibration.c',fw/'services/protocol/star_protocol.c',root/'tests/flight_tests.c']))
    cases.append(('gui_navigation',[fw/'gui/gui_dashboard.c',fw/'gui/gui_menu.c',root/'tests/gui_navigation_tests.c']))
    cases.append(('imu_processing',[fw/'algorithms/filter/biquad.c',fw/'algorithms/attitude/fusion.c',fw/'algorithms/calibration/gyro_calibration.c',fw/'services/imu_pipeline.c',fw/'platform/stm32/imu_sample_decode.c',root/'tests/imu_processing_tests.c']))
includes=[fw/'algorithms/calibration',fw/'services/parameters',fw/'services/protocol',fw/'services',fw/'algorithms/chassis',fw/'boards/stm32',fw/'app']
includes += [fw/'gui',fw/'third_party/u8g2/csrc']
includes += [fw/'algorithms/filter',fw/'algorithms/attitude']
includes += [fw/'platform/stm32']
for name,sources in cases:
    exe=build/(name+('_tests.exe' if os.name=='nt' else '_tests'))
    subprocess.run([a.cc,'-std=c11','-Wall','-Wextra','-Werror','-O2',*(['-flto'] if name == 'gui_navigation' else []),'-ffunction-sections','-fdata-sections',*['-I'+str(x) for x in includes],*[str(x) for x in sources],'-Wl,--gc-sections','-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
